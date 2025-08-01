#!/bin/python
import json
import os
import hashlib
import logging
import threading
import time
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv
from compute_hash import compute_program_hash, ComputeHashError
from RedisStorageBackend import RedisStorageBackend
from MemoryStorageBackend import MemoryStorageBackend
import sys

from web3 import Web3
from web3.middleware import geth_poa_middleware
from agent_logic import handle_agent_logic
import blockchain_interface

load_dotenv()

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()

# Enhanced logging setup
if LOG_LEVEL == "NONE":
    logging.disable(logging.CRITICAL)
else:
    numeric_level = getattr(logging, LOG_LEVEL, logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )


# Backend setup
USE_REDIS = os.getenv("USE_REDIS", "false").lower() == "true"
try:
    storage = RedisStorageBackend() if USE_REDIS else MemoryStorageBackend()
except:
    logging.warning("Redis unavailable, using Memory backend")
    storage = MemoryStorageBackend()

# Globals
threshold = int(os.getenv("THRESHOLD", 300))
BOOTSTRAP = os.getenv("BOOTSTRAP", "FALSE").upper() == "TRUE"
AUTO_START = os.getenv("AUTO_START", "FALSE").upper() == "TRUE"
ConfigSidecar = os.getenv("CONFIGSIDECAR", "")
agent_name = os.getenv('AGENT_NAME', '')
MEMORY_STORAGE_FILE = os.getenv('MEMORY_STORAGE_FILE', '')

logging.info(f"BOOTSTRAP={BOOTSTRAP}, AUTO_START={AUTO_START}, CONFIGSIDECAR='{ConfigSidecar}', THRESHOLD={threshold}, AGENT_NAME={agent_name}")
if MEMORY_STORAGE_FILE:
    logging.info(f"Memory storage will be saved to: {MEMORY_STORAGE_FILE}")

measuring = False
attestation_thread = None
lock = threading.Lock()

digests = []
digests_count = 0
final_digest = ""

app = FastAPI()


# Charger les variables d'environnement nécessaires ou définir les chemins ici
eth_node_url = os.getenv('ETH_NODE_URL', '')
#contract_address = os.getenv('CONTRACT_ADDRESS', '')
abi_path = os.getenv('CONTRACT_ABI_PATH', '')

web3 = None
mas_contract = None
contract_abi = None

# Load configuration
json_file_path = os.getenv("CONFIGSIDECAR", "")

try:
    with open(json_file_path, "r") as json_file:
        data = json.load(json_file)

    eth_address = data["eth_address"]
    private_key = data["private_key"]
    eth_node_url = data["eth_node"]
    contract_address = Web3.toChecksumAddress(data["contract_address"])

except FileNotFoundError:
    logging.error(f"Configuration file not found: {json_file_path}")
    sys.exit(1)
except KeyError as e:
    logging.error(f"Missing key {e} in configuration file")
    sys.exit(1)
except Exception as e:
    logging.error(f"Unexpected error: {str(e)}")
    sys.exit(1)

logging.info(f"Configuration loaded from {json_file_path}")

# Initialize Web3 + contract via blockchain_interface
mas_contract=blockchain_interface.init_web3_interface(
    _eth_address    = eth_address,
    _private_key    = private_key,
    eth_node_url    = eth_node_url,
    abi_path        = abi_path,
    contract_address= contract_address
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def read_and_validate_config():
    """Lire et valider la configuration pour l'attestation"""
    logging.info(f"Reading configuration from {ConfigSidecar}")
    try:
        with open(ConfigSidecar, 'r') as f:
            config = json.load(f)
            return {
                'cmd_name': config.get('cmd_name'),
                'text_section_size': config.get('text_section_size'),
                'offset': config.get('offset')  ,
                'eth_address': config.get('eth_address'),
                'private_key': config.get('private_key'),
                'eth_node_url': config.get('eth_node_url'),
                'contract_address': config.get('contract_address')
            }
    except Exception as e:
        logging.error(f"Error reading config file {ConfigSidecar}: {str(e)}")
        return None


def process_digest(digest, config):
    """Traiter un digest calculé et mettre à jour les compteurs"""
    global digests, digests_count, final_digest
    
    # Suppression du verrou pour éviter les blocages
    digests.append(digest)
    digests_count += 1
    # count = storage.incr_digest_count(f"measure_digests:{agent_name}")
    # logging.info(f"Digest added: {digests_count}/{threshold} collected (total: {count})")

    aggregated_digests = "".join(digests)
    computed_digest = hashlib.sha256(aggregated_digests.encode()).hexdigest()
    logging.debug(f"Current aggregated digest: {computed_digest[:8]}...{computed_digest[-8:]}")

    if digests_count >= threshold:
        final_digest = computed_digest
        logging.info(f"Threshold reached! Final digest: {final_digest[:16]}...{final_digest[-16:]}")
        storage.push_final_digest(f"final_digests:{agent_name}", {
            "final_digest": final_digest,
            "threshold": threshold
        })
        logging.info(f"Final digest stored in key: final_digests:{agent_name}")
        digests = []
        digests_count = 0
        logging.info("Digest counters reset")


def compute_single_hash(config):
    """Calculer un seul hash pour une configuration donnée"""
    if not all(k in config and config[k] is not None for k in ['cmd_name', 'text_section_size', 'offset']):
        missing = [k for k in ['cmd_name', 'text_section_size', 'offset'] if k not in config or config[k] is None]
        logging.error(f"Missing required configuration values: {', '.join(missing)}")
        return None
    
    logging.info(f"Computing hash for {config['cmd_name']}")
    logging.debug(f"Parameters: text_size={config['text_section_size']}, offset={config['offset']}")
    
    try:
        digest = compute_program_hash(
            config['cmd_name'],
            config['text_section_size'],
            config['offset']
        )
        logging.info(f"Hash computed: {digest[:8]}...{digest[-8:]}")
        return digest
    except ComputeHashError as e:
        logging.error(f"Hash computation error: {str(e)}")
        logging.debug(f"Error details: {e.__class__.__name__}")
        return None
    except Exception as e:
        logging.error(f"Error computing hash: {str(e)}")
        logging.error(f"Error type: {e.__class__.__name__}")
        import traceback
        logging.debug(f"Traceback: {traceback.format_exc()}")
        return None


def attestation_process_bootstrap():
    """Fonction d'attestation pour le mode bootstrap (une seule exécution)"""
    global measuring, nonce
    logging.info("=== Starting bootstrap attestation process ===")
    
    # Lire et valider la configuration
    config = read_and_validate_config()
    if not config:
        return
    
    # Calculer un seul hash
    digest = compute_single_hash(config)
    if digest:
        logging.info("Bootstrap attestation completed successfully")
        
        # Stocker le digest et le timestamp dans le storage
        timestamp = time.time()
        # Charger l'UUID du fichier de configsidecar
        config_uuid = None
        if ConfigSidecar:
            try:
                import json
                with open(ConfigSidecar, 'r') as f:
                    config_data = json.load(f)
                    config_uuid = config_data.get('uuid') or config_data.get('id')
            except Exception as e:
                logging.warning(f"Could not read uuid from {ConfigSidecar}: {e}")
                
        storage.unpack_set(
            digest=digest,
            timestamp=timestamp,
            agent_name=agent_name,
            uuid=config_uuid
        )

        logging.info(f"Digest and timestamp stored in storage with key bootstrap_digest:{agent_name}")
        # --- Appel logique agent/prover ici ---
        # Exemple d'appel pour agent avec les paramètres web3 et private_key
        blockchain_interface.request_attestation(eth_address, mas_contract)
        time.sleep(3)  # Attendre un peu pour s'assurer que l'attestation est traitée

    else:
        logging.error("Bootstrap attestation failed")

    # Export storage to file if MemoryStorageBackend is used
    if isinstance(storage, MemoryStorageBackend) and MEMORY_STORAGE_FILE:
        if storage.export_to_file(MEMORY_STORAGE_FILE):
            logging.info(f"Memory storage exported to {MEMORY_STORAGE_FILE} after bootstrap")
        else:
            logging.error(f"Failed to export memory storage to {MEMORY_STORAGE_FILE}")


    measuring = False
    # storage.set_status(f"attestation_running:{agent_name}", str(measuring))
    logging.info("=== Bootstrap attestation process stopped ===")


def attestation_process_continuous():
    """Fonction d'attestation pour le mode continu (en boucle)"""
    global measuring, digests, digests_count, final_digest
    logging.info("=== Starting continuous attestation process ===")
    
    # Lire et valider la configuration
    config = read_and_validate_config()
    if not config:
        return

    config_uuid = None
    if ConfigSidecar:
        try:
            import json
            with open(ConfigSidecar, 'r') as f:
                config_data = json.load(f)
                config_uuid = config_data.get('uuid') or config_data.get('id')
        except Exception as e:
            logging.warning(f"Could not read uuid from {ConfigSidecar}: {e}")
    
    iteration = 0
    while measuring:
        iteration += 1
        try:
            # Vérifier que les valeurs requises sont présentes dans la configuration
            if not all(k in config and config[k] is not None for k in ['cmd_name', 'text_section_size', 'offset']):
                missing = [k for k in ['cmd_name', 'text_section_size', 'offset'] if k not in config or config[k] is None]
                logging.error(f"Missing required configuration values: {', '.join(missing)}")
                time.sleep(5)  # Attendre avant de réessayer
                continue
            
            logging.info(f"Attestation iteration {iteration}: Computing hash for {config['cmd_name']}")
            
            digest = compute_program_hash(
                config['cmd_name'],
                config['text_section_size'],
                config['offset']
            )


            digests.append(digest)
            digests_count += 1
            # count = storage.incr_digest_count(f"measure_digests:{agent_name}")
            # logging.info(f"Digest added: {digests_count}/{threshold} collected (total: {count})")

            aggregated_digests = "".join(digests)
            computed_digest = hashlib.sha256(aggregated_digests.encode()).hexdigest()
            logging.debug(f"Current aggregated digest: {computed_digest[:8]}...{computed_digest[-8:]}")

            if digests_count >= threshold:
                final_digest = computed_digest
                logging.info(f"Threshold reached! Final digest: {final_digest[:16]}...{final_digest[-16:]}")
                storage.push_final_digest(f"final_digests:{agent_name}", {
                    "final_digest": final_digest,
                    "threshold": threshold,
                    "timestamp": time.time()
                })

                logging.info("Requesting attestation...")
                blockchain_interface.request_attestation(eth_address, mas_contract)
                time.sleep(3)  # Attendre un peu pour s'assurer que l'attestation est traitée

                
                logging.info(f"Final digest stored in key: final_digests:{agent_name}")
                digests = []
                digests_count = 0
                logging.info("Digest counters reset")
                
                # Export storage to file if MemoryStorageBackend is used
                if isinstance(storage, MemoryStorageBackend) and MEMORY_STORAGE_FILE:
                    if storage.export_to_file(MEMORY_STORAGE_FILE):
                        logging.info(f"Memory storage exported to {MEMORY_STORAGE_FILE} after threshold reached")
                    else:
                        logging.error(f"Failed to export memory storage to {MEMORY_STORAGE_FILE}")
            
            # Traiter le digest calculé
            # process_digest(digest, config)
            
        except ComputeHashError as e:
            logging.error(f"Hash computation error in iteration {iteration}: {str(e)}")
            logging.debug(f"Error details: {e.__class__.__name__}")
            time.sleep(2)  # Attendre un peu avant de réessayer
        except Exception as e:
            logging.error(f"Error in attestation process (iteration {iteration}): {str(e)}")
            logging.error(f"Error type: {e.__class__.__name__}")
            import traceback
            logging.debug(f"Traceback: {traceback.format_exc()}")
            time.sleep(5)  # Attendre plus longtemps pour les erreurs génériques

    logging.info("=== Continuous attestation process stopped ===")


def start_attestation():

    # TODO: 
    # - append register agent function
    global measuring, attestation_thread
    if measuring and attestation_thread and attestation_thread.is_alive():
        logging.info("Attestation process already running")
        return
    measuring = True

    config = read_and_validate_config()
    if not config:
        logging.error("Failed to read sidecar configuration")
        return

    if BOOTSTRAP:
        logging.info("BOOTSTRAP mode: running attestation once synchronously.")
        attestation_process_bootstrap()
        measuring = False
    else:
        logging.info("Continuous mode: starting attestation thread.")
        attestation_thread = threading.Thread(target=attestation_process_continuous, daemon=True)
        attestation_thread.start()


@app.get("/startAttestation")
def start_attestation_endpoint():
    start_attestation()
    return JSONResponse(content={"message": "Attestation process started"})

@app.get("/deactivateAttestation")
def stop_attestation_endpoint():
    global measuring
    measuring = False
    # storage.set_status(f"attestation_running:{agent_name}", str(measuring))
    return JSONResponse(content={"message": "Attestation deactivated"})

# Startup event to auto-start attestation if AUTO_START is True
@app.on_event("startup")
async def startup_event():
    import threading
    import time
    from pause_on_process import pause_container_on_proc1
    # threading.Thread(target=pause_container_on_proc1, daemon=True).start()

    if AUTO_START:
        logging.info("AUTO_START is enabled, starting attestation automatically")
        time.sleep(8)  # Sleep for 5 seconds before starting attestation
        start_attestation()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8080, reload=True)
