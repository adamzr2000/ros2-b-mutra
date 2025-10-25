# prover.py
import time
import logging
from fastapi import FastAPI
from blockchain_client import AttestationState
from compute_hash import compute_program_hash, ComputeHashError
from RedisStorageBackend import RedisStorageBackend
from MemoryStorageBackend import MemoryStorageBackend
import utils
import hashlib
import threading

GREEN = "\033[92m"
RESET = "\033[0m"
logger = logging.getLogger(__name__)

def _should_stop(stop_event):
    return bool(stop_event and stop_event.is_set())

def process_prover_attestation(app: FastAPI, attestation_id: str, measurements, stop_event=None):
    timestamps = {}
    blockchain_client = app.state.blockchain_client
    log_prefix = f"[Prover-{utils.short_att_id(attestation_id)}]"

    try:
        # Initiate attestation here
        timestamps["evidence_sent"] = int(time.time() * 1000)
        tx_hash = blockchain_client.send_evidence(attestation_id, measurements)
        logger.info(f"{log_prefix} Evidence sent (attestation started)")

    except Exception as e:
        logger.error(f"{log_prefix} Failed to send evidence: {e}")
        return
    
    while not blockchain_client.is_prover_agent(attestation_id):
        time.sleep(0.05)
    
    # Wait until this agent is recognized as the prover (or stop/timeout)
    start_wait = time.time()
    TIMEOUT_S = 60
    while not blockchain_client.is_prover_agent(attestation_id):
        if _should_stop(stop_event):
            logger.info(f"{log_prefix} Stopping while waiting to become prover.")
            return
        if time.time() - start_wait > TIMEOUT_S:
            logger.error(f"{log_prefix} Timeout while waiting to become prover.")
            return
        time.sleep(0.05)
        
    logger.info(f"{log_prefix} Waiting for verification result...")
    # Wait for attestation to close (or stop/timeout)
    start_wait = time.time()
    while blockchain_client.get_attestation_state(attestation_id) != AttestationState.Closed:
        if _should_stop(stop_event):
            logger.info(f"{log_prefix} Stopping while waiting for attestation to close.")
            return
        if time.time() - start_wait > TIMEOUT_S:
            logger.error(f"{log_prefix} Timeout while waiting for attestation to close.")
            return
        time.sleep(0.05)

    timestamps["result_received"] = int(time.time() * 1000)
    # Retrieve attestation details
    try:
        prover_address, verifier_address, attestation_result, timestamp = blockchain_client.get_attestation_info(attestation_id)
        is_success = (attestation_result == 2)
        logger.info(f"{log_prefix} Attestation closed (result: {'✅ SUCCESS' if is_success else '❌ FAILURE'})")
    except Exception as e:
        logger.error(f"Failed to read attestation info: {e}")

    if app.state.export_enabled:
        try:
            utils.export_attestation_times_json(
                app.state.participant_name, attestation_id, "prover", timestamps, app.state.results_dir
            )
        except Exception as e:
            logger.error(f"{log_prefix} Failed exporting prover timings: {e}")

def run_prover_and_cleanup(app: FastAPI, attestation_id: str, measurements, stop_event=None):
    start = time.time()
    try:
        process_prover_attestation(app, attestation_id, measurements, stop_event=stop_event)
    finally:
        elapsed = time.time() - start
        log_prefix = f"[Prover-{utils.short_att_id(attestation_id)}]"
        logger.info(f"{GREEN}{log_prefix} completed in {elapsed:.2f}s{RESET}")
        app.state.active_attestations.pop(attestation_id, None)

def run_prover_logic_continuous_mode(app: FastAPI, stop_event=None):
    
    iteration = 0
    config = {
        'cmd_name': app.state.cmd_name,
        'text_section_size': app.state.text_section_size,
        'offset': app.state.offset,
    }
    threshold = app.state.prover_threshold
    storage = app.state.storage
    memory_storage_file = app.state.memory_storage_file
    agent_name = app.state.participant_name

    logging.info(f"Starting attestation process in continuous mode with threshold {threshold}...")

    # Only measure while the flag is on; also allow external stop via stop_event
    while getattr(app.state, "measuring", False) and not _should_stop(stop_event):
        iteration += 1
        try:
            logging.debug(f"Attestation iteration {iteration}: Computing hash for {config['cmd_name']}")
            
            digest = compute_program_hash(
                config['cmd_name'],
                config['text_section_size'],
                config['offset']
            )
            app.state.digests.append(digest)
            app.state.digests_count += 1

            if _should_stop(stop_event):
                break
        
            # count = storage.incr_digest_count(f"measure_digests:{agent_name}")
            # logging.info(f"Digest added: {app.state.digests_count}/{threshold} collected (total: {count})")

            aggregated_digests = "".join(app.state.digests)
            computed_digest = hashlib.sha256(aggregated_digests.encode()).hexdigest()
            # logging.debug(f"Current aggregated digest: {computed_digest[:8]}...{computed_digest[-8:]}")

            if app.state.digests_count >= threshold:
                final_digest = computed_digest
                app.state.final_digest = final_digest

                logging.info(f"[Prover] Threshold reached. Final digest: {final_digest[:16]}...{final_digest[-16:]}")
                storage.push_final_digest(f"final_digests:{agent_name}", {
                    "final_digest": final_digest,
                    "threshold": threshold,
                    "timestamp": time.time()
                })

                # Send evidence to smart contract (new attestation process)
                attestation_id = utils.generate_attestation_id()
                signatures = [final_digest, final_digest, final_digest]
                thread = threading.Thread(
                    target=run_prover_and_cleanup,
                    args=(app, attestation_id, signatures, stop_event),
                    name=agent_name,
                    daemon=True
                )
                app.state.active_attestations[attestation_id] = thread
                thread.start()

                # Also track the thread globally so shutdown can join
                if hasattr(app.state, "threads"):
                    app.state.threads.append(thread)

                time.sleep(3)
                
                logging.debug(f"Final digest stored in key: final_digests:{app.state.participant_name}")
                app.state.digests = []
                app.state.digests_count = 0
                logging.debug("Digest counters reset")
                
                # Export storage to file if MemoryStorageBackend is used
                if isinstance(storage, MemoryStorageBackend) and memory_storage_file:
                    if storage.export_to_file(memory_storage_file):
                        logging.debug(f"💾 Memory storage exported to {memory_storage_file} after threshold reached")
                    else:
                        logging.error(f"Failed to export memory storage to {memory_storage_file}")
            
            # Traiter le digest calculé
            # process_digest(digest, config)
            time.sleep(0.05)

            
        except ComputeHashError as e:
            logging.error(f"Hash computation error in iteration {iteration}: {str(e)}")
            logging.debug(f"Error details: {e.__class__.__name__}")
            time.sleep(3)  # Attendre un peu avant de réessayer
        except Exception as e:
            logging.error(f"Error in attestation process (iteration {iteration}): {str(e)}")
            logging.error(f"Error type: {e.__class__.__name__}")
            import traceback
            logging.debug(f"Traceback: {traceback.format_exc()}")
            time.sleep(3)  # Attendre plus longtemps pour les erreurs génériques

    logging.info("Attestation process stopped")