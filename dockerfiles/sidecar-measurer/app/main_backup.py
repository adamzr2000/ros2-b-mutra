#!/bin/python
import json
import os
import hashlib
import logging
import threading
import time
import redis
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv
from app.compute_hash import compute_program_hash

load_dotenv()

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

# Redis setup
redis_client = redis.Redis(host='redis', port=6379, db=0)

# Globals
threshold = 300
measuring = False
attestation_thread = None
lock = threading.Lock()

# Digests
digests = []
digests_count = 0
final_digest = ""
agent_name = os.getenv('AGENT_NAME', 'agent1')


# FastAPI app
app = FastAPI()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def get_agent_data():
    try:
        with open(f'/jsonfiles/{agent_name}.json', 'r') as file:
            return json.load(file)
    except Exception as e:
        logging.error(f"Error reading {agent_name}.json: {str(e)}")
        return None


def attestation_process():
    global measuring, digests, digests_count, final_digest
    logging.info("Starting attestation background thread")
    data_agent = get_agent_data()
    if not data_agent:
        logging.error("Agent data not found")
        return

    while measuring:
        try:
            digest = compute_program_hash(
                data_agent["cmd_name"],
                data_agent["text_section_size"],
                data_agent["offset"]
            )
            with lock:
                digests.append(digest)
                digests_count += 1
                redis_client.incr(f"measure_digests:{agent_name}")

                aggregated_digests = "".join(digests)
                computed_digest = hashlib.sha256(aggregated_digests.encode()).hexdigest()

                if digests_count >= threshold:
                    final_digest = computed_digest
                    redis_client.lpush(f"final_digests:{agent_name}", json.dumps({
                        "final_digest": final_digest,
                        "threshold": threshold
                    }))
                    digests = []
                    digests_count = 0

        except Exception as e:
            logging.error(f"Error in attestation process: {str(e)}")
        # time.sleep(1)

    logging.info("Attestation thread stopped")


def start_attestation():
    global measuring, attestation_thread
    if measuring and attestation_thread and attestation_thread.is_alive():
        logging.info("Attestation process already running")
        return
    measuring = True
    
    agent_data = get_agent_data()
    if not agent_data:
        logging.error("Agent data not found")
        return

    redis_client.set(f"attestation_running:{agent_name}", str(measuring))
    redis_client.hmset(f"agent_info:{agent_name}", {
        "cmd_name": agent_data["cmd_name"],
        "text_section_size": agent_data["text_section_size"],
        "offset": agent_data["offset"],
        "uuid": agent_data["id"],
        "sha256": agent_data["sha256"]
    })
    attestation_thread = threading.Thread(target=attestation_process, daemon=True)
    attestation_thread.start()


@app.get("/startAttestation")
def start_attestation_endpoint():
    start_attestation()
    return JSONResponse(content={"message": "Attestation process started"})


@app.get("/deactivateAttestation")
def stop_attestation_endpoint():
    global measuring
    measuring = False
    redis_client.set(f"attestation_running:{agent_name}", str(measuring))
    return JSONResponse(content={"message": "Attestation deactivated"})


@app.get("/digest")
def get_current_digest():
    with lock:
        return {
            "agent_name": agent_name,
            "final_digest": final_digest,
            "threshold": threshold
        }


@app.get("/activateAttestation")
def activate_attestation():
    data_agent = get_agent_data()
    if not data_agent:
        return JSONResponse(content={"error": "Agent data not found"}, status_code=404)
    return JSONResponse(content={"message": "Attestation process started"}, status_code=200)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8080, reload=True)
