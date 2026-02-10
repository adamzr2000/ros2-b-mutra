# app/internal/attestation/prover.py

import time
from fastapi import FastAPI
from app.internal.blockchain.types import AttestationState
from app.internal.compute.hash import compute_program_hash, ComputeHashError
from app.internal.storage.redis import RedisStorageBackend
from app.internal.storage.memory import MemoryStorageBackend
from app.internal.utils import helpers
from app.internal.logger import info, warn, error, debug, green_text

import hashlib
import threading

def _should_stop(stop_event):
    return bool(stop_event and stop_event.is_set())

def process_prover_attestation(app: FastAPI, attestation_id: str, measurements, start_ts: int, stop_event=None):
    timestamps = {}

    # Use the passed timestamp (when measuring started)
    timestamps["prover_start"] = start_ts

    blockchain_client = app.state.blockchain_client
    log_prefix = f"[Prover-{helpers.short_att_id(attestation_id)}]"

    try:
        # Initiate attestation here
        timestamps["evidence_sent"] = helpers.now_ms()
        p0 = helpers.perf_ns()

        tx_hash = blockchain_client.send_evidence(attestation_id, measurements, wait=True)
        
        p1 = helpers.perf_ns()
        timestamps["evidence_sent_tx_confirmed"] = helpers.now_ms()
        timestamps["dur_send_evidence_call_us"] = helpers.ns_to_us(p0, p1)

        info(f"{log_prefix} Evidence sent (attestation started)")

    except Exception as e:
        error(f"{log_prefix} Failed to send evidence: {e}")
        return
    
    # Wait until this agent is recognized as the prover (or stop/timeout)
    start_wait = time.time()
    TIMEOUT_S = 60
    while not blockchain_client.is_prover_agent(attestation_id):
        if _should_stop(stop_event):
            info(f"{log_prefix} Stopping while waiting to become prover.")
            return
        if time.time() - start_wait > TIMEOUT_S:
            error(f"{log_prefix} Timeout while waiting to become prover.")
            return
        time.sleep(0.2)
        
    info(f"{log_prefix} Waiting for verification result...")

    # Wait for attestation to close (or stop/timeout)
    start_wait = time.time()
    while blockchain_client.get_attestation_state(attestation_id) != AttestationState.Closed:
        if _should_stop(stop_event):
            info(f"{log_prefix} Stopping while waiting for attestation to close.")
            return
        if time.time() - start_wait > TIMEOUT_S:
            error(f"{log_prefix} Timeout while waiting for attestation to close.")
            return
        time.sleep(0.2)

    timestamps["result_received"] = helpers.now_ms()
    # Retrieve attestation details
    try:
        prover_address, verifier_address, attestation_result, timestamp = blockchain_client.get_attestation_info(attestation_id)
        is_success = (attestation_result == 2)
        info(f"{log_prefix} Attestation closed (result: {'✅ SUCCESS' if is_success else '❌ FAILURE'})")
    except Exception as e:
        error(f"Failed to read attestation info: {e}")

    timestamps["prover_finished"] = helpers.now_ms()
    
    debug(f"{log_prefix} summary: send_call_us={timestamps.get('dur_send_evidence_call_us','NA')} | tx_confirm_ms={((timestamps['evidence_sent_tx_confirmed']-timestamps['evidence_sent']) if ('evidence_sent_tx_confirmed' in timestamps and 'evidence_sent' in timestamps) else 'NA')} | e2e_ms={((timestamps['result_received']-timestamps['evidence_sent']) if ('result_received' in timestamps and 'evidence_sent' in timestamps) else 'NA')} | total_ms={((timestamps['prover_finished']-timestamps['prover_start']) if ('prover_finished' in timestamps and 'prover_start' in timestamps) else 'NA')}")

    if app.state.export_enabled:
        try:
            helpers.export_attestation_times_json(
                app.state.participant_name, attestation_id, "prover", timestamps, app.state.results_dir, 
                json_path=getattr(app.state, "results_file", None)
            )
        except Exception as e:
            error(f"{log_prefix} Failed exporting prover timings: {e}")

def run_prover_and_cleanup(app: FastAPI, attestation_id: str, measurements, start_ts: int, stop_event=None):
    start = time.time()
    try:
        process_prover_attestation(app, attestation_id, measurements, start_ts, stop_event=stop_event)
    finally:
        elapsed = time.time() - start
        log_prefix = f"[Prover-{helpers.short_att_id(attestation_id)}]"
        info(green_text(f"{log_prefix} completed in {elapsed:.2f}s"))
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

    info(f"Starting attestation process in continuous mode with threshold {threshold}...")

    # Initialize batch start time if it doesn't exist
    if not hasattr(app.state, "batch_start_ts") or app.state.digests_count == 0:
        app.state.batch_start_ts = helpers.now_ms()

    # Only measure while the flag is on; also allow external stop via stop_event
    while getattr(app.state, "measuring", False) and not _should_stop(stop_event):
        iteration += 1
        try:
            debug(f"Attestation iteration {iteration}: Computing hash for {config['cmd_name']}")
            
            # HEAVY CPU ACTIVITY: Reading the binary and hashing it
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
            # info(f"Digest added: {app.state.digests_count}/{threshold} collected (total: {count})")

            aggregated_digests = "".join(app.state.digests)
            computed_digest = hashlib.sha256(aggregated_digests.encode()).hexdigest()

            if app.state.digests_count >= threshold:
                final_digest = computed_digest
                app.state.final_digest = final_digest

                # Capture the start time for THIS batch
                current_batch_start = app.state.batch_start_ts

                info(f"[Prover] Threshold reached. Final digest: {final_digest[:16]}...{final_digest[-16:]}")
                storage.push_final_digest(f"final_digests:{agent_name}", {
                    "final_digest": final_digest,
                    "threshold": threshold,
                    "timestamp": time.time()
                })

                # Send evidence to smart contract (new attestation process)
                attestation_id = helpers.generate_attestation_id()
                signatures = [final_digest, final_digest, final_digest]

                # Should we run this synchronously and wait?
                thread = threading.Thread(
                    target=run_prover_and_cleanup,
                    args=(app, attestation_id, signatures, current_batch_start, stop_event),
                    name=agent_name,
                    daemon=True
                )
                app.state.active_attestations[attestation_id] = thread
                thread.start()

                # Also track the thread globally so shutdown can join
                if hasattr(app.state, "threads"):
                    app.state.threads.append(thread)

                # time.sleep(3)
                
                debug(f"Final digest stored in key: final_digests:{app.state.participant_name}")

                # RESET STATE FOR NEXT BATCH
                app.state.digests = []
                app.state.digests_count = 0
                # Reset the timer for the NEXT batch
                app.state.batch_start_ts = helpers.now_ms()

                debug("Digest counters reset and new batch timer started") 

                if isinstance(storage, MemoryStorageBackend) and memory_storage_file:
                    if storage.export_to_file(memory_storage_file):
                        debug(f"💾 Memory storage exported to {memory_storage_file} after threshold reached")
                    else:
                        error(f"Failed to export memory storage to {memory_storage_file}")
            
            time.sleep(0.05)

            
        except ComputeHashError as e:
            error(f"Hash computation error in iteration {iteration}: {str(e)}") 
            time.sleep(5)
        except Exception as e:
            error(f"Error in attestation process (iteration {iteration}): {str(e)}")
            time.sleep(5)

    info("Attestation process stopped")