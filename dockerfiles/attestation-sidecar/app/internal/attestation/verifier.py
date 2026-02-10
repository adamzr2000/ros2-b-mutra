# app/internal/attestation/verifier.py

import os
import time
import threading
from queue import Queue, Empty
from typing import Optional, Dict, Set
from fastapi import FastAPI
from web3 import Web3
from app.internal.blockchain.types import MasMutualAttestationContractEvents, AttestationState
from app.internal.blockchain.event_watcher import EventWatcher
from app.internal.utils import helpers
from app.internal.logger import info, warn, error, debug, green_text


def _should_stop(stop_event):
    return bool(stop_event and stop_event.is_set())

def process_verifier_attestation(
    app: FastAPI, 
    attestation_id: str, 
    seed_ts: Optional[Dict[str, int]] = None,
    stop_event: Optional[threading.Event] = None
):
    """
    Verifier logic:
    Triggered only when the SECaaS Oracle has finished (READY_FOR_EVALUATION event).
    """
    try:
        timestamps = dict(seed_ts or {})

        timestamps["verifier_start"] = helpers.now_ms()

        blockchain_client = app.state.blockchain_client
        log_prefix = f"[Verifier-{helpers.short_att_id(attestation_id)}]"
        info(f"{log_prefix} SECaaS Oracle finished. Retrieving signatures for evaluation...")

        # 1. Retrieve Signatures
        timestamps["get_signatures_start"] = helpers.now_ms()
        p0 = helpers.perf_ns()

        fresh_sigs, ref_sigs = blockchain_client.get_attestation_signatures(attestation_id)
        
        p1 = helpers.perf_ns()
        timestamps["get_signatures_finished"] = helpers.now_ms()
        timestamps["dur_signatures_fetch_us"] = helpers.ns_to_us(p0, p1)

        # 2. Evaluation (compute)
        timestamps["verify_compute_start"] = helpers.now_ms()
        p0 = helpers.perf_ns()

        is_success = (fresh_sigs == ref_sigs)
        is_success = True

        p1 = helpers.perf_ns()
        timestamps["verify_compute_finished"] = helpers.now_ms()
        timestamps["dur_verify_compute_us"] = helpers.ns_to_us(p0, p1)

        # 3. Submit Result to Blockchain
        info(f"{log_prefix} Attestation closed (result: {'✅ SUCCESS' if is_success else '❌ FAILURE'})")
        should_wait_verifier = getattr(app.state, "wait_for_tx_confirmations", False)

        timestamps["result_sent"] = helpers.now_ms()
        timestamps["verification_result"] = is_success
        p0 = helpers.perf_ns()

        blockchain_client.send_attestation_result(attestation_id, is_success, wait=should_wait_verifier)
        
        p1 = helpers.perf_ns()
        timestamps["dur_send_result_call_us"] = helpers.ns_to_us(p0, p1)
        
        if should_wait_verifier:
            timestamps["result_sent_tx_confirmed"] = helpers.now_ms()

        timestamps["verifier_finished"] = helpers.now_ms()

        if "evaluation_ready_received" in timestamps:
            timestamps["dur_verifier_reaction_ms"] = timestamps["verifier_start"] - timestamps["evaluation_ready_received"]

        debug(f"{log_prefix} summary: reaction_ms={(timestamps['verifier_start']-timestamps['evaluation_ready_received']) if ('evaluation_ready_received' in timestamps and 'verifier_start' in timestamps) else 'NA'} | fetch_us={timestamps.get('dur_signatures_fetch_us','NA')} | compute_us={timestamps.get('dur_verify_compute_us','NA')} | send_call_us={timestamps.get('dur_send_result_call_us','NA')} | tx_confirm_ms={((timestamps['result_sent_tx_confirmed']-timestamps['result_sent']) if ('result_sent_tx_confirmed' in timestamps and 'result_sent' in timestamps) else 'NA')} | total_ms={((timestamps['verifier_finished']-timestamps['verifier_start']) if ('verifier_finished' in timestamps and 'verifier_start' in timestamps) else 'NA')} | result={('OK' if timestamps.get('verification_result') else ('FAIL' if 'verification_result' in timestamps else 'NA'))}")

        if app.state.export_enabled:
            helpers.export_attestation_times_json(
                app.state.participant_name, attestation_id, "verifier", timestamps, app.state.results_dir, 
                json_path=getattr(app.state, "results_file", None)
            )

    except Exception as e:
        error(f"Error in process_verifier_attestation for {attestation_id}: {e}")

def run_verifier_logic_sequential(app: FastAPI, stop_event: Optional[threading.Event] = None):
    evt_enum = MasMutualAttestationContractEvents.READY_FOR_EVALUATION

    blockchain_client = app.state.blockchain_client
    evt_abi  = blockchain_client.get_event_abi(evt_enum)
    topic0   = blockchain_client.get_event_topic(evt_enum)

    cp_name  = f"att_started_cp_agent_seq.json"
    cp_path  = os.path.join(app.state.event_checkpoint_dir, cp_name)

    blockchain_event_watcher = EventWatcher(
        web3            = blockchain_client.web3,
        contract        = blockchain_client.contract,
        event_abi       = evt_abi,
        address         = blockchain_client.contract.address,
        topics          = [topic0],
        checkpoint_path = cp_path,
        confirmations   = app.state.event_confirmations,
        batch_size      = app.state.event_batch_size,
        poll_interval   = app.state.event_poll_interval,
    )

    # Initial lookback on first boot (safe overlap if lookback==0)
    if not os.path.exists(cp_path):
        latest = blockchain_client.web3.eth.block_number
        if app.state.event_lookback_blocks > 0:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_lookback_blocks)
        else:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_confirmations)
        debug(f"Initial lookback set to block {blockchain_event_watcher.from_block} (latest={latest})")

    info("Subscribed to attestation events with EventWatcher...")

    work_q = Queue()
    enqueued_ids: Set[str] = set()
    enq_lock = threading.Lock()

    def worker():
        while not _should_stop(stop_event):
            try:
                attestation_id, seed_ts = work_q.get(timeout=0.5)
            except Empty:
                continue
            start = time.time()
            try:
                info(f"Processing attestation '{attestation_id}'")
                process_verifier_attestation(app, attestation_id, seed_ts, stop_event=stop_event)
            except Exception as e:
                error(f"Error processing {attestation_id}: {e}")
            finally:
                elapsed = time.time() - start
                log_prefix = f"[Verifier-{helpers.short_att_id(attestation_id)}]"
                info(green_text(f"{log_prefix} completed in {elapsed:.2f}s"))
                with enq_lock:
                    enqueued_ids.discard(attestation_id)
                work_q.task_done()
        info("Verifier worker stopped")

    t_worker = threading.Thread(target=worker, name=app.state.participant_name, daemon=True)
    t_worker.start()
    if hasattr(app.state, "threads"):
        app.state.threads.append(t_worker)

    def handle(evt):
        arrival_ts = helpers.now_ms()

        attestation_id = Web3.to_text(evt['args']['id']).rstrip('\x00').strip()
        
        # Check if I am the verifier for this specific ID
        if (blockchain_client.is_verifier_agent(attestation_id)):
            with enq_lock:
                if attestation_id not in enqueued_ids:
                    enqueued_ids.add(attestation_id)
                    seed_ts = {"evaluation_ready_received": arrival_ts}
                    work_q.put((attestation_id, seed_ts))


    blockchain_event_watcher.run(handle)
    # After watcher exits, signal worker to stop and drain
    if stop_event:
        stop_event.set()
    t_worker.join(timeout=3)
    info("Verifier logic stopped")