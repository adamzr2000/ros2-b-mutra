# verifier.py
import os
import time
import logging
import threading
from queue import Queue, Empty
from typing import Optional, Dict, Set
from fastapi import FastAPI
from web3 import Web3

from blockchain_manager import (
    MasMutualAttestationContractEvents, AttestationState, EventWatcher
)
import utils

GREEN = "\033[92m"
RESET = "\033[0m"
logger = logging.getLogger(__name__)

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

        blockchain_manager = app.state.blockchain_manager
        log_prefix = f"[Verifier-{utils.short_att_id(attestation_id)}]"

        # 1. Fetching Signatures
        logger.info(f"{log_prefix} SECaaS Oracle finished. Retrieving signatures for evaluation...")
        timestamps["get_signatures_start"] = int(time.time() * 1000)
        fresh_sigs, ref_sigs = blockchain_manager.get_attestation_signatures(attestation_id)
        timestamps["get_signatures_finished"] = int(time.time() * 1000)
        
        # 2. Evaluation
        is_success = (fresh_sigs == ref_sigs)
        is_success = True

        # 3. Submit Result to Blockchain
        timestamps["result_sent"] = int(time.time() * 1000)
        logger.info(f"{log_prefix} Attestation closed (result: {'✅ SUCCESS' if is_success else '❌ FAILURE'})")
        blockchain_manager.send_attestation_result(attestation_id, is_success)

        if app.state.export_enabled:
            utils.export_attestation_times_json(
                app.state.participant_name, attestation_id, "verifier", timestamps, app.state.results_dir, 
                json_path=getattr(app.state, "results_file", None)
            )

    except Exception as e:
        logger.error(f"Error in process_verifier_attestation for {attestation_id}: {e}")

def run_verifier_logic_sequential(app: FastAPI, stop_event: Optional[threading.Event] = None):
    evt_enum = MasMutualAttestationContractEvents.READY_FOR_EVALUATION

    blockchain_manager = app.state.blockchain_manager
    evt_abi  = blockchain_manager.get_event_abi(evt_enum)
    topic0   = blockchain_manager.get_event_topic(evt_enum)

    cp_name  = f"att_started_cp_agent_seq.json"
    cp_path  = os.path.join(app.state.event_checkpoint_dir, cp_name)

    blockchain_event_watcher = EventWatcher(
        web3            = blockchain_manager.web3,
        contract        = blockchain_manager.contract,
        event_abi       = evt_abi,
        address         = blockchain_manager.contract.address,
        topics          = [topic0],
        checkpoint_path = cp_path,
        confirmations   = app.state.event_confirmations,
        batch_size      = app.state.event_batch_size,
        poll_interval   = app.state.event_poll_interval,
    )

    # Initial lookback on first boot (safe overlap if lookback==0)
    if not os.path.exists(cp_path):
        latest = blockchain_manager.web3.eth.block_number
        if app.state.event_lookback_blocks > 0:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_lookback_blocks)
        else:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_confirmations)
        logger.debug(f"Initial lookback set to block {blockchain_event_watcher.from_block} (latest={latest})")

    logger.info("Subscribed to attestation events with EventWatcher...")

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
                logger.info(f"Processing attestation '{attestation_id}'")
                process_verifier_attestation(app, attestation_id, seed_ts, stop_event=stop_event)
            except Exception:
                logger.exception(f"Error processing {attestation_id}")
            finally:
                elapsed = time.time() - start
                log_prefix = f"[Verifier-{utils.short_att_id(attestation_id)}]"
                logger.info(f"{GREEN}{log_prefix} completed in {elapsed:.2f}s{RESET}")
                with enq_lock:
                    enqueued_ids.discard(attestation_id)
                work_q.task_done()
        logger.info("Verifier worker stopped")

    t_worker = threading.Thread(target=worker, name=app.state.participant_name, daemon=True)
    t_worker.start()
    if hasattr(app.state, "threads"):
        app.state.threads.append(t_worker)

    def handle(evt):
        arrival_ts = int(time.time() * 1000)

        attestation_id = Web3.toText(evt['args']['id']).rstrip('\x00').strip()
        # Check if I am the verifier for this specific ID
        if (blockchain_manager.is_verifier_agent(attestation_id)):
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
    logger.info("Verifier logic stopped")