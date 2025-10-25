# secaas.py
import os
import time
import logging
import threading
from queue import Queue, Empty
from typing import Optional, Dict, Set
from fastapi import FastAPI
from web3 import Web3

from blockchain_client import (
    MasMutualAttestationContractEvents, AttestationState, EventWatcher
)
import utils

logger = logging.getLogger(__name__)

def _should_stop(stop_event):
    return bool(stop_event and stop_event.is_set())

def process_secaas_attestation(
    app: FastAPI, 
    attestation_id: str, 
    seed_ts: Optional[Dict[str, int]] = None,
    stop_event: Optional[threading.Event] = None,
):
    try:
        timestamps = dict(seed_ts or {})
        log_prefix = f"[Oracle-{utils.short_att_id(attestation_id)}]"
        blockchain_client = app.state.blockchain_client

        # --- ORACLE PHASE ---
        timestamps["get_prover_addr_start"] = int(time.time() * 1000)
        logger.info(f"{log_prefix} Retrieving prover address from SC...")
        agent_address = blockchain_client.get_prover_address(attestation_id)
        timestamps["get_prover_addr_finished"] = int(time.time() * 1000)

        timestamps["get_prover_ref_signatures_db_start"] = int(time.time() * 1000)
        logger.info(f"{log_prefix} Retrieving prover reference signature from DB...")
        # TO BE MODIFIED TO READ FROM DB
        agent_info = utils.get_agent_by_blockchain_address("/config", agent_address)
        if not agent_info:
            logger.error(f"{log_prefix} Agent with address '{agent_address}' not found in the DB.")
            return
        reference_agent_measurements = agent_info['ref_signatures']
        timestamps["get_prover_ref_signatures_db_finished"] = int(time.time() * 1000)

        timestamps["prover_ref_signatures_sent"] = int(time.time() * 1000)
        logger.info(f"{log_prefix} Reference signatures sent for agent with address '{agent_address}'")
        tx_hash = blockchain_client.send_reference_signatures(attestation_id, reference_agent_measurements)

        if app.state.export_enabled:
            utils.export_attestation_times_json(
                app.state.participant_name, attestation_id, "oracle", timestamps, app.state.results_dir
            )

        # --- VERIFIER PHASE (if SECaaS also verifier) ---
        if blockchain_client.is_verifier_agent(attestation_id):
            vts = {}
            logger.info("SECaaS is also the verifier for this attestation.")

            log_prefix = f"[Verifier-{utils.short_att_id(attestation_id)}]"

            # Wait until the contract signals ReadyForEvaluation
            start_wait = time.time()
            TIMEOUT_S = 60
            while blockchain_client.get_attestation_state(attestation_id) != AttestationState.ReadyForEvaluation:
                if _should_stop(stop_event):
                    logger.info(f"{log_prefix} Stopping while waiting for ReadyForEvaluation.")
                    return
                if time.time() - start_wait > TIMEOUT_S:
                    logger.error(f"{log_prefix} Timeout waiting for ReadyForEvaluation.")
                    return
                time.sleep(0.05)

            # Do the actual comparison/evaluation — TO BE MODIFIED
            result = True

            # Save attestation result
            vts["result_sent"] = int(time.time() * 1000)
            logger.info(f"{log_prefix} Attestation closed (result: {'✅ SUCCESS' if result else '❌ FAILURE'})")
            tx_hash = blockchain_client.send_attestation_result(attestation_id, result)
        
            if app.state.export_enabled:
                utils.export_attestation_times_json(
                    app.state.participant_name, attestation_id, "verifier", vts, app.state.results_dir
                )

    except Exception as e:
        logger.error(f"Error while processing attestation '{attestation_id}': {e}")

def run_secaas_logic(app: FastAPI, stop_event: Optional[threading.Event] = None):
    # Build blockchain_event_watcher for AttestationStarted
    evt_enum = MasMutualAttestationContractEvents.ATTESTATION_STARTED
    blockchain_client = app.state.blockchain_client
    evt_abi  = blockchain_client.get_event_abi(evt_enum)
    topic0   = blockchain_client.get_event_topic(evt_enum)

    cp_name  = f"att_started_cp_secaas.json"
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

    # Initial lookback (when no checkpoint)
    if not os.path.exists(cp_path):
        latest = blockchain_client.web3.eth.block_number
        if app.state.event_lookback_blocks > 0:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_lookback_blocks)
        else:
            # small safe overlap even when lookback is 0
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_confirmations)
        logger.info(f"Initial lookback set to block {blockchain_event_watcher.from_block} (latest={latest})")

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
            try:
                logger.info(f"Processing attestation '{attestation_id}'")
                process_secaas_attestation(app, attestation_id, seed_ts, stop_event=stop_event)
            except Exception:
                logger.exception(f"Error processing {attestation_id}")
            finally:
                with enq_lock:
                    enqueued_ids.discard(attestation_id)
                work_q.task_done()

        logger.info("SECaaS worker stopped")

    t_worker = threading.Thread(target=worker, name="SECaaS-Worker", daemon=True)
    t_worker.start()
    if hasattr(app.state, "threads"):
        app.state.threads.append(t_worker)

    def handle(evt):
        attestation_id = Web3.toText(evt['args']['id']).rstrip('\x00').strip()
        if blockchain_client.get_attestation_state(attestation_id) == AttestationState.Open:
           with enq_lock:
                if attestation_id not in enqueued_ids:
                    enqueued_ids.add(attestation_id)
                    seed_ts = {"attestation_started_received": int(time.time() * 1000)}
                    work_q.put((attestation_id, seed_ts))

    blockchain_event_watcher.run(handle)
    # After watcher exits, signal worker to stop and drain
    if stop_event:
        stop_event.set()
    t_worker.join(timeout=3)
    logger.info("SECaaS logic stopped")