# app/internal/attestation/oracle_verifier.py

import os
import time
import threading
from queue import Queue, Empty
from typing import Optional, Dict, Set
from fastapi import FastAPI
from web3 import Web3
from app.internal.logger import info, warn, error, debug, green_text
from app.internal.blockchain.types import (
    AttestationState,
    AttestationManagerContractEvents
)
from app.internal.blockchain.event_watcher import EventWatcher
from app.internal.utils import helpers
from app.internal import results

def _should_stop(stop_event):
    return bool(stop_event and stop_event.is_set())

def process_secaas_attestation(
    app: FastAPI,
    attestation_id: str,
    detected: dict,
    stop_event: Optional[threading.Event] = None,
):
    """detected: when the event was seen, t_ms (Unix ms), t (perf_counter)
    and detect_ms (its block imported -> seen, None when not known). A verdict
    sent, or tried, is recorded as a row."""
    log_prefix = f"[SECaaS-{helpers.short_att_id(attestation_id)}]"
    row = {"round_id": attestation_id, "verifier": "secaas", "role": "secaas",
           "t_detected_ms": detected["t_ms"], "detect_ms": detected["detect_ms"],
           "queue_ms": results.ms_since(detected["t"])}

    def record(outcome):
        row["outcome"] = outcome
        row["total_ms"] = results.ms_since(detected["t"])
        results.add(row)

    t_check = time.perf_counter()
    try:
        blockchain_client = app.state.blockchain_client
        should_wait = getattr(app.state, "wait_for_tx_confirmations", False)

        # V2 and V3 anchor the reference on chain and the SECaaS compares
        # against it, so a wrong or tampered SECaaS database cannot change the
        # verdict. Only LV takes the reference from the database, by prover.
        anchored = blockchain_client.has_anchored_reference()
        ref_hash = None
        if not anchored:
            agent_address = blockchain_client.get_prover_address(attestation_id)
            signatures_dict = app.state.db_client.get_ref_signatures(agent_address)
            if not signatures_dict:
                error(f"{log_prefix} Agent '{agent_address}' not found in Database.")
                return
            ref_hash = signatures_dict["combined_hash"]

        if not blockchain_client.is_verifier_agent(attestation_id):
            if anchored:
                # V2 and V3 announce a cycle to the SECaaS only when it verifies it.
                info(f"{log_prefix} Cycle already closed, nothing to do")
                return
            info(f"{log_prefix} Prover ref. measurement sent to the smart contract")
            blockchain_client.submit_reference(attestation_id, ref_hash, wait=should_wait)
            return
        row["check_ms"] = results.ms_since(t_check)

        log_prefix = f"[Verifier-{helpers.short_att_id(attestation_id)}]"

        # 1. Read the evidence and the reference
        t0 = time.perf_counter()
        fresh_sig_hex, chain_ref_hex, iter_count, bound_block = blockchain_client.get_attestation_signatures(attestation_id)
        if anchored:
            ref_hash = chain_ref_hex
        block_hash = blockchain_client.get_block_hash(bound_block) if bound_block is not None else None
        row["read_ms"] = results.ms_since(t0)

        # 2. Compare. V3: the prover bound each measurement to the hash of
        # block bound_block; the reference is bound the same way. Then the
        # chained reference is re-derived under the no-tamper hypothesis: all
        # K m_i = ref_hash, so the prover's H_K equals chain(ref_hash, K).
        # Must stay bit-for-bit identical to compute.RollingHashStep in Go.
        t0 = time.perf_counter()
        if block_hash is not None:
            ref_hash = helpers.bind_to_block(ref_hash, block_hash)
        try:
            derived_ref = helpers.rolling_hash(ref_hash, int(iter_count))
        except Exception as ex:
            # No verdict without a reference to compare against.
            error(f"{log_prefix} Failed to derive rolling reference (K={iter_count}): {ex}")
            record("error")
            return

        def _strip0x(s: str) -> str:
            return s[2:] if s.lower().startswith("0x") else s
        is_success = (_strip0x(fresh_sig_hex).lower() == _strip0x(derived_ref).lower())
        row["compute_ms"] = results.ms_since(t0)
        row["verdict"] = "success" if is_success else "failure"

        # 3. Close the cycle. Retried once on a transient failure (RPC
        # timeout); a second failure is a real error.
        info(f"{log_prefix} Resolved atomically (result: {'✅ SUCCESS' if is_success else '❌ FAILURE'})")
        t0 = time.perf_counter()
        for _attempt in range(2):
            row["attempts"] = _attempt + 1
            try:
                blockchain_client.verify_by_secaas(attestation_id, is_success, wait=should_wait)
                break
            except Exception as resolve_err:
                if _attempt == 0:
                    warn(f"{log_prefix} verify_by_secaas failed (attempt 1), retrying: {resolve_err}")
                    time.sleep(0.5)
                else:
                    raise
        row["send_ms"] = results.ms_since(t0)
        record("sent")

    except Exception as e:
        error(f"Error while processing attestation '{attestation_id}': {e}")
        import traceback
        debug(traceback.format_exc())
        if "verdict" in row:
            record("error")

def run_expiry_logic(app: FastAPI, stop_event: Optional[threading.Event] = None):
    """V3: expire the jury cycles still waiting for votes after the jury timeout.
    The prover expires its own cycle too; this covers a prover that went down.
    The expired cycle then reaches the SECaaS through AttestationStarted."""
    blockchain_client = app.state.blockchain_client
    evt_enum = AttestationManagerContractEvents.READY_FOR_EVALUATION
    watcher = EventWatcher(
        web3            = blockchain_client.web3,
        contract        = blockchain_client.contract,
        event_abi       = blockchain_client.get_event_abi(evt_enum),
        address         = blockchain_client.contract.address,
        topics          = [blockchain_client.get_event_topic(evt_enum)],
        checkpoint_path = os.path.join(app.state.event_checkpoint_dir, "ready_cp_secaas.json"),
        confirmations   = app.state.event_confirmations,
        batch_size      = app.state.event_batch_size,
        poll_interval   = app.state.event_poll_interval,
        heads           = app.state.heads,
    )
    timeout_blocks = blockchain_client.jury_timeout_blocks()
    pending: Dict[str, int] = {}  # attestation id -> block of its evidence
    lock = threading.Lock()

    def handle(evt):
        attestation_id = Web3.to_text(evt['args']['id']).rstrip('\x00').strip()
        with lock:
            pending[attestation_id] = evt['blockNumber']

    def expire_due():
        while not _should_stop(stop_event):
            heads = app.state.heads
            seq = heads.seq
            # The block just notified, else asked to the node.
            head = heads.num if heads.live else blockchain_client.web3.eth.block_number
            with lock:
                # A backup for the prover, which expires its cycle a block earlier.
                due = [i for i, b in pending.items() if head >= b + timeout_blocks]
            for attestation_id in due:
                log_prefix = f"[SECaaS-{helpers.short_att_id(attestation_id)}]"
                if blockchain_client.get_attestation_state(attestation_id) == AttestationState.ReadyForEvaluation:
                    try:
                        blockchain_client.expire(attestation_id, wait=True)
                        info(f"{log_prefix} Jury timeout reached, cycle {attestation_id} expired by the SECaaS")
                    except Exception as e:
                        # The jury or the prover may have acted first: the state is read again next round.
                        warn(f"{log_prefix} Expire not sent: {e}")
                        continue
                with lock:
                    pending.pop(attestation_id, None)
            app.state.heads.wait(seq)

    t = threading.Thread(target=expire_due, name="SECaaS-expiry", daemon=True)
    t.start()
    info(f"Expiring jury cycles left without a verdict after {timeout_blocks} blocks")
    watcher.run(handle, stop_event=stop_event)
    t.join(timeout=30)

def run_secaas_logic(app: FastAPI, stop_event: Optional[threading.Event] = None):
    # Build blockchain_event_watcher for AttestationStarted
    evt_enum = AttestationManagerContractEvents.ATTESTATION_STARTED
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
        heads           = app.state.heads,
    )

    # Initial lookback (when no checkpoint)
    if not os.path.exists(cp_path):
        latest = blockchain_client.web3.eth.block_number
        if app.state.event_lookback_blocks > 0:
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_lookback_blocks)
        else:
            # small safe overlap even when lookback is 0
            blockchain_event_watcher.from_block = max(0, latest - app.state.event_confirmations)
        info(f"Initial lookback set to block {blockchain_event_watcher.from_block} (latest={latest})")

    info("Subscribed to attestation events with EventWatcher...")

    work_q = Queue()
    enqueued_ids: Set[str] = set()
    enq_lock = threading.Lock()

    def worker():
        # Exit only when BOTH: stop is set AND the queue is empty.
        # This guarantees every in-flight / already-queued item is processed and
        # task_done() is called before the thread exits, so work_q.join() in
        # stop_attestation never deadlocks and every verdict is recorded.
        while True:
            try:
                attestation_id, detected = work_q.get(timeout=0.05)
            except Empty:
                if _should_stop(stop_event):
                    break  # queue empty + stop requested → exit cleanly
                continue
            try:
                info(f"Processing attestation '{attestation_id}'")
                process_secaas_attestation(app, attestation_id, detected, stop_event=stop_event)
            except Exception as e:
                error(f"Error processing {attestation_id}: {e}")
            finally:
                with enq_lock:
                    enqueued_ids.discard(attestation_id)
                work_q.task_done()

        info("SECaaS worker stopped")

    t_worker = threading.Thread(target=worker, name="SECaaS", daemon=True)
    t_worker.start()
    if hasattr(app.state, "threads"):
        app.state.threads.append(t_worker)

    def handle(evt):
        heads = app.state.heads
        detected = {"t_ms": int(time.time() * 1000), "t": time.perf_counter(), "detect_ms": None}
        if heads.live and evt["blockNumber"] == heads.num:
            detected["detect_ms"] = round((detected["t"] - heads.at) * 1000, 3)
        attestation_id = Web3.to_text(evt['args']['id']).rstrip('\x00').strip()
        with enq_lock:
            if attestation_id not in enqueued_ids:
                enqueued_ids.add(attestation_id)
                work_q.put((attestation_id, detected))

    blockchain_event_watcher.run(handle, stop_event=stop_event)
    # After watcher exits, signal worker to stop and drain.
    # Worker exits only when queue is empty + stop set, so joining here
    # guarantees every verdict is recorded before run_secaas_logic returns.
    if stop_event:
        stop_event.set()
    t_worker.join(timeout=30)
    info("SECaaS logic stopped")