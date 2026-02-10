# app/internal/blockchain/formatters.py

from typing import Callable, Any, Sequence

from prettytable import PrettyTable
from web3 import Web3


def build_attestation_chain_table(
    attestation_chain: Sequence[bytes],
    get_attestation_info: Callable[[str], Any],
    get_agent_info: Callable[[str], Any],
    secaas_address: str = "0xed9d02e382b34818e88b88a309c7fe71e65f419d",
    last_n: int = 10,
) -> PrettyTable:
    """
    Returns a PrettyTable with the last N attestations.
    - get_attestation_info(attestation_id_str) -> (prover_address, verifier_address, attestation_result, timestamp)
    - get_agent_info(address) -> (name, is_registered, completed_attestations)
    """
    if not attestation_chain:
        table = PrettyTable()
        table.field_names = ["info"]
        table.add_row(["Attestation chain is empty"])
        return table

    last_n = min(last_n, len(attestation_chain))
    recent_chain = attestation_chain[-last_n:]

    table = PrettyTable()
    table.field_names = ["#", "Attestation ID", "Prover", "Verifier", "Result", "Timestamp"]
    table.align = "l"

    for i, attestation_id_b in enumerate(recent_chain, start=1):
        attestation_id = attestation_id_b.rstrip(b"\x00").decode("utf-8")

        prover_address, verifier_address, attestation_result, timestamp = get_attestation_info(attestation_id)
        status = "✅ SUCCESS" if attestation_result == 2 else "❌ FAILURE"

        prover_name, _, _ = get_agent_info(prover_address)

        if Web3.to_checksum_address(verifier_address) == Web3.to_checksum_address(secaas_address):
            verifier_name = "SECaaS"
        else:
            verifier_name, _, _ = get_agent_info(verifier_address)

        prover_info = f"{prover_name} - {prover_address}"
        verifier_info = f"{verifier_name} - {verifier_address}"

        table.add_row([i, attestation_id, prover_info, verifier_info, status, timestamp])

    return table
