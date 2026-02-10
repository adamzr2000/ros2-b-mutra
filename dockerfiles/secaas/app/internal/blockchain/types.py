# app/internal/blockchain/types.py

from enum import IntEnum, Enum
from web3 import Web3

def text_to_bytes32(text_input: str) -> bytes:
    """
    Converts a text string to a 32-byte bytes object, right-padded with null bytes.
    Used for converting Attestation IDs (strings) to bytes32 for the contract.
    """
    b = Web3.to_bytes(text=text_input)
    if len(b) > 32:
        raise ValueError(f"String '{text_input}' exceeds 32 bytes ({len(b)})")
    return b.ljust(32, b'\0')

def as_bytes32(v) -> bytes:
    """
    Accepts hex string (with or without 0x) or raw 32-byte value and returns bytes32.
    Raises ValueError if not 32 bytes after parsing.
    """
    if isinstance(v, (bytes, bytearray)):
        if len(v) != 32:
            raise ValueError(f"bytes32 must be 32 bytes, got {len(v)}")
        return bytes(v)
    if isinstance(v, str):
        hs = v if v.startswith("0x") else f"0x{v}"

        # 2. Use Static Method directly
        b = Web3.to_bytes(hexstr=hs)

        if len(b) != 32:
            raise ValueError(f"bytes32 must be 32 bytes, got {len(b)} after parsing '{v}'")
        return b
    raise TypeError(f"Unsupported type for bytes32: {type(v)}")

def as_bytes32_triplet(values) -> list:
    if values is None or len(values) != 3:
        raise ValueError("Expected 3 signatures: [robot, prover, verifier]")
    # 4. Update internal calls
    return [as_bytes32(values[0]), as_bytes32(values[1]), as_bytes32(values[2])]


class AttestationState(IntEnum):
    Open = 0
    ReadyForEvaluation = 1
    Closed = 2


class MasMutualAttestationContractEvents(str, Enum):
    AGENT_REGISTERED = "AgentRegistered"
    AGENT_REMOVED = "AgentRemoved"
    ATTESTATION_STARTED = "AttestationStarted"
    READY_FOR_EVALUATION = "ReadyForEvaluation"
    ATTESTATION_COMPLETED = "AttestationCompleted"