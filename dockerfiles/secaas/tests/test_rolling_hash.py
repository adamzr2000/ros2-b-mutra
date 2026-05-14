"""Cross-language test vectors for helpers.rolling_hash.

The fixtures in tests/rolling-hash-vectors.json are the source of truth for
both this test and the Go test in
dockerfiles/attestation-sidecar-optimized/app/internal/compute/digest_test.go.
If this test ever drifts from the Go test, the prover-side rolling-hash
chain and the verifier-side re-derivation will produce different bytes and
every attestation closes as FAILURE silently. Do not edit either side
without re-running both.
"""
import json
import sys
from pathlib import Path

import pytest

# Make `app.internal.utils.helpers` importable: the secaas package layout
# expects /src/app/... ; here the test runs from outside the container.
SECAAS_APP = Path(__file__).resolve().parent.parent
if str(SECAAS_APP) not in sys.path:
    sys.path.insert(0, str(SECAAS_APP))

from app.internal.utils.helpers import rolling_hash  # noqa: E402

FIXTURES = Path(__file__).resolve().parents[3] / "tests" / "rolling-hash-vectors.json"


@pytest.fixture(scope="module")
def vectors():
    with FIXTURES.open() as f:
        doc = json.load(f)
    assert doc["vectors"], f"no vectors in {FIXTURES}"
    return doc["vectors"]


def test_rolling_hash_vectors(vectors):
    for v in vectors:
        got = rolling_hash(v["seed"], v["k"])
        assert got == v["expected"], (
            f"{v['name']}: K={v['k']}\n"
            f"  got      {got}\n"
            f"  expected {v['expected']}"
        )


def test_rolling_hash_k1_is_identity():
    seed = "8c55173fa6a85295a83378cc84dc18baf8810ee21a0d0013b77d0fbd904ebffe"
    assert rolling_hash(seed, 1) == seed


def test_rolling_hash_accepts_0x_prefix():
    seed = "8c55173fa6a85295a83378cc84dc18baf8810ee21a0d0013b77d0fbd904ebffe"
    a = rolling_hash(seed, 3)
    b = rolling_hash("0x" + seed, 3)
    assert a == b


def test_rolling_hash_rejects_wrong_length():
    with pytest.raises(ValueError, match="32 bytes"):
        rolling_hash("ab", 1)


def test_rolling_hash_rejects_invalid_hex():
    with pytest.raises(ValueError):
        rolling_hash("zz" * 32, 1)
