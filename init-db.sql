-- Connect to the attestation database
\c attestation;

-- Create the mapping table using _signature suffix
CREATE TABLE IF NOT EXISTS measures (
    eth_address VARCHAR(42) PRIMARY KEY,
    prover_signature TEXT NOT NULL,
    verifier_signature TEXT NOT NULL,
    payload_signature TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for optimized Ethereum address lookups
CREATE INDEX IF NOT EXISTS idx_measures_eth_lower ON measures (LOWER(eth_address));