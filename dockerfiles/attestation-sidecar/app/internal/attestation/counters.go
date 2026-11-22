package attestation

import (
	"sync/atomic"

	"app/internal/blockchain"
)

// Counters of this sidecar since it started, served by GET /status.
// Self-reported, for monitoring and experiments: the chain's events are the
// source of truth.
var counters struct {
	measurements, proved, stale, success, failure, timeouts, verified, votes, errors atomic.Int64
}

// CountersSnapshot returns the current counters.
func CountersSnapshot() map[string]int64 {
	return map[string]int64{
		"measurements": counters.measurements.Load(), // memory measurements taken (K per round)
		"proved":       counters.proved.Load(),       // evidence accepted on chain
		"stale":        counters.stale.Load(),        // evidence past the freshness window, not sent (V3)
		"success":      counters.success.Load(),      // verdicts received as prover
		"failure":      counters.failure.Load(),
		"timeouts":     counters.timeouts.Load(), // rounds without a verdict in time
		"verified":     counters.verified.Load(), // rounds closed as verifier (LV, V2)
		"votes":        counters.votes.Load(),    // votes cast as juror (V3)
		"errors":       counters.errors.Load(),   // failed transactions, chain reads or measurements
		// gas estimates tried again after a Besu "Internal error" (+~100 ms each)
		"gas_estimate_retries": blockchain.EstimateRetries(),
	}
}
