// Package results keeps the timing rows of this agent, served by GET /results
// and cleared by DELETE /results. Durations use the monotonic clock of this
// process; t_*_ms fields are Unix wall-clock times, to order rows only.
package results

import (
	"sync"
	"time"
)

// Prover is one round sent by this agent. A duration is empty when the
// round did not reach it.
type Prover struct {
	RoundID     string   `json:"round_id"`
	Robot       string   `json:"robot"`
	TMeasuredMs int64    `json:"t_measured_ms"` // when its first measurement started (wall clock)
	TSentMs     int64    `json:"t_sent_ms,omitempty"`
	EvidenceMs  float64  `json:"evidence_ms"`            // K measurements, salted, bound and folded; sleeps excluded
	PresendMs   *float64 `json:"presend_ms,omitempty"`   // evidence ready -> Prove sending: freshness check, previous round still open
	SendMs      *float64 `json:"send_ms,omitempty"`      // Prove sent: gas estimate, sign, submit
	InclusionMs *float64 `json:"inclusion_ms,omitempty"` // Prove submitted -> its block imported by the node
	JudgementMs *float64 `json:"judgement_ms,omitempty"` // its block -> the block closing the round
	TotalMs     *float64 `json:"total_ms,omitempty"`     // evidence -> round seen closed: the sum of the above
	Outcome     string   `json:"outcome"`                // success, failure, sent, stale, rejected, timeout, error
}

// Verifier is the work of this agent on one round it was asked to judge. A
// duration is empty when the work did not reach it.
type Verifier struct {
	RoundID     string   `json:"round_id"`
	Verifier    string   `json:"verifier"`
	Role        string   `json:"role"` // juror (drawn), candidate (not drawn), or verifier with a contract without jury
	TDetectedMs int64    `json:"t_detected_ms"`
	DetectMs    *float64 `json:"detect_ms,omitempty"`  // block with the event imported -> event seen
	QueueMs     *float64 `json:"queue_ms,omitempty"`   // event seen -> work started (rounds before it)
	CheckMs     *float64 `json:"check_ms,omitempty"`   // juror check (Vote rehearsal)
	ReadMs      *float64 `json:"read_ms,omitempty"`    // evidence, reference and block hash
	ComputeMs   *float64 `json:"compute_ms,omitempty"` // bind, fold, compare
	SendMs      *float64 `json:"send_ms,omitempty"`    // verdict transaction sent (gas estimate, sign, submit)
	TotalMs     *float64 `json:"total_ms,omitempty"`   // event seen -> last step done: queue to send
	Attempts    int      `json:"attempts,omitempty"`   // verdict transactions tried
	Verdict     string   `json:"verdict,omitempty"`    // success, failure
	Outcome     string   `json:"outcome"`              // sent, not_drawn, late, error
}

var (
	mu       sync.Mutex
	prover   []Prover
	verifier []Verifier
)

func AddProver(r Prover) {
	mu.Lock()
	prover = append(prover, r)
	mu.Unlock()
}

func AddVerifier(r Verifier) {
	mu.Lock()
	verifier = append(verifier, r)
	mu.Unlock()
}

// Snapshot returns copies of the rows recorded since the last Clear.
func Snapshot() ([]Prover, []Verifier) {
	mu.Lock()
	defer mu.Unlock()
	return append([]Prover{}, prover...), append([]Verifier{}, verifier...)
}

func Clear() {
	mu.Lock()
	prover, verifier = nil, nil
	mu.Unlock()
}

// Ms is d in milliseconds, to the microsecond.
func Ms(d time.Duration) float64 {
	return float64(d.Microseconds()) / 1000
}

// Dur is Ms for an optional duration.
func Dur(d time.Duration) *float64 {
	ms := Ms(d)
	return &ms
}

// Verdict names a verdict as the rows do.
func Verdict(ok bool) string {
	if ok {
		return "success"
	}
	return "failure"
}
