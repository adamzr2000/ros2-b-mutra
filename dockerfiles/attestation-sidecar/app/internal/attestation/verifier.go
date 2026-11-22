package attestation

import (
	"context"
	"encoding/hex"
	"fmt"
	"path/filepath"
	"strings"
	"sync"
	"time"

	"app/internal/blockchain"
	"app/internal/compute"
	"app/internal/logger"
	"app/internal/results"
	"app/internal/utils"

	"github.com/ethereum/go-ethereum/core/types"
)

// VerifierConfig holds all configuration required by the Verifier.
type VerifierConfig struct {
	ParticipantName        string
	WaitForTxConfirmations bool

	// Event Watcher Config
	EventCheckpointDir string
	EventConfirmations uint64
	EventBatchSize     uint64
	EventLookback      uint64

	ForceVerdict *bool // experiment only: vote this verdict without comparing
}

// verificationJob represents a task in the worker queue
type verificationJob struct {
	AttestationID string
	Detected      time.Time // event seen
	DetectMs      *float64  // block with the event imported -> event seen; nil when not known
	Juror         bool      // V3: vote if drawn in the jury of the cycle
}

// RunVerifierLogicSequential starts the verifier event loop and worker.
func RunVerifierLogicSequential(
	ctx context.Context,
	client *blockchain.BlockchainClient,
	cfg VerifierConfig,
	stopWg *sync.WaitGroup,
) {
	defer stopWg.Done()

	eventName := blockchain.EventReadyForEvaluation // Ensure this constant exists in types.go

	// Setup Checkpoint Path
	cpName := "att_started_cp_agent_seq.json"
	cpPath := filepath.Join(cfg.EventCheckpointDir, cpName)

	// Create Event Watcher
	watcher, err := blockchain.NewEventWatcher(
		client,
		eventName,
		cpPath,
		cfg.EventConfirmations,
		cfg.EventBatchSize,
	)
	if err != nil {
		logger.Error("Verifier failed to create EventWatcher: %v", err)
		return
	}

	// NOTE: In Python, you manually set from_block with lookback here.
	// The Go EventWatcher auto-loads checkpoint or defaults to latest.
	// If you strictly need the lookback on first run, we rely on the watcher's robust default.

	logger.Info("Subscribed to attestation events with EventWatcher...")

	// Worker Queue (Channel)
	// Python: work_q = Queue()
	// Rounds wait here for the worker; a full queue drops them, so it holds
	// far more than a block brings even with many robots.
	workChan := make(chan verificationJob, 4096)

	// Deduplication Set
	enqueuedIDs := make(map[string]bool)
	var enqLock sync.Mutex

	// ---------------------------------------------------------
	// Worker Goroutine
	// ---------------------------------------------------------
	go func() {
		for {
			select {
			case <-ctx.Done():
				logger.Info("Verifier worker stopped")
				return
			case job, ok := <-workChan:
				if !ok {
					logger.Info("Verifier worker stopped")
					return
				}
				start := time.Now()

				// A prover is never in its own jury.
				_, own := ownRounds.Load(job.AttestationID)
				if !(job.Juror && own) &&
					processVerifierAttestation(ctx, client, cfg, job, start) {
					elapsed := time.Since(start).Seconds()
					logPrefix := fmt.Sprintf("[Verifier-%s]", utils.ShortAttID(job.AttestationID, 4))
					logger.Info("%s", logger.GreenText(fmt.Sprintf("%s completed in %.2fs", logPrefix, elapsed)))
				}

				// Remove from dedup set
				enqLock.Lock()
				delete(enqueuedIDs, job.AttestationID)
				enqLock.Unlock()
			}
		}
	}()

	// ---------------------------------------------------------
	// Event Handler Callback
	// ---------------------------------------------------------
	handle := func(log types.Log, parsedData map[string]interface{}) error {
		detected := time.Now()
		var detectMs *float64
		if num, at, live := client.LastHead(); live && log.BlockNumber == num {
			detectMs = results.Dur(detected.Sub(at))
		}

		// Extract Attestation ID
		// It usually comes as [32]byte from the watcher. Convert to string.
		var attID string
		if idVal, ok := parsedData["id"]; ok { // Assumes event arg is named "id"
			if idBytes, ok := idVal.([32]byte); ok {
				// Strip null bytes
				attID = strings.TrimRight(string(idBytes[:]), "\x00")
			} else if idStr, ok := idVal.(string); ok {
				// fallback if it was parsed as string/hash
				attID = idStr
			}
		}

		if attID == "" {
			return fmt.Errorf("could not extract attestation ID from event")
		}

		// With a jury (V3) no single verifier is elected: the vote itself tells
		// whether this agent was drawn.
		juror := client.HasJury()
		isVerifier := false
		if !juror {
			var err error
			if isVerifier, err = client.IsVerifier(attID); err != nil {
				logger.Error("IsVerifier check failed for %s: %v", attID, err)
				return nil // Don't crash handler
			}
		}
		if isVerifier || juror {
			enqLock.Lock()
			if !enqueuedIDs[attID] {
				enqueuedIDs[attID] = true

				// Non-blocking send to avoid stalling watcher if queue is full
				select {
				case workChan <- verificationJob{AttestationID: attID, Detected: detected, DetectMs: detectMs, Juror: juror}:
				default:
					logger.Warn("Verifier queue full, dropping %s", attID)
				}
			}
			enqLock.Unlock()
		}
		return nil
	}

	// Run Watcher in a goroutine; close workChan once it exits so the worker
	// drains any remaining buffered jobs and then stops cleanly.
	watcherDone := make(chan struct{})
	go func() {
		defer close(watcherDone)
		watcher.Run(ctx, handle)
	}()

	// Block until context cancelled, then wait for watcher to finish before
	// closing workChan (guarantees no send-on-closed-channel panic).
	<-ctx.Done()
	<-watcherDone
	close(workChan)
	logger.Info("Verifier logic stopped")
}

// mayVote tells a juror (V3), before it verifies, whether it may vote in the
// cycle: the rehearsal of its vote runs the contract's jury draw in the next
// block, and the jury does not depend on the verdict. A node that rehearses
// in the latest block cannot draw before the evidence block is sealed: the
// rehearsal is then tried once more a block later.
//
// Returns "" when it may vote, else why not, as a row outcome: not_drawn,
// late (closed or with the SECaaS), error; stopped when the agent stops.
func mayVote(ctx context.Context, client *blockchain.BlockchainClient, attID, logPrefix string) string {
	err := client.CheckVote(attID)
	if blockchain.IsRevert(err, "evidence block not sealed yet") {
		if werr := waitNextBlock(ctx, client); werr != nil {
			return "stopped"
		}
		err = client.CheckVote(attID)
	}
	switch {
	case err == nil:
		return ""
	case blockchain.IsRevert(err, "caller is not in the jury"):
		logger.Debug("%s Not drawn in the jury", logPrefix)
		return "not_drawn"
	case blockchain.IsRevert(err, "cycle closed"), blockchain.IsRevert(err, "cycle is with the SECaaS"):
		logger.Debug("%s Cycle closed or with the SECaaS, no vote", logPrefix)
		return "late"
	default:
		logger.Error("%s Failed to check the jury: %v", logPrefix, err)
		counters.errors.Add(1)
		return "error"
	}
}

// waitNextBlock waits until the chain head moves past the current block.
func waitNextBlock(ctx context.Context, client *blockchain.BlockchainClient) error {
	start, err := client.BlockNumber()
	if err != nil {
		return err
	}
	for {
		next := client.NextHead()
		if head, err := client.BlockNumber(); err == nil && head > start {
			return nil
		}
		if !blockchain.WaitHead(next, ctx.Done()) {
			return ctx.Err()
		}
	}
}

// processVerifierAttestation evaluates a cycle and sends the verdict. A juror
// (V3) first checks that it may vote (mayVote), then verifies and votes at
// once: the vote goes in the next block at best. Returns whether a verdict was
// sent or attempted, false when this agent is not a juror of the cycle. A
// verdict sent or attempted is recorded as a row.
func processVerifierAttestation(
	ctx context.Context,
	client *blockchain.BlockchainClient,
	cfg VerifierConfig,
	job verificationJob,
	started time.Time,
) bool {
	attestationID, juror := job.AttestationID, job.Juror
	row := results.Verifier{
		RoundID:     attestationID,
		Verifier:    cfg.ParticipantName,
		Role:        "verifier",
		TDetectedMs: job.Detected.UnixMilli(),
		DetectMs:    job.DetectMs,
		QueueMs:     results.Dur(started.Sub(job.Detected)),
	}
	record := func(outcome string) bool {
		row.Outcome = outcome
		row.TotalMs = results.Dur(time.Since(job.Detected))
		results.AddVerifier(row)
		return true
	}
	logPrefix := fmt.Sprintf("[Verifier-%s]", utils.ShortAttID(attestationID, 4))
	if juror {
		logPrefix = fmt.Sprintf("[Juror-%s]", utils.ShortAttID(attestationID, 4))
		t := time.Now()
		why := mayVote(ctx, client, attestationID, logPrefix)
		row.CheckMs = results.Dur(time.Since(t))
		switch why {
		case "":
			row.Role = "juror"
		case "stopped":
			return false
		default: // a candidate that does not vote: its check is its work on the round
			row.Role = "candidate"
			record(why)
			return false
		}
	}
	logger.Debug("%s Retrieving the evidence and the reference...", logPrefix)

	// 1. Read the evidence and the reference
	t := time.Now()
	freshSig, refSig, iterCount, boundBlock, err := client.GetAttestationSignatures(attestationID)
	if err != nil {
		// A juror catching up on events (after a restart) finds closed cycles;
		// an agent not eligible at the evidence, or the prover, cannot judge it.
		if juror && blockchain.IsRevert(err, "not ready for evaluation") {
			logger.Debug("%s Cycle already closed, no vote", logPrefix)
			return false
		}
		if juror && blockchain.IsRevert(err, "caller does not judge this cycle") {
			logger.Debug("%s Not a candidate juror of this cycle", logPrefix)
			return false
		}
		logger.Error("%s Failed to get signatures: %v", logPrefix, err)
		counters.errors.Add(1)
		return record("error")
	}
	// V3: the prover bound each measurement to the hash of block boundBlock;
	// the reference is bound the same way, sha256(ref || blockhash).
	var blockHash [32]byte
	if client.HasFreshness() {
		if blockHash, err = client.BlockHash(boundBlock); err != nil {
			logger.Error("%s Failed to read the hash of block %d: %v", logPrefix, boundBlock, err)
			counters.errors.Add(1)
			return record("error")
		}
	}
	row.ReadMs = results.Dur(time.Since(t))

	// 2. Evaluation (compute)
	t = time.Now()
	if client.HasFreshness() {
		ref, err := blockchain.AsBytes32(refSig)
		if err != nil {
			logger.Error("%s Invalid reference: %v", logPrefix, err)
			counters.errors.Add(1)
			return record("error")
		}
		bound := compute.BindToBlock(ref, blockHash)
		refSig = hex.EncodeToString(bound[:])
	}
	// Re-derive the chained reference: under the no-tamper hypothesis the
	// binary is static, so all K m_i = refSig and the prover's H_K equals
	// chain(refSig, K). For K=1 this is identity. Must stay bit-for-bit
	// identical to compute.RollingHashStep used in the prover.
	derivedRef, derr := compute.RollingHashHex(refSig, iterCount)
	if derr != nil {
		logger.Error("%s Failed to derive rolling reference (K=%d): %v", logPrefix, iterCount, derr)
		counters.errors.Add(1)
		return record("error")
	}
	// freshSig is "0x"-prefixed (Web3 convention); derivedRef is bare hex.
	freshNorm := strings.TrimPrefix(strings.ToLower(freshSig), "0x")
	isSuccess := strings.EqualFold(freshNorm, derivedRef)
	if cfg.ForceVerdict != nil {
		isSuccess = *cfg.ForceVerdict
		logger.Info("%s Verdict forced to %v (FORCE_VERDICT)", logPrefix, isSuccess)
	}
	row.ComputeMs = results.Dur(time.Since(t))
	row.Verdict = results.Verdict(isSuccess)

	// 3. Submit Result
	statusIcon := "❌ FAILURE"
	if isSuccess {
		statusIcon = "✅ SUCCESS"
	}
	if !juror {
		logger.Info("%s Attestation closed (result: %s)", logPrefix, statusIcon)
	}

	timeout := 60
	t = time.Now()
	row.Attempts = 1
	if juror {
		_, err = client.Vote(attestationID, isSuccess, cfg.WaitForTxConfirmations, timeout)
		row.SendMs = results.Dur(time.Since(t))
		switch {
		case err == nil:
			logger.Info("%s Drawn in the jury, vote sent (verdict: %s)", logPrefix, statusIcon)
			counters.votes.Add(1)
		case blockchain.IsRevert(err, "caller is not in the jury"):
			logger.Debug("%s Not drawn in the jury", logPrefix)
			return false
		case blockchain.IsRevert(err, "cycle closed"), blockchain.IsRevert(err, "cycle is with the SECaaS"):
			logger.Info("%s Cycle closed or handed to the SECaaS before this vote", logPrefix)
			return record("late")
		default:
			logger.Error("%s Failed to vote: %v", logPrefix, err)
			counters.errors.Add(1)
			return record("error")
		}
	} else {
		_, err = client.Verify(attestationID, isSuccess, cfg.WaitForTxConfirmations, timeout)
		row.SendMs = results.Dur(time.Since(t))
		if err != nil {
			logger.Error("%s Failed to send result: %v", logPrefix, err)
			counters.errors.Add(1)
			return record("error")
		}
		counters.verified.Add(1)
	}
	return record("sent")
}
