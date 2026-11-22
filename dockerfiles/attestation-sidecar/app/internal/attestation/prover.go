package attestation

import (
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"app/internal/blockchain"
	"app/internal/compute"
	"app/internal/logger"
	"app/internal/results"
	"app/internal/utils"
)

// SelfIntegrityInfo holds self-integrity check configuration
type SelfIntegrityInfo struct {
	Enabled           bool
	CmdName           string
	TextSectionSize   int
	TextSectionOffset int
	TextSectionPrefix []byte
	ExpectedSHA256    string
}

// ProverConfig holds configuration for the prover
type ProverConfig struct {
	CmdName                   string
	TextSectionSize           int
	Offset                    int
	TextSectionPrefix         string    // Hex string of first N bytes of .text section
	IterQThreshold            uint32    // K = fresh measurements folded per on-chain attestation
	Salt                      *[32]byte // nil = unsalted measurements (contracts LV and RR)
	AgentName                 string
	SelfIntegrity             SelfIntegrityInfo
	Rounds                    int           // batches to send, then stop; 0 = until stopped
	StartOffset               time.Duration // wait before the first batch
	WaitForVerificationResult bool
}

// ProcessProverAttestation sends one round and waits for its verdict. row
// holds the round id, the robot and the evidence time; ready is when the
// evidence was ready. The row is recorded unless the prover is stopped.
func ProcessProverAttestation(
	bc *blockchain.BlockchainClient,
	row results.Prover,
	ready time.Time,
	measurement string,
	iterCount uint32,
	boundBlock uint64,
	stopCh <-chan struct{},
	waitForResult bool,
) {
	attestationID := row.RoundID
	logPrefix := fmt.Sprintf("[Prover-%s]", utils.ShortAttID(attestationID, 4))
	record := func(outcome string) {
		row.Outcome = outcome
		results.AddProver(row)
	}

	// 1. Send Evidence. A jury contract refuses it while a round left open by
	// a stop is still with the jury. Rounds slower than the SSP leave batches
	// waiting: one already past the freshness window would be rejected, so it
	// is not sent.
	select {
	case <-stopCh:
		return // stopped: the batches still waiting are not sent
	default:
	}
	if !waitForOpenRound(bc, stopCh, logPrefix) {
		return
	}
	if stale, err := pastFreshnessWindow(bc, boundBlock); err == nil && stale {
		logger.Warn("%s Evidence past the freshness window, not sent (rounds slower than the SSP)", logPrefix)
		counters.stale.Add(1)
		record("stale")
		return
	}
	// Marked before it is sent: this agent's verifier, notified of the same
	// block as the prover, must not take it for another agent's round.
	ownRounds.Store(attestationID, true)
	defer ownRounds.Delete(attestationID)

	start := time.Now()
	row.TSentMs = start.UnixMilli()
	row.PresendMs = results.Dur(start.Sub(ready))

	// iterCount carries K for the on-chain record so the verifier can
	// re-derive the chained reference. Sent without waiting: the wait below
	// is timed apart.
	txHash, err := bc.Prove(attestationID, measurement, iterCount, boundBlock, false, 60)
	sent := time.Now()
	row.SendMs = results.Dur(sent.Sub(start))
	if err != nil {
		logger.Error("%s Failed to send evidence: %v", logPrefix, err)
		counters.errors.Add(1)
		record("error")
		return
	}
	logger.Info("%s Evidence sent (attestation started)", logPrefix)

	// 2. Wait for the evidence to be mined, checked at each new block: a
	// rejected Prove (stale evidence, open round) is reported at once
	// instead of after the close timeout.
	if err := bc.WaitForReceipt(txHash, 60); err != nil {
		logger.Error("%s Evidence not accepted: %v", logPrefix, err)
		if errors.Is(err, blockchain.ErrReceiptTimeout) {
			counters.timeouts.Add(1)
			record("timeout")
		} else {
			counters.errors.Add(1)
			record("rejected")
		}
		return
	}
	mined := time.Now()
	row.InclusionMs = results.Dur(mined.Sub(sent))
	counters.proved.Add(1)

	if !waitForResult {
		record("sent")
		return
	}
	if bc.HasJury() {
		openRound.Store(attestationID)
	}

	logger.Info("%s Waiting for verification result...", logPrefix)

	// 3. Wait for Attestation to Close. With a jury (V3), hand the cycle to
	// the SECaaS once its timeout has passed without enough votes.
	jury := bc.HasJury()
	var expireAt uint64 // first block where Expire is accepted, 0 = not known yet
	expired := false
	var expiryChecked time.Time // the timeout is blocks long: checked once a second
	timeout := 60 * time.Second
	for {
		next := bc.NextHead()
		state, err := bc.GetAttestationState(attestationID)
		if err == nil && state == blockchain.Closed {
			openRound.Store("")
			break
		}
		if jury && !expired && err == nil && state == blockchain.ReadyForEvaluation && time.Since(expiryChecked) >= time.Second {
			expiryChecked = time.Now()
			expired = expireIfDue(bc, attestationID, txHash, &expireAt, logPrefix)
		}

		if time.Since(mined) > timeout {
			logger.Error("%s Timeout while waiting for attestation to close.", logPrefix)
			counters.timeouts.Add(1)
			record("timeout")
			return
		}
		if !blockchain.WaitHead(next, stopCh) {
			logger.Info("%s Stopping while waiting for attestation to close.", logPrefix)
			return
		}
	}
	closed := time.Now()
	row.JudgementMs = results.Dur(closed.Sub(mined))
	total := row.EvidenceMs + results.Ms(closed.Sub(ready))
	row.TotalMs = &total

	// 4. Retrieve Result
	_, verifier, resultEnum, err := bc.GetAttestationInfo(attestationID)
	if err != nil {
		logger.Error("%s Failed to read attestation info: %v", logPrefix, err)
		counters.errors.Add(1)
		record("error")
		return
	}
	// Enum: 0=None, 1=Failure, 2=Success
	isSuccess := (resultEnum == 2)
	if isSuccess {
		counters.success.Add(1)
	} else {
		counters.failure.Add(1)
	}
	record(results.Verdict(isSuccess))
	statusIcon := "❌ FAILURE"
	if isSuccess {
		statusIcon = "✅ SUCCESS"
	}
	logger.Info("%s Attestation Complete", logPrefix)
	logger.Info("%s Result: %s", logPrefix, statusIcon)
	verifiedBy := verifier.Hex()
	votes := ""
	if jury {
		// V3: a cycle closed by its jury names the juror whose vote closed it
		jurors, summary, err := bc.JuryVotes(attestationID)
		if err != nil {
			logger.Error("%s Failed to read the jury votes: %v", logPrefix, err)
		}
		for _, j := range jurors {
			if j == verifier {
				verifiedBy = "the jury, closing vote from " + verifier.Hex()
			}
		}
		votes = summary
	}
	logger.Info("%s Verified by: %s", logPrefix, verifiedBy)
	if votes != "" {
		logger.Info("%s Jury votes: %s", logPrefix, votes)
	}
}

// expireIfDue sends Expire once the head block reaches the jury timeout of
// the cycle. Returns true once Expire has been tried, whatever its outcome:
// a revert means the jury closed the cycle in the meantime.
func expireIfDue(bc *blockchain.BlockchainClient, attID, txHash string, expireAt *uint64, logPrefix string) bool {
	if *expireAt == 0 {
		seed, err := bc.TxBlockNumber(txHash)
		if err != nil {
			return false // evidence not mined yet
		}
		timeout, err := bc.JuryTimeoutBlocks()
		if err != nil {
			logger.Error("%s Failed to read the jury timeout: %v", logPrefix, err)
			return true
		}
		*expireAt = seed + timeout
	}
	// Expire goes in the next block at best, where its gas is also estimated.
	head, err := bc.BlockNumber()
	if err != nil || head+1 < *expireAt {
		return false
	}
	if _, err := bc.Expire(attID, true, 60); err != nil {
		logger.Warn("%s Expire not sent: %v", logPrefix, err)
		return true
	}
	logger.Info("%s Jury timeout reached, cycle handed to the SECaaS", logPrefix)
	return true
}

// RunProverAndCleanup runs the prover and cleans up
func RunProverAndCleanup(
	bc *blockchain.BlockchainClient,
	row results.Prover,
	ready time.Time,
	measurement string,
	boundBlock uint64,
	stopCh <-chan struct{},
	config ProverConfig,
	activeAttestations *sync.Map,
) {
	start := time.Now()
	logPrefix := fmt.Sprintf("[Prover-%s]", utils.ShortAttID(row.RoundID, 4))

	defer func() {
		elapsed := time.Since(start)
		logger.Info("%s", logger.CyanText(fmt.Sprintf("%s completed in %.2fs", logPrefix, elapsed.Seconds())))
		activeAttestations.Delete(row.RoundID)
	}()

	ProcessProverAttestation(
		bc, row, ready, measurement, config.IterQThreshold, boundBlock, stopCh,
		config.WaitForVerificationResult,
	)
}

// measureCombinedOnce takes one measurement m_i: the hash of the attested
// process, combined with the sidecar's own hash (self-integrity) and the
// loaded libraries when enabled, as hex.
func measureCombinedOnce(config ProverConfig) (string, error) {
	var robotHash, sidecarHash, libsHash string
	var err error
	if config.TextSectionPrefix != "" {
		robotHash, err = compute.ComputeProgramHashWithPrefix(
			config.CmdName,
			config.TextSectionSize,
			config.TextSectionPrefix,
		)
	} else {
		robotHash, err = compute.ComputeProgramHash(
			config.CmdName,
			config.TextSectionSize,
			config.Offset,
		)
	}
	if err != nil {
		return "", err
	}
	combinedDigest := robotHash

	if config.SelfIntegrity.Enabled {
		sidecarHash, err = compute.ComputeProgramHash(
			config.SelfIntegrity.CmdName,
			config.SelfIntegrity.TextSectionSize,
			config.SelfIntegrity.TextSectionOffset,
		)
		if err != nil {
			return "", err
		}
		if sidecarHash != config.SelfIntegrity.ExpectedSHA256 {
			logger.Warn("[Prover] ⚠️ Self-integrity mismatch!")
		}
		sum := sha256.Sum256([]byte(robotHash + sidecarHash))
		combinedDigest = hex.EncodeToString(sum[:])
	}

	if utils.GetEnvBool("ENABLE_LIBS_HASH", false) {
		exePath, e := os.Readlink("/proc/self/exe")
		if e == nil {
			libs, e2 := compute.ComputeLoadedLibrariesHash(os.Getpid(), exePath)
			if e2 == nil && libs.Aggregate != "" {
				libsHash = libs.Aggregate
				sum := sha256.Sum256([]byte(robotHash + sidecarHash + libsHash))
				combinedDigest = hex.EncodeToString(sum[:])
			}
		}
		expected := utils.GetEnvStr("EXPECTED_LIBS_HASH", "")
		if expected != "" && libsHash != expected {
			logger.Warn("[Prover] ⚠️ Loaded libs hash mismatch!")
		}
	}
	return combinedDigest, nil
}

// RunProverLogicContinuousMode runs the continuous prover loop.
//
// Per cycle: takes K = config.IterQThreshold fresh measurements spaced by
// ATTESTATION_INTERVAL_MS, folds them into a single rolling hash H_K via
// byte-level chained SHA-256 (compute.RollingHashStep), and submits one
// Prove(id, H_K, K) on-chain. K=1 reduces to single-shot. With a contract
// that checks freshness, each measurement is first bound to the hash of a
// recent block, read before the batch, and that block goes with the evidence.
func RunProverLogicContinuousMode(
	stopCh <-chan struct{},
	bc *blockchain.BlockchainClient,
	config ProverConfig,
	activeAttestations *sync.Map,
	wg *sync.WaitGroup,
) {
	defer wg.Done()

	K := config.IterQThreshold
	if K < 1 {
		K = 1
	}
	logger.Info("Starting attestation process in continuous mode (IterQ K=%d)...", K)
	if config.StartOffset > 0 {
		select {
		case <-stopCh:
			return
		case <-time.After(config.StartOffset):
		}
	}

	bindToBlock := bc.HasFreshness()
	var boundBlock uint64
	batches := 0

	// A jury contract refuses a new round while the prover's last one is open,
	// and accepts rounds bound to increasing blocks only: they are sent one
	// after the other, in the order of their batches, by a single worker.
	// ponytail: the measuring loop blocks once 64 rounds wait, if the SSP stays
	// shorter than a round.
	var serial chan func()
	if bc.HasJury() {
		serial = make(chan func(), 64)
		defer close(serial)
		go func() {
			for round := range serial {
				round()
			}
		}()
	}

	for {
		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		attestationID := utils.GenerateAttestationID()
		logPrefix := fmt.Sprintf("[Prover-%s]", utils.ShortAttID(attestationID, 4))

		var boundHash [32]byte
		if bindToBlock {
			var ok bool
			if boundBlock, boundHash, ok = nextBoundBlock(bc, boundBlock, stopCh); !ok {
				logger.Info("Attestation process stopped")
				return
			}
		}

		// Fold K fresh measurements into a rolling-hash accumulator.
		evidenceStart := time.Now()
		var slept time.Duration
		var H [32]byte
		failed := false
		for i := uint32(1); i <= K; i++ {
			if i > 1 {
				sleepStart := time.Now()
				select {
				case <-stopCh:
					logger.Info("Attestation process stopped")
					return
				case <-time.After(measurementDelay()):
				}
				slept += time.Since(sleepStart)
			}

			logger.Info("%s Measurement %d/%d", logPrefix, i, K)
			combined, err := measureCombinedOnce(config)
			if err == nil {
				counters.measurements.Add(1)
			}
			if err != nil {
				if compute.IsComputeHashError(err) {
					logger.Debug("Hash computation error at iter %d/%d: %v", i, K, err)
				} else {
					logger.Error("Error in attestation process at iter %d/%d: %v", i, K, err)
					counters.errors.Add(1)
				}
				failed = true
				break
			}

			raw, err := hex.DecodeString(combined)
			if err != nil || len(raw) != 32 {
				logger.Error("Invalid combined digest at iter %d/%d (len=%d): %v", i, K, len(raw), err)
				counters.errors.Add(1)
				failed = true
				break
			}
			var m [32]byte
			copy(m[:], raw)
			if config.Salt != nil {
				m = compute.SaltMeasurement(m, *config.Salt)
			}
			if bindToBlock {
				m = compute.BindToBlock(m, boundHash)
			}

			if i == 1 {
				H = m
			} else {
				H = compute.RollingHashStep(H, m)
			}
			logger.Debug("[Prover] iter %d/%d: m=%s...%s", i, K, combined[:16], combined[len(combined)-16:])
		}
		if failed {
			time.Sleep(1 * time.Second)
			continue
		}
		ready := time.Now()
		row := results.Prover{
			RoundID:     attestationID,
			Robot:       config.AgentName,
			TMeasuredMs: evidenceStart.UnixMilli(),
			EvidenceMs:  results.Ms(ready.Sub(evidenceStart) - slept),
		}

		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		freshSig := hex.EncodeToString(H[:])
		if K == 1 {
			logger.Info("[Prover] Final digest (K=1): %s...%s",
				freshSig[:16], freshSig[len(freshSig)-16:])
		} else {
			logger.Info("[Prover] Rolling-hash digest (K=%d): %s...%s",
				K, freshSig[:16], freshSig[len(freshSig)-16:])
		}

		activeAttestations.Store(attestationID, true)
		wg.Add(1)
		round := func(row results.Prover, ready time.Time, evidence string, boundBlock uint64) func() {
			return func() {
				defer wg.Done()
				RunProverAndCleanup(
					bc, row, ready, evidence, boundBlock, stopCh, config, activeAttestations,
				)
			}
		}(row, ready, freshSig, boundBlock)
		if serial != nil {
			serial <- round
		} else {
			go round()
		}

		batches++
		if config.Rounds > 0 && batches >= config.Rounds {
			logger.Info("%d round(s) sent; stopping prover loop", batches)
			return
		}

		// Inter-cycle sleep — the period between m_K of this cycle and m_1 of
		// the next equals the SSP exactly, with a 500ms floor that only applies
		// when SSP < 500ms (e.g. SSP=0 back-to-back mode), so that back-to-back
		// mode sends at most two Prove per second. For SSP >= 500ms the cycle
		// period is precisely the SSP, so detection latency models can use
		// P = SSP without a +0.5s constant. In exp mode there is no floor: the
		// end of a batch is visible on chain, so a floor would give an attacker
		// 500ms of safe time after it.
		sleep := measurementDelay()
		if os.Getenv("ATTESTATION_INTERVAL_MODE") != "exp" && sleep < 500*time.Millisecond {
			sleep = 500 * time.Millisecond
		}
		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		case <-time.After(sleep):
		}
	}
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// attestationInterval uses utils.GetEnvInt to respect environment configuration
// attestationInterval is the sleep between consecutive measurements (SSP).
// ATTESTATION_INTERVAL_MS=0 is explicitly allowed and means "no sleep" — the
// prover loop runs back-to-back measurements, bounded only by the cgroup
// CPU_LIMIT. Negative values are coerced to 0 for safety.
func attestationInterval() time.Duration {
	ms := utils.GetEnvInt("ATTESTATION_INTERVAL_MS", 0)
	if ms < 0 {
		ms = 0
	}
	return time.Duration(ms) * time.Millisecond
}

// ownRounds are the rounds this agent proves, from just before each Prove
// until its verdict: its verifier skips them.
var ownRounds sync.Map

// openRound is the prover's last jury round until it is seen closed: set
// still when the prover stopped while waiting for it.
var openRound atomic.Value

// waitForOpenRound waits until the round left open, if any, is closed or with
// the SECaaS (a jury that does not vote is expired by the SECaaS). false
// when the prover is stopped meanwhile.
func waitForOpenRound(bc *blockchain.BlockchainClient, stopCh <-chan struct{}, logPrefix string) bool {
	id, _ := openRound.Load().(string)
	if id == "" {
		return true
	}
	logger.Info("%s Waiting for the previous round to leave the jury", logPrefix)
	for {
		next := bc.NextHead()
		if state, err := bc.GetAttestationState(id); err == nil && state != blockchain.ReadyForEvaluation {
			openRound.Store("")
			return true
		}
		if !blockchain.WaitHead(next, stopCh) {
			return false
		}
	}
}

// pastFreshnessWindow tells whether evidence bound to boundBlock would be
// rejected as stale: a transaction sent now goes in the next block at best.
func pastFreshnessWindow(bc *blockchain.BlockchainClient, boundBlock uint64) (bool, error) {
	if !bc.HasFreshness() {
		return false, nil
	}
	window, err := bc.FreshnessBlocks()
	if err != nil {
		return false, err
	}
	// The block just notified, else asked to the node.
	head, _, live := bc.LastHead()
	if !live {
		if head, err = bc.BlockNumber(); err != nil {
			return false, err
		}
	}
	return head+1-boundBlock > window, nil
}

// nextBoundBlock returns the block the next batch is bound to, the parent of
// the latest one, once it is newer than last: the contract accepts each round
// bound to a newer block than the one before. ok is false when the prover is
// stopped.
func nextBoundBlock(bc *blockchain.BlockchainClient, last uint64, stopCh <-chan struct{}) (uint64, [32]byte, bool) {
	for {
		next := bc.NextHead()
		n, hash, err := bc.ParentBlock()
		if err == nil && n > last {
			return n, hash, true
		}
		if err != nil {
			logger.Error("[Prover] Failed to read the block to bind the evidence to: %v", err)
			counters.errors.Add(1)
		}
		if !blockchain.WaitHead(next, stopCh) {
			return 0, [32]byte{}, false
		}
	}
}

// measurementDelay is the wait before a measurement: the SSP in fixed mode,
// a draw of mean SSP in exp mode. Like the SSP, the mode is read on every
// call, so that POST /config can change it between two batches.
func measurementDelay() time.Duration {
	if os.Getenv("ATTESTATION_INTERVAL_MODE") == "exp" {
		return nextInterval(attestationInterval())
	}
	return attestationInterval()
}
