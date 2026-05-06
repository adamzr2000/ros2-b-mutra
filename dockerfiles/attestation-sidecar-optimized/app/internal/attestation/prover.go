package attestation

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"os"
	"sync"
	"time"

	"app/internal/blockchain"
	"app/internal/compute"
	"app/internal/logger"
	// "app/internal/metrics"
	"app/internal/storage"
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
	CmdName           string
	TextSectionSize   int
	Offset            int
	TextSectionPrefix string // Hex string of first N bytes of .text section
	Threshold         int    // Kept for config compatibility, but logic uses interval
	IterQThreshold    uint32 // K = fresh measurements folded per on-chain attestation
	AgentName         string
	ExportEnabled     bool
	ResultsDir        string
	ResultsFile       string
	MemoryStorageFile string
	SelfIntegrity     SelfIntegrityInfo
	OneShot           bool
	WaitForTxConfirmations bool
}

// ProverState holds the runtime state of the prover
type ProverState struct {
	Digests      []string
	DigestsCount int
	FinalDigest  string
	Measuring    bool
	mu           sync.Mutex
}

// ProcessProverAttestation processes a single attestation as prover
func ProcessProverAttestation(
	bc *blockchain.BlockchainClient,
	attestationID string,
	measurement string,
	iterCount uint32,
	stopCh <-chan struct{},
	exportEnabled bool,
	participantName string,
	resultsDir string,
	resultsFile string,
	startTs int64,
    startP int64,
    waitForTx bool,
) {
	timestamps := make(map[string]interface{})

	// 1. Capture Prover Start (from the loop's start time)
	timestamps["prover_start"] = startTs
	timestamps["p_prover_start"] = startP

	logPrefix := fmt.Sprintf("[Prover-%s]", utils.ShortAttID(attestationID, 4))

	// 2. Send Evidence
	timestamps["evidence_sent"] = utils.NowMs()
	timestamps["p_send_evidence_start"] = utils.PerfNs()

	// Send evidence with wait=true and timeout=60s. iterCount carries K for
	// the on-chain record so the verifier can re-derive the chained reference.
	_, err := bc.SendEvidence(attestationID, measurement, iterCount, waitForTx, 60)

	timestamps["p_send_evidence_finished"] = utils.PerfNs()

	// Only record confirmation if we actually waited for it!
    if waitForTx {
        timestamps["p_send_evidence_finished_tx_confirmed"] = timestamps["p_send_evidence_finished"]
        timestamps["evidence_sent_tx_confirmed"] = utils.NowMs()
    }

	if err != nil {
		logger.Error("%s Failed to send evidence: %v", logPrefix, err)
		return
	}

	logger.Info("%s Evidence sent (attestation started)", logPrefix)

	// 3. Wait for AttestationCompleted event (event-driven, replaces IsProver +
	//    GetAttestationState + GetAttestationInfo polling).
	//    The contract emits AttestationCompleted(id, verified) for every attestation,
	//    including positive mid-window ones where the struct is deleted on-chain.
	//    Using a context lets us honour both the 60 s timeout and stopCh cancellation.
	watchCtx, watchCancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer watchCancel()
	go func() {
		select {
		case <-stopCh:
			watchCancel()
		case <-watchCtx.Done():
		}
	}()

	logger.Info("%s Waiting for verification result...", logPrefix)
	verified, err := bc.WatchAttestationCompleted(watchCtx, attestationID)

	timestamps["p_result_received"] = utils.PerfNs()
	timestamps["result_received"] = utils.NowMs()

	if err != nil {
		if watchCtx.Err() != nil {
			logger.Info("%s Stopping (context cancelled).", logPrefix)
		} else {
			logger.Error("%s Failed waiting for AttestationCompleted: %v", logPrefix, err)
		}
		return
	}

	statusIcon := "❌ FAILURE"
	if verified {
		statusIcon = "✅ SUCCESS"
	}
	logger.Info("%s Attestation Complete: %s", logPrefix, statusIcon)

	timestamps["p_prover_finished"] = utils.PerfNs()
	timestamps["prover_finished"] = utils.NowMs()

	// 6. Export Results
	if exportEnabled {
		utils.ExportAttestationTimesJSON(
			participantName,
			attestationID,
			"prover",
			timestamps,
			resultsDir,
			resultsFile, // result file path handled by helper if empty
		)
	}
}

// RunProverAndCleanup runs the prover and cleans up
func RunProverAndCleanup(
	bc *blockchain.BlockchainClient,
	attestationID string,
	measurement string,
	startTs int64,
	startP int64,
	stopCh <-chan struct{},
	config ProverConfig,
	activeAttestations *sync.Map,
) {
	start := time.Now()
	logPrefix := fmt.Sprintf("[Prover-%s]", utils.ShortAttID(attestationID, 4))

	defer func() {
		elapsed := time.Since(start)
		logger.Info(logger.GreenText(fmt.Sprintf("%s completed in %.2fs", logPrefix, elapsed.Seconds())))
		activeAttestations.Delete(attestationID)
	}()

	ProcessProverAttestation(
		bc, attestationID, measurement, config.IterQThreshold, stopCh,
		config.ExportEnabled, config.AgentName, config.ResultsDir, config.ResultsFile,
		startTs, startP, config.WaitForTxConfirmations,
	)
}

// measureCombinedOnce performs a single integrity measurement cycle and
// returns the combined digest as a hex string (the same "fresh measurement
// m_i" that, in K=1 mode, would be sent as-is on-chain). The combined digest
// is sha256( robotHash_hex || sidecarHash_hex [|| libsHash_hex] ) — kept as
// hex-string concat for backward compat with the original combined formula.
func measureCombinedOnce(config ProverConfig) (combinedDigest, robotHash, sidecarHash, libsHash string, err error) {
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
		return "", "", "", "", err
	}
	combinedDigest = robotHash

	if config.SelfIntegrity.Enabled {
		sidecarHash, err = compute.ComputeProgramHash(
			config.SelfIntegrity.CmdName,
			config.SelfIntegrity.TextSectionSize,
			config.SelfIntegrity.TextSectionOffset,
		)
		if err != nil {
			return "", robotHash, "", "", err
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
	return combinedDigest, robotHash, sidecarHash, libsHash, nil
}

// RunProverLogicContinuousMode runs the continuous prover loop.
//
// Per cycle: takes K = config.IterQThreshold fresh measurements spaced by
// ATTESTATION_INTERVAL_MS, folds them into a single rolling hash H_K via
// byte-level chained SHA-256 (compute.RollingHashStep), and submits one
// SendEvidence(id, H_K, K) on-chain. K=1 reduces to single-shot.
func RunProverLogicContinuousMode(
	stopCh <-chan struct{},
	bc *blockchain.BlockchainClient,
	store storage.StorageBackend,
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

	state := &ProverState{
		Digests:   make([]string, 0),
		Measuring: true,
	}

	for {
		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		if !state.Measuring {
			time.Sleep(25 * time.Millisecond)
			continue
		}

		// Cycle start timestamps (taken at the FIRST measurement so that
		// p_send_evidence_start - prover_start captures the full K-fold loop).
		batchStartTs := utils.NowMs()
		batchStartP := utils.PerfNs()

		// Fold K fresh measurements into a rolling-hash accumulator.
		var H [32]byte
		var lastCombined, lastRobotHash, lastSidecarHash, lastLibsHash string
		failed := false
		for i := uint32(1); i <= K; i++ {
			// Sleep BEFORE each measurement except the first — the SSP between
			// consecutive measurements is what defines the "vulnerability
			// window granularity" the paper talks about.
			if i > 1 {
				select {
				case <-stopCh:
					logger.Info("Attestation process stopped")
					return
				case <-time.After(attestationInterval()):
				}
			}

			combined, robotHash, sidecarHash, libsHash, err := measureCombinedOnce(config)
			if err != nil {
				if compute.IsComputeHashError(err) {
					logger.Debug("Hash computation error at iter %d/%d: %v", i, K, err)
				} else {
					logger.Error("Error in attestation process at iter %d/%d: %v", i, K, err)
				}
				failed = true
				break
			}
			lastCombined, lastRobotHash, lastSidecarHash, lastLibsHash = combined, robotHash, sidecarHash, libsHash

			raw, err := hex.DecodeString(combined)
			if err != nil || len(raw) != 32 {
				logger.Error("Invalid combined digest at iter %d/%d (len=%d): %v", i, K, len(raw), err)
				failed = true
				break
			}
			var m [32]byte
			copy(m[:], raw)

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

		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		freshSig := hex.EncodeToString(H[:])
		state.FinalDigest = freshSig
		if K == 1 {
			logger.Info("[Prover] Final digest (K=1): %s...%s",
				freshSig[:16], freshSig[len(freshSig)-16:])
		} else {
			logger.Info("[Prover] Rolling-hash digest (K=%d): %s...%s",
				K, freshSig[:16], freshSig[len(freshSig)-16:])
		}

		// Storage trace for debugging — record the LAST combined digest that
		// fed the chain plus the chain output.
		store.PushFinalDigest("final_digests:"+config.AgentName, map[string]interface{}{
			"final_digest":    lastCombined, // last m_i, useful for debugging
			"sidecar_hash":    lastSidecarHash,
			"libs_hash":       lastLibsHash,
			"robot_hash":      lastRobotHash,
			"combined_digest": freshSig, // H_K — what actually goes on-chain
			"iter_count":      K,
			"timestamp":       float64(time.Now().Unix()),
		})

		attestationID := utils.GenerateAttestationID()

		wg.Add(1)
		go func(attID, evidence string, startTs, startP int64) {
			defer wg.Done()
			RunProverAndCleanup(
				bc, attID, evidence, startTs, startP, stopCh, config, activeAttestations,
			)
		}(attestationID, freshSig, batchStartTs, batchStartP)

		activeAttestations.Store(attestationID, true)

		// Short sleep to let the goroutine kick off before next cycle.
		time.Sleep(500 * time.Millisecond)

		if memStor, ok := store.(*storage.MemoryStorageBackend); ok && config.MemoryStorageFile != "" {
			memStor.ExportToFile(config.MemoryStorageFile)
		}

		if config.OneShot {
			logger.Info("One-shot attestation completed; stopping prover loop")
			return
		}

		// Inter-cycle sleep — preserves the SSP between m_K of this cycle
		// and m_1 of the next cycle, so the cadence remains constant across
		// cycle boundaries.
		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		case <-time.After(attestationInterval()):
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
func attestationInterval() time.Duration {
	ms := utils.GetEnvInt("ATTESTATION_INTERVAL_MS", 15000)
	if ms <= 0 {
		ms = 15
	}
	return time.Duration(ms) * time.Millisecond
}