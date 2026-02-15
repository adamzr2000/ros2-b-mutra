package attestation

import (
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

	// Send evidence with wait=true and timeout=60s
	_, err := bc.SendEvidence(attestationID, measurement, waitForTx, 60)

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

	// 3. Wait to become Prover (Poll Loop)
	startWait := time.Now()
	timeout := 60 * time.Second

	for {
		isProver, err := bc.IsProver(attestationID)
		if err == nil && isProver {
			break
		}

		select {
		case <-stopCh:
			logger.Info("%s Stopping while waiting to become prover.", logPrefix)
			return
		default:
		}

		if time.Since(startWait) > timeout {
			logger.Error("%s Timeout while waiting to become prover.", logPrefix)
			return
		}
		time.Sleep(15 * time.Millisecond)
	}

	logger.Info("%s Waiting for verification result...", logPrefix)

	// 4. Wait for Attestation to Close
	startWait = time.Now()
	for {
		state, err := bc.GetAttestationState(attestationID)
		if err == nil && state == blockchain.Closed {
			break
		}

		select {
		case <-stopCh:
			logger.Info("%s Stopping while waiting for attestation to close.", logPrefix)
			return
		default:
		}

		if time.Since(startWait) > timeout {
			logger.Error("%s Timeout while waiting for attestation to close.", logPrefix)
			return
		}
		time.Sleep(15 * time.Millisecond)
	}

	timestamps["p_result_received"] = utils.PerfNs()
	timestamps["result_received"] = utils.NowMs()

	// 5. Retrieve Result
	_, verifier, resultEnum, ts, err := bc.GetAttestationInfo(attestationID)
	if err != nil {
		logger.Error("%s Failed to read attestation info: %v", logPrefix, err)
	} else {
		// Enum: 0=None, 1=Failure, 2=Success
		isSuccess := (resultEnum == 2)
		statusIcon := "❌ FAILURE"
		if isSuccess {
			statusIcon = "✅ SUCCESS"
		}
		logger.Info("%s Attestation Complete", logPrefix)
		logger.Info("%s Result: %s", logPrefix, statusIcon)
		logger.Info("%s Verified by: %s at %v", logPrefix, verifier.Hex(), ts)
	}

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
		bc, attestationID, measurement, stopCh, 
		config.ExportEnabled, config.AgentName, config.ResultsDir, config.ResultsFile,
		startTs, startP, config.WaitForTxConfirmations,
	)
}

// RunProverLogicContinuousMode runs the continuous prover loop
func RunProverLogicContinuousMode(
	stopCh <-chan struct{},
	bc *blockchain.BlockchainClient,
	store storage.StorageBackend,
	config ProverConfig,
	activeAttestations *sync.Map,
	wg *sync.WaitGroup,
) {
	defer wg.Done()

	logger.Info("Starting attestation process in continuous mode (single-hash)...")

	// Create local state
	state := &ProverState{
		Digests:      make([]string, 0),
		Measuring:    true,
	}

	for {
		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		// Simple measuring check
		measuring := state.Measuring
		if !measuring {
			time.Sleep(25 * time.Millisecond)
			continue
		}

		// Record the start time of *this specific attestation cycle*
		// This will be passed down to become timestamps["prover_start"]
		batchStartTs := utils.NowMs()
		batchStartP := utils.PerfNs()

		// 1. Measure / Compute Hash
		var digest string
		var err error

		if config.TextSectionPrefix != "" {
			logger.Debug("Using dynamic offset calculation with prefix: %s...", config.TextSectionPrefix[:min(16, len(config.TextSectionPrefix))])
			digest, err = compute.ComputeProgramHashWithPrefix(
				config.CmdName,
				config.TextSectionSize,
				config.TextSectionPrefix,
			)
		} else {
			digest, err = compute.ComputeProgramHash(
				config.CmdName,
				config.TextSectionSize,
				config.Offset,
			)
		}

		if err != nil {
			if compute.IsComputeHashError(err) {
				logger.Error("Hash computation error: %v", err)
			} else {
				logger.Error("Error in attestation process: %v", err)
			}
			time.Sleep(1 * time.Second)
			continue
		}

		select {
		case <-stopCh:
			logger.Info("Attestation process stopped")
			return
		default:
		}

		finalDigest := digest
		state.FinalDigest = finalDigest

		logger.Info("[Prover] Final digest: %s...%s",
			finalDigest[:16], finalDigest[len(finalDigest)-16:])

		// 2. Compute Self-Integrity (if enabled)
		combinedDigest := finalDigest
		sidecarHash := ""
		libsHash := ""

		if config.SelfIntegrity.Enabled {
			sidecarHash, err = compute.ComputeProgramHash(
				config.SelfIntegrity.CmdName,
				config.SelfIntegrity.TextSectionSize,
				config.SelfIntegrity.TextSectionOffset,
			)
			if err == nil {
				if sidecarHash != config.SelfIntegrity.ExpectedSHA256 {
					logger.Warn("[Prover] ⚠️ Self-integrity mismatch!")
				}
				// Combine
				combined := finalDigest + sidecarHash
				combinedHashBytes := sha256.Sum256([]byte(combined))
				combinedDigest = hex.EncodeToString(combinedHashBytes[:])
			} else {
				logger.Error("[Prover] Failed to compute self-integrity hash: %v", err)
			}
		}

		// 3. Libs Hash Logic
		libsEnabled := utils.GetEnvBool("ENABLE_LIBS_HASH", false)
		if libsEnabled {
			exePath, err := os.Readlink("/proc/self/exe")
			if err == nil {
				libsDigest, err := compute.ComputeLoadedLibrariesHash(os.Getpid(), exePath)
				if err == nil && libsDigest.Aggregate != "" {
					libsHash = libsDigest.Aggregate
				}
			}
		}

		if libsHash != "" {
			combined := finalDigest + sidecarHash + libsHash
			combinedHashBytes := sha256.Sum256([]byte(combined))
			combinedDigest = hex.EncodeToString(combinedHashBytes[:])
		}

		// Expected Libs Hash
		expectedLibsHash := utils.GetEnvStr("EXPECTED_LIBS_HASH", "")
		if libsEnabled && expectedLibsHash != "" {
			if libsHash != expectedLibsHash {
				logger.Warn("[Prover] ⚠️ Loaded libs hash mismatch!")
			}
		}

		// 4. Push to Storage
		store.PushFinalDigest("final_digests:"+config.AgentName, map[string]interface{}{
			"final_digest":    finalDigest,
			"sidecar_hash":    sidecarHash,
			"libs_hash":       libsHash,
			"combined_digest": combinedDigest,
			"threshold":       1,
			"timestamp":       float64(time.Now().Unix()),
		})

		// 5. Start Attestation Transaction
		attestationID := utils.GenerateAttestationID()

		wg.Add(1)
		go func(attID string, evidence string, startTs int64, startP int64) {
			defer wg.Done()
			RunProverAndCleanup(
				bc,
				attID,
				evidence,
				startTs,
				startP,
				stopCh,
				config,
				activeAttestations,
			)
		}(attestationID, combinedDigest, batchStartTs, batchStartP)

		activeAttestations.Store(attestationID, true)

		// Short sleep to allow async start
		time.Sleep(500 * time.Millisecond)

		logger.Debug("Final digest stored in key: final_digests:%s", config.AgentName)

		// Export storage if memory backend
		if memStor, ok := store.(*storage.MemoryStorageBackend); ok && config.MemoryStorageFile != "" {
			memStor.ExportToFile(config.MemoryStorageFile)
		}

		if config.OneShot {
			logger.Info("One-shot attestation completed; stopping prover loop")
			return
		}

		time.Sleep(attestationInterval())
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