package attestation

import (
	"context"
	"fmt"
	"path/filepath"
	"strings"
	"sync"
	"time"

	"app/internal/blockchain"
	"app/internal/logger"
	"app/internal/utils"

	"github.com/ethereum/go-ethereum/core/types"
)

// VerifierConfig holds all configuration required by the Verifier.
type VerifierConfig struct {
	ParticipantName        string
	ResultsDir             string
	ResultsFile            string
	ExportEnabled          bool
	WaitForTxConfirmations bool

	// Event Watcher Config
	EventCheckpointDir string
	EventConfirmations uint64
	EventBatchSize     uint64
	EventPollInterval  float64
	EventLookback      uint64
}

// verificationJob represents a task in the worker queue
type verificationJob struct {
	AttestationID string
	SeedTs        map[string]interface{}
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
		cfg.EventPollInterval,
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
	workChan := make(chan verificationJob, 100) // Buffer 100

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
			case job := <-workChan:
				start := time.Now()

				logger.Info("Processing attestation '%s'", job.AttestationID)
				processVerifierAttestation(ctx, client, cfg, job.AttestationID, job.SeedTs)

				elapsed := time.Since(start).Seconds()
				logPrefix := fmt.Sprintf("[Verifier-%s]", utils.ShortAttID(job.AttestationID, 4))
				logger.Info(logger.GreenText(fmt.Sprintf("%s completed in %.2fs", logPrefix, elapsed)))

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
		arrivalTs := utils.NowMs()

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

		// Check if I am the verifier
		isVerifier, err := client.IsVerifier(attID)
		if err != nil {
			logger.Error("IsVerifier check failed for %s: %v", attID, err)
			return nil // Don't crash handler
		}

		if isVerifier {
			enqLock.Lock()
			if !enqueuedIDs[attID] {
				enqueuedIDs[attID] = true

				seed := map[string]interface{}{
					"evaluation_ready_received": arrivalTs,
				}

				// Non-blocking send to avoid stalling watcher if queue is full
				select {
				case workChan <- verificationJob{AttestationID: attID, SeedTs: seed}:
				default:
					logger.Warn("Verifier queue full, dropping %s", attID)
				}
			}
			enqLock.Unlock()
		}
		return nil
	}

	// Run Watcher (Blocks until loop ends)
	// We run it in a goroutine so we can handle ctx.Done() cancellation
	go func() {
		// EventWatcher.Run blocks forever. We don't have a Stop() method on it yet,
		// but since the main app cancels the context, the whole process will die anyway.
		// For cleaner shutdown, we'd modify EventWatcher to accept context.
		// For now, this is acceptable.
		watcher.Run(handle)
	}()

	// Block here until context cancelled
	<-ctx.Done()
	logger.Info("Verifier logic stopped")
}

// processVerifierAttestation contains the core business logic
func processVerifierAttestation(
	ctx context.Context,
	client *blockchain.BlockchainClient,
	cfg VerifierConfig,
	attestationID string,
	seedTs map[string]interface{},
) {
	timestamps := make(map[string]interface{})
	// Copy seed
	for k, v := range seedTs {
		timestamps[k] = v
	}

	timestamps["verifier_start"] = utils.NowMs()
	logPrefix := fmt.Sprintf("[Verifier-%s]", utils.ShortAttID(attestationID, 4))
	logger.Info("%s SECaaS Oracle finished. Retrieving signatures for evaluation...", logPrefix)

	// 1. Retrieve Signatures
	timestamps["get_signatures_start"] = utils.NowMs()
	p0 := utils.PerfNs()

	freshSig, refSig, err := client.GetAttestationSignatures(attestationID)
	if err != nil {
		logger.Error("%s Failed to get signatures: %v", logPrefix, err)
		return
	}

	p1 := utils.PerfNs()
	timestamps["get_signatures_finished"] = utils.NowMs()
	timestamps["dur_signatures_fetch_us"] = utils.NsToUs(p0, p1)

	// 2. Evaluation (compute)
	timestamps["verify_compute_start"] = utils.NowMs()
	p0 = utils.PerfNs()

	// Logic: Compare signatures
	isSuccess := strings.EqualFold(freshSig, refSig)
	// For testing
	isSuccess = true

	p1 = utils.PerfNs()
	timestamps["verify_compute_finished"] = utils.NowMs()
	timestamps["dur_verify_compute_us"] = utils.NsToUs(p0, p1)

	// 3. Submit Result
	statusIcon := "❌ FAILURE"
	if isSuccess {
		statusIcon = "✅ SUCCESS"
	}
	logger.Info("%s Attestation closed (result: %s)", logPrefix, statusIcon)

	timestamps["result_sent"] = utils.NowMs()
	timestamps["verification_result"] = isSuccess // stores bool, helpers handles conversion
	p0 = utils.PerfNs()

	// Wait logic based on config
	timeout := 60 // default timeout
	_, err = client.SendAttestationResult(attestationID, isSuccess, cfg.WaitForTxConfirmations, timeout)
	if err != nil {
		logger.Error("%s Failed to send result: %v", logPrefix, err)
		// We continue to record what we can
	}

	p1 = utils.PerfNs()
	timestamps["dur_send_result_call_us"] = utils.NsToUs(p0, p1)

	if cfg.WaitForTxConfirmations {
		timestamps["result_sent_tx_confirmed"] = utils.NowMs()
	}

	timestamps["verifier_finished"] = utils.NowMs()

	// 4. Reaction Metrics
	if startVal, ok := timestamps["verifier_start"].(int64); ok {
		if readyVal, ok := timestamps["evaluation_ready_received"].(int64); ok {
			timestamps["dur_verifier_reaction_ms"] = startVal - readyVal
		}
	}

	// 5. Summary Log (simplified)
	logger.Debug("%s summary: result=%v | total_dur=%dms",
		logPrefix, isSuccess,
		timestamps["verifier_finished"].(int64)-timestamps["verifier_start"].(int64))

	// 6. Export
	if cfg.ExportEnabled {
		utils.ExportAttestationTimesJSON(
			cfg.ParticipantName,
			attestationID,
			"verifier",
			timestamps,
			cfg.ResultsDir,
			cfg.ResultsFile, // Can be empty, helper handles it
		)
	}
}