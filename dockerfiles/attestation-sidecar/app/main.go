package main

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"sync"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"

	"app/internal/attestation"
	"app/internal/blockchain"
	"app/internal/compute"
	"app/internal/config"
	"app/internal/logger"
	"app/internal/results"
)

// StatusResponse represents the JSON structure returned by /status
type StatusResponse struct {
	Status     string           `json:"status"` // "idle", "running", or "finished"
	Rounds     int              `json:"rounds"` // per run, 0 = until stopped
	LastResult string           `json:"last_result,omitempty"`
	Counters   map[string]int64 `json:"counters"` // since the sidecar started
	// websocket: new blocks notified by the node; polling: checked on a timer
	BlockNotifications string `json:"block_notifications"`
}

// lastResult tracks the last attestation result for /status
var lastResult string

// -----------------------------------------------------------------------------
// App State
// -----------------------------------------------------------------------------

type AppState struct {
	Config           *config.AppConfig
	BlockchainClient *blockchain.BlockchainClient

	// Runtime Control
	mu                 sync.Mutex
	measuring          bool
	cancelFunc         context.CancelFunc
	workersWg          sync.WaitGroup
	activeAttestations *sync.Map
}

// -----------------------------------------------------------------------------
// Core Logic
// -----------------------------------------------------------------------------

func startAttestation(state *AppState) {
	state.mu.Lock()
	defer state.mu.Unlock()

	// Reset lastResult at the start (protected by mutex)
	lastResult = ""

	if state.measuring {
		return
	}

	var salt *[32]byte
	if state.Config.SaltMeasurement {
		s, err := compute.MeasurementSalt(state.Config.Agent.PrivateKey)
		if err != nil {
			logger.Error("Failed to derive the measurement salt: %v", err)
			return
		}
		salt = &s
	}

	ctx, cancel := context.WithCancel(context.Background())
	state.cancelFunc = cancel
	state.measuring = true

	// 1. Map Config -> Prover Config
	proverCfg := attestation.ProverConfig{
		Salt:                      salt,
		CmdName:                   state.Config.Agent.CmdName,
		TextSectionSize:           state.Config.Agent.TextSectionSize,
		Offset:                    state.Config.Agent.Offset,
		TextSectionPrefix:         state.Config.Agent.TextSectionPrefix,
		IterQThreshold:            state.Config.IterQThreshold,
		AgentName:                 state.Config.Agent.Name,
		Rounds:                    state.Config.Rounds,
		StartOffset:               time.Duration(state.Config.StartOffsetMs) * time.Millisecond,
		WaitForVerificationResult: state.Config.WaitForVerificationResult,
		SelfIntegrity: attestation.SelfIntegrityInfo{
			Enabled:           state.Config.SelfIntegrity.Enabled,
			CmdName:           state.Config.SelfIntegrity.CmdName,
			TextSectionSize:   state.Config.SelfIntegrity.TextSectionSize,
			TextSectionOffset: state.Config.SelfIntegrity.TextSectionOffset,
			TextSectionPrefix: state.Config.SelfIntegrity.TextSectionPrefix,
			ExpectedSHA256:    state.Config.SelfIntegrity.ExpectedSHA256,
		},
	}

	state.workersWg.Add(1)
	go func() {
		attestation.RunProverLogicContinuousMode(
			ctx.Done(),
			state.BlockchainClient,
			proverCfg,
			state.activeAttestations,
			&state.workersWg,
		)
		// With a round limit the prover finishes on its own, once its last
		// round is closed. The verifier keeps running: other robots may still
		// draw this one in their jury, until /stop.
		if state.Config.Rounds > 0 {
			deadline := time.Now().Add(120 * time.Second)
			for time.Now().Before(deadline) {
				count := 0
				state.activeAttestations.Range(func(_, _ interface{}) bool {
					count++
					return true
				})
				if count == 0 {
					break
				}
				time.Sleep(50 * time.Millisecond)
			}

			state.mu.Lock()
			defer state.mu.Unlock()
			lastResult = "success"
			state.measuring = false
		}
	}()

	// 2. Map Config -> Verifier Config
	verifierCfg := attestation.VerifierConfig{
		ParticipantName:        state.Config.Agent.Name,
		WaitForTxConfirmations: state.Config.VerifierWaitTx,
		EventCheckpointDir:     state.Config.EventCheckpointDir,
		EventConfirmations:     state.Config.EventConfirmations,
		EventBatchSize:         state.Config.EventBatchSize,
		EventLookback:          state.Config.EventLookback,
		ForceVerdict:           state.Config.ForceVerdict,
	}

	state.workersWg.Add(1)
	go func() {
		attestation.RunVerifierLogicSequential(
			ctx,
			state.BlockchainClient,
			verifierCfg,
			&state.workersWg,
		)
	}()
}

func stopAttestation(state *AppState) {
	state.mu.Lock()
	defer state.mu.Unlock()

	// Set lastResult to "stopped" if not already set (protected by mutex)
	if lastResult == "" {
		lastResult = "stopped"
	}

	// Always cancel the context so background goroutines (e.g. RunProverAndCleanup
	// goroutines that outlive a prover loop with a round limit) are signalled to
	// exit immediately rather than running until their 60 s internal timeout.
	if state.cancelFunc != nil {
		state.cancelFunc()
	}

	// With a round limit the prover goroutine may have set measuring=false
	// already: the context above still stops the verifier.
	if !state.measuring {
		return
	}

	done := make(chan struct{})
	go func() {
		state.workersWg.Wait()
		close(done)
	}()

	select {
	case <-done:
		// Clean join
	case <-time.After(5 * time.Second):
		logger.Warn("Timeout waiting for workers to join")
	}

	state.measuring = false
}

// -----------------------------------------------------------------------------
// Main Entry Point
// -----------------------------------------------------------------------------

func main() {
	// 1. Load Centralized Config
	cfg, err := config.Load()
	if err != nil {
		logger.Error("Failed to load configuration: %v", err)
		os.Exit(1)
	}

	// Validate IterQ threshold: must match the contract's MAX_ITER_COUNT
	// otherwise Prove reverts. K=0 is meaningless; K=1 is single-shot.
	if cfg.IterQThreshold < 1 || cfg.IterQThreshold > 10000 {
		logger.Error("ITERQ_THRESHOLD=%d is out of range [1, 10000]", cfg.IterQThreshold)
		os.Exit(1)
	}
	logger.Info("IterQ threshold (K) = %d", cfg.IterQThreshold)
	if cfg.ForceVerdict != nil {
		logger.Warn("FORCE_VERDICT is set: this agent votes %v on every cycle without comparing (experiment only)", *cfg.ForceVerdict)
	}

	if err := attestation.CheckIntervalMode(os.Getenv("ATTESTATION_INTERVAL_MODE")); err != nil {
		logger.Error("%v", err)
		os.Exit(1)
	}
	logger.Info("Measurement delay mode = %s", os.Getenv("ATTESTATION_INTERVAL_MODE"))

	// ---------------------------------------------------------
	// 1.5 Compute Self-Integrity Info (Auto-Detection)
	// ---------------------------------------------------------
	if cfg.SelfIntegrityEnabled {
		logger.Info("Computing self-integrity info from /proc/self/exe...")

		selfInfo, err := compute.ComputeSelfIntegrityInfo(32) // 32 byte prefix
		if err != nil {
			logger.Warn("Failed to compute self-integrity info: %v", err)
			logger.Warn("Self-integrity check will be DISABLED.")
			cfg.SelfIntegrity.Enabled = false
		} else {
			selfPid := os.Getpid()
			textOffset, err := compute.FindTextSectionOffsetInMemory(selfPid, selfInfo.TextSectionPrefix)
			if err != nil {
				logger.Warn("Failed to find .text offset in memory: %v", err)
				logger.Warn("Self-integrity check will be DISABLED.")
				cfg.SelfIntegrity.Enabled = false
			} else {
				cfg.SelfIntegrity.Enabled = true
				cfg.SelfIntegrity.CmdName = "attestation-si"
				cfg.SelfIntegrity.TextSectionSize = selfInfo.Size
				cfg.SelfIntegrity.TextSectionOffset = textOffset
				cfg.SelfIntegrity.TextSectionPrefix = selfInfo.TextSectionPrefix
				cfg.SelfIntegrity.ExpectedSHA256 = selfInfo.SHA256

				logger.Info("Self-integrity ENABLED: size=%d, offset=%d, SHA256=%s...",
					cfg.SelfIntegrity.TextSectionSize,
					cfg.SelfIntegrity.TextSectionOffset,
					cfg.SelfIntegrity.ExpectedSHA256[:16])
			}
		}
	} else {
		logger.Info("Self-integrity check explicitly disabled via config.")
		cfg.SelfIntegrity.Enabled = false
	}
	// ---------------------------------------------------------

	// 2. Initialize Blockchain Client
	bcClient, err := blockchain.NewBlockchainClient(
		cfg.Agent.EthAddress,
		cfg.Agent.PrivateKey,
		cfg.Agent.EthNodeURL,
		cfg.Agent.ContractAddress,
		cfg.Agent.ContractABI,
	)
	if err != nil {
		logger.Error("Failed to init blockchain client: %v", err)
		os.Exit(1)
	}
	defer bcClient.Close()
	// New blocks are notified over WebSocket; without one, polled.
	bcClient.FollowHeads(cfg.Agent.EthWSURL, time.Duration(cfg.EventPollInterval*float64(time.Second)))

	// 4. Initialize State
	appState := &AppState{
		Config:             cfg,
		BlockchainClient:   bcClient,
		activeAttestations: &sync.Map{},
	}

	// 5. Register on chain if needed, then optionally start
	isReg, err := bcClient.IsRegistered()
	if err != nil {
		logger.Error("Failed to check registration: %v", err)
	} else if !isReg {
		logger.Info("Registering agent '%s' (%s) ...", cfg.Agent.Name, cfg.Agent.EthAddress)
		tx, err := bcClient.RegisterAgent(cfg.Agent.Name, true, 30)
		if err != nil {
			logger.Error("Registration failed: %v", err)
			os.Exit(1)
		}
		logger.Info("%s registered (Tx: %s)", cfg.Agent.Name, tx)
	}

	if cfg.AutoStart {
		startAttestation(appState)
	}

	logger.Info(logger.GreenText("Startup completed for participant '%s'"), cfg.Agent.Name)

	// 6. API Server (Gin)
	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Recovery())

	r.GET("/", func(c *gin.Context) {
		c.JSON(200, gin.H{"status": "running", "agent": cfg.Agent.Name})
	})

	r.GET("/status", func(c *gin.Context) {
		appState.mu.Lock()
		defer appState.mu.Unlock()
		resp := StatusResponse{
			Rounds:             cfg.Rounds,
			LastResult:         lastResult,
			Counters:           attestation.CountersSnapshot(),
			BlockNotifications: appState.BlockchainClient.HeadsMode(),
		}
		if appState.measuring {
			resp.Status = "running"
		} else if cfg.Rounds > 0 && lastResult == "success" {
			resp.Status = "finished"
		} else {
			resp.Status = "idle"
		}
		c.JSON(200, resp)
	})

	r.GET("/digest", func(c *gin.Context) {
		si := compute.SelfIntegrityParams{
			Enabled:           cfg.SelfIntegrity.Enabled,
			CmdName:           cfg.SelfIntegrity.CmdName,
			TextSectionSize:   cfg.SelfIntegrity.TextSectionSize,
			TextSectionOffset: cfg.SelfIntegrity.TextSectionOffset,
		}
		d, err := compute.ComputeAttestationDigest(
			cfg.Agent.CmdName,
			cfg.Agent.TextSectionSize,
			cfg.Agent.Offset,
			cfg.Agent.TextSectionPrefix,
			si,
		)
		if err != nil {
			c.JSON(500, gin.H{"error": err.Error()})
			return
		}
		c.JSON(200, d)
	})

	r.POST("/start", func(c *gin.Context) {
		startAttestation(appState)
		c.JSON(200, gin.H{"message": "Attestation started."})
	})

	r.POST("/stop", func(c *gin.Context) {
		stopAttestation(appState)
		c.JSON(200, gin.H{"message": "Attestation stopped."})
	})

	// GET /results: the timing rows recorded since the last DELETE /results.
	r.GET("/results", func(c *gin.Context) {
		prover, verifier := results.Snapshot()
		c.JSON(200, gin.H{"prover": prover, "verifier": verifier})
	})

	r.DELETE("/results", func(c *gin.Context) {
		results.Clear()
		c.JSON(200, gin.H{"message": "Results cleared."})
	})

	// POST /config {"rounds": N, "start_offset_ms": N, "attestation_interval_ms": N,
	// "attestation_interval_mode": "fixed"|"exp", "iterq_threshold": K}
	// Partial update, refused while attestation runs. Applies from the next /start
	// (the interval and its mode from the next wait).
	r.POST("/config", func(c *gin.Context) {
		var body struct {
			Rounds                  *int    `json:"rounds"`
			StartOffsetMs           *int    `json:"start_offset_ms"`
			AttestationIntervalMs   *int    `json:"attestation_interval_ms"`
			AttestationIntervalMode *string `json:"attestation_interval_mode"`
			IterQThreshold          *int    `json:"iterq_threshold"`
		}
		if err := c.ShouldBindJSON(&body); err != nil {
			c.JSON(400, gin.H{"error": "invalid body"})
			return
		}
		if body.Rounds == nil && body.StartOffsetMs == nil && body.AttestationIntervalMs == nil && body.AttestationIntervalMode == nil && body.IterQThreshold == nil {
			c.JSON(400, gin.H{"error": "no recognized fields: use rounds, start_offset_ms, attestation_interval_ms, attestation_interval_mode, iterq_threshold"})
			return
		}

		appState.mu.Lock()
		defer appState.mu.Unlock()

		if appState.measuring {
			c.JSON(409, gin.H{"error": "cannot change config while attestation is running"})
			return
		}

		applied := gin.H{}

		if body.Rounds != nil {
			if *body.Rounds < 0 {
				c.JSON(400, gin.H{"error": "rounds must be >= 0"})
				return
			}
			appState.Config.Rounds = *body.Rounds
			lastResult = ""
			applied["rounds"] = *body.Rounds
			logger.Info("Rounds per run set to: %d", *body.Rounds)
		}

		if body.StartOffsetMs != nil {
			if *body.StartOffsetMs < 0 {
				c.JSON(400, gin.H{"error": "start_offset_ms must be >= 0"})
				return
			}
			appState.Config.StartOffsetMs = *body.StartOffsetMs
			applied["start_offset_ms"] = *body.StartOffsetMs
			logger.Info("Prover start offset set to: %dms", *body.StartOffsetMs)
		}

		if body.AttestationIntervalMs != nil {
			ms := *body.AttestationIntervalMs
			if ms < 0 {
				ms = 0
			}
			os.Setenv("ATTESTATION_INTERVAL_MS", strconv.Itoa(ms))
			applied["attestation_interval_ms"] = ms
			logger.Info("Attestation interval set to: %dms", ms)
		}

		if body.AttestationIntervalMode != nil {
			if err := attestation.CheckIntervalMode(*body.AttestationIntervalMode); err != nil {
				c.JSON(400, gin.H{"error": err.Error()})
				return
			}
			os.Setenv("ATTESTATION_INTERVAL_MODE", *body.AttestationIntervalMode)
			applied["attestation_interval_mode"] = *body.AttestationIntervalMode
			logger.Info("Measurement delay mode set to: %s", *body.AttestationIntervalMode)
		}

		if body.IterQThreshold != nil {
			k := *body.IterQThreshold
			if k < 1 || k > 10000 {
				c.JSON(400, gin.H{"error": "iterq_threshold must be in [1, 10000]"})
				return
			}
			appState.Config.IterQThreshold = uint32(k)
			applied["iterq_threshold"] = k
			logger.Info("IterQ threshold (K) set to: %d", k)
		}

		c.JSON(200, gin.H{"applied": applied})
	})

	// 7. Graceful Shutdown
	srvPort := 8000
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)

	go func() {
		if err := r.Run(fmt.Sprintf(":%d", srvPort)); err != nil {
			logger.Error("Server error: %v", err)
		}
	}()

	logger.Info("Server listening on port %d", srvPort)
	<-quit

	logger.Info("Shutting down...")
	stopAttestation(appState)
	logger.Info("Bye.")
}
