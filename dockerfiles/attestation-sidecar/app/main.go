package main

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"

	"app/internal/attestation"
	"app/internal/blockchain"
	"app/internal/compute"
	"app/internal/config"
	"app/internal/logger"
	"app/internal/storage"
	"app/internal/utils"
)

// -----------------------------------------------------------------------------
// App State
// -----------------------------------------------------------------------------

type AppState struct {
	Config           *config.AppConfig
	BlockchainClient *blockchain.BlockchainClient
	Storage          storage.StorageBackend

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

	if state.measuring {
		logger.Info("Agent attestation already running; ignoring start request")
		return
	}

	if state.Config.ExportEnabled {
		utils.EnsureResultsInitialized(state.Config.ResultsDir, state.Config.Agent.Name, state.Config.ResultsFile)
	}

	ctx, cancel := context.WithCancel(context.Background())
	state.cancelFunc = cancel
	state.measuring = true

	logger.Info("Agent attestation loop started")

	// 1. Map Config -> Prover Config
	proverCfg := attestation.ProverConfig{
		CmdName:           state.Config.Agent.CmdName,
		TextSectionSize:   state.Config.Agent.TextSectionSize,
		Offset:            state.Config.Agent.Offset,
		TextSectionPrefix: state.Config.Agent.TextSectionPrefix,
		Threshold:         state.Config.ProverThreshold,
		AgentName:         state.Config.Agent.Name,
		ExportEnabled:     state.Config.ExportEnabled,
		ResultsDir:        state.Config.ResultsDir,
		MemoryStorageFile: state.Config.MemoryStorageFile,
		ResultsFile:       state.Config.ResultsFile,
		OneShot:           state.Config.OneShot,
		// Pass the auto-discovered SelfIntegrity config
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
			state.Storage,
			proverCfg,
			state.activeAttestations,
			&state.workersWg,
		)
	}()

	// 2. Map Config -> Verifier Config
	verifierCfg := attestation.VerifierConfig{
		ParticipantName:        state.Config.Agent.Name,
		ResultsDir:             state.Config.ResultsDir,
		ResultsFile:            state.Config.ResultsFile,
		ExportEnabled:          state.Config.ExportEnabled,
		WaitForTxConfirmations: state.Config.VerifierWaitTx,
		EventCheckpointDir:     state.Config.EventCheckpointDir,
		EventConfirmations:     state.Config.EventConfirmations,
		EventBatchSize:         state.Config.EventBatchSize,
		EventPollInterval:      state.Config.EventPollInterval,
		EventLookback:          state.Config.EventLookback,
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

	if !state.measuring {
		return
	}

	if state.cancelFunc != nil {
		state.cancelFunc()
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

	if state.Config.ExportEnabled {
		utils.MarkExperimentStop(state.Config.ResultsDir, state.Config.Agent.Name, state.Config.ResultsFile)
	}
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

	// ---------------------------------------------------------
	// 1.5 Compute Self-Integrity Info (Auto-Detection)
	// ---------------------------------------------------------
	if cfg.SelfIntegrityEnabled {
		logger.Info("Computing self-integrity info from /proc/self/exe...")
		
		// Use internal/compute to analyze the running binary
		selfInfo, err := compute.ComputeSelfIntegrityInfo(32) // 32 byte prefix
		if err != nil {
			logger.Warn("Failed to compute self-integrity info: %v", err)
			logger.Warn("Self-integrity check will be DISABLED.")
			cfg.SelfIntegrity.Enabled = false
		} else {
			// Find the offset of .text section in memory
			selfPid := os.Getpid()
			textOffset, err := compute.FindTextSectionOffsetInMemory(selfPid, selfInfo.TextSectionPrefix)
			if err != nil {
				logger.Warn("Failed to find .text offset in memory: %v", err)
				logger.Warn("Self-integrity check will be DISABLED.")
				cfg.SelfIntegrity.Enabled = false
			} else {
				// Success! Populate the config struct
				cfg.SelfIntegrity.Enabled = true
				cfg.SelfIntegrity.CmdName = "attestation-si" // Specific requirement
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

	// 3. Initialize Storage
	var store storage.StorageBackend
	if cfg.UseRedis {
		store = storage.NewRedisStorageBackend("localhost", 6379, 0)
	} else {
		store = storage.NewMemoryStorageBackend()
	}

	// 4. Initialize State
	appState := &AppState{
		Config:             cfg,
		BlockchainClient:   bcClient,
		Storage:            store,
		activeAttestations: &sync.Map{},
	}

	// 5. Handle Background Tasks (Registration / Display)
	if cfg.DisplayMode {
		n := utils.GetEnvInt("CHAIN_DISPLAY_N", 10)
		sec := utils.GetEnvInt("CHAIN_DISPLAY_SEC", 30)
		go bcClient.PeriodicAttestationChainDisplay(n, sec)
	} else {
		// Check registration
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

		// Auto-Start loop
		if cfg.AutoStart {
			startAttestation(appState)
		}
	}

	logger.Info(logger.GreenText("Startup completed for participant '%s'"), cfg.Agent.Name)

	// 6. API Server (Gin)
	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Recovery())

	r.GET("/", func(c *gin.Context) {
		c.JSON(200, gin.H{"status": "running", "agent": cfg.Agent.Name})
	})

	r.POST("/start", func(c *gin.Context) {
		startAttestation(appState)
		c.JSON(200, gin.H{"message": "Attestation started."})
	})

	r.POST("/stop", func(c *gin.Context) {
		stopAttestation(appState)
		c.JSON(200, gin.H{"message": "Attestation stopped."})
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
	<-quit // Block until signal

	logger.Info("Shutting down...")
	stopAttestation(appState)
	logger.Info("Bye.")
}