package config

import (
	"encoding/json"
	"fmt"
	"os"

	"app/internal/utils"
)

// ---------------------------------------------------------
// Shared Struct Definitions
// ---------------------------------------------------------

// SelfIntegrityConfig holds settings for the optional self-check.
type SelfIntegrityConfig struct {
	Enabled           bool   `json:"enabled"`
	CmdName           string `json:"cmd_name"`
	TextSectionSize   int    `json:"text_section_size"`
	TextSectionOffset int    `json:"offset"`
	TextSectionPrefix []byte `json:"text_section_prefix"`
	ExpectedSHA256    string `json:"expected_sha256"`
}

// ---------------------------------------------------------
// JSON File Structure
// ---------------------------------------------------------

// AgentConfig represents the raw config.json file content.
type AgentConfig struct {
	Name              string      `json:"name"`
	EthAddress        string      `json:"eth_address"`
	PrivateKey        string      `json:"private_key"`
	EthNodeURL        string      `json:"eth_node_url"`
	EthWSURL          string      `json:"eth_ws_url"` // new-block notifications; empty: polling
	ContractAddress   string      `json:"contract_address"`
	ContractABI       interface{} `json:"contract_abi"`
	CmdName           string      `json:"cmd_name"`
	TextSectionSize   int         `json:"text_section_size"`
	Offset            int         `json:"offset"`
	TextSectionPrefix string      `json:"text_section_prefix"`
}

// ---------------------------------------------------------
// Aggregated App Config
// ---------------------------------------------------------

// AppConfig holds the Agent config plus all Environment overrides
type AppConfig struct {
	// Source Data (JSON)
	Agent AgentConfig

	// Runtime Flags (Env)
	AutoStart                 bool
	Rounds                    int // batches per run, then the prover stops; 0 = until stopped
	StartOffsetMs             int // prover wait after /start, the verifier starts at once
	WaitForVerificationResult bool

	// Attestation Params (Env)
	IterQThreshold       uint32 // K = number of fresh measurements folded into one on-chain attestation
	SelfIntegrityEnabled bool
	SaltMeasurement      bool  // send SHA256(m || salt) instead of m (contract V2)
	ForceVerdict         *bool // experiment only: vote this verdict without comparing, nil = compare

	// Self-integrity info, filled in by main.go
	SelfIntegrity SelfIntegrityConfig

	// Verifier Params (Env)
	VerifierWaitTx     bool
	EventCheckpointDir string
	EventConfirmations uint64
	EventBatchSize     uint64
	EventPollInterval  float64 // seconds between checks while block notifications are down
	EventLookback      uint64
}

// ---------------------------------------------------------
// Loaders
// ---------------------------------------------------------

func LoadAgentConfig(path string) (*AgentConfig, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var cfg AgentConfig
	if err := json.Unmarshal(data, &cfg); err != nil {
		return nil, err
	}
	return &cfg, nil
}

func Load() (*AppConfig, error) {
	configPath := utils.GetEnvStr("CONFIG_PATH", "")
	if configPath == "" {
		return nil, fmt.Errorf("CONFIG_PATH environment variable is required")
	}

	agentCfg, err := LoadAgentConfig(configPath)
	if err != nil {
		return nil, fmt.Errorf("failed to load agent config: %v", err)
	}

	if agentCfg.Name == "" {
		agentCfg.Name = "agent"
	}

	appCfg := &AppConfig{
		Agent:                     *agentCfg,
		AutoStart:                 utils.GetEnvBool("AUTO_START", false),
		Rounds:                    utils.GetEnvInt("ROUNDS", 0),
		WaitForVerificationResult: utils.GetEnvBool("WAIT_FOR_VERIFICATION_RESULT", true),

		IterQThreshold:       uint32(utils.GetEnvInt("ITERQ_THRESHOLD", 1)),
		SelfIntegrityEnabled: utils.GetEnvBool("SELF_INTEGRITY_ENABLED", false),
		SaltMeasurement:      utils.GetEnvBool("SALT_MEASUREMENT", false),

		// Initialize Empty SelfIntegrity (Will be populated in main.go)
		SelfIntegrity: SelfIntegrityConfig{
			Enabled: false,
		},

		VerifierWaitTx:     utils.GetEnvBool("WAIT_FOR_TX_CONFIRMATIONS", false),
		EventCheckpointDir: utils.GetEnvStr("EVENT_CHECKPOINT_DIR", "/checkpoints"),
		EventConfirmations: uint64(utils.GetEnvInt("EVENT_CONFIRMATIONS", 1)),
		EventBatchSize:     uint64(utils.GetEnvInt("EVENT_BATCH_SIZE", 1000)),
		EventPollInterval:  utils.GetEnvFloat("EVENT_POLL_INTERVAL", 1.0),
		EventLookback:      uint64(utils.GetEnvInt("EVENT_LOOKBACK_BLOCKS", 0)),
	}

	switch v := os.Getenv("FORCE_VERDICT"); v {
	case "":
	case "valid", "invalid":
		forced := v == "valid"
		appCfg.ForceVerdict = &forced
	default:
		return nil, fmt.Errorf("FORCE_VERDICT must be valid, invalid or unset, got %q", v)
	}

	return appCfg, nil
}
