package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"app/internal/blockchain"
	"app/internal/config"
)

type TxSample struct {
	Function         string  `json:"function"`
	Iteration        int     `json:"iteration"`
	TxHash           string  `json:"tx_hash,omitempty"`
	Success          bool    `json:"success"`
	Error            string  `json:"error,omitempty"`
	LatencyMs        float64 `json:"latency_ms"`
	GasUsed          uint64  `json:"gas_used,omitempty"`
	BlockNumber      uint64  `json:"block_number,omitempty"`
	BlockTimestamp   uint64  `json:"block_timestamp,omitempty"`
	BlockSize        uint64  `json:"block_size,omitempty"`
	TransactionCount uint64  `json:"transaction_count,omitempty"`
}

type FunctionSummary struct {
	Function             string  `json:"function"`
	Total                int     `json:"total"`
	Success              int     `json:"success"`
	Failed               int     `json:"failed"`
	SuccessRate          float64 `json:"success_rate"`
	LatencyMinMs         float64 `json:"latency_min_ms"`
	LatencyMaxMs         float64 `json:"latency_max_ms"`
	LatencyMeanMs        float64 `json:"latency_mean_ms"`
	LatencyStdMs         float64 `json:"latency_std_ms"`
	LatencyP25Ms         float64 `json:"latency_p25_ms"`
	LatencyP50Ms         float64 `json:"latency_p50_ms"`
	LatencyP75Ms         float64 `json:"latency_p75_ms"`
	LatencyP95Ms         float64 `json:"latency_p95_ms"`
	GasUsedTotal         uint64  `json:"gas_used_total"`
	GasUsedMean          float64 `json:"gas_used_mean"`
	GasUsedStd           float64 `json:"gas_used_std"`
	GasUsedMin           uint64  `json:"gas_used_min"`
	GasUsedMax           uint64  `json:"gas_used_max"`
	BlockSizeMean        float64 `json:"block_size_mean"`
	BlockSizeMin         uint64  `json:"block_size_min"`
	BlockSizeMax         uint64  `json:"block_size_max"`
	TxCountMean          float64 `json:"tx_count_mean"`
	TxCountMin           uint64  `json:"tx_count_min"`
	TxCountMax           uint64  `json:"tx_count_max"`
	FirstErrorSampleText string  `json:"first_error_sample_text,omitempty"`
}

type BenchReport struct {
	GeneratedAt   string            `json:"generated_at"`
	ConfigPath    string            `json:"config_path"`
	RPCURL        string            `json:"rpc_url"`
	Contract      string            `json:"contract_address"`
	BenchAccount  string            `json:"bench_account"`
	RunsPerMethod int               `json:"runs_per_method"`
	Functions     []string          `json:"functions"`
	Samples       []TxSample        `json:"samples"`
	Summary       []FunctionSummary `json:"summary"`
}

type TxRunner func(iter int) TxSample

func main() {
	defaultFunctions := "RegisterAgent,SendEvidence,SendRefSignature,CloseAttestationProcess"

	var (
		configPath = flag.String("config", os.Getenv("CONFIG_PATH"), "Path to robot/secaas config JSON")
		outDir = flag.String("out", "./benchmarks/output", "Output directory for benchmark report")
		functionsCSV = flag.String("functions", defaultFunctions, "Comma-separated smart contract function names to benchmark")
		runs = flag.Int("runs", 5, "Sequential transactions per function")
		timeoutSec = flag.Int("timeout", 60, "Confirmation timeout in seconds")
		agentName = flag.String("agent-name", "txbench-agent", "Agent name used for RegisterAgent setup")
	)
	flag.Parse()

	if *configPath == "" {
		exitf("missing -config flag (or CONFIG_PATH env var)")
	}
	if *runs <= 0 {
		exitf("-runs must be > 0")
	}

	agentCfg, err := config.LoadAgentConfig(*configPath)
	if err != nil {
		exitf("failed to load config: %v", err)
	}

	bcClient, err := blockchain.NewBlockchainClient(
		agentCfg.EthAddress,
		agentCfg.PrivateKey,
		agentCfg.EthNodeURL,
		agentCfg.ContractAddress,
		agentCfg.ContractABI,
	)
	if err != nil {
		exitf("failed to init blockchain client: %v", err)
	}
	defer bcClient.Close()

	if err := ensureRegistered(bcClient, *agentName, *timeoutSec); err != nil {
		exitf("failed to ensure benchmark account is registered: %v", err)
	}

	requestedFunctions := parseFunctions(*functionsCSV)
	runners := map[string]TxRunner{
		"RegisterAgent": func(iter int) TxSample {
			return runTx("RegisterAgent", iter, func() (string, error) {
				return bcClient.RegisterAgent(fmt.Sprintf("%s-%d", *agentName, iter), true, *timeoutSec)
			}, bcClient)
		},
		"SendEvidence": func(iter int) TxSample {
			if err := ensureRegistered(bcClient, *agentName, *timeoutSec); err != nil {
				return failedSample("SendEvidence", iter, err)
			}
			attID := makeAttID("ev", iter)
			sig := makeBytes32Hex("evidence", iter)
			return runTx("SendEvidence", iter, func() (string, error) {
				return bcClient.SendEvidence(attID, sig, 1, true, *timeoutSec)
			}, bcClient)
		},
		"SendRefSignature": func(iter int) TxSample {
			if err := ensureRegistered(bcClient, *agentName, *timeoutSec); err != nil {
				return failedSample("SendRefSignature", iter, err)
			}
			attID := makeAttID("ref", iter)
			if _, err := bcClient.SendEvidence(attID, makeBytes32Hex("seed-evidence", iter), 1, true, *timeoutSec); err != nil {
				return failedSample("SendRefSignature", iter, fmt.Errorf("setup SendEvidence failed: %w", err))
			}
			return runTx("SendRefSignature", iter, func() (string, error) {
				return bcClient.SendRefSignature(attID, makeBytes32Hex("ref", iter), true, *timeoutSec)
			}, bcClient)
		},
		"CloseAttestationProcess": func(iter int) TxSample {
			if err := ensureRegistered(bcClient, *agentName, *timeoutSec); err != nil {
				return failedSample("CloseAttestationProcess", iter, err)
			}
			attID := makeAttID("close", iter)
			if _, err := bcClient.SendEvidence(attID, makeBytes32Hex("close-evidence", iter), 1, true, *timeoutSec); err != nil {
				return failedSample("CloseAttestationProcess", iter, fmt.Errorf("setup SendEvidence failed: %w", err))
			}
			if _, err := bcClient.SendRefSignature(attID, makeBytes32Hex("close-ref", iter), true, *timeoutSec); err != nil {
				return failedSample("CloseAttestationProcess", iter, fmt.Errorf("setup SendRefSignature failed: %w", err))
			}
			isVerifier, err := bcClient.IsVerifier(attID)
			if err != nil {
				return failedSample("CloseAttestationProcess", iter, fmt.Errorf("setup IsVerifier check failed: %w", err))
			}
			if !isVerifier {
				return failedSample("CloseAttestationProcess", iter, fmt.Errorf("bench account is not verifier for attestation %s", attID))
			}
			return runTx("CloseAttestationProcess", iter, func() (string, error) {
				return bcClient.SendAttestationResult(attID, true, true, *timeoutSec)
			}, bcClient)
		},
	}

	samples := make([]TxSample, 0, len(requestedFunctions)*(*runs))
	for _, functionName := range requestedFunctions {
		runner, ok := runners[functionName]
		if !ok {
			samples = append(samples, failedSample(functionName, 0, fmt.Errorf("function is not supported by benchmark runner")))
			continue
		}
		fmt.Printf("\n=== Benchmarking %s (%d sequential tx) ===\n", functionName, *runs)
		for i := 1; i <= *runs; i++ {
			sample := runner(i)
			samples = append(samples, sample)
			if sample.Success {
				fmt.Printf("[%s %d/%d] ok latency=%.2fms gas=%d tx=%s\n", functionName, i, *runs, sample.LatencyMs, sample.GasUsed, sample.TxHash)
			} else {
				fmt.Printf("[%s %d/%d] fail error=%s\n", functionName, i, *runs, sample.Error)
			}
		}
	}

	summary := summarize(samples)
	report := BenchReport{
		GeneratedAt:   time.Now().UTC().Format(time.RFC3339),
		ConfigPath:    *configPath,
		RPCURL:        agentCfg.EthNodeURL,
		Contract:      agentCfg.ContractAddress,
		BenchAccount:  agentCfg.EthAddress,
		RunsPerMethod: *runs,
		Functions:     requestedFunctions,
		Samples:       samples,
		Summary:       summary,
	}

	if err := os.MkdirAll(*outDir, 0o755); err != nil {
		exitf("failed to create output dir: %v", err)
	}
	outFile := filepath.Join(*outDir, fmt.Sprintf("txbench_%s.json", time.Now().Format("20060102_150405")))
	bytes, err := json.MarshalIndent(report, "", "  ")
	if err != nil {
		exitf("failed to serialize report: %v", err)
	}
	if err := os.WriteFile(outFile, bytes, 0o644); err != nil {
		exitf("failed to write report file: %v", err)
	}

	fmt.Printf("\nReport written: %s\n", outFile)
}

func runTx(functionName string, iter int, sender func() (string, error), bc *blockchain.BlockchainClient) TxSample {
	start := time.Now()
	txHash, err := sender()
	latencyMs := float64(time.Since(start).Milliseconds())

	if err != nil {
		return TxSample{
			Function:  functionName,
			Iteration: iter,
			Success:   false,
			Error:     err.Error(),
			LatencyMs: latencyMs,
		}
	}

	receipt, err := bc.GetTransactionReceipt(txHash)
	if err != nil {
		return TxSample{
			Function:  functionName,
			Iteration: iter,
			TxHash:    txHash,
			Success:   false,
			Error:     fmt.Sprintf("tx mined but receipt read failed: %v", err),
			LatencyMs: latencyMs,
		}
	}

	return TxSample{
		Function:         functionName,
		Iteration:        iter,
		TxHash:           txHash,
		Success:          receipt.Status == 1,
		LatencyMs:        latencyMs,
		GasUsed:          receipt.GasUsed,
		BlockNumber:      receipt.BlockNumber,
		BlockTimestamp:   receipt.BlockTimestamp,
		BlockSize:        receipt.BlockSize,
		TransactionCount: receipt.TransactionCount,
	}
}

func summarize(samples []TxSample) []FunctionSummary {
	groups := make(map[string][]TxSample)
	for _, s := range samples {
		groups[s.Function] = append(groups[s.Function], s)
	}

	keys := make([]string, 0, len(groups))
	for k := range groups {
		keys = append(keys, k)
	}
	sort.Strings(keys)

	summaries := make([]FunctionSummary, 0, len(keys))
	for _, fn := range keys {
		group := groups[fn]
		latencies := make([]float64, 0, len(group))
		gasValues := make([]uint64, 0, len(group))
		blockSizes := make([]uint64, 0, len(group))
		txCounts := make([]uint64, 0, len(group))
		total := len(group)
		success := 0
		firstErr := ""
		var gasTotal uint64

		for _, sample := range group {
			latencies = append(latencies, sample.LatencyMs)
			if sample.Success {
				success++
				gasValues = append(gasValues, sample.GasUsed)
				gasTotal += sample.GasUsed
				blockSizes = append(blockSizes, sample.BlockSize)
				txCounts = append(txCounts, sample.TransactionCount)
			} else if firstErr == "" {
				firstErr = sample.Error
			}
		}
		sort.Float64s(latencies)
		sort.Slice(gasValues, func(i, j int) bool { return gasValues[i] < gasValues[j] })
		sort.Slice(blockSizes, func(i, j int) bool { return blockSizes[i] < blockSizes[j] })
		sort.Slice(txCounts, func(i, j int) bool { return txCounts[i] < txCounts[j] })

		s := FunctionSummary{
			Function:             fn,
			Total:                total,
			Success:              success,
			Failed:               total - success,
			SuccessRate:          safeRate(success, total),
			LatencyMinMs:         minF(latencies),
			LatencyMaxMs:         maxF(latencies),
			LatencyMeanMs:        meanF(latencies),
			LatencyStdMs:         stdF(latencies),
			LatencyP25Ms:         percentile(latencies, 25),
			LatencyP50Ms:         percentile(latencies, 50),
			LatencyP75Ms:         percentile(latencies, 75),
			LatencyP95Ms:         percentile(latencies, 95),
			GasUsedTotal:         gasTotal,
			GasUsedMean:          meanU(gasValues),
			GasUsedStd:           stdU(gasValues),
			GasUsedMin:           minU(gasValues),
			GasUsedMax:           maxU(gasValues),
			BlockSizeMean:        meanU(blockSizes),
			BlockSizeMin:         minU(blockSizes),
			BlockSizeMax:         maxU(blockSizes),
			TxCountMean:          meanU(txCounts),
			TxCountMin:           minU(txCounts),
			TxCountMax:           maxU(txCounts),
			FirstErrorSampleText: firstErr,
		}
		summaries = append(summaries, s)
	}
	return summaries
}

func ensureRegistered(bc *blockchain.BlockchainClient, name string, timeoutSec int) error {
	isReg, err := bc.IsRegistered()
	if err != nil {
		return err
	}
	if isReg {
		return nil
	}
	_, err = bc.RegisterAgent(name, true, timeoutSec)
	return err
}

func parseFunctions(csv string) []string {
	parts := strings.Split(csv, ",")
	out := make([]string, 0, len(parts))
	for _, p := range parts {
		trimmed := strings.TrimSpace(p)
		if trimmed == "" {
			continue
		}
		out = append(out, trimmed)
	}
	return out
}

func makeAttID(tag string, iter int) string {
	seed := fmt.Sprintf("%s-%d-%d", tag, iter, time.Now().UnixNano())
	hash := sha256.Sum256([]byte(seed))
	return fmt.Sprintf("b%s", hex.EncodeToString(hash[:])[:24])
}

func makeBytes32Hex(tag string, iter int) string {
	seed := fmt.Sprintf("%s-%d-%d", tag, iter, time.Now().UnixNano())
	hash := sha256.Sum256([]byte(seed))
	return "0x" + hex.EncodeToString(hash[:])
}

func failedSample(function string, iter int, err error) TxSample {
	return TxSample{
		Function:  function,
		Iteration: iter,
		Success:   false,
		Error:     err.Error(),
	}
}

func safeRate(success, total int) float64 {
	if total == 0 {
		return 0
	}
	return (float64(success) / float64(total)) * 100.0
}

func percentile(values []float64, p float64) float64 {
	if len(values) == 0 {
		return 0
	}
	if len(values) == 1 {
		return values[0]
	}
	if p <= 0 {
		return values[0]
	}
	if p >= 100 {
		return values[len(values)-1]
	}

	idx := (p / 100.0) * float64(len(values)-1)
	lower := int(math.Floor(idx))
	upper := int(math.Ceil(idx))
	if lower == upper {
		return values[lower]
	}
	weight := idx - float64(lower)
	return values[lower]*(1-weight) + values[upper]*weight
}

func meanF(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	total := 0.0
	for _, v := range values {
		total += v
	}
	return total / float64(len(values))
}

func minF(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	return values[0]
}

func maxF(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	return values[len(values)-1]
}

func stdF(values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	mean := meanF(values)
	var sumSq float64
	for _, v := range values {
		diff := v - mean
		sumSq += diff * diff
	}
	variance := sumSq / float64(len(values))
	return math.Sqrt(variance)
}

func meanU(values []uint64) float64 {
	if len(values) == 0 {
		return 0
	}
	var total uint64
	for _, v := range values {
		total += v
	}
	return float64(total) / float64(len(values))
}

func stdU(values []uint64) float64 {
	if len(values) == 0 {
		return 0
	}
	mean := meanU(values)
	var sumSq float64
	for _, v := range values {
		diff := float64(v) - mean
		sumSq += diff * diff
	}
	variance := sumSq / float64(len(values))
	return math.Sqrt(variance)
}

func minU(values []uint64) uint64 {
	if len(values) == 0 {
		return 0
	}
	return values[0]
}

func maxU(values []uint64) uint64 {
	if len(values) == 0 {
		return 0
	}
	return values[len(values)-1]
}

func exitf(format string, args ...interface{}) {
	fmt.Fprintf(os.Stderr, format+"\n", args...)
	os.Exit(1)
}
