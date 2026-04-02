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
	"strconv"
	"strings"
	"sync"
	"time"

	"app/internal/blockchain"
	"app/internal/config"
)

type TxSample struct {
	Account          string  `json:"account"`
	Executor         string  `json:"executor"`
	Function         string  `json:"function"`
	Iteration        int     `json:"iteration"`
	Skipped          bool    `json:"skipped,omitempty"`
	SkipReason       string  `json:"skip_reason,omitempty"`
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

type AccountSummary struct {
	Account          string  `json:"account"`
	Function         string  `json:"function"`
	Total            int     `json:"total"`
	Success          int     `json:"success"`
	Failed           int     `json:"failed"`
	SuccessRate      float64 `json:"success_rate"`
	LatencyMinMs     float64 `json:"latency_min_ms"`
	LatencyMaxMs     float64 `json:"latency_max_ms"`
	LatencyMeanMs    float64 `json:"latency_mean_ms"`
	LatencyStdMs     float64 `json:"latency_std_ms"`
	LatencyP25Ms     float64 `json:"latency_p25_ms"`
	LatencyP50Ms     float64 `json:"latency_p50_ms"`
	LatencyP75Ms     float64 `json:"latency_p75_ms"`
	LatencyP95Ms     float64 `json:"latency_p95_ms"`
	GasUsedTotal     uint64  `json:"gas_used_total"`
	GasUsedMean      float64 `json:"gas_used_mean"`
	GasUsedStd       float64 `json:"gas_used_std"`
	GasUsedMin       uint64  `json:"gas_used_min"`
	GasUsedMax       uint64  `json:"gas_used_max"`
	FirstErrorText   string  `json:"first_error_text,omitempty"`
}

type FunctionSummary struct {
	Function        string  `json:"function"`
	TotalSamples    int     `json:"total_samples"`
	SuccessSamples  int     `json:"success_samples"`
	FailedSamples   int     `json:"failed_samples"`
	SuccessRate     float64 `json:"success_rate"`
	LatencyMinMs    float64 `json:"latency_min_ms"`
	LatencyMaxMs    float64 `json:"latency_max_ms"`
	LatencyMeanMs   float64 `json:"latency_mean_ms"`
	LatencyStdMs    float64 `json:"latency_std_ms"`
	LatencyP25Ms    float64 `json:"latency_p25_ms"`
	LatencyP50Ms    float64 `json:"latency_p50_ms"`
	LatencyP75Ms    float64 `json:"latency_p75_ms"`
	LatencyP95Ms    float64 `json:"latency_p95_ms"`
	GasUsedTotal    uint64  `json:"gas_used_total"`
	GasUsedMean     float64 `json:"gas_used_mean"`
	GasUsedStd      float64 `json:"gas_used_std"`
	GasUsedMin      uint64  `json:"gas_used_min"`
	GasUsedMax      uint64  `json:"gas_used_max"`
	UniqueAccounts  int     `json:"unique_accounts"`
	AvgTxPerAccount float64 `json:"avg_tx_per_account"`
}

type ParallelBenchReport struct {
	GeneratedAt      string             `json:"generated_at"`
	ConfigDir        string             `json:"config_dir"`
	Accounts         []string           `json:"accounts"`
	RunsPerAccount   int                `json:"runs_per_account"`
	Concurrency      int                `json:"concurrency"`
	Functions        []string           `json:"functions"`
	Samples          []TxSample         `json:"samples"`
	AccountSummaries []AccountSummary   `json:"account_summaries"`
	FunctionSummary  []FunctionSummary  `json:"function_summary"`
	ExecutionTimeMs  float64            `json:"execution_time_ms"`
}

type TxRunner func(iter int) TxSample

type SignerContext struct {
	Name   string
	Client *blockchain.BlockchainClient
	Mutex  *sync.Mutex
}

func main() {
	defaultFunctions := "RegisterAgent,SendEvidence,SendRefSignature,CloseAttestationProcess"

	var (
		configDir    = flag.String("config-dir", "./config", "Path to config directory containing robot/secaas JSON files")
		accountsCSV  = flag.String("accounts", "", "Account range or list (e.g., '1-10', '1,5,10,50' or leave empty for all)")
		outDir       = flag.String("out", "./benchmarks/output", "Output directory for benchmark report")
		functionsCSV = flag.String("functions", defaultFunctions, "Comma-separated smart contract function names to benchmark")
		runs         = flag.Int("runs", 5, "Sequential transactions per function per account")
		timeoutSec   = flag.Int("timeout", 60, "Confirmation timeout in seconds")
		concurrency  = flag.Int("concurrency", 10, "Max concurrent accounts")
		warmup       = flag.Bool("warmup", true, "Run 1 successful warm-up attestation per account before benchmark")
		closeVerified = flag.Bool("close-verified", true, "Use verified=true when closing attestations")
	)
	flag.Parse()

	if *concurrency <= 0 {
		exitf("-concurrency must be > 0")
	}
	if *runs <= 0 {
		exitf("-runs must be > 0")
	}

	// Load account configs
	accounts, err := loadAccounts(*configDir, *accountsCSV)
	if err != nil {
		exitf("failed to load accounts: %v", err)
	}
	if len(accounts) == 0 {
		exitf("no accounts loaded")
	}

	requestedFunctions := parseFunctions(*functionsCSV)
	agentNames := make(map[string]string)
	for _, acc := range accounts {
		agentNames[acc] = fmt.Sprintf("txbench-agent-%s", acc)
	}

	signers, err := buildSignerPool(*configDir, accounts)
	if err != nil {
		exitf("failed to initialize signer pool: %v", err)
	}
	defer closeSignerPool(signers)

	fmt.Printf("=== Parallel Smart Contract Benchmark ===\n")
	fmt.Printf("Accounts: %d\n", len(accounts))
	fmt.Printf("Functions: %v\n", requestedFunctions)
	fmt.Printf("Runs per account: %d\n", *runs)
	fmt.Printf("Concurrency: %d\n\n", *concurrency)

	if *warmup && containsFunction(requestedFunctions, "CloseAttestationProcess") {
		fmt.Printf("Running warm-up attestations (verified=%v) to seed verifier rotation...\n", *closeVerified)
		if err := runWarmup(accounts, agentNames, signers, *timeoutSec, *closeVerified); err != nil {
			fmt.Printf("Warm-up warning: %v\n", err)
			fmt.Printf("Continuing benchmark with partial/zero warm-up coverage.\n")
		}
		fmt.Printf("Warm-up complete.\n\n")
	}

	startTime := time.Now()

	// Run benchmarks in parallel
	samples := runParallelBenchmark(
		accounts,
		requestedFunctions,
		*runs,
		*timeoutSec,
		*concurrency,
		agentNames,
		signers,
		*closeVerified,
	)

	executionTimeMs := float64(time.Since(startTime).Milliseconds())

	// Generate summaries
	accountSummaries := summarizeByAccount(samples)
	functionSummaries := summarizeByFunction(samples)

	report := ParallelBenchReport{
		GeneratedAt:      time.Now().UTC().Format(time.RFC3339),
		ConfigDir:        *configDir,
		Accounts:         accounts,
		RunsPerAccount:   *runs,
		Concurrency:      *concurrency,
		Functions:        requestedFunctions,
		Samples:          samples,
		AccountSummaries: accountSummaries,
		FunctionSummary:  functionSummaries,
		ExecutionTimeMs:  executionTimeMs,
	}

	if err := os.MkdirAll(*outDir, 0o755); err != nil {
		exitf("failed to create output dir: %v", err)
	}
	outFile := filepath.Join(*outDir, fmt.Sprintf("txbench-parallel_%s.json", time.Now().Format("20060102_150405")))
	bytes, err := json.MarshalIndent(report, "", "  ")
	if err != nil {
		exitf("failed to serialize report: %v", err)
	}
	if err := os.WriteFile(outFile, bytes, 0o644); err != nil {
		exitf("failed to write report file: %v", err)
	}

	fmt.Printf("\n=== Results ===\n")
	fmt.Printf("Total samples: %d\n", len(samples))
	fmt.Printf("Execution time: %.2f seconds\n", executionTimeMs/1000)
	fmt.Printf("Report written: %s\n", outFile)
}

func loadAccounts(configDir string, accountsCSV string) ([]string, error) {
	// Read directory
	entries, err := os.ReadDir(configDir)
	if err != nil {
		return nil, err
	}

	availableAccounts := make([]string, 0)
	for _, entry := range entries {
		if !entry.IsDir() && strings.HasSuffix(entry.Name(), ".json") {
			name := strings.TrimSuffix(entry.Name(), ".json")
			if strings.HasPrefix(name, "robot") || name == "secaas" {
				availableAccounts = append(availableAccounts, name)
			}
		}
	}
	sort.Strings(availableAccounts)

	// Parse requested accounts
	if accountsCSV == "" {
		result := make([]string, 0, len(availableAccounts))
		for _, acc := range availableAccounts {
			if strings.HasPrefix(acc, "robot") {
				result = append(result, acc)
			}
		}
		return result, nil
	}

	requested := make(map[string]bool)
	parts := strings.Split(accountsCSV, ",")
	for _, part := range parts {
		part = strings.TrimSpace(part)
		if strings.Contains(part, "-") {
			// Range: e.g., "1-10"
			rangeParts := strings.Split(part, "-")
			if len(rangeParts) != 2 {
				return nil, fmt.Errorf("invalid range format: %s", part)
			}
			start, err := strconv.Atoi(strings.TrimSpace(rangeParts[0]))
			if err != nil {
				return nil, fmt.Errorf("invalid range start: %s", rangeParts[0])
			}
			end, err := strconv.Atoi(strings.TrimSpace(rangeParts[1]))
			if err != nil {
				return nil, fmt.Errorf("invalid range end: %s", rangeParts[1])
			}
			for i := start; i <= end; i++ {
				requested[fmt.Sprintf("robot%d", i)] = true
			}
		} else {
			// Single account
			requested[part] = true
		}
	}

	// Filter available accounts
	result := make([]string, 0)
	for _, acc := range availableAccounts {
		if requested[acc] {
			result = append(result, acc)
		}
	}
	return result, nil
}

func buildSignerPool(configDir string, accounts []string) (map[string]*SignerContext, error) {
	names := make([]string, 0, len(accounts)+1)
	names = append(names, accounts...)
	names = append(names, "secaas")

	uniqueNames := make(map[string]bool)
	for _, name := range names {
		uniqueNames[name] = true
	}

	pool := make(map[string]*SignerContext)
	for name := range uniqueNames {
		configPath := filepath.Join(configDir, name+".json")
		agentCfg, err := config.LoadAgentConfig(configPath)
		if err != nil {
			return nil, fmt.Errorf("load config for %s: %w", name, err)
		}

		client, err := blockchain.NewBlockchainClient(
			agentCfg.EthAddress,
			agentCfg.PrivateKey,
			agentCfg.EthNodeURL,
			agentCfg.ContractAddress,
			agentCfg.ContractABI,
		)
		if err != nil {
			return nil, fmt.Errorf("init blockchain client for %s: %w", name, err)
		}

		pool[name] = &SignerContext{
			Name:   name,
			Client: client,
			Mutex:  &sync.Mutex{},
		}
	}

	return pool, nil
}

func closeSignerPool(pool map[string]*SignerContext) {
	for _, signer := range pool {
		signer.Client.Close()
	}
}

func containsFunction(functions []string, name string) bool {
	for _, function := range functions {
		if function == name {
			return true
		}
	}
	return false
}

func runWarmup(accounts []string, agentNames map[string]string, signers map[string]*SignerContext, timeoutSec int, closeVerified bool) error {
	failures := make([]string, 0)
	for index, account := range accounts {
		proverSigner, ok := signers[account]
		if !ok {
			failures = append(failures, fmt.Sprintf("%s: signer missing", account))
			continue
		}
		secaasSigner, ok := signers["secaas"]
		if !ok {
			return fmt.Errorf("warm-up secaas signer missing")
		}

		if err := ensureRegistered(proverSigner.Client, agentNames[account], timeoutSec); err != nil {
			failures = append(failures, fmt.Sprintf("%s: ensure registered failed: %v", account, err))
			continue
		}

		attID := makeAttID("warmup", index+1)

		proverSigner.Mutex.Lock()
		_, err := proverSigner.Client.SendEvidence(attID, makeBytes32Hex("warmup-evidence", index+1), true, timeoutSec)
		proverSigner.Mutex.Unlock()
		if err != nil {
			if regErr := ensureRegistered(proverSigner.Client, agentNames[account], timeoutSec); regErr == nil {
				proverSigner.Mutex.Lock()
				_, err = proverSigner.Client.SendEvidence(attID, makeBytes32Hex("warmup-evidence-retry", index+1), true, timeoutSec)
				proverSigner.Mutex.Unlock()
			}
			if err != nil {
				failures = append(failures, fmt.Sprintf("%s: SendEvidence failed: %v", account, err))
				continue
			}
		}

		secaasSigner.Mutex.Lock()
		_, err = secaasSigner.Client.SendRefSignature(attID, makeBytes32Hex("warmup-ref", index+1), true, timeoutSec)
		secaasSigner.Mutex.Unlock()
		if err != nil {
			failures = append(failures, fmt.Sprintf("%s: SendRefSignature failed: %v", account, err))
			continue
		}

		verifierSigner, err := findVerifierSigner(attID, signers)
		if err != nil {
			failures = append(failures, fmt.Sprintf("%s: verifier selection failed: %v", account, err))
			continue
		}

		verifierSigner.Mutex.Lock()
		_, err = verifierSigner.Client.SendAttestationResult(attID, closeVerified, true, timeoutSec)
		verifierSigner.Mutex.Unlock()
		if err != nil {
			failures = append(failures, fmt.Sprintf("%s: CloseAttestationProcess by %s failed: %v", account, verifierSigner.Name, err))
			continue
		}

		fmt.Printf("[warmup %d/%d] prover=%s verifier=%s\n", index+1, len(accounts), account, verifierSigner.Name)
	}

	if len(failures) > 0 {
		return fmt.Errorf("warm-up had %d failed account(s): %s", len(failures), strings.Join(failures, " | "))
	}

	return nil
}

func findVerifierSigner(attID string, signers map[string]*SignerContext) (*SignerContext, error) {
	names := make([]string, 0, len(signers))
	for name := range signers {
		names = append(names, name)
	}
	sort.Strings(names)

	for _, name := range names {
		signer := signers[name]
		isVerifier, err := signer.Client.IsVerifier(attID)
		if err != nil {
			continue
		}
		if isVerifier {
			return signer, nil
		}
	}

	return nil, fmt.Errorf("no verifier found for attestation %s", attID)
}

func runParallelBenchmark(
	accounts []string,
	functions []string,
	runs int,
	timeoutSec int,
	concurrency int,
	agentNames map[string]string,
	signers map[string]*SignerContext,
	closeVerified bool,
) []TxSample {
	samples := make([]TxSample, 0, len(accounts)*len(functions)*runs)
	samplesMutex := &sync.Mutex{}

	semaphore := make(chan struct{}, concurrency)
	var wg sync.WaitGroup

	for _, accountName := range accounts {
		signer, ok := signers[accountName]
		if !ok {
			fmt.Printf("[%s] ERROR signer not found\n", accountName)
			continue
		}
		wg.Add(1)
		go func(accName string, accountSigner *SignerContext) {
			defer wg.Done()
			semaphore <- struct{}{}
			defer func() { <-semaphore }()

			// Ensure registered
			if err := ensureRegistered(accountSigner.Client, agentNames[accName], timeoutSec); err != nil {
				fmt.Printf("[%s] ERROR ensuring registered: %v\n", accName, err)
				return
			}

			// Build runners
			runners := buildRunners(accName, agentNames[accName], accountSigner, timeoutSec, signers, closeVerified)

			// Run each function
			for _, functionName := range functions {
				runner, ok := runners[functionName]
				if !ok {
					fmt.Printf("[%s] function not supported: %s\n", accName, functionName)
					continue
				}

				for i := 1; i <= runs; i++ {
					sample := runner(i)
					samplesMutex.Lock()
					samples = append(samples, sample)
					samplesMutex.Unlock()
					if sample.Success {
						fmt.Printf("[%s %s %d/%d] ok latency=%.2fms\n", accName, functionName, i, runs, sample.LatencyMs)
					} else {
						fmt.Printf("[%s %s %d/%d] fail error=%s\n", accName, functionName, i, runs, sample.Error)
					}
				}
			}
		}(accountName, signer)
	}

	wg.Wait()
	sort.Slice(samples, func(i, j int) bool {
		if samples[i].Account != samples[j].Account {
			return samples[i].Account < samples[j].Account
		}
		if samples[i].Function != samples[j].Function {
			return samples[i].Function < samples[j].Function
		}
		return samples[i].Iteration < samples[j].Iteration
	})

	return samples
}

func buildRunners(account string, agentName string, accountSigner *SignerContext, timeoutSec int, signers map[string]*SignerContext, closeVerified bool) map[string]func(int) TxSample {
	secaasSigner, ok := signers["secaas"]
	if !ok {
		return map[string]func(int) TxSample{
			"RegisterAgent": func(iter int) TxSample {
				return failedSample(account, "RegisterAgent", iter, fmt.Errorf("secaas signer missing"))
			},
		}
	}

	return map[string]func(int) TxSample{
		"RegisterAgent": func(iter int) TxSample {
			isReg, err := accountSigner.Client.IsRegistered()
			if err != nil {
				return failedSample(account, "RegisterAgent", iter, fmt.Errorf("IsRegistered failed: %w", err))
			}
			if isReg {
				return skippedSample(account, account, "RegisterAgent", iter, "already registered")
			}
			return runTx(account, account, "RegisterAgent", iter, func() (string, error) {
				accountSigner.Mutex.Lock()
				defer accountSigner.Mutex.Unlock()
				return accountSigner.Client.RegisterAgent(fmt.Sprintf("%s-%d", agentName, iter), true, timeoutSec)
			}, accountSigner.Client)
		},
		"SendEvidence": func(iter int) TxSample {
			if err := ensureRegistered(accountSigner.Client, agentName, timeoutSec); err != nil {
				return failedSample(account, "SendEvidence", iter, err)
			}
			attID := makeAttID("ev", iter)
			sig := makeBytes32Hex("evidence", iter)
			return runTx(account, account, "SendEvidence", iter, func() (string, error) {
				accountSigner.Mutex.Lock()
				defer accountSigner.Mutex.Unlock()
				return accountSigner.Client.SendEvidence(attID, sig, true, timeoutSec)
			}, accountSigner.Client)
		},
		"SendRefSignature": func(iter int) TxSample {
			if err := ensureRegistered(accountSigner.Client, agentName, timeoutSec); err != nil {
				return failedSample(account, "SendRefSignature", iter, err)
			}
			attID := makeAttID("ref", iter)
			accountSigner.Mutex.Lock()
			_, err := accountSigner.Client.SendEvidence(attID, makeBytes32Hex("seed-evidence", iter), true, timeoutSec)
			accountSigner.Mutex.Unlock()
			if err != nil {
				return failedSample(account, "SendRefSignature", iter, fmt.Errorf("setup SendEvidence failed: %w", err))
			}
			return runTx(account, "secaas", "SendRefSignature", iter, func() (string, error) {
				secaasSigner.Mutex.Lock()
				defer secaasSigner.Mutex.Unlock()
				return secaasSigner.Client.SendRefSignature(attID, makeBytes32Hex("ref", iter), true, timeoutSec)
			}, secaasSigner.Client)
		},
		"CloseAttestationProcess": func(iter int) TxSample {
			if err := ensureRegistered(accountSigner.Client, agentName, timeoutSec); err != nil {
				return failedSample(account, "CloseAttestationProcess", iter, err)
			}
			attID := makeAttID("close", iter)
			accountSigner.Mutex.Lock()
			_, err := accountSigner.Client.SendEvidence(attID, makeBytes32Hex("close-evidence", iter), true, timeoutSec)
			accountSigner.Mutex.Unlock()
			if err != nil {
				return failedSample(account, "CloseAttestationProcess", iter, fmt.Errorf("setup SendEvidence failed: %w", err))
			}
			secaasSigner.Mutex.Lock()
			_, err = secaasSigner.Client.SendRefSignature(attID, makeBytes32Hex("close-ref", iter), true, timeoutSec)
			secaasSigner.Mutex.Unlock()
			if err != nil {
				return failedSample(account, "CloseAttestationProcess", iter, fmt.Errorf("setup SendRefSignature failed: %w", err))
			}

			verifierSigner, err := findVerifierSigner(attID, signers)
			if err != nil {
				return failedSample(account, "CloseAttestationProcess", iter, fmt.Errorf("failed to find verifier signer: %w", err))
			}
			return runTx(account, verifierSigner.Name, "CloseAttestationProcess", iter, func() (string, error) {
				verifierSigner.Mutex.Lock()
				defer verifierSigner.Mutex.Unlock()
				return verifierSigner.Client.SendAttestationResult(attID, closeVerified, true, timeoutSec)
			}, verifierSigner.Client)
		},
	}
}

func runTx(account string, executor string, functionName string, iter int, sender func() (string, error), bc *blockchain.BlockchainClient) TxSample {
	start := time.Now()
	txHash, err := sender()
	latencyMs := float64(time.Since(start).Milliseconds())

	if err != nil {
		return TxSample{
			Account:   account,
			Executor:  executor,
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
			Account:   account,
			Executor:  executor,
			Function:  functionName,
			Iteration: iter,
			TxHash:    txHash,
			Success:   false,
			Error:     fmt.Sprintf("tx mined but receipt read failed: %v", err),
			LatencyMs: latencyMs,
		}
	}

	return TxSample{
		Account:          account,
		Executor:         executor,
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

func summarizeByAccount(samples []TxSample) []AccountSummary {
	groups := make(map[string]map[string][]TxSample)
	for _, s := range samples {
		if groups[s.Account] == nil {
			groups[s.Account] = make(map[string][]TxSample)
		}
		groups[s.Account][s.Function] = append(groups[s.Account][s.Function], s)
	}

	summaries := make([]AccountSummary, 0)
	accounts := make([]string, 0, len(groups))
	for acc := range groups {
		accounts = append(accounts, acc)
	}
	sort.Strings(accounts)

	for _, acc := range accounts {
		functions := make([]string, 0, len(groups[acc]))
		for fn := range groups[acc] {
			functions = append(functions, fn)
		}
		sort.Strings(functions)

		for _, fn := range functions {
			group := groups[acc][fn]
			latencies := make([]float64, 0, len(group))
			gasValues := make([]uint64, 0, len(group))
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
				} else if firstErr == "" {
					firstErr = sample.Error
				}
			}
			sort.Float64s(latencies)
			sort.Slice(gasValues, func(i, j int) bool { return gasValues[i] < gasValues[j] })

			s := AccountSummary{
				Account:       acc,
				Function:      fn,
				Total:         total,
				Success:       success,
				Failed:        total - success,
				SuccessRate:   safeRate(success, total),
				LatencyMinMs:  minF(latencies),
				LatencyMaxMs:  maxF(latencies),
				LatencyMeanMs: meanF(latencies),
				LatencyStdMs:  stdF(latencies),
				LatencyP25Ms:  percentile(latencies, 25),
				LatencyP50Ms:  percentile(latencies, 50),
				LatencyP75Ms:  percentile(latencies, 75),
				LatencyP95Ms:  percentile(latencies, 95),
				GasUsedTotal:  gasTotal,
				GasUsedMean:   meanU(gasValues),
				GasUsedStd:    stdU(gasValues),
				GasUsedMin:    minU(gasValues),
				GasUsedMax:    maxU(gasValues),
				FirstErrorText: firstErr,
			}
			summaries = append(summaries, s)
		}
	}
	return summaries
}

func summarizeByFunction(samples []TxSample) []FunctionSummary {
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
		accountSet := make(map[string]bool)
		total := len(group)
		success := 0
		var gasTotal uint64

		for _, sample := range group {
			latencies = append(latencies, sample.LatencyMs)
			accountSet[sample.Account] = true
			if sample.Success {
				success++
				gasValues = append(gasValues, sample.GasUsed)
				gasTotal += sample.GasUsed
			}
		}
		sort.Float64s(latencies)
		sort.Slice(gasValues, func(i, j int) bool { return gasValues[i] < gasValues[j] })

		s := FunctionSummary{
			Function:        fn,
			TotalSamples:    total,
			SuccessSamples:  success,
			FailedSamples:   total - success,
			SuccessRate:     safeRate(success, total),
			LatencyMinMs:    minF(latencies),
			LatencyMaxMs:    maxF(latencies),
			LatencyMeanMs:   meanF(latencies),
			LatencyStdMs:    stdF(latencies),
			LatencyP25Ms:    percentile(latencies, 25),
			LatencyP50Ms:    percentile(latencies, 50),
			LatencyP75Ms:    percentile(latencies, 75),
			LatencyP95Ms:    percentile(latencies, 95),
			GasUsedTotal:    gasTotal,
			GasUsedMean:     meanU(gasValues),
			GasUsedStd:      stdU(gasValues),
			GasUsedMin:      minU(gasValues),
			GasUsedMax:      maxU(gasValues),
			UniqueAccounts:  len(accountSet),
			AvgTxPerAccount: float64(total) / float64(len(accountSet)),
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

func failedSample(account string, function string, iter int, err error) TxSample {
	return TxSample{
		Account:   account,
		Executor:  account,
		Function:  function,
		Iteration: iter,
		Success:   false,
		Error:     err.Error(),
	}
}

func skippedSample(account string, executor string, function string, iter int, reason string) TxSample {
	return TxSample{
		Account:    account,
		Executor:   executor,
		Function:   function,
		Iteration:  iter,
		Skipped:    true,
		SkipReason: reason,
		Success:    true,
		LatencyMs:  0,
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
