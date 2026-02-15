package utils

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"app/internal/logger"
)

// -------------------------------------------------------------------------
// Global Locking Mechanism (Process-Local)
// -------------------------------------------------------------------------

var (
	// Map to store one mutex per file path
	jsonFileLocks = make(map[string]*sync.Mutex)
	// Mutex to protect the map creation itself
	globalLockCreation sync.Mutex

	// Provides a monotonic baseline for PerfNs
	programStartTime = time.Now()
)

// getJSONLock returns a thread-safe mutex for a specific file path.
func getJSONLock(path string) *sync.Mutex {
	globalLockCreation.Lock()
	defer globalLockCreation.Unlock()

	if _, exists := jsonFileLocks[path]; !exists {
		jsonFileLocks[path] = &sync.Mutex{}
	}
	return jsonFileLocks[path]
}

// -------------------------------------------------------------------------
// High-precision timing helpers
// -------------------------------------------------------------------------

// NowMs returns Unix epoch time in milliseconds (wall clock).
func NowMs() int64 {
	return time.Now().UnixMilli()
}

// PerfNs returns a high-resolution monotonic clock in nanoseconds.
func PerfNs() int64 {
	return time.Now().UnixNano()
}

// -------------------------------------------------------------------------
// File System & Atomic Writes
// -------------------------------------------------------------------------

func EnsureDir(path string) {
	dir := filepath.Dir(path)
	if dir == "." || dir == "" {
		return
	}
	os.MkdirAll(dir, 0755)
}

// atomicWriteJSON writes data to a temp file and renames it to ensure atomicity.
func atomicWriteJSON(path string, data interface{}) error {
	dir := filepath.Dir(path)
	if dir == "" {
		dir = "."
	}
	os.MkdirAll(dir, 0755)

	// Create temp file
	tmpFile, err := os.CreateTemp(dir, "tmp_*.json")
	if err != nil {
		return err
	}
	defer os.Remove(tmpFile.Name()) // Clean up if something fails before rename

	// Write JSON
	encoder := json.NewEncoder(tmpFile)
	encoder.SetEscapeHTML(false)
	if err := encoder.Encode(data); err != nil {
		tmpFile.Close()
		return err
	}

	// Flush and Sync
	if err := tmpFile.Sync(); err != nil {
		tmpFile.Close()
		return err
	}
	tmpFile.Close()

	// Set permissions (0644)
	os.Chmod(tmpFile.Name(), 0644)

	// Atomic Rename
	return os.Rename(tmpFile.Name(), path)
}

// -------------------------------------------------------------------------
// Result Data Structures
// -------------------------------------------------------------------------

// ResultsData represents the schema of the export file.
type ResultsData struct {
	Participant string                   `json:"participant"`
	Clock       string                   `json:"clock"`
	TimeUnit    string                   `json:"time_unit"`
	TStart      int64                    `json:"t_start"`
	TEnd        int64                    `json:"t_end,omitempty"`
	Stopped     bool                     `json:"stopped,omitempty"`
	LastWriteMs int64                    `json:"last_write_ms,omitempty"`
	Prover      []map[string]interface{} `json:"prover"`
	Verifier    []map[string]interface{} `json:"verifier"`
	Oracle      []map[string]interface{} `json:"oracle"`
}

func loadOrInitResults(path string, participant string, nowMs int64) ResultsData {
	// Default structure
	data := ResultsData{
		Participant: participant,
		Clock:       "unix_epoch",
		TimeUnit:    "ms",
		TStart:      nowMs,
		Prover:      []map[string]interface{}{},
		Verifier:    []map[string]interface{}{},
		Oracle:      []map[string]interface{}{},
	}

	if _, err := os.Stat(path); err == nil {
		fileBytes, err := os.ReadFile(path)
		if err == nil {
			var loaded ResultsData
			if err := json.Unmarshal(fileBytes, &loaded); err == nil {
				// Return loaded only if it seems valid (has t_start)
				if loaded.TStart != 0 {
					return loaded
				}
			}
		}
	}
	return data
}

func upsertByAttestationID(roleList *[]map[string]interface{}, att map[string]interface{}) {
	attID, ok := att["attestation_id"].(string)
	if !ok || attID == "" {
		*roleList = append(*roleList, att)
		return
	}

	// Try to find existing record to update
	for i, existing := range *roleList {
		if exID, ok := existing["attestation_id"].(string); ok && exID == attID {
			// Merge: update existing map with new values
			for k, v := range att {
				existing[k] = v
			}
			(*roleList)[i] = existing
			return
		}
	}

	// If not found, append
	*roleList = append(*roleList, att)
}

// EnsureResultsInitialized ensures the results file exists and has the header.
func EnsureResultsInitialized(jsonDir string, participant string, jsonPath string) {
	EnsureDir(jsonDir)
	if jsonPath == "" {
		jsonPath = filepath.Join(jsonDir, fmt.Sprintf("%s.json", participant))
	}

	lock := getJSONLock(jsonPath)
	lock.Lock()
	defer lock.Unlock()

	nowMs := NowMs()
	
	// Check if already valid
	if _, err := os.Stat(jsonPath); err == nil {
		fileBytes, err := os.ReadFile(jsonPath)
		if err == nil {
			var loaded map[string]interface{}
			if err := json.Unmarshal(fileBytes, &loaded); err == nil {
				if _, ok := loaded["t_start"]; ok {
					return
				}
			}
		}
	}

	// Fresh Init
	data := ResultsData{
		Participant: participant,
		Clock:       "unix_epoch",
		TimeUnit:    "ms",
		TStart:      nowMs,
		Prover:      []map[string]interface{}{},
		Verifier:    []map[string]interface{}{},
		Oracle:      []map[string]interface{}{},
	}
	atomicWriteJSON(jsonPath, data)
}

// -------------------------------------------------------------------------
// Export Logic
// -------------------------------------------------------------------------

// Constants for mapping logic
var (
	roleFields = map[string]map[string][]string{
		"prover": {
			"t_prover_start":               {"prover_start"},
			"t_evidence_sent":              {"evidence_sent"},
			"t_evidence_sent_tx_confirmed": {"evidence_sent_tx_confirmed"},
			"t_result_received":            {"result_received"},
			"t_prover_finished":            {"prover_finished"},
		},
		"verifier": {
			"meta_verification_result":        {"verification_result"},
			"meta_is_secaas_verifier":         {"is_secaas_verifier"},
			"t_ready_for_evaluation_received": {"ready_for_evaluation_received"},
			"t_verifier_start":                {"verifier_start"},
			"t_get_signatures_start":          {"get_signatures_start"},
			"t_get_signatures_finished":       {"get_signatures_finished"},
			"t_verify_compute_start":          {"verify_compute_start"},
			"t_verify_compute_finished":       {"verify_compute_finished"},
			"t_result_sent":                   {"result_sent"},
			"t_result_sent_tx_confirmed":      {"result_sent_tx_confirmed"},
			"t_verifier_finished":             {"verifier_finished"},
		},
		"oracle": {
			"meta_is_secaas_verifier":                  {"is_secaas_verifier"},
			"t_attestation_started_received":           {"attestation_started_received"},
			"t_oracle_start":                           {"oracle_start"},
			"t_get_prover_addr_start":                  {"get_prover_addr_start"},
			"t_get_prover_addr_finished":               {"get_prover_addr_finished"},
			"t_get_prover_ref_signature_db_start":      {"get_prover_ref_signatures_db_start"},
			"t_get_prover_ref_signature_db_finished":   {"get_prover_ref_signatures_db_finished"},
			"t_prover_ref_signature_sent":              {"prover_ref_signatures_sent"},
			"t_prover_ref_signature_sent_tx_confirmed": {"prover_ref_signatures_sent_tx_confirmed"},
			"t_oracle_finished":                        {"oracle_finished"},
		},
	}

	rolePrecisionMap = map[string][][3]string{
		"prover": {
			{"dur_send_evidence_call", "p_send_evidence_start", "p_send_evidence_finished"},
			{"dur_send_evidence_tx_confirm", "p_send_evidence_start", "p_send_evidence_finished_tx_confirmed"},
			{"dur_prover_e2e", "p_send_evidence_start", "p_result_received"},
			{"dur_prover_total", "p_prover_start", "p_prover_finished"},
		},
		"verifier": {
			{"dur_signatures_fetch", "p_get_signatures_start", "p_get_signatures_finished"},
			{"dur_verify_compute", "p_verify_compute_start", "p_verify_compute_finished"},
			{"dur_send_result_call", "p_send_result_start", "p_send_result_finished"},
			{"dur_send_result_tx_confirm", "p_send_result_start", "p_send_result_finished_tx_confirmed"},
			{"dur_verifier_reaction", "p_ready_for_evaluation_received", "p_verifier_start"},
			{"dur_verifier_total", "p_verifier_start", "p_verifier_finished"},
		},
		"oracle": {
			{"dur_prover_addr_fetch", "p_get_prover_addr_start", "p_get_prover_addr_finished"},
			{"dur_db_fetch", "p_get_prover_ref_signatures_db_start", "p_get_prover_ref_signatures_db_finished"},
			{"dur_send_prover_ref_signature_call", "p_send_prover_ref_signature_start", "p_send_prover_ref_signature_finished"},
			{"dur_send_prover_ref_signatures_tx_confirm", "p_send_prover_ref_signature_start", "p_send_prover_ref_signature_finished_tx_confirmed"},
			{"dur_oracle_reaction", "p_attestation_started_received", "p_oracle_start"},
			{"dur_oracle_total", "p_oracle_start", "p_oracle_finished"},
		},
	}
)

func ExportAttestationTimesJSON(participantName string, attestationID string, role string, timestamps map[string]interface{}, jsonDir string, jsonPath string) {
	EnsureDir(jsonDir)
	if jsonPath == "" {
		jsonPath = filepath.Join(jsonDir, fmt.Sprintf("%s.json", participantName))
	}

	lock := getJSONLock(jsonPath)
	lock.Lock()
	defer lock.Unlock()

	// Prepare record
	att := make(map[string]interface{})
	att["attestation_id"] = attestationID

	// 1) Copy Wall-clock timestamps and Metadata based on mapping
	fields, ok := roleFields[role]
	if ok {
		for outKey, candidates := range fields {
			for _, srcKey := range candidates {
				if val, exists := timestamps[srcKey]; exists {
					att[outKey] = val
					break
				}
			}
		}
	}

	// Helper to extract int64 safely from the original timestamps map
	getInt := func(m map[string]interface{}, k string) (int64, bool) {
		v, exists := m[k]
		if !exists {
			return 0, false
		}
		switch n := v.(type) {
		case int64:
			return n, true
		case int:
			return int64(n), true
		case float64:
			return int64(n), true
		default:
			return 0, false
		}
	}

	// 2) Compute High-Precision Durations (p_ fields)
	durs, ok := rolePrecisionMap[role]
	if ok {
		for _, triplet := range durs {
			prefix, pStart, pEnd := triplet[0], triplet[1], triplet[2]
			startVal, ok1 := getInt(timestamps, pStart)
			endVal, ok2 := getInt(timestamps, pEnd)

			if ok1 && ok2 {
				deltaNs := endVal - startVal
				
				// Export native nanoseconds
				att[fmt.Sprintf("%s_ns", prefix)] = deltaNs
				
				// Export float ms rounded to 4 decimals (e.g. 1.2345 ms)
				msVal := float64(deltaNs) / 1000000.0
				att[fmt.Sprintf("%s_ms", prefix)] = math.Round(msVal*10000) / 10000
			}
		}
	}

	// 3) Atomic Write
	nowMs := NowMs()
	data := loadOrInitResults(jsonPath, participantName, nowMs)

	if data.Stopped {
		return
	}

	switch role {
	case "prover":
		upsertByAttestationID(&data.Prover, att)
	case "verifier":
		upsertByAttestationID(&data.Verifier, att)
	case "oracle":
		upsertByAttestationID(&data.Oracle, att)
	}

	data.LastWriteMs = nowMs

	if err := atomicWriteJSON(jsonPath, data); err == nil {
		logger.Info("💾 Results saved to '%s'", jsonPath)
	}
}

func MarkExperimentStop(jsonDir string, participant string, jsonPath string) {
	if jsonPath == "" {
		jsonPath = filepath.Join(jsonDir, fmt.Sprintf("%s.json", participant))
	}
	lock := getJSONLock(jsonPath)
	lock.Lock()
	defer lock.Unlock()

	nowMs := NowMs()
	data := loadOrInitResults(jsonPath, participant, nowMs)

	if data.TEnd == 0 {
		data.TEnd = nowMs
		data.Stopped = true
		atomicWriteJSON(jsonPath, data)
	}
}

// -------------------------------------------------------------------------
// Environment Helpers
// -------------------------------------------------------------------------

func GetEnvInt(name string, def int) int {
	v := os.Getenv(name)
	if v == "" {
		return def
	}
	i, err := strconv.Atoi(v)
	if err != nil {
		return def
	}
	return i
}

func GetEnvFloat(name string, def float64) float64 {
	v := os.Getenv(name)
	if v == "" {
		return def
	}
	f, err := strconv.ParseFloat(v, 64)
	if err != nil {
		return def
	}
	return f
}

func GetEnvStr(name string, def string) string {
	v := os.Getenv(name)
	if v != "" {
		return v
	}
	return def
}

func GetEnvBool(name string, def bool) bool {
	v := strings.TrimSpace(os.Getenv(name))
	if v == "" {
		return def
	}
	switch strings.ToLower(v) {
	case "1", "true", "t", "yes", "y", "on":
		return true
	case "0", "false", "f", "no", "n", "off":
		return false
	default:
		return def
	}
}

// -------------------------------------------------------------------------
// Misc Helpers
// -------------------------------------------------------------------------

func GenerateAttestationID() string {
	timestamp := time.Now().Unix() // 10 digits usually
	// Seed random if strictly needed, though Go 1.20+ seeds global rand automatically.
	// For older Go versions or safety:
	r := rand.New(rand.NewSource(time.Now().UnixNano()))
	suffix := r.Intn(9000) + 1000 // 1000 to 9999
	return fmt.Sprintf("attestation%d%d", timestamp%1000000000, suffix)
}

func NormalizeEthAddress(addr string) string {
	if addr == "" {
		return ""
	}
	a := strings.ToLower(strings.TrimSpace(addr))
	return strings.TrimPrefix(a, "0x")
}

func ShortAttID(attID string, n int) string {
	if attID == "" {
		return fmt.Sprintf("att...%s", strings.Repeat("?", n))
	}
	
	// Try to find trailing digits using regex
	re := regexp.MustCompile(`(\d+)$`)
	match := re.FindString(attID)
	
	var tail string
	if match != "" && len(match) >= n {
		tail = match[len(match)-n:]
	} else {
		if len(attID) > n {
			tail = attID[len(attID)-n:]
		} else {
			tail = attID
		}
	}
	return fmt.Sprintf("att...%s", tail)
}