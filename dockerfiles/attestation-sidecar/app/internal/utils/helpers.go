package utils

import (
	crand "crypto/rand"
	"encoding/hex"
	"fmt"
	"os"
	"regexp"
	"strconv"
	"strings"
	"time"
)

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
	// Use crypto/rand for 8 bytes of entropy → 16 hex chars.
	// Total length: "attestation" (11) + 16 = 27 chars ≤ 32 bytes (bytes32 safe).
	// This avoids the seed-collision bug where containers starting simultaneously
	// share the same UnixNano value and produce identical IDs.
	var b [8]byte
	if _, err := crand.Read(b[:]); err != nil {
		// Unreachable in practice; fall back to nanosecond timestamp.
		return fmt.Sprintf("attestation%016x", uint64(time.Now().UnixNano()))
	}
	return fmt.Sprintf("attestation%s", hex.EncodeToString(b[:]))
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
