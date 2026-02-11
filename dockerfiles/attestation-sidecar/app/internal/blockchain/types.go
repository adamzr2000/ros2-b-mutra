package blockchain

import (
	"encoding/hex"
	"fmt"
	"strings"
)

// AttestationState matches your Python IntEnum
type AttestationState int

const (
	Open               AttestationState = 0
	ReadyForEvaluation AttestationState = 1
	Closed             AttestationState = 2
)

// TextToBytes32 mimics text_to_bytes32 (right-padded with null bytes)
func TextToBytes32(textInput string) ([32]byte, error) {
	var b32 [32]byte
	b := []byte(textInput)
	if len(b) > 32 {
		return b32, fmt.Errorf("string '%s' exceeds 32 bytes", textInput)
	}
	copy(b32[:], b)
	return b32, nil
}

// AsBytes32 converts hex strings or bytes to [32]byte
func AsBytes32(v interface{}) ([32]byte, error) {
	var b32 [32]byte
	switch val := v.(type) {
	case []byte:
		if len(val) != 32 {
			return b32, fmt.Errorf("must be 32 bytes, got %d", len(val))
		}
		copy(b32[:], val)
	case string:
		hexStr := strings.TrimPrefix(val, "0x")
		b, err := hex.DecodeString(hexStr)
		if err != nil || len(b) != 32 {
			return b32, fmt.Errorf("invalid bytes32 hex: %v", val)
		}
		copy(b32[:], b)
	default:
		return b32, fmt.Errorf("unsupported type for bytes32")
	}
	return b32, nil
}

// Event names as constants to prevent typos
const (
	EventAgentRegistered      = "AgentRegistered"
	EventAttestationStarted   = "AttestationStarted"
	EventReadyForEvaluation   = "ReadyForEvaluation"
	EventAttestationCompleted = "AttestationCompleted"
)