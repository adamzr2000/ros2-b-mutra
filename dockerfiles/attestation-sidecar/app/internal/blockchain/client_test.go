package blockchain

import (
	"errors"
	"strings"
	"testing"

	"github.com/ethereum/go-ethereum/accounts/abi"
	"github.com/ethereum/go-ethereum/common/hexutil"
)

type revertErr struct{ data string }

func (e revertErr) Error() string          { return "Execution reverted" }
func (e revertErr) ErrorData() interface{} { return e.data }

func TestRevertReason(t *testing.T) {
	stringTy, _ := abi.NewType("string", "", nil)
	packed, err := abi.Arguments{{Type: stringTy}}.Pack("Prove : evidence outside the freshness window")
	if err != nil {
		t.Fatal(err)
	}
	data := append([]byte{0x08, 0xc3, 0x79, 0xa0}, packed...) // Error(string)
	got := revertReason(revertErr{hexutil.Encode(data)}).Error()
	if want := "reverted: Prove : evidence outside the freshness window"; got != want {
		t.Fatalf("revertReason = %q, want %q", got, want)
	}
	// Without revert data the node's error is kept.
	if got := revertReason(errors.New("connection refused")).Error(); !strings.Contains(got, "connection refused") {
		t.Fatalf("revertReason dropped the node error: %q", got)
	}
}
