package compute

import (
	"crypto/sha256"
	"encoding/hex"
	"os"

	"app/internal/utils"
)

// AttestationDigest is the triplet a prover would submit on-chain
// (combined_hash) along with its constituents, useful for bootstrapping
// reference measurements before deployment.
type AttestationDigest struct {
	RobotHash    string `json:"robot_hash"`
	SidecarHash  string `json:"attestation_sidecar_hash"`
	LibsHash     string `json:"libs_hash,omitempty"`
	CombinedHash string `json:"combined_hash"`
}

// SelfIntegrityParams mirrors config.SelfIntegrityConfig but kept here to
// avoid importing config (which would create an import cycle once prover.go
// is refactored to use this helper).
type SelfIntegrityParams struct {
	Enabled           bool
	CmdName           string
	TextSectionSize   int
	TextSectionOffset int
}

// ComputeAttestationDigest performs a single measurement cycle and returns
// the same triplet the prover would submit. It mirrors the body of
// RunProverLogicContinuousMode's per-cycle computation (sha256 over hex
// concatenation of robot/sidecar/libs digests). Kept as a standalone helper
// so the /digest endpoint and the prover loop stay bit-for-bit consistent.
func ComputeAttestationDigest(
	cmdName string,
	textSectionSize int,
	offset int,
	textSectionPrefix string,
	si SelfIntegrityParams,
) (AttestationDigest, error) {
	var out AttestationDigest

	var (
		robotHash string
		err       error
	)
	if textSectionPrefix != "" {
		robotHash, err = ComputeProgramHashWithPrefix(cmdName, textSectionSize, textSectionPrefix)
	} else {
		robotHash, err = ComputeProgramHash(cmdName, textSectionSize, offset)
	}
	if err != nil {
		return out, err
	}
	out.RobotHash = robotHash
	out.CombinedHash = robotHash

	if si.Enabled {
		sidecarHash, err := ComputeProgramHash(si.CmdName, si.TextSectionSize, si.TextSectionOffset)
		if err != nil {
			return out, err
		}
		out.SidecarHash = sidecarHash
		sum := sha256.Sum256([]byte(robotHash + sidecarHash))
		out.CombinedHash = hex.EncodeToString(sum[:])
	}

	if utils.GetEnvBool("ENABLE_LIBS_HASH", false) {
		exePath, err := os.Readlink("/proc/self/exe")
		if err == nil {
			libs, err := ComputeLoadedLibrariesHash(os.Getpid(), exePath)
			if err == nil && libs.Aggregate != "" {
				out.LibsHash = libs.Aggregate
				sum := sha256.Sum256([]byte(robotHash + out.SidecarHash + libs.Aggregate))
				out.CombinedHash = hex.EncodeToString(sum[:])
			}
		}
	}

	return out, nil
}
