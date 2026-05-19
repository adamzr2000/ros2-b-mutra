package compute

import (
	"bytes"
	"crypto/sha256"
	"debug/elf"
	"encoding/hex"
	"fmt"
	"os"

	"app/internal/logger"
)

// ELFTextSectionInfo contains information about the .text section of an ELF binary
type ELFTextSectionInfo struct {
	SHA256            string
	Size              int
	Offset            int
	TextSectionPrefix []byte
}

// ComputeELFTextSectionInfo computes SHA256, size and prefix of the .text section of an ELF binary
// binaryPath: path to the ELF binary (use /proc/self/exe for current process)
// prefixLen: number of bytes to extract from the beginning of .text section (0 = none)
func ComputeELFTextSectionInfo(binaryPath string, prefixLen int) (*ELFTextSectionInfo, error) {
	file, err := elf.Open(binaryPath)
	if err != nil {
		return nil, fmt.Errorf("failed to open ELF file %s: %w", binaryPath, err)
	}
	defer file.Close()

	textSection := file.Section(".text")
	if textSection == nil {
		return nil, fmt.Errorf("section .text not found in %s", binaryPath)
	}

	// Read the .text section data
	data, err := textSection.Data()
	if err != nil {
		return nil, fmt.Errorf("failed to read .text section: %w", err)
	}

	// Compute SHA256
	hash := sha256.Sum256(data)
	hashHex := hex.EncodeToString(hash[:])

	// Get size
	size := int(textSection.Size)

	// Get offset in file (for reference)
	offset := int(textSection.Offset)

	// Extract prefix
	var prefix []byte
	if prefixLen > 0 {
		if prefixLen > len(data) {
			prefix = data
		} else {
			prefix = data[:prefixLen]
		}
	}

	return &ELFTextSectionInfo{
		SHA256:            hashHex,
		Size:              size,
		Offset:            offset,
		TextSectionPrefix: prefix,
	}, nil
}

// ComputeSelfIntegrityInfo computes the .text section info for the current running process
// This reads /proc/self/exe to get the binary of the current process
func ComputeSelfIntegrityInfo(prefixLen int) (*ELFTextSectionInfo, error) {
	// /proc/self/exe is a symlink to the current process's executable
	exePath, err := os.Readlink("/proc/self/exe")
	if err != nil {
		return nil, fmt.Errorf("failed to read /proc/self/exe: %w", err)
	}

	logger.Debug("[ELF] Reading self binary from: %s", exePath)

	info, err := ComputeELFTextSectionInfo(exePath, prefixLen)
	if err != nil {
		return nil, err
	}

	logger.Info("[ELF] Self .text section: size=%d, SHA256=%s...%s",
		info.Size, info.SHA256[:16], info.SHA256[len(info.SHA256)-16:])

	return info, nil
}

// FindTextSectionOffsetInMemory finds the offset of the .text section in memory
// by searching for the prefix bytes in executable memory segments
func FindTextSectionOffsetInMemory(pid int, prefix []byte) (int, error) {
	if len(prefix) == 0 {
		return 0, fmt.Errorf("prefix is empty")
	}

	logger.Debug("[ELF] Searching for .text prefix in memory of PID %d (prefix: %s...)",
		pid, hex.EncodeToString(prefix[:min(8, len(prefix))]))

	// Read memory maps
	mapsPath := fmt.Sprintf("/proc/%d/maps", pid)
	mapsData, err := os.ReadFile(mapsPath)
	if err != nil {
		return 0, fmt.Errorf("failed to read %s: %w", mapsPath, err)
	}

	var firstRXStart uint64 = 0
	lines := bytes.Split(mapsData, []byte("\n"))

	for _, line := range lines {
		lineStr := string(line)
		if len(lineStr) == 0 {
			continue
		}

		// Parse line: "address-range perms offset dev inode pathname"
		// Example: "00400000-00452000 r-xp 00000000 08:01 123456 /app/binary"
		parts := bytes.Fields(line)
		if len(parts) < 2 {
			continue
		}

		perms := string(parts[1])
		if !bytes.Contains(parts[1], []byte("r-xp")) {
			continue
		}

		// Parse address range
		addrRange := bytes.Split(parts[0], []byte("-"))
		if len(addrRange) != 2 {
			continue
		}

		var start, end uint64
		fmt.Sscanf(string(addrRange[0]), "%x", &start)
		fmt.Sscanf(string(addrRange[1]), "%x", &end)

		if firstRXStart == 0 {
			firstRXStart = start
		}

		segmentSize := int(end - start)
		logger.Debug("[ELF] Scanning segment 0x%x-0x%x (%s, size=%d)", start, end, perms, segmentSize)

		// Read this segment
		memData, err := ReadMemory(pid, start, segmentSize)
		if err != nil {
			logger.Debug("[ELF] Failed to read segment: %v", err)
			continue
		}

		// Search for prefix
		idx := bytes.Index(memData, prefix)
		if idx != -1 {
			absoluteAddr := start + uint64(idx)
			relativeOffset := int(absoluteAddr - firstRXStart)
			logger.Info("[ELF] Found .text prefix at offset %d (0x%x) from first RX page",
				relativeOffset, relativeOffset)
			return relativeOffset, nil
		}
	}

	return 0, fmt.Errorf("text section prefix not found in any executable segment")
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
