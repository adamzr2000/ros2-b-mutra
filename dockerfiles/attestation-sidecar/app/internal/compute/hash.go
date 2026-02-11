package compute

import (
	"bufio"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"strconv"
	"strings"

	"app/internal/logger"
)

// ComputeHashError represents hash computation errors
type ComputeHashError struct {
	Message string
}

func (e *ComputeHashError) Error() string {
	return e.Message
}

// GetPID returns the PID of a running process by name
// Scans /proc/*/comm to find the process (like Python version)
func GetPID(programName string) (int, error) {
	procDir, err := os.Open("/proc")
	if err != nil {
		return 0, &ComputeHashError{Message: fmt.Sprintf("Cannot open /proc: %v", err)}
	}
	defer procDir.Close()

	entries, err := procDir.Readdirnames(-1)
	if err != nil {
		return 0, &ComputeHashError{Message: fmt.Sprintf("Cannot read /proc: %v", err)}
	}

	var foundPIDs []int
	for _, entry := range entries {
		// Check if entry is a PID (all digits)
		pid, err := strconv.Atoi(entry)
		if err != nil {
			continue
		}

		// Read /proc/PID/comm
		commPath := fmt.Sprintf("/proc/%d/comm", pid)
		commData, err := os.ReadFile(commPath)
		if err != nil {
			continue
		}

		comm := strings.TrimSpace(string(commData))
		// comm is truncated to 15 chars, so compare with truncated name too
		if comm == programName || strings.HasPrefix(programName, comm) || strings.HasPrefix(comm, programName) {
			foundPIDs = append(foundPIDs, pid)
		}
	}

	if len(foundPIDs) == 0 {
		return 0, &ComputeHashError{Message: "Program not found"}
	}

	// Return the minimum PID (like Python's min())
	minPID := foundPIDs[0]
	for _, pid := range foundPIDs[1:] {
		if pid < minPID {
			minPID = pid
		}
	}

	return minPID, nil
}

// GetTextSectionSize returns the size of the .text section of a binary
func GetTextSectionSize(binaryPath string) (int, error) {
	cmd := exec.Command("readelf", "-W", "-S", binaryPath)
	output, err := cmd.Output()
	if err != nil {
		return 0, &ComputeHashError{Message: fmt.Sprintf("Error running readelf: %v", err)}
	}

	lines := strings.Split(string(output), "\n")
	for _, line := range lines {
		if strings.Contains(line, ".text") {
			fields := strings.Fields(line)
			if len(fields) >= 6 {
				size, err := strconv.ParseInt(fields[5], 16, 64)
				if err != nil {
					return 0, &ComputeHashError{Message: fmt.Sprintf("Error parsing text section size: %v", err)}
				}
				return int(size), nil
			}
		}
	}

	return 0, &ComputeHashError{Message: "No .text section found"}
}

// GetFirstRXPageAddress returns the first r-xp memory region address for a process
func GetFirstRXPageAddress(pid int) (uint64, error) {
	mapsPath := fmt.Sprintf("/proc/%d/maps", pid)
	file, err := os.Open(mapsPath)
	if err != nil {
		return 0, &ComputeHashError{Message: fmt.Sprintf("Error reading memory maps: %v", err)}
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		parts := strings.Fields(line)
		if len(parts) >= 2 && strings.Contains(parts[1], "r-xp") {
			addressRange := strings.Split(parts[0], "-")
			if len(addressRange) >= 1 {
				addr, err := strconv.ParseUint(addressRange[0], 16, 64)
				if err != nil {
					return 0, &ComputeHashError{Message: fmt.Sprintf("Error parsing address: %v", err)}
				}
				return addr, nil
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return 0, &ComputeHashError{Message: fmt.Sprintf("Error reading memory maps: %v", err)}
	}

	return 0, &ComputeHashError{Message: "No RX page found"}
}

// ReadMemory reads memory from a process
func ReadMemory(pid int, address uint64, size int) ([]byte, error) {
	memPath := fmt.Sprintf("/proc/%d/mem", pid)
	file, err := os.Open(memPath)
	if err != nil {
		return nil, &ComputeHashError{Message: fmt.Sprintf("Error opening process memory: %v", err)}
	}
	defer file.Close()

	_, err = file.Seek(int64(address), 0)
	if err != nil {
		return nil, &ComputeHashError{Message: fmt.Sprintf("Error seeking in process memory: %v", err)}
	}

	data := make([]byte, size)
	n, err := file.Read(data)
	if err != nil {
		return nil, &ComputeHashError{Message: fmt.Sprintf("Error reading process memory: %v", err)}
	}

	if n == 0 {
		return nil, &ComputeHashError{Message: "Failed to read memory data"}
	}

	return data[:n], nil
}

// ComputeSHA256 computes SHA256 hash of data
func ComputeSHA256(data []byte) string {
	hash := sha256.Sum256(data)
	return hex.EncodeToString(hash[:])
}

// ComputeProgramHash computes the hash of a running program's .text section
func ComputeProgramHash(programName string, textSize, offset int) (string, error) {
	logger.Debug("Computing hash for process '%s' with text_size=%d, offset=%d", programName, textSize, offset)

	pid, err := GetPID(programName)
	if err != nil {
		logger.Error("Process '%s' not found", programName)
		return "", err
	}
	logger.Debug("Found PID: %d for process '%s'", pid, programName)

	rxAddress, err := GetFirstRXPageAddress(pid)
	if err != nil {
		logger.Error("Failed to get RX page address for PID %d", pid)
		return "", err
	}
	logger.Debug("Found RX page address: 0x%x for PID %d", rxAddress, pid)

	if offset < 0 {
		return "", &ComputeHashError{Message: fmt.Sprintf("Invalid offset value: %d", offset)}
	}
	logger.Debug("Using offset: %d", offset)

	beginAddress := rxAddress + uint64(offset)
	logger.Debug("Calculated begin_address: 0x%x", beginAddress)

	if textSize <= 0 {
		return "", &ComputeHashError{Message: fmt.Sprintf("Invalid text_section_size: %d", textSize)}
	}
	logger.Debug("Using text_size: %d", textSize)

	logger.Debug("Reading %d bytes of memory from address 0x%x for PID %d", textSize, beginAddress, pid)
	memoryContent, err := ReadMemory(pid, beginAddress, textSize)
	if err != nil {
		return "", err
	}

	hashResult := ComputeSHA256(memoryContent)
	logger.Debug("Computed hash: %s...%s", hashResult[:8], hashResult[len(hashResult)-8:])

	return hashResult, nil
}

// ComputeProgramHashWithPrefix computes the hash of a running program's .text section
// by dynamically finding the offset using the text section prefix bytes.
// prefixHex: hex string of the first N bytes of .text section (e.g., "4c8da42440ffffff...")
func ComputeProgramHashWithPrefix(programName string, textSize int, prefixHex string) (string, error) {
	logger.Debug("Computing hash for process '%s' with text_size=%d, prefix=%s...",
		programName, textSize, prefixHex[:min(16, len(prefixHex))])

	// Decode hex prefix to bytes
	prefix, err := hex.DecodeString(prefixHex)
	if err != nil {
		return "", &ComputeHashError{Message: fmt.Sprintf("Invalid prefix hex string: %v", err)}
	}

	if len(prefix) == 0 {
		return "", &ComputeHashError{Message: "Prefix is empty"}
	}

	pid, err := GetPID(programName)
	if err != nil {
		logger.Error("Process '%s' not found", programName)
		return "", err
	}
	logger.Debug("Found PID: %d for process '%s'", pid, programName)

	// Find offset dynamically using the prefix (reuse the function from elf.go)
	offset, err := FindTextSectionOffsetInMemory(pid, prefix)
	if err != nil {
		logger.Error("Failed to find .text section offset for PID %d: %v", pid, err)
		return "", err
	}
	logger.Debug("Dynamically computed offset: %d (0x%x)", offset, offset)

	rxAddress, err := GetFirstRXPageAddress(pid)
	if err != nil {
		logger.Error("Failed to get RX page address for PID %d", pid)
		return "", err
	}
	logger.Debug("Found RX page address: 0x%x for PID %d", rxAddress, pid)

	beginAddress := rxAddress + uint64(offset)
	logger.Debug("Calculated begin_address: 0x%x", beginAddress)

	if textSize <= 0 {
		return "", &ComputeHashError{Message: fmt.Sprintf("Invalid text_section_size: %d", textSize)}
	}
	logger.Debug("Using text_size: %d", textSize)

	logger.Debug("Reading %d bytes of memory from address 0x%x for PID %d", textSize, beginAddress, pid)
	memoryContent, err := ReadMemory(pid, beginAddress, textSize)
	if err != nil {
		return "", err
	}

	hashResult := ComputeSHA256(memoryContent)
	logger.Debug("Computed hash: %s...%s", hashResult[:8], hashResult[len(hashResult)-8:])

	return hashResult, nil
}

// IsComputeHashError checks if error is a ComputeHashError
func IsComputeHashError(err error) bool {
	var hashErr *ComputeHashError
	return errors.As(err, &hashErr)
}
