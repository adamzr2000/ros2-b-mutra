package compute

import (
	"bufio"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"

	"app/internal/logger"
)

type procMapEntry struct {
	Start    uint64
	End      uint64
	Perms    string
	Offset   uint64
	Pathname string
}

type LoadedLibsDigest struct {
	Aggregate string
	PerLib    map[string]string
	LibCount  int
	TotalSize int
}

func parseProcMaps(pid int) ([]procMapEntry, error) {
	mapsPath := fmt.Sprintf("/proc/%d/maps", pid)
	file, err := os.Open(mapsPath)
	if err != nil {
		return nil, fmt.Errorf("failed to open %s: %w", mapsPath, err)
	}
	defer file.Close()

	var entries []procMapEntry
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		fields := strings.Fields(line)
		if len(fields) < 5 {
			continue
		}

		addrRange := strings.Split(fields[0], "-")
		if len(addrRange) != 2 {
			continue
		}

		start, err := strconv.ParseUint(addrRange[0], 16, 64)
		if err != nil {
			continue
		}
		end, err := strconv.ParseUint(addrRange[1], 16, 64)
		if err != nil {
			continue
		}

		offset, err := strconv.ParseUint(fields[2], 16, 64)
		if err != nil {
			continue
		}

		pathname := ""
		if len(fields) >= 6 {
			pathname = strings.Join(fields[5:], " ")
		}

		entries = append(entries, procMapEntry{
			Start:    start,
			End:      end,
			Perms:    fields[1],
			Offset:   offset,
			Pathname: pathname,
		})
	}

	if err := scanner.Err(); err != nil {
		return nil, fmt.Errorf("failed to scan %s: %w", mapsPath, err)
	}

	return entries, nil
}

func normalizePath(path string) string {
	return strings.TrimSuffix(path, " (deleted)")
}

// ComputeLoadedLibrariesHash computes a stable aggregate hash over executable segments
// of shared libraries loaded in the target process. The aggregate is computed by
// sorting libraries by path and hashing "path:digest;" entries.
func ComputeLoadedLibrariesHash(pid int, excludePath string) (LoadedLibsDigest, error) {
	entries, err := parseProcMaps(pid)
	if err != nil {
		return LoadedLibsDigest{}, err
	}

	excludeNorm := normalizePath(excludePath)
	byLib := make(map[string][]procMapEntry)

	for _, entry := range entries {
		if !strings.Contains(entry.Perms, "r-xp") {
			continue
		}
		if entry.Pathname == "" || !strings.HasPrefix(entry.Pathname, "/") {
			continue
		}

		entryPathNorm := normalizePath(entry.Pathname)
		if excludeNorm != "" && entryPathNorm == excludeNorm {
			continue
		}

		byLib[entry.Pathname] = append(byLib[entry.Pathname], entry)
	}

	if len(byLib) == 0 {
		return LoadedLibsDigest{}, fmt.Errorf("no executable library mappings found")
	}

	perLib := make(map[string]string)
	paths := make([]string, 0, len(byLib))
	totalSize := 0
	failed := 0

	for path, segments := range byLib {
		sort.Slice(segments, func(i, j int) bool {
			if segments[i].Offset == segments[j].Offset {
				return segments[i].Start < segments[j].Start
			}
			return segments[i].Offset < segments[j].Offset
		})

		hasher := sha256.New()
		libSize := 0
		for _, seg := range segments {
			segSize := int(seg.End - seg.Start)
			if segSize <= 0 {
				continue
			}
			data, err := ReadMemory(pid, seg.Start, segSize)
			if err != nil {
				failed++
				logger.Warn("[LIBS] Failed to read %s segment 0x%x-0x%x: %v", path, seg.Start, seg.End, err)
				libSize = 0
				break
			}
			hasher.Write(data)
			libSize += len(data)
		}

		if libSize == 0 {
			continue
		}

		perLib[path] = hex.EncodeToString(hasher.Sum(nil))
		totalSize += libSize
		paths = append(paths, path)
	}

	if len(perLib) == 0 {
		return LoadedLibsDigest{}, fmt.Errorf("failed to hash executable library mappings")
	}

	sort.Strings(paths)
	aggregateHasher := sha256.New()
	for _, path := range paths {
		digest := perLib[path]
		aggregateHasher.Write([]byte(path))
		aggregateHasher.Write([]byte(":"))
		aggregateHasher.Write([]byte(digest))
		aggregateHasher.Write([]byte(";"))
	}

	aggregate := hex.EncodeToString(aggregateHasher.Sum(nil))
	if failed > 0 {
		return LoadedLibsDigest{
			Aggregate: aggregate,
			PerLib:    perLib,
			LibCount:  len(perLib),
			TotalSize: totalSize,
		}, fmt.Errorf("failed to read %d library segments", failed)
	}

	return LoadedLibsDigest{
		Aggregate: aggregate,
		PerLib:    perLib,
		LibCount:  len(perLib),
		TotalSize: totalSize,
	}, nil
}
