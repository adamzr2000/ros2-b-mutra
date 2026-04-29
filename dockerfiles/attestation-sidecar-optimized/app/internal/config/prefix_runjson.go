
package config

import (
	"fmt"
	"path/filepath"
	"regexp"
	"strconv"
	"app/internal/utils"
)

// GetNextRunJSONWithPrefix returns the next run JSON file with a custom prefix (e.g., "startup-" or "continuous-")
func GetNextRunJSONWithPrefix(resultsDir, prefix, participant string) string {
	utils.EnsureDir(resultsDir)
	pattern := filepath.Join(resultsDir, fmt.Sprintf("%s%s-run*.json", prefix, participant))

	files, _ := filepath.Glob(pattern)
	maxIdx := 0
	re := regexp.MustCompile(fmt.Sprintf(`%s%s-run(\d+)\.json$`, regexp.QuoteMeta(prefix), regexp.QuoteMeta(participant)))

	for _, f := range files {
		baseName := filepath.Base(f)
		matches := re.FindStringSubmatch(baseName)
		if len(matches) == 2 {
			if idx, err := strconv.Atoi(matches[1]); err == nil {
				if idx > maxIdx {
					maxIdx = idx
				}
			}
		}
	}
	return filepath.Join(resultsDir, fmt.Sprintf("%s%s-run%d.json", prefix, participant, maxIdx+1))
}
