package blockchain

import (
	"context"
	"encoding/json"
	"fmt"
	"math/big"
	"os"
	"path/filepath"
	"sort"
	"time"

	"app/internal/logger"

	"github.com/ethereum/go-ethereum"
	"github.com/ethereum/go-ethereum/accounts/abi"
	"github.com/ethereum/go-ethereum/common"
	"github.com/ethereum/go-ethereum/core/types"
)

// EventHandler is the callback signature for processing logs.
type EventHandler func(log types.Log, parsedData map[string]interface{}) error

type EventWatcher struct {
	client         *BlockchainClient
	eventName      string
	eventABI       abi.Event
	address        common.Address
	topics         [][]common.Hash
	checkpointPath string
	confirmations  uint64
	batchSize      uint64
	pollInterval   time.Duration

	// State
	fromBlock *big.Int
	seenKeys  map[string]bool
}

// CheckpointData matches the JSON structure of the Python version
type CheckpointData struct {
	FromBlock uint64   `json:"from_block"`
	SeenKeys  []string `json:"seen_keys"`
}

func NewEventWatcher(
	client *BlockchainClient,
	eventName string,
	checkpointPath string,
	confirmations uint64,
	batchSize uint64,
	pollIntervalSec float64,
) (*EventWatcher, error) {

	// 1. Find Event ABI
	evABI, ok := client.contractABI.Events[eventName]
	if !ok {
		return nil, fmt.Errorf("event '%s' not found in contract ABI", eventName)
	}

	// 2. Prepare Topics (Topic[0] is the event signature hash)
	topic0 := evABI.ID
	topics := [][]common.Hash{{topic0}}

	// 3. Init Watcher
	watcher := &EventWatcher{
		client:         client,
		eventName:      eventName,
		eventABI:       evABI,
		address:        client.contractAddress,
		topics:         topics,
		checkpointPath: checkpointPath,
		confirmations:  confirmations,
		batchSize:      batchSize,
		pollInterval:   time.Duration(pollIntervalSec * float64(time.Second)),
		seenKeys:       make(map[string]bool),
		fromBlock:      big.NewInt(0),
	}

	// 4. Load Checkpoint
	watcher.loadCheckpoint()

	return watcher, nil
}

func (ew *EventWatcher) loadCheckpoint() {
	if _, err := os.Stat(ew.checkpointPath); err == nil {
		fileBytes, err := os.ReadFile(ew.checkpointPath)
		if err == nil {
			var cp CheckpointData
			if err := json.Unmarshal(fileBytes, &cp); err == nil {
				ew.fromBlock = big.NewInt(int64(cp.FromBlock))
				for _, k := range cp.SeenKeys {
					ew.seenKeys[k] = true
				}
				logger.Debug("Loaded checkpoint for %s: from_block=%d, seen=%d", ew.eventName, cp.FromBlock, len(cp.SeenKeys))
				return
			}
		}
	}
	// Fallback logic
	header, err := ew.client.client.HeaderByNumber(context.Background(), nil)
	if err == nil && header != nil {
		ew.fromBlock = header.Number
	}
}

func (ew *EventWatcher) saveCheckpoint() {
	dir := filepath.Dir(ew.checkpointPath)
	if dir != "" {
		os.MkdirAll(dir, 0755)
	}

	keys := make([]string, 0, len(ew.seenKeys))
	for k := range ew.seenKeys {
		keys = append(keys, k)
	}

	cp := CheckpointData{
		FromBlock: ew.fromBlock.Uint64(),
		SeenKeys:  keys,
	}

	bytes, _ := json.MarshalIndent(cp, "", "  ")
	_ = os.WriteFile(ew.checkpointPath, bytes, 0644)
}

func (ew *EventWatcher) latestSafeBlock() *big.Int {
	header, err := ew.client.client.HeaderByNumber(context.Background(), nil)
	if err != nil {
		return big.NewInt(0)
	}
	current := header.Number.Uint64()
	if current < ew.confirmations {
		return big.NewInt(0)
	}
	return big.NewInt(int64(current - ew.confirmations))
}

func (ew *EventWatcher) uniqueKey(log types.Log) string {
	return fmt.Sprintf("%s-%d", log.BlockHash.Hex(), log.Index)
}

// Run starts the infinite polling loop. It blocks, so run in a goroutine.
func (ew *EventWatcher) Run(handler EventHandler) {
	logger.Info("EventWatcher start: event=%s address=%s from_block=%s confirm=%d",
		ew.eventName, ew.address.Hex(), ew.fromBlock.String(), ew.confirmations)

	for {
		safeTo := ew.latestSafeBlock()

		// If fromBlock > safeTo, we are ahead of safe chain, wait.
		if ew.fromBlock.Cmp(safeTo) > 0 {
			time.Sleep(ew.pollInterval)
			continue
		}

		// Loop to catch up range by range
		for ew.fromBlock.Cmp(safeTo) <= 0 {
			// toBlock = min(fromBlock + batchSize - 1, safeTo)
			batch := new(big.Int).SetUint64(ew.batchSize - 1)
			toBlock := new(big.Int).Add(ew.fromBlock, batch)
			if toBlock.Cmp(safeTo) > 0 {
				toBlock.Set(safeTo)
			}

			// Fetch Logs
			query := ethereum.FilterQuery{
				FromBlock: ew.fromBlock,
				ToBlock:   toBlock,
				Addresses: []common.Address{ew.address},
				Topics:    ew.topics,
			}

			logs, err := ew.client.client.FilterLogs(context.Background(), query)
			if err != nil {
				logger.Error("Watcher fetch error: %v. Backing off...", err)
				time.Sleep(3 * time.Second)
				break // break inner loop to retry
			}

			// Sort logs strictly by BlockNumber then Index
			sort.Slice(logs, func(i, j int) bool {
				if logs[i].BlockNumber == logs[j].BlockNumber {
					return logs[i].Index < logs[j].Index
				}
				return logs[i].BlockNumber < logs[j].BlockNumber
			})

			// Process
			for _, log := range logs {
				key := ew.uniqueKey(log)
				if ew.seenKeys[key] {
					continue
				}

				// --- Full Argument Decoding ---
				parsedData := make(map[string]interface{})

				// 1. Unpack non-indexed fields (Data)
				if len(log.Data) > 0 {
					if err := ew.eventABI.Inputs.UnpackIntoMap(parsedData, log.Data); err != nil {
						logger.Error("Failed to unpack non-indexed args for %s: %v", ew.eventName, err)
						continue
					}
				}

				// 2. Unpack indexed fields (Topics)
				// Topic[0] is signature. Args start at Topic[1].
				topicIndex := 1
				for _, arg := range ew.eventABI.Inputs {
					if arg.Indexed {
						if topicIndex >= len(log.Topics) {
							break
						}
						
						// Manual extraction based on type
						switch arg.Type.T {
						case abi.AddressTy:
							parsedData[arg.Name] = common.HexToAddress(log.Topics[topicIndex].Hex())
						case abi.IntTy, abi.UintTy:
							parsedData[arg.Name] = log.Topics[topicIndex].Big()
						case abi.BoolTy:
							parsedData[arg.Name] = (log.Topics[topicIndex].Big().Cmp(big.NewInt(0)) != 0)
						default:
							// Fallback: raw hash for strings/bytes/arrays
							parsedData[arg.Name] = log.Topics[topicIndex]
						}
						topicIndex++
					}
				}

				// Handle
				if err := handler(log, parsedData); err != nil {
					logger.Error("Handler error for log %s: %v", key, err)
				}
				ew.seenKeys[key] = true
			}

			// Clear seen keys and advance
			ew.seenKeys = make(map[string]bool)
			ew.fromBlock.Add(toBlock, big.NewInt(1))
			ew.saveCheckpoint()

			// Refresh safeTo
			safeTo = ew.latestSafeBlock()
		}

		time.Sleep(ew.pollInterval)
	}
}