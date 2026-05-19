package storage

import (
	"encoding/json"
	"os"
	"sync"

	"app/internal/logger"
)

// MemoryStorageBackend implements in-memory storage
type MemoryStorageBackend struct {
	data        map[string]interface{}
	mu          sync.RWMutex
	storageFile string
	modCount    int
}

// NewMemoryStorageBackend creates a new memory storage backend
func NewMemoryStorageBackend() *MemoryStorageBackend {
	return &MemoryStorageBackend{
		data:        make(map[string]interface{}),
		storageFile: os.Getenv("MEMORY_STORAGE_FILE"),
	}
}

func (m *MemoryStorageBackend) Set(key string, value interface{}) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.data[key] = value
	return nil
}

func (m *MemoryStorageBackend) Get(key string) (interface{}, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.data[key], nil
}

func (m *MemoryStorageBackend) Delete(key string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	delete(m.data, key)
	return nil
}

func (m *MemoryStorageBackend) Exists(key string) bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	_, exists := m.data[key]
	return exists
}

func (m *MemoryStorageBackend) SetStatus(key, value string) error {
	return m.Set(key, value)
}

func (m *MemoryStorageBackend) StoreAgentInfo(key string, info map[string]interface{}) error {
	return m.Set(key, info)
}

func (m *MemoryStorageBackend) PushFinalDigest(key string, digestData map[string]interface{}) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	var list []map[string]interface{}
	if existing, ok := m.data[key]; ok {
		if existingList, ok := existing.([]map[string]interface{}); ok {
			list = existingList
		}
	}

	list = append(list, digestData)
	m.data[key] = list
	return nil
}

func (m *MemoryStorageBackend) IncrDigestCount(key string) (int, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	count := 0
	if existing, ok := m.data[key]; ok {
		if existingCount, ok := existing.(int); ok {
			count = existingCount
		}
	}

	count++
	m.data[key] = count

	// Auto-save every 10 increments
	m.modCount++
	if m.modCount%10 == 0 && m.storageFile != "" {
		m.saveToFileIfConfigured()
	}

	return count, nil
}

func (m *MemoryStorageBackend) ExportToFile(filePath string) error {
	m.mu.RLock()
	defer m.mu.RUnlock()

	data, err := json.MarshalIndent(m.data, "", "  ")
	if err != nil {
		logger.Error("Failed to marshal storage data: %v", err)
		return err
	}

	if err := os.WriteFile(filePath, data, 0644); err != nil {
		logger.Error("Failed to export storage data to %s: %v", filePath, err)
		return err
	}

	logger.Debug("Storage data exported to %s", filePath)
	return nil
}

func (m *MemoryStorageBackend) LoadFromFile(filePath string) error {
	data, err := os.ReadFile(filePath)
	if err != nil {
		logger.Error("Failed to load storage data from %s: %v", filePath, err)
		return err
	}

	m.mu.Lock()
	defer m.mu.Unlock()

	if err := json.Unmarshal(data, &m.data); err != nil {
		logger.Error("Failed to unmarshal storage data: %v", err)
		return err
	}

	logger.Info("Storage data loaded from %s", filePath)
	return nil
}

func (m *MemoryStorageBackend) saveToFileIfConfigured() {
	if m.storageFile == "" {
		return
	}

	// Note: caller should already hold the lock
	data, err := json.MarshalIndent(m.data, "", "  ")
	if err != nil {
		logger.Error("Failed to auto-save memory storage: %v", err)
		return
	}

	if err := os.WriteFile(m.storageFile, data, 0644); err != nil {
		logger.Error("Failed to auto-save memory storage: %v", err)
	}
}
