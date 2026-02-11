package storage

// StorageBackend defines the interface for storage operations
type StorageBackend interface {
	Set(key string, value interface{}) error
	Get(key string) (interface{}, error)
	Delete(key string) error
	Exists(key string) bool
	SetStatus(key, value string) error
	StoreAgentInfo(key string, info map[string]interface{}) error
	PushFinalDigest(key string, digestData map[string]interface{}) error
	IncrDigestCount(key string) (int, error)
	ExportToFile(filePath string) error
	LoadFromFile(filePath string) error
}

// DigestData represents the data stored for a final digest
type DigestData struct {
	FinalDigest string  `json:"final_digest"`
	Threshold   int     `json:"threshold"`
	Timestamp   float64 `json:"timestamp"`
}
