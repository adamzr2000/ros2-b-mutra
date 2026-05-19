package storage

import (
	"context"
	"encoding/json"
	"fmt"
	"os"

	"github.com/redis/go-redis/v9"

	"app/internal/logger"
)

// RedisStorageBackend implements Redis-based storage
type RedisStorageBackend struct {
	client *redis.Client
	ctx    context.Context
}

// NewRedisStorageBackend creates a new Redis storage backend
func NewRedisStorageBackend(host string, port int, db int) *RedisStorageBackend {
	if host == "" {
		host = "localhost"
	}
	if port == 0 {
		port = 6379
	}

	client := redis.NewClient(&redis.Options{
		Addr: fmt.Sprintf("%s:%d", host, port),
		DB:   db,
	})

	return &RedisStorageBackend{
		client: client,
		ctx:    context.Background(),
	}
}

func (r *RedisStorageBackend) Set(key string, value interface{}) error {
	data, err := json.Marshal(value)
	if err != nil {
		return err
	}
	return r.client.Set(r.ctx, key, string(data), 0).Err()
}

// Get intelligently retrieves data based on the Redis type (String, List, or Hash)
func (r *RedisStorageBackend) Get(key string) (interface{}, error) {
	// 1. Check Key Type
	keyType, err := r.client.Type(r.ctx, key).Result()
	if err != nil {
		return nil, err
	}

	switch keyType {
	case "none":
		return nil, nil // Key does not exist

	case "string":
		// Standard JSON blob (from Set) or Counter (from Incr)
		val, err := r.client.Get(r.ctx, key).Result()
		if err != nil {
			return nil, err
		}
		var result interface{}
		// Try to unmarshal as JSON. If it fails, return raw string/int logic?
		// Usually Incr stores "1", which unmarshals to float64(1).
		if err := json.Unmarshal([]byte(val), &result); err != nil {
			return val, nil // Return raw string if not JSON
		}
		return result, nil

	case "list":
		// Created by PushFinalDigest
		items, err := r.client.LRange(r.ctx, key, 0, -1).Result()
		if err != nil {
			return nil, err
		}
		var list []map[string]interface{}
		for _, item := range items {
			var entry map[string]interface{}
			if err := json.Unmarshal([]byte(item), &entry); err == nil {
				list = append(list, entry)
			}
		}
		return list, nil

	case "hash":
		// Created by StoreAgentInfo
		all, err := r.client.HGetAll(r.ctx, key).Result()
		if err != nil {
			return nil, err
		}
		// Reconstruct the map. We assume values were stored as JSON strings
		result := make(map[string]interface{})
		for k, v := range all {
			var fieldVal interface{}
			if err := json.Unmarshal([]byte(v), &fieldVal); err == nil {
				result[k] = fieldVal
			} else {
				result[k] = v
			}
		}
		return result, nil

	default:
		return nil, fmt.Errorf("unsupported redis type: %s", keyType)
	}
}

func (r *RedisStorageBackend) Delete(key string) error {
	return r.client.Del(r.ctx, key).Err()
}

func (r *RedisStorageBackend) Exists(key string) bool {
	result, err := r.client.Exists(r.ctx, key).Result()
	if err != nil {
		return false
	}
	return result == 1
}

func (r *RedisStorageBackend) SetStatus(key, value string) error {
	// Store as simple JSON string for consistency
	return r.Set(key, value)
}

func (r *RedisStorageBackend) StoreAgentInfo(key string, info map[string]interface{}) error {
	// Store as Hash so we can update partial fields if needed, 
	// or just to match the structure implied by your provided code.
	fields := make(map[string]interface{})
	for k, v := range info {
		data, err := json.Marshal(v)
		if err != nil {
			return err
		}
		fields[k] = string(data)
	}
	return r.client.HSet(r.ctx, key, fields).Err()
}

func (r *RedisStorageBackend) PushFinalDigest(key string, digestData map[string]interface{}) error {
	data, err := json.Marshal(digestData)
	if err != nil {
		return err
	}
	// Use RPUSH to append to end (Python lists usually append). LPUSH prepends.
	// Memory backend uses append(). So RPUSH is more accurate to memory.go behavior.
	return r.client.RPush(r.ctx, key, string(data)).Err()
}

func (r *RedisStorageBackend) IncrDigestCount(key string) (int, error) {
	result, err := r.client.Incr(r.ctx, key).Result()
	if err != nil {
		return 0, err
	}
	return int(result), nil
}

func (r *RedisStorageBackend) ExportToFile(filePath string) error {
	// Get all keys
	keys, err := r.client.Keys(r.ctx, "*").Result()
	if err != nil {
		return err
	}

	data := make(map[string]interface{})
	for _, key := range keys {
		// Use our Smart Get() to handle Lists/Hashes correctly
		val, err := r.Get(key)
		if err != nil {
			logger.Warn("Failed to get key %s: %v", key, err)
			continue
		}
		data[key] = val
	}

	jsonData, err := json.MarshalIndent(data, "", "  ")
	if err != nil {
		return err
	}

	return os.WriteFile(filePath, jsonData, 0644)
}

func (r *RedisStorageBackend) LoadFromFile(filePath string) error {
	data, err := os.ReadFile(filePath)
	if err != nil {
		return err
	}

	var kvData map[string]interface{}
	if err := json.Unmarshal(data, &kvData); err != nil {
		return err
	}

	// Note: Loading complex types (Lists/Hashes) from a flat JSON export 
	// back into Redis structures is hard because we lost the type info in export.
	// For simplicity, we just Set() them as strings. 
	// If full restoration is needed, ExportToFile needs to save metadata about types.
	for k, v := range kvData {
		if err := r.Set(k, v); err != nil {
			logger.Warn("Failed to set key %s: %v", k, err)
		}
	}

	return nil
}

func (r *RedisStorageBackend) Close() error {
	return r.client.Close()
}