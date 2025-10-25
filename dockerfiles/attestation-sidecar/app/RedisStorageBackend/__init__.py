# RedisStorageBackend implementation
import redis
import json
from typing import Any, Dict, Optional

class RedisStorageBackend:
    def __init__(self, host: str = 'localhost', port: int = 6379, db: int = 0):
        self.client = redis.Redis(host=host, port=port, db=db)

    def set(self, key: str, value: Any) -> None:
        self.client.set(key, json.dumps(value))

    def get(self, key: str) -> Optional[Any]:
        value = self.client.get(key)
        if value is not None:
            return json.loads(value)
        return None

    def delete(self, key: str) -> None:
        self.client.delete(key)

    def exists(self, key: str) -> bool:
        return self.client.exists(key) == 1
        
    def set_status(self, key: str, value: str) -> None:
        self.client.set(key, value)
    
    def store_agent_info(self, key: str, info: Dict[str, Any]) -> None:
        self.client.hmset(key, {k: json.dumps(v) for k, v in info.items()})
    
    def push_final_digest(self, key: str, digest_data: Dict[str, Any]) -> None:
        self.client.lpush(key, json.dumps(digest_data))
    
    def incr_digest_count(self, key: str) -> int:
        return self.client.incr(key)
