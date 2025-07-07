import redis
import json
import os
from datetime import datetime

class RedisLogger:
    def __init__(self):
        self.redis_client = redis.Redis(
            host=os.getenv('REDIS_HOST', 'redis'),
            port=int(os.getenv('REDIS_PORT', 6379)),
            decode_responses=True
        )
        
    def log_operation(self, operation_type, message):
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'type': operation_type,
            'message': message
        }
        self.redis_client.rpush('logs:wrapper', json.dumps(log_entry))
        
    def log_error(self, error_message, context=None):
        error_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'error': error_message,
            'context': context
        }
        self.redis_client.rpush('logs:wrapper', json.dumps(error_entry))
        
    def get_operation_logs(self):
        logs = self.redis_client.lrange('logs:wrapper', 0, -1)
        return [json.loads(log) for log in logs]
        
    def get_error_logs(self):
        logs = self.redis_client.lrange('logs:wrapper', 0, -1)
        return [json.loads(log) for log in logs]
