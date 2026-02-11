#!/usr/bin/env python3
"""
Database Client (PostgreSQL) for querying reference digests and deployment configs.
"""

from typing import Optional, Dict
import psycopg2
import time
from psycopg2 import pool
from app.internal.logger import info, warn, error, debug

class DatabaseClient:
    def __init__(self, host: str, port: int, database: str, user: str, password: str):
        self.params = {
            "host": host,
            "port": port,
            "database": database,
            "user": user,
            "password": password
        }
        self.connection_pool = None
        self._initialize_pool()
        
    def _initialize_pool(self):
        """Initialize connection pool with a retry loop for Docker environments."""
        max_retries = 10
        delay = 2  # seconds
        
        for i in range(max_retries):
            try:
                self.connection_pool = psycopg2.pool.SimpleConnectionPool(
                    minconn=1,
                    maxconn=10,
                    **self.params
                )
                info(f"Successfully connected to DB on attempt {i+1}")
                return
            except Exception as e:
                warn(f"Database not ready (attempt {i+1}/{max_retries}): {e}")
                time.sleep(delay)
        
        error("Final attempt failed. Could not connect to database.")
        self.connection_pool = None
            
    def get_ref_signatures(self, eth_address: str) -> Optional[Dict[str, str]]:
        """
        Fetches the triplet of reference hashes for a specific agent.
        """
        if not self.connection_pool:
            return None
            
        # Normalize address for lookup
        addr = eth_address.lower() if eth_address.startswith("0x") else f"0x{eth_address.lower()}"
        
        conn = None
        try:
            conn = self.connection_pool.getconn()
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT prover_hash, verifier_hash, robot_hash 
                    FROM measures 
                    WHERE LOWER(eth_address) = %s
                """, (addr,))
                
                row = cur.fetchone()
                if row:
                    return {
                        "prover_hash": row[0],
                        "verifier_hash": row[1],
                        "robot_hash": row[2]
                    }
                return None
        except Exception as e:
            print(f"[DBClient] Query error: {e}")
            return None
        finally:
            if conn:
                self.connection_pool.putconn(conn)


    def add_ref_signatures(self, eth_address: str, prover_hash: str, verifier_hash: str, robot_hash: str) -> bool:
        """
        Adds or updates reference hashes for an agent (prover_hash, verifier_hash, robot_hash).
        """
        if not self.connection_pool: return False

        addr = eth_address.lower() if eth_address.startswith("0x") else f"0x{eth_address.lower()}"
        
        conn = None
        try:
            conn = self.connection_pool.getconn()
            with conn.cursor() as cur:
                # We unpack the list here for the SQL query
                cur.execute("""
                    INSERT INTO measures (eth_address, prover_hash, verifier_hash, robot_hash)
                    VALUES (%s, %s, %s, %s)
                    ON CONFLICT (eth_address) 
                    DO UPDATE SET 
                        prover_hash = EXCLUDED.prover_hash,
                        verifier_hash = EXCLUDED.verifier_hash,
                        robot_hash = EXCLUDED.robot_hash,
                        updated_at = CURRENT_TIMESTAMP
                """, (addr, prover_hash, verifier_hash, robot_hash))
                conn.commit()
                info(f"Updated signatures for: {addr}")
                return True
        except Exception as e:
            error(f"Upsert failed for {addr}: {e}")
            if conn: conn.rollback()
            return False
        finally:
            if conn: self.connection_pool.putconn(conn)

    def close(self):
        if self.connection_pool:
            self.connection_pool.closeall()
            info("DB pool closed")