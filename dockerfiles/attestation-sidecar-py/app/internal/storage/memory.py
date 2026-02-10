# app/internal/storage/memory.py

import threading
import json
import os
from typing import Any, Dict, Optional
from app.internal.logger import info, warn, error, debug, green_text

class MemoryStorageBackend:
    def __init__(self):
        self._data: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._storage_file = os.getenv("MEMORY_STORAGE_FILE", "")
        
        # Charger les données depuis le fichier si spécifié
        # if self._storage_file and os.path.exists(self._storage_file):
        #     self.load_from_file(self._storage_file)

    def unpack_set(self, **kwargs: Any) -> None:
        with self._lock:
            self._data.update(kwargs)

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._data[key] = value

    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            return self._data.get(key)

    def delete(self, key: str) -> None:
        with self._lock:
            if key in self._data:
                del self._data[key]

    def exists(self, key: str) -> bool:
        with self._lock:
            return key in self._data
            
    def set_status(self, key: str, value: str) -> None:
        self.set(key, value)
    
    def store_agent_info(self, key: str, info: Dict[str, Any]) -> None:
        self.set(key, info)
    
    def push_final_digest(self, key: str, digest_data: Dict[str, Any]) -> None:
        if key not in self._data:
            with self._lock:
                self._data[key] = []
        with self._lock:
            self._data[key].append(digest_data)
    
    def incr_digest_count(self, key: str) -> int:
        with self._lock:
            if key not in self._data:
                self._data[key] = 0
            self._data[key] += 1
            self._save_to_file_if_configured()
            return self._data[key]
    
    def export_to_file(self, file_path: str) -> bool:
        """Exporte toutes les données du stockage vers un fichier JSON"""
        try:
            with self._lock:
                with open(file_path, 'w') as f:
                    json.dump(self._data, f, indent=2)
            debug(f"Storage data exported to {file_path}")
            return True
        except Exception as e:
            error(f"Failed to export storage data to {file_path}: {str(e)}")
            return False
    
    def load_from_file(self, file_path: str) -> bool:
        """Charge les données depuis un fichier JSON vers le stockage"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            with self._lock:
                self._data = data
            info(f"Storage data loaded from {file_path}")
            return True
        except Exception as e:
            error(f"Failed to load storage data from {file_path}: {str(e)}")
            return False
    
    def _save_to_file_if_configured(self):
        """Sauvegarde les données dans le fichier configuré si spécifié
        Optimisé pour ne pas sauvegarder à chaque incrément (seulement tous les 10)
        """
        if not self._storage_file:
            return
            
        # Récupérer ou initialiser le compteur de modifications
        with self._lock:
            mod_count = self._data.get('_mod_count', 0) + 1
            self._data['_mod_count'] = mod_count
            
            # Sauvegarder seulement tous les 10 incréments pour éviter trop d'I/O
            if mod_count % 10 == 0:
                try:
                    debug(f"Saving memory storage to {self._storage_file} (modification count: {mod_count})")
                    self.export_to_file(self._storage_file)
                except Exception as e:
                    error(f"Failed to auto-save memory storage: {str(e)}")
                    # Continuer l'exécution même en cas d'erreur
