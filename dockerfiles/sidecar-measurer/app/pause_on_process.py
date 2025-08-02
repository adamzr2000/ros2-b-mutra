import time
import subprocess
import os
import logging

# === Logging Setup ===
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
if LOG_LEVEL == "NONE":
    logging.disable(logging.CRITICAL)
else:
    numeric_level = getattr(logging, LOG_LEVEL, logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )

def pause_container_on_proc1():
    container = os.environ.get("CONTAINER", "")
    proc_comm = os.environ.get("PROC_COMM", "")

    logging.info(f"Surveillance de /proc/1/comm pour {container}, attente de : {proc_comm}")
    cur_proc_comm = ""
    # Boucle jusqu'à ce que /proc/1/comm == proc_comm
    while True:
        try:
            with open("/proc/1/comm", "r") as f:
                cur_proc_comm = f.read().strip()
            logging.debug(f"Processus actuel (PID 1): {cur_proc_comm}")
            if cur_proc_comm == proc_comm:
                logging.info(f"Process {proc_comm} détecté (PID 1). Mise en pause du container {container}.")
                subprocess.run(["docker", "pause", container])
                break
        except Exception as e:
            logging.warning(f"Erreur lors de la lecture de /proc/1/comm : {e}")
        time.sleep(1)
