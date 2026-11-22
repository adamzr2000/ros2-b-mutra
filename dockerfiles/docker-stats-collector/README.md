# Resource collector

Samples CPU, memory and network of containers from the Docker stats stream,
about once per second each, and keeps the samples in memory for the
experiment runner (`experiment_attestation_protocol.py`). See `app/main.py` for the API and
the columns. One runs on every host with containers to measure (port 6000).
