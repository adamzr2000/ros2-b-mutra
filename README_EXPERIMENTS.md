# Experiments

How to run each experiment for the final results (contract V3). Each one is a
script `experiment_<name>.py` whose data goes to `experiments/data/<name>/`.

## Before the final runs

- Delete the test data: `rm -rf experiments/data/{attestation-protocol,blockchain-impact,robot-impact,tamper-detection}`
  (keep `attestation-sc/`, `tamper-impact/` and `toctou/` if their runs are the final ones).
- If the sidecar image was rebuilt (`cd dockerfiles && make attestation-sidecar`), run `./start.sh` again:
  the references must match the new binary.
- One experiment at a time: a second one refuses to start.
- Every run writes a `meta.json`. Check `problems` (empty when the run is consistent) and
  `gas_estimate_retries` (Besu estimates retried, +~100 ms each: flag the run if needed).
- Long runs survive a closed SSH session if started detached:
  `setsid nohup python3 -u experiment_<name>.py ... > experiments/data/.<name>.log 2>&1 < /dev/null &`
- Where they run: experiments 1 and 3 put the robots (dummy) and their sidecars on nextnet
  (`--remote`: `nextnet@10.5.1.20`, set in `start.sh`), Besu, the SECaaS and monitoring on this
  host; the experiments read the hosts from `.env`. The others need Gazebo and the chain on one
  host and run locally. Before the first remote run, check that nextnet reaches this host's
  ports 21001-21004 and 22001-22004, and rebuild the sidecar image there.

## 1. Attestation protocol: latency and resources (~40 min)

```bash
./start.sh --dummy --robots 4 --remote
python3 experiment_attestation_protocol.py --phase continuous --runs 20 --rounds 10
```

`--phase bootstrap|both` adds the first rounds after a chain reset, judged by the SECaaS.

## 2. Robot impact: effect of the measurement rate on the robot (~25 min)

```bash
./start.sh --robots 1 --no-cpu-limit --ssp 10 --iterq 500
python3 experiment_robot_impact.py --runs 3 --window 60
```

Baseline and SSP 10 s, 5 s, 1 s, 100 ms, 10 ms.

## 3. Blockchain impact: ledger growth and validators (~50 min per N)

```bash
for n in 4 8 16 32 64; do
  ./stop.sh && ./start.sh --dummy --robots $n --ssp 10000 --iterq 10 --remote
  python3 experiment_blockchain_impact.py
done
```

Conditions idle, K1, K5, K10 at SSP 10 s, 600 s each. Up to 64 robots on nextnet; above 64,
`REMOTE2_HOST` must be set in `start.sh`.

## 4. Smart contract: gas per function and path (~2 min, no stack needed)

```bash
python3 experiment_attestation_sc.py
```

## 5. Tamper detection: latency (~25 min)

```bash
./start.sh --robots 4 --iterq 1
python3 experiment_tamper_detection.py
```

SSP 5, 10, 20, 40 s; 10 trials each.

## 6. Tamper impact: formation mission (~2 h)

```bash
python3 experiment_tamper_impact.py --reps 3      # add --gpu on a host with an NVIDIA GPU
```

Redeploys the stack itself for every run. Scenarios: baseline, unmitigated (SSP 10 s),
mitigated (SSP 10 s and 40 s).

## 7. TOCTOU: tamper-run-restore attacks (~2 h)

```bash
./start.sh --robots 4 --iterq 1
python3 experiment_toctou.py --trials 20
```

Fixed and random (exp) SSP of 5 s and 20 s; tau/SSP 0.05 to 2.

## After

```bash
./stop.sh
```
