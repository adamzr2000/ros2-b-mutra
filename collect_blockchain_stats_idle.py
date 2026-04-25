#!/usr/bin/env python3
"""Collect blockchain stats for an idle window.

Flow:
1) Verify collector health endpoint is reachable.
2) Start monitoring with the provided settings.
3) Wait for N seconds (default: 120).
4) Stop monitoring.
"""

import argparse
import json
import sys
import time

import requests


def _post_json(url: str, payload: dict | None = None, timeout: int = 10) -> dict:
	resp = requests.post(url, json=payload, timeout=timeout)
	resp.raise_for_status()
	try:
		return resp.json()
	except ValueError:
		return {"raw": resp.text}


def _get_json(url: str, timeout: int = 5) -> dict:
	resp = requests.get(url, timeout=timeout)
	resp.raise_for_status()
	try:
		return resp.json()
	except ValueError:
		return {"raw": resp.text}


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Collect idle Ethereum blockchain stats for a fixed duration.",
	)
	parser.add_argument(
		"--collector-url",
		default="http://localhost:7000",
		help="Eth stats collector base URL (default: http://localhost:7000)",
	)
	parser.add_argument(
		"--rpc-url",
		default="http://host.docker.internal:21001",
		help="Ethereum RPC URL used by the collector.",
	)
	parser.add_argument(
		"--poll-interval",
		type=float,
		default=1.0,
		help="Polling interval in seconds (default: 1.0)",
	)
	parser.add_argument(
		"--csv-dir",
		default="/experiments/data/blockchain-stats/results/idle",
		help="CSV output directory from the collector container perspective.",
	)
	parser.add_argument(
		"--csv-name",
		default=None,
		help=(
			"CSV file name (with or without .csv). "
			"If omitted, uses blockchain-idle-<seconds>s.csv"
		),
	)
	parser.add_argument(
		"--seconds",
		type=int,
		default=120,
		help="Idle collection duration in seconds (default: 120)",
	)
	return parser.parse_args()


def main() -> int:
	args = parse_args()

	if args.seconds <= 0:
		print("Error: --seconds must be > 0", file=sys.stderr)
		return 2

	base = args.collector_url.rstrip("/")
	health_url = f"{base}/"
	start_url = f"{base}/monitor/start"
	stop_url = f"{base}/monitor/stop"

	csv_name = args.csv_name or f"blockchain-idle-{args.seconds}s.csv"

	payload = {
		"rpc_url": args.rpc_url,
		"poll_interval": args.poll_interval,
		"csv_dir": args.csv_dir,
		"csv_name": csv_name,
	}

	started = False
	try:
		print(f"[1/4] Health check -> {health_url}")
		health = _get_json(health_url, timeout=5)
		print(json.dumps(health, indent=2))

		print("[2/4] Starting monitor")
		start_resp = _post_json(start_url, payload=payload, timeout=10)
		started = True
		print(json.dumps(start_resp, indent=2))

		print(f"[3/4] Collecting idle stats for {args.seconds}s...")
		time.sleep(args.seconds)

		print("[4/4] Stopping monitor")
		stop_resp = _post_json(stop_url, timeout=20)
		print(json.dumps(stop_resp, indent=2))
		return 0

	except requests.RequestException as exc:
		print(f"HTTP error: {exc}", file=sys.stderr)
		if started:
			try:
				print("Attempting cleanup: stop monitor...")
				cleanup = _post_json(stop_url, timeout=20)
				print(json.dumps(cleanup, indent=2))
			except Exception as cleanup_exc:
				print(f"Cleanup failed: {cleanup_exc}", file=sys.stderr)
		return 1
	except KeyboardInterrupt:
		print("Interrupted by user.", file=sys.stderr)
		if started:
			try:
				print("Attempting cleanup: stop monitor...")
				cleanup = _post_json(stop_url, timeout=20)
				print(json.dumps(cleanup, indent=2))
			except Exception as cleanup_exc:
				print(f"Cleanup failed: {cleanup_exc}", file=sys.stderr)
		return 130


if __name__ == "__main__":
	raise SystemExit(main())
