#!/usr/bin/env python3
"""
Generates docker-compose files for a given number of robots (1–128).

LOCAL MODE (default):
  Everything runs on d-mutra.
  N ≤ GAZEBO_LIMIT = 4: Gazebo mode (gazebo-server + turtlebots on ros-net).
  N > GAZEBO_LIMIT:     dummy mode (namespaced ROS2 dummy_publisher, no Gazebo).
  Always outputs docker-compose.robots.yml and docker-compose.attestation.yml.

REMOTE MODE (--mode remote):
  d-mutra hosts Besu + SECaaS + monitoring only.
  All robots + sidecars run on remote host(s).
  N ≤ GAZEBO_LIMIT: Gazebo mode on remote1.
  N > GAZEBO_LIMIT: dummy mode on remote1 (and remote2 if N > REMOTE1_LIMIT).
  Outputs under generated/:
      docker-compose.robots-N{N}-remote1.yml
      docker-compose.attestation-N{N}-remote1.yml
      docker-compose.robots-N{N}-remote2.yml        (only when N > REMOTE1_LIMIT)
      docker-compose.attestation-N{N}-remote2.yml   (only when N > REMOTE1_LIMIT)

Usage:
    python3 generate_compose.py --robots 4             # local, Gazebo
    python3 generate_compose.py --robots 32            # local, dummy
    python3 generate_compose.py --robots 4  --mode remote  # remote, Gazebo on remote1
    python3 generate_compose.py --robots 64 --mode remote  # remote, dummy on remote1

Called automatically by start.sh via --robots N --mode MODE.
"""
import argparse
import os
import sys

MAX_ROBOTS    = 128
GAZEBO_LIMIT  = 4    # N ≤ 4: real Gazebo simulation; N > 4: dummy mode
REMOTE1_LIMIT = 64   # remote mode: robots 1..64 on remote1
REMOTE2_LIMIT = 128  # remote mode: robots 65..128 on remote2

CONTRACT_TO_IMAGE = {
    "AttestationManager":          "attestation-sidecar:latest",
    "AttestationManagerOptimized": "attestation-sidecar-optimized:latest",
}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def robot_position(i: int, spacing: int = 5):
    """Grid layout: 10 robots per row, `spacing` units apart."""
    row = (i - 1) // 10
    col = (i - 1) % 10
    return col * spacing, row * spacing


# ── Robot compose generators ──────────────────────────────────────────────────

def generate_robots_yml_gazebo(n: int) -> str:
    """N ≤ GAZEBO_LIMIT: gazebo-server + N turtlebots on ros-net bridge."""
    lines = [
        "# docker-compose.robots.yml",
        f"# AUTO-GENERATED for {n} robot(s) [Gazebo mode] — edit generate_compose.py, not this file.",
        "",
        "services:",
        "  gazebo-server:",
        "    image: turtlebot3-gazebo",
        "    container_name: gazebo-server",
        "    hostname: gazebo-server",
        "    environment:",
        "      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}",
        "      - GAZEBO_AUDIO_DEVICE=none",
        "    ports:",
        '      - "11345:11345"',
        "    tty: true",
        "    stdin_open: true",
        '    command: bash -c "./start_gazebo_server.sh"',
        "    networks:",
        "      - ros-net",
    ]

    for i in range(1, n + 1):
        x, y = robot_position(i)
        lines += [
            "",
            f"  robot{i}:",
            f"    image: turtlebot3-gazebo",
            f"    container_name: robot{i}",
            f"    hostname: robot{i}",
            f"    environment:",
            f"        - ROS_DOMAIN_ID=${{ROS_DOMAIN_ID}}",
            f"        - GAZEBO_MASTER_URI=http://gazebo-server:11345",
            f"        - NAMESPACE=robot{i}",
            f"        - X_POSE={x}",
            f"        - Y_POSE={y}",
            f'    command: bash -c "./start_turtlebot.sh"',
            f"    depends_on:",
            f"      - gazebo-server",
            f"    stdin_open: true",
            f"    tty: true",
            f"    networks:",
            f"      - ros-net",
        ]

    lines += [
        "",
        "networks:",
        "  ros-net:",
        "    driver: bridge",
        "    name: ros-net",
    ]

    return "\n".join(lines) + "\n"


def generate_robots_yml_dummy(n: int) -> str:
    """N > GAZEBO_LIMIT: each robot runs a namespaced ROS2 dummy_publisher.
    Loads the full rclcpp/rmw/FastDDS library stack (realistic memory map for
    attestation) without requiring Gazebo or a DDS discovery server.
    """
    lines = [
        "# docker-compose.robots.yml",
        f"# AUTO-GENERATED for {n} robot(s) [dummy mode] — edit generate_compose.py, not this file.",
        "",
        "services:",
    ]

    for i in range(1, n + 1):
        lines += [
            "",
            f"  robot{i}:",
            f"    image: turtlebot3-gazebo",
            f"    container_name: robot{i}",
            f"    hostname: robot{i}",
            f"    environment:",
            f"        - NAMESPACE=robot{i}",
            f'    command: bash -c "source /opt/ros/humble/setup.bash && source /home/agent/ros2_ws/install/setup.bash && ros2 launch c_example_package dummy_publisher.launch.py namespace:=robot{i}"',
            f"    stdin_open: true",
            f"    tty: true",
        ]

    return "\n".join(lines) + "\n"


def generate_robots_yml(n: int) -> str:
    if n <= GAZEBO_LIMIT:
        return generate_robots_yml_gazebo(n)
    return generate_robots_yml_dummy(n)


def generate_robots_remote_yml(n_start: int, n_end: int) -> str:
    """Remote robots: namespaced ROS2 dummy_publisher (same as local dummy mode)."""
    lines = [
        "# docker-compose.robots-remote.yml",
        f"# AUTO-GENERATED for robots {n_start}–{n_end} on remote host [dummy mode].",
        "# edit generate_compose.py, not this file.",
        "",
        "services:",
    ]

    for i in range(n_start, n_end + 1):
        lines += [
            "",
            f"  robot{i}:",
            f"    image: turtlebot3-gazebo",
            f"    container_name: robot{i}",
            f"    hostname: robot{i}",
            f"    environment:",
            f"        - NAMESPACE=robot{i}",
            f'    command: bash -c "source /opt/ros/humble/setup.bash && source /home/agent/ros2_ws/install/setup.bash && ros2 launch c_example_package dummy_publisher.launch.py namespace:=robot{i}"',
            f"    stdin_open: true",
            f"    tty: true",
        ]

    return "\n".join(lines) + "\n"


# ── Attestation compose generators ───────────────────────────────────────────

def generate_attestation_yml(n: int, start: int = 1, config_dir: str = "./config",
                             sidecar_image: str = "attestation-sidecar:latest") -> str:
    """Sidecars start..n on a single host. Reaches Besu via host-exposed ports."""
    lines = [
        "# docker-compose.attestation.yml",
        f"# AUTO-GENERATED for {n} robot(s) — edit generate_compose.py, not this file.",
        "",
        "services:",
    ]

    for i in range(start, n + 1):
        port = 8000 + i
        lines += [
            f"  robot{i}-sidecar:",
            f"    container_name: robot{i}-sidecar",
            f"    image: {sidecar_image}",
            f"    environment:",
            f"      - CONFIG_PATH=/config/robot{i}.json",
            f"      - EXPORT_RESULTS=${{EXPORT_RESULTS}}",
            f"      - RESULTS_DIR=${{RESULTS_DIR}}",
            f"      - USE_REDIS=${{USE_REDIS}}",
            f"      - ONE_SHOT=${{ONE_SHOT}}",
            f"      - AUTO_START=${{AUTO_START}}",
            f"      - WAIT_FOR_TX_CONFIRMATIONS=${{WAIT_FOR_TX_CONFIRMATIONS}}",
            f"      - WAIT_FOR_VERIFICATION_RESULT=${{WAIT_FOR_VERIFICATION_RESULT}}",
            f"      - MEMORY_STORAGE_FILE=/measurements/robot{i}_measurements.json",
            f"      - ATTESTATION_INTERVAL_MS=${{ATTESTATION_INTERVAL_MS}}",
            f"      - EVENT_CONFIRMATIONS=${{EVENT_CONFIRMATIONS}}",
            f"      - EVENT_POLL_INTERVAL=${{EVENT_POLL_INTERVAL}}",
            f"      - EVENT_BATCH_SIZE=${{EVENT_BATCH_SIZE}}",
            f"      - LOG_LEVEL=${{LOG_LEVEL}}",
            f"      - ENABLE_LIBS_HASH=${{ENABLE_LIBS_HASH}}",
            f"      - SELF_INTEGRITY_ENABLED=${{SELF_INTEGRITY_ENABLED}}",
            f"    ports:",
            f'      - "{port}:8000"',
            f"    volumes:",
            f"      - {config_dir}/robot{i}.json:/config/robot{i}.json",
            f"      - ./experiments/data/:/experiments/data:rw",
            f"      - /var/run/docker.sock:/var/run/docker.sock",
            f"      - ./measurements/:/measurements:rw",
            f"      - ./checkpoints/robot{i}-sidecar:/checkpoints",
            f'    pid: "container:robot{i}"',
            f"    cap_add:",
            f"      - SYS_PTRACE",
            f"    deploy:",
            f"      resources:",
            f"        limits:",
            f"          cpus: '${{CPU_LIMIT}}'",
            f"    extra_hosts:",
            f'      - "host.docker.internal:host-gateway"',
            f"",
        ]

    return "\n".join(lines) + "\n"


def generate_attestation_remote_yml(n_start: int, n_end: int, config_dir: str = "./config-dummy",
                                    blockchain_host: str = "",
                                    sidecar_image: str = "attestation-sidecar:latest") -> str:
    """Sidecars n_start..n_end on remote host.
    blockchain_host is local machine's IP reachable from the remote host; it is injected
    as the resolution target for host.docker.internal so the same config JSON works
    on both local and remote sidecars.
    """
    lines = [
        "# docker-compose.attestation-remote.yml",
        f"# AUTO-GENERATED for sidecars {n_start}–{n_end} on remote host.",
        "# edit generate_compose.py, not this file.",
        "",
        "services:",
    ]

    for i in range(n_start, n_end + 1):
        port = 8000 + i
        lines += [
            f"  robot{i}-sidecar:",
            f"    container_name: robot{i}-sidecar",
            f"    image: {sidecar_image}",
            f"    environment:",
            f"      - CONFIG_PATH=/config/robot{i}.json",
            f"      - EXPORT_RESULTS=${{EXPORT_RESULTS}}",
            f"      - RESULTS_DIR=${{RESULTS_DIR}}",
            f"      - USE_REDIS=${{USE_REDIS}}",
            f"      - ONE_SHOT=${{ONE_SHOT}}",
            f"      - AUTO_START=${{AUTO_START}}",
            f"      - WAIT_FOR_TX_CONFIRMATIONS=${{WAIT_FOR_TX_CONFIRMATIONS}}",
            f"      - WAIT_FOR_VERIFICATION_RESULT=${{WAIT_FOR_VERIFICATION_RESULT}}",
            f"      - MEMORY_STORAGE_FILE=/measurements/robot{i}_measurements.json",
            f"      - ATTESTATION_INTERVAL_MS=${{ATTESTATION_INTERVAL_MS}}",
            f"      - EVENT_CONFIRMATIONS=${{EVENT_CONFIRMATIONS}}",
            f"      - EVENT_POLL_INTERVAL=${{EVENT_POLL_INTERVAL}}",
            f"      - EVENT_BATCH_SIZE=${{EVENT_BATCH_SIZE}}",
            f"      - LOG_LEVEL=${{LOG_LEVEL}}",
            f"      - ENABLE_LIBS_HASH=${{ENABLE_LIBS_HASH}}",
            f"      - SELF_INTEGRITY_ENABLED=${{SELF_INTEGRITY_ENABLED}}",
            f"    ports:",
            f'      - "{port}:8000"',
            f"    volumes:",
            f"      - {config_dir}/robot{i}.json:/config/robot{i}.json",
            f"      - ./experiments/data/:/experiments/data:rw",
            f"      - /var/run/docker.sock:/var/run/docker.sock",
            f"      - ./measurements/:/measurements:rw",
            f"      - ./checkpoints/robot{i}-sidecar:/checkpoints",
            f'    pid: "container:robot{i}"',
            f"    cap_add:",
            f"      - SYS_PTRACE",
            f"    deploy:",
            f"      resources:",
            f"        limits:",
            f"          cpus: '${{CPU_LIMIT}}'",
        ]
        if blockchain_host:
            lines += [
                f"    extra_hosts:",
                f'      - "host.docker.internal:{blockchain_host}"',
            ]
        lines.append("")

    return "\n".join(lines) + "\n"


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Generate docker-compose files for N robots (1–{MAX_ROBOTS})"
    )
    parser.add_argument(
        "--robots", type=int, required=True,
        help=f"Number of robots (1–{MAX_ROBOTS}). "
             f"N≤{GAZEBO_LIMIT}: Gazebo mode (local). "
             f"N>{GAZEBO_LIMIT}: dummy mode (local).",
    )
    parser.add_argument(
        "--blockchain-host", default="",
        help="Local machine IP reachable from remote hosts (injected as host.docker.internal "
             "so remote sidecars can reach the Besu validators on d-mutra).",
    )
    parser.add_argument(
        "--mode", choices=["local", "remote"], default="local",
        help="Deployment mode. "
             "local (default): robots on d-mutra, split to remote1 when N > LOCAL_LIMIT. "
             "remote: d-mutra hosts Besu+SECaaS only; all robots+sidecars on remote host(s).",
    )
    parser.add_argument(
        "--contract", default="AttestationManager",
        choices=list(CONTRACT_TO_IMAGE.keys()),
        help="Smart contract in use — determines which sidecar image is written into compose files "
             "(default: AttestationManager → attestation-sidecar:latest).",
    )
    args = parser.parse_args()

    if not 1 <= args.robots <= MAX_ROBOTS:
        print(f"❌ --robots must be between 1 and {MAX_ROBOTS} (got {args.robots})")
        sys.exit(1)

    n = args.robots
    sidecar_image = CONTRACT_TO_IMAGE[args.contract]

    if args.mode == "remote":
        # ── REMOTE MODE: all robots on remote host(s) ─────────────────────────
        gen_dir = os.path.join(SCRIPT_DIR, "generated")
        os.makedirs(gen_dir, exist_ok=True)

        # Config dir depends on Gazebo vs dummy
        cfg_dir = "./config" if n <= GAZEBO_LIMIT else "./config-dummy"

        # Remote1: robots 1..min(n, REMOTE1_LIMIT)
        r1_end = min(n, REMOTE1_LIMIT)

        robots_r1_path = os.path.join(gen_dir, f"docker-compose.robots-N{n}-remote1.yml")
        if n <= GAZEBO_LIMIT:
            # Gazebo mode: use full Gazebo compose (includes ros-net bridge)
            with open(robots_r1_path, "w") as f:
                f.write(generate_robots_yml_gazebo(n))
            print(f"✅ {robots_r1_path}  (robots 1–{n}, Gazebo mode)")
        else:
            with open(robots_r1_path, "w") as f:
                f.write(generate_robots_remote_yml(1, r1_end))
            print(f"✅ {robots_r1_path}  (robots 1–{r1_end}, dummy mode)")

        attest_r1_path = os.path.join(gen_dir, f"docker-compose.attestation-N{n}-remote1.yml")
        with open(attest_r1_path, "w") as f:
            f.write(generate_attestation_remote_yml(1, r1_end, config_dir=cfg_dir,
                                                    blockchain_host=args.blockchain_host,
                                                    sidecar_image=sidecar_image))
        print(f"✅ {attest_r1_path}  (sidecars 1–{r1_end}, image: {sidecar_image})")

        # Remote2: robots REMOTE1_LIMIT+1..min(n, REMOTE2_LIMIT) (only when N > REMOTE1_LIMIT)
        if n > REMOTE1_LIMIT:
            r2_end = min(n, REMOTE2_LIMIT)

            robots_r2_path = os.path.join(gen_dir, f"docker-compose.robots-N{n}-remote2.yml")
            with open(robots_r2_path, "w") as f:
                f.write(generate_robots_remote_yml(REMOTE1_LIMIT + 1, r2_end))
            print(f"✅ {robots_r2_path}  (robots {REMOTE1_LIMIT + 1}–{r2_end}, dummy mode)")

            attest_r2_path = os.path.join(gen_dir, f"docker-compose.attestation-N{n}-remote2.yml")
            with open(attest_r2_path, "w") as f:
                f.write(generate_attestation_remote_yml(REMOTE1_LIMIT + 1, r2_end,
                                                        config_dir="./config-dummy",
                                                        blockchain_host=args.blockchain_host,
                                                        sidecar_image=sidecar_image))
            print(f"✅ {attest_r2_path}  (sidecars {REMOTE1_LIMIT + 1}–{r2_end}, image: {sidecar_image})")

    else:
        # ── LOCAL MODE: everything on d-mutra ─────────────────────────────────
        mode = "Gazebo" if n <= GAZEBO_LIMIT else "dummy"
        robots_path      = os.path.join(SCRIPT_DIR, "docker-compose.robots.yml")
        attestation_path = os.path.join(SCRIPT_DIR, "docker-compose.attestation.yml")

        with open(robots_path, "w") as f:
            f.write(generate_robots_yml(n))
        print(f"✅ {robots_path}  ({n} robots, {mode} mode)")

        cfg_dir = "./config" if n <= GAZEBO_LIMIT else "./config-dummy"
        with open(attestation_path, "w") as f:
            f.write(generate_attestation_yml(n, config_dir=cfg_dir, sidecar_image=sidecar_image))
        print(f"✅ {attestation_path}  ({n} sidecars, image: {sidecar_image}, ports 8001–{8000 + n})")
