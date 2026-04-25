#!/usr/bin/env python3
"""
Generates docker-compose files for a given number of robots (1–100).

Gazebo mode (N ≤ GAZEBO_LIMIT = 4):
    Real simulation — gazebo-server + N turtlebots on a shared ros-net bridge.
    Outputs docker-compose.robots.yml and docker-compose.attestation.yml.

Dummy mode, single-host (GAZEBO_LIMIT < N ≤ LOCAL_LIMIT = 8):
    Robots run `sleep infinity` — no Gazebo or DDS needed.
    Outputs docker-compose.robots.yml and docker-compose.attestation.yml.

Dummy mode, multi-host (N > LOCAL_LIMIT = 8):
    Robots 1..LOCAL_LIMIT on local machine, LOCAL_LIMIT+1..N on remote host.
    Outputs four files under generated/:
        docker-compose.robots-N{N}-local.yml
        docker-compose.robots-N{N}-remote.yml
        docker-compose.attestation-N{N}-local.yml
        docker-compose.attestation-N{N}-remote.yml

Usage:
    python3 generate_compose.py --robots 4    # Gazebo mode
    python3 generate_compose.py --robots 15   # dummy, single-host
    python3 generate_compose.py --robots 50   # dummy, multi-host

Called automatically by start.sh via --robots N.
"""
import argparse
import os
import sys

MAX_ROBOTS   = 100
GAZEBO_LIMIT = 4    # N ≤ 4: real Gazebo simulation
LOCAL_LIMIT  = 8    # N ≤ 8: single host; N > 8: multi-host split

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

def generate_attestation_yml(n: int, start: int = 1, config_dir: str = "./config") -> str:
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
            f"    image: attestation-sidecar:latest",
            f"    environment:",
            f"      - CONFIG_PATH=/config/robot{i}.json",
            f"      - EXPORT_RESULTS=${{EXPORT_RESULTS}}",
            f"      - RESULTS_DIR=${{RESULTS_DIR}}",
            f"      - USE_REDIS=${{USE_REDIS}}",
            f"      - ONE_SHOT=${{ONE_SHOT}}",
            f"      - AUTO_START=${{AUTO_START}}",
            f"      - WAIT_FOR_TX_CONFIRMATIONS=${{WAIT_FOR_TX_CONFIRMATIONS}}",
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
            f"          cpus: '0.4'",
            f"    extra_hosts:",
            f'      - "host.docker.internal:host-gateway"',
            f"",
        ]

    return "\n".join(lines) + "\n"


def generate_attestation_remote_yml(n_start: int, n_end: int, config_dir: str = "./config-dummy",
                                    blockchain_host: str = "") -> str:
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
            f"    image: attestation-sidecar:latest",
            f"    environment:",
            f"      - CONFIG_PATH=/config/robot{i}.json",
            f"      - EXPORT_RESULTS=${{EXPORT_RESULTS}}",
            f"      - RESULTS_DIR=${{RESULTS_DIR}}",
            f"      - USE_REDIS=${{USE_REDIS}}",
            f"      - ONE_SHOT=${{ONE_SHOT}}",
            f"      - AUTO_START=${{AUTO_START}}",
            f"      - WAIT_FOR_TX_CONFIRMATIONS=${{WAIT_FOR_TX_CONFIRMATIONS}}",
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
            f"          cpus: '0.4'",
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
             f"N≤{GAZEBO_LIMIT}: Gazebo mode. "
             f"N≤{LOCAL_LIMIT}: dummy single-host. "
             f"N>{LOCAL_LIMIT}: dummy multi-host.",
    )
    parser.add_argument(
        "--blockchain-host", default="",
        help="local machine IP reachable from the remote host (only used for multi-host remote "
             "attestation compose). Injected as host.docker.internal so the same config "
             "JSON works on both local and remote sidecars.",
    )
    args = parser.parse_args()

    if not 1 <= args.robots <= MAX_ROBOTS:
        print(f"❌ --robots must be between 1 and {MAX_ROBOTS} (got {args.robots})")
        sys.exit(1)

    n = args.robots

    if n > LOCAL_LIMIT:
        # ── Multi-host: four files in generated/ ──────────────────────────────
        gen_dir = os.path.join(SCRIPT_DIR, "generated")
        os.makedirs(gen_dir, exist_ok=True)

        paths = {
            "robots-local":       os.path.join(gen_dir, f"docker-compose.robots-N{n}-local.yml"),
            "robots-remote":      os.path.join(gen_dir, f"docker-compose.robots-N{n}-remote.yml"),
            "attestation-local":  os.path.join(gen_dir, f"docker-compose.attestation-N{n}-local.yml"),
            "attestation-remote": os.path.join(gen_dir, f"docker-compose.attestation-N{n}-remote.yml"),
        }

        with open(paths["robots-local"], "w") as f:
            f.write(generate_robots_yml_dummy(LOCAL_LIMIT))
        print(f"✅ {paths['robots-local']}  (robots 1–{LOCAL_LIMIT}, dummy)")

        with open(paths["robots-remote"], "w") as f:
            f.write(generate_robots_remote_yml(LOCAL_LIMIT + 1, n))
        print(f"✅ {paths['robots-remote']}  (robots {LOCAL_LIMIT + 1}–{n}, dummy)")

        with open(paths["attestation-local"], "w") as f:
            f.write(generate_attestation_yml(LOCAL_LIMIT, config_dir="./config-dummy"))
        print(f"✅ {paths['attestation-local']}  (sidecars 1–{LOCAL_LIMIT})")

        with open(paths["attestation-remote"], "w") as f:
            f.write(generate_attestation_remote_yml(LOCAL_LIMIT + 1, n, config_dir="./config-dummy",
                                                    blockchain_host=args.blockchain_host))
        print(f"✅ {paths['attestation-remote']}  (sidecars {LOCAL_LIMIT + 1}–{n})")

    else:
        # ── Single-host: two files in root ────────────────────────────────────
        mode = "Gazebo" if n <= GAZEBO_LIMIT else "dummy"
        robots_path      = os.path.join(SCRIPT_DIR, "docker-compose.robots.yml")
        attestation_path = os.path.join(SCRIPT_DIR, "docker-compose.attestation.yml")

        with open(robots_path, "w") as f:
            f.write(generate_robots_yml(n))
        print(f"✅ {robots_path}  ({n} robots, {mode} mode)")

        cfg_dir = "./config" if n <= GAZEBO_LIMIT else "./config-dummy"
        with open(attestation_path, "w") as f:
            f.write(generate_attestation_yml(n, config_dir=cfg_dir))
        print(f"✅ {attestation_path}  ({n} sidecars, ports 8001–{8000 + n})")
