#!/usr/bin/env python3
"""
Generates docker-compose.robots.yml and docker-compose.attestation.yml
for a given number of robots (1–100).

Usage:
    python3 generate_compose.py --robots 16

Called automatically by start.sh via --robots N.
The two output files are generated artifacts — edit this script, not them.
"""
import argparse
import os
import sys

MAX_ROBOTS = 100
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def robot_position(i: int, spacing: int = 5):
    """Grid layout: 10 robots per row, `spacing` units apart.
    robot1 → (0,0), robot10 → (45,0), robot11 → (0,5), …
    """
    row = (i - 1) // 10
    col = (i - 1) % 10
    return col * spacing, row * spacing


def generate_robots_yml(n: int) -> str:
    lines = [
        "# docker-compose.robots.yml",
        f"# AUTO-GENERATED for {n} robot(s) — edit generate_compose.py, not this file.",
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
        "",
        "  gazebo-client:",
        "    image: gazebo-vnc:latest",
        "    container_name: gazebo-client",
        "    hostname: gazebo-client",
        "    environment:",
        "      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}",
        "      - GAZEBO_MASTER_URI=http://gazebo-server:11345",
        "    depends_on:",
        "      - gazebo-server",
        "    ports:",
        '      - "6080:80"',
        "    security_opt:",
        "      - seccomp=unconfined",
        "    restart: unless-stopped",
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


def generate_attestation_yml(n: int) -> str:
    lines = [
        "# docker-compose.attestation.yml",
        f"# AUTO-GENERATED for {n} robot(s) — edit generate_compose.py, not this file.",
        "",
        "services:",
    ]

    for i in range(1, n + 1):
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
            f"      - ./config/robot{i}.json:/config/robot{i}.json",
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
            f"    networks:",
            f"      - quorum-dev-quickstart",
            f"",
        ]

    lines += [
        "networks:",
        "  quorum-dev-quickstart:",
        "    external: true",
    ]

    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Generate docker-compose files for N robots (1–{MAX_ROBOTS})"
    )
    parser.add_argument(
        "--robots", type=int, required=True,
        help=f"Number of robots to generate (1–{MAX_ROBOTS})",
    )
    args = parser.parse_args()

    if not 1 <= args.robots <= MAX_ROBOTS:
        print(f"❌ --robots must be between 1 and {MAX_ROBOTS} (got {args.robots})")
        sys.exit(1)

    robots_path     = os.path.join(SCRIPT_DIR, "docker-compose.robots.yml")
    attestation_path = os.path.join(SCRIPT_DIR, "docker-compose.attestation.yml")

    with open(robots_path, "w") as f:
        f.write(generate_robots_yml(args.robots))
    print(f"✅ {robots_path}  ({args.robots} robots)")

    with open(attestation_path, "w") as f:
        f.write(generate_attestation_yml(args.robots))
    print(f"✅ {attestation_path}  ({args.robots} sidecars, ports 8001–{8000 + args.robots})")
