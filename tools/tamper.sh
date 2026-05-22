#!/usr/bin/env bash
# tamper.sh - inject a single-byte tampering into the .text of a target
# process inside a robot's sidecar PID namespace. Pure injection: the
# script does not observe the framework's reaction. Pair it with
# ./tools/observe.sh or container logs to see whether the corruption
# is detected as a FAILURE attestation.
#
# Read-modify-write of /proc/<pid>/mem from inside the sidecar container,
# which has CAP_SYS_PTRACE and shares the PID namespace with its robot.
# The change is in-memory only; restart the affected container to undo.
#
# Usage:
#   ./tools/tamper.sh <robot> [--target T] [--offset 0xN]
#
# Targets:
#   state_publisher (default)  - robot_state_publisher in Gazebo robots
#   dummy                      - dummy_publisher in dummy robots
#   sidecar                    - the attestation-sidecar binary itself
#
# Examples:
#   ./tools/tamper.sh robot1
#   ./tools/tamper.sh robot1 --target sidecar
#   ./tools/tamper.sh robot2 --target dummy --offset 0x6000
#
# Exit code: 0 if the byte was written, 1 on any error.

set -uo pipefail

TARGET="state_publisher"
OFFSET=""
ROBOT=""

usage() {
  sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target) TARGET="$2"; shift 2 ;;
    --offset) OFFSET="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown flag: $1" >&2; usage >&2; exit 1 ;;
    *)
      if [[ -z "$ROBOT" ]]; then ROBOT="$1"; shift
      else echo "unexpected positional arg: $1" >&2; exit 1
      fi
      ;;
  esac
done

[[ -z "$ROBOT" ]] && { echo "missing robot name" >&2; usage >&2; exit 1; }

SIDECAR="${ROBOT}-sidecar"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIDECAR"; then
  echo "container $SIDECAR is not running" >&2
  exit 1
fi

case "$TARGET" in
  state_publisher)
    PROC_GREP="robot_state_publisher"
    DEFAULT_OFFSET="0x2000"        # 8 KB into the 42223-byte .text
    ;;
  dummy)
    PROC_GREP="dummy_publisher"
    DEFAULT_OFFSET="0x4e20"        # 5744 + 14256, ~20 KB into the .text window
    ;;
  sidecar)
    PROC_GREP=""                   # the sidecar binary is PID 1 in its container
    DEFAULT_OFFSET="0x100000"      # 1 MB into the ~14 MB .text
    ;;
  *) echo "unknown target: $TARGET" >&2; exit 1 ;;
esac

OFFSET="${OFFSET:-$DEFAULT_OFFSET}"

echo "── Tampering ───────────────────────────────────────────"
echo "  robot:   $ROBOT (container $SIDECAR)"
echo "  target:  $TARGET"
echo "  offset:  $OFFSET"
echo

if [[ "$TARGET" == "sidecar" ]]; then
  docker exec -e TAMPER_OFFSET="$OFFSET" "$SIDECAR" sh -c '
    ADDR=$(grep "r-xp.*attestation-sidecar$" /proc/1/maps | head -1 | cut -d- -f1)
    [ -z "$ADDR" ] && { echo "no r-xp mapping for attestation-sidecar in /proc/1/maps"; exit 1; }
    printf "\220" | dd of=/proc/1/mem bs=1 count=1 \
      seek=$(( 0x$ADDR + TAMPER_OFFSET )) conv=notrunc 2>/dev/null
    echo "  tampered PID=1 (self) at 0x$ADDR + $TAMPER_OFFSET"
  ' || { echo "tampering failed" >&2; exit 1; }
else
  docker exec -e TAMPER_OFFSET="$OFFSET" -e TAMPER_PROC="$PROC_GREP" "$SIDECAR" sh -c '
    PID=$(pgrep -f "$TAMPER_PROC" | head -1)
    [ -z "$PID" ] && { echo "$TAMPER_PROC not found in PID namespace"; exit 1; }
    ADDR=$(grep "r-xp.*$TAMPER_PROC" /proc/$PID/maps | head -1 | cut -d- -f1)
    [ -z "$ADDR" ] && { echo "no r-xp mapping for $TAMPER_PROC in /proc/$PID/maps"; exit 1; }
    printf "\220" | dd of=/proc/$PID/mem bs=1 count=1 \
      seek=$(( 0x$ADDR + TAMPER_OFFSET )) conv=notrunc 2>/dev/null
    echo "  tampered PID=$PID ($TAMPER_PROC) at 0x$ADDR + $TAMPER_OFFSET"
  ' || { echo "tampering failed" >&2; exit 1; }
fi

echo
echo "Recovery: docker restart $ROBOT       (reload the binary fresh)"
