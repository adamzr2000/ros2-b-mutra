#!/usr/bin/env bash
# test_formation_mitigation.sh — validate the ROS mitigation half of the
# formation experiment WITHOUT the blockchain stack.
#
# Flow: fresh run -> let the square travel -> inject the constant-turn attack on
# robot3 (the cohesive swarm stalls) -> simulate D-MUTRA detection by publishing
# /formation/excluded robot3 -> robots 1/2/4 drop robot3 and reform the triangle
# -> the (active) swarm should RESUME advancing toward the goal.
#
# Prints a centroid timeline so you can see: travel -> HALT -> (mitigate) -> RESUME.
# The active-3 centroid (robots 1,2,4) is the recovery signal after exclusion.
#
# Prereq: the control-only formation stack is brought up by this script
# (gazebo + 4 robots + orchestrator). No password / no blockchain needed.
#
# Usage:  ./tools/test_formation_mitigation.sh [--no-mitigate]
#   --no-mitigate : inject the attack but DON'T isolate (shows permanent halt =
#                   the unmitigated case). Default: mitigate after the halt.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

PROJECT="ros2-b-mutra"
MITIGATE=1
[[ "${1:-}" == "--no-mitigate" ]] && MITIGATE=0

ATTACK_OFFSET="0xbfa7"     # steer_cmd ang-gain mulsd; NOP -> constant turn
TRAVEL_S=12                # travel before attack
HALT_WATCH_S=18            # observe the stall
RECOVER_WATCH_S=35         # observe recovery after isolation

upsert(){ local k="$1" v="$2"; if grep -qE "^[[:space:]]*$k=" .env; then sed -i "s|^[[:space:]]*$k=.*|$k=$v|g" .env; else echo "$k=$v" >> .env; fi; }

echo "── 1. fresh formation stack (control-only) ──────────────────────────────"
upsert ENABLE_FORMATION TRUE; upsert ENABLE_NAV2 FALSE; upsert GAZEBO_WORLD empty
upsert FORMATION_SCALE 2.0; upsert K_ATT 0.8; upsert K_FORM 3.0
upsert GOAL_X 40.0; upsert GOAL_Y 0.0; upsert MITIGATION false
upsert RUN_DURATION_S 120; upsert RUN_TAG mit-test
python3 generate_compose.py --robots 4 --world empty --formation --formation-scale 2.0 \
  --gpu --vnc --mode local --contract AttestationManagerLV --blockchain-host host.docker.internal >/dev/null
docker compose -p "$PROJECT" -f docker-compose.robots.yml up -d --force-recreate >/dev/null 2>&1
echo "   waiting for formation_agent ready..."
for i in $(seq 1 50); do docker logs robot1 2>&1 | grep -q "formation_agent ready" && break; sleep 3; done
# Bring up the orchestrator ONLY (no recorder needed for the live test).
docker compose -p "$PROJECT" -f docker-compose.formation.yml up -d --force-recreate orchestrator >/dev/null 2>&1
echo "   waiting for goal publish..."
for i in $(seq 1 30); do docker logs orchestrator 2>&1 | grep -q "Goal published" && break; sleep 2; done

# Centroid sampler (active = robots minus any excluded; here we print both the
# all-4 centroid and the 1/2/4 centroid so recovery is visible after exclusion).
cat > /tmp/_mit_sample.py <<'PY'
import subprocess, sys
def pos(r):
    out=subprocess.run(["ros2","topic","echo","--once","--field","pose.pose.position",
                        "/"+r+"/ground_truth"],capture_output=True,text=True,timeout=5).stdout
    x=y=None
    for ln in out.splitlines():
        s=ln.strip()
        if s.startswith("x:") and x is None: x=float(s.split()[1])
        elif s.startswith("y:") and y is None: y=float(s.split()[1])
    return x,y
tag=sys.argv[1] if len(sys.argv)>1 else ""
P={r:pos(r) for r in ["robot1","robot2","robot3","robot4"]}
c4x=sum(P[r][0] for r in P)/4
c3x=(P["robot1"][0]+P["robot2"][0]+P["robot4"][0])/3   # active-3 (1,2,4)
print("  %-9s centroid_x(all4)=%6.2f  centroid_x(1,2,4)=%6.2f  r3=(%6.2f,%6.2f)"
      % (tag, c4x, c3x, P["robot3"][0], P["robot3"][1]))
PY
docker cp /tmp/_mit_sample.py robot1:/tmp/ >/dev/null 2>&1
samp(){ docker exec robot1 bash -lc "source /opt/ros/humble/setup.bash && python3 /tmp/_mit_sample.py '$1'"; }

echo "── 2. baseline travel (${TRAVEL_S}s) — centroid should climb ────────────"
for i in $(seq 1 4); do samp "travel"; sleep $((TRAVEL_S/4)); done

echo "── 3. INJECT constant-turn attack on robot3 ─────────────────────────────"
docker run --rm --pid=container:robot3 --cap-add=SYS_PTRACE --user root turtlebot3-gazebo bash -lc "
  PID=\$(pgrep -f formation_agent | head -1)
  BASE=\$(grep 'r-xp.*formation_agent' /proc/\$PID/maps | head -1 | cut -d- -f1)
  printf '\x90\x90\x90\x90\x90' | dd of=/proc/\$PID/mem bs=1 seek=\$(( 0x\$BASE + $ATTACK_OFFSET )) count=5 conv=notrunc 2>/dev/null
  echo '   injected (robot3 now spins ~1.8 rad/s)'"

echo "── 4. observe STALL (${HALT_WATCH_S}s) — centroid should flatten ────────"
for i in $(seq 1 6); do samp "HALT"; sleep $((HALT_WATCH_S/6)); done

if [[ "$MITIGATE" == "1" ]]; then
  echo "── 5. MITIGATE: publish /formation/excluded robot3 (simulated detection) ─"
  docker exec robot1 bash -lc "source /opt/ros/humble/setup.bash && \
    timeout 4 ros2 topic pub -r 2 --qos-durability transient_local \
    /formation/excluded std_msgs/msg/String \"{data: 'robot3'}\" >/dev/null 2>&1; echo '   published exclusion'"
  echo "── 6. observe RECOVERY (${RECOVER_WATCH_S}s) — centroid_x(1,2,4) should climb again ─"
  for i in $(seq 1 7); do samp "RECOVER"; sleep $((RECOVER_WATCH_S/7)); done
  echo
  echo "✅ PASS criteria: centroid_x(1,2,4) flat during HALT, then RISING during RECOVER."
else
  echo "── (--no-mitigate) observe PERMANENT halt (${RECOVER_WATCH_S}s) ─────────"
  for i in $(seq 1 7); do samp "no-mit"; sleep $((RECOVER_WATCH_S/7)); done
  echo
  echo "✅ Unmitigated: centroid stays flat (no recovery)."
fi
echo "   (watch live at http://localhost:6080)"
