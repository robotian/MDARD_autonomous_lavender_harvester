#!/bin/bash
# Step test of a sim robot's drive: cmd_vel steps vs the sim's ground truth, for tuning model_params.yaml's vel_ctl
# and wheel_friction (sim/CLAUDE.md "Velocity control").
#
#   scripts/vel_tune/step_test.sh <robot> <label> [--segments full|user|screen] [--reset] [--compare <label>]
#
#   robot       container name = namespace (j100_0921)
#   --segments  full (default): 0.5, 0.3, -0.2 m/s, in place 0.3 and 0.1 rad/s, arcs 0.3/0.3 and 0.5/0.5;
#               user: 0.3 m/s, 0.3 rad/s, both; screen: short open-loop check
#   --reset     sim reset first (robot back at its spawn pose; restarts the robot containers)
#   --compare   also plot this run next to an earlier one of the same robot
#   --in C      run in container C (e.g. basestation) instead of the robot's own: for a model whose robot container
#               doesn't start (no robot_data/<id>/robot.yaml, e.g. a200_0333); its arm, if any, isn't stowed there
#
# Results in sim/generated/vel_check/<robot>_<label>.{jsonl,metrics.json,png}. Nothing else may publish the robot's
# cmd_vel meanwhile (no Nav2, no teleop); the robot drives a few metres east of where it stands and turns in place.
# A change of vel_ctl / wheel_friction takes effect at the next spawn: `docker restart a300-isaac-sim` (sim only).
set -u
cd "$(dirname "$0")/../.."
[ $# -ge 2 ] || { sed -n '2,17p' "$0"; exit 1; }
R=$1; L=$2; shift 2
SEG=full; RESET=0; CMP=""; C=$R
while [ $# -gt 0 ]; do
  case $1 in
    --segments) SEG=$2; shift 2;; --reset) RESET=1; shift;; --compare) CMP=$2; shift 2;; --in) C=$2; shift 2;;
    *) echo "unknown option $1"; exit 1;;
  esac
done
OUT=sim/generated/vel_check; mkdir -p "$OUT"
if [ $RESET = 1 ]; then
  python3 scripts/fleet_ctl.py reset 2>&1 | tail -1
  # wait for the boot's new robot.srdf, not the copy from before the reset (see nav2_tune/trial.sh)
  start=$(date -d "$(docker inspect -f '{{.State.StartedAt}}' "$C")" +%s)
  for i in $(seq 90); do
    [ "$(docker exec "$C" stat -c %Y /etc/clearpath/robot.srdf 2>/dev/null || echo 0)" -gt "$start" ] && break; sleep 2
  done
fi
pubs=$(timeout -k 2 30 docker exec "$C" bash -c "source /opt/ros/jazzy/setup.bash; ros2 topic info /$R/cmd_vel --no-daemon" \
       2>/dev/null | sed -n 's/Publisher count: //p')
[ "${pubs:-0}" = 0 ] || { echo "$R/cmd_vel already has $pubs publisher(s): stop Nav2 / teleop first"; exit 1; }
docker exec "$C" mkdir -p /tmp/vel_tune
for f in step.py metrics.py plot.py compare.py; do docker cp "scripts/vel_tune/$f" "$C":/tmp/vel_tune/ >/dev/null; done
timeout -k 5 600 docker exec -e VEL_SEGMENTS="$SEG" -e ROBOT_NAMESPACE="$R" "$C" bash -c \
  "source /opt/ros/jazzy/setup.bash; python3 /tmp/vel_tune/step.py /tmp/vel_tune/$L.jsonl" 2>&1 | grep -E "stowed|skipped|done"
docker exec "$C" python3 /tmp/vel_tune/metrics.py "/tmp/vel_tune/$L.jsonl" "$R $L" 2>&1 | grep -v Warning
docker exec "$C" python3 /tmp/vel_tune/plot.py "/tmp/vel_tune/$L.jsonl" "/tmp/vel_tune/$L.png" "$R" >/dev/null 2>&1
for ext in jsonl metrics.json png json; do docker cp "$C:/tmp/vel_tune/$L.$ext" "$OUT/${R}_$L.$ext" >/dev/null 2>&1; done
if [ -n "$CMP" ]; then
  docker cp "$OUT/${R}_$CMP.jsonl" "$C":/tmp/vel_tune/before.jsonl >/dev/null
  docker exec "$C" python3 /tmp/vel_tune/compare.py /tmp/vel_tune/before.jsonl "/tmp/vel_tune/$L.jsonl" \
    "/tmp/vel_tune/cmp.png" "$R" "$CMP" "$L" >/dev/null 2>&1
  docker cp "$C":/tmp/vel_tune/cmp.png "$OUT/${R}_${CMP}_vs_$L.png" >/dev/null && echo "comparison: $OUT/${R}_${CMP}_vs_$L.png"
fi
echo "results: $OUT/${R}_$L.*"
