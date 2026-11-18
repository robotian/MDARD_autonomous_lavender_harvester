#!/bin/bash
# One Nav2 benchmark trial on a sim robot: reset the sim, start the robot's stack without Nav2 and without the
# operations manager (the coordinator must not hand it tasks), start Nav2 the way bringup_main does
# (bringup_nav2_map.launch.py) with an optional params file, drive the course, and keep the results.
#
#   scripts/nav2_tune/trial.sh <robot> <label> [--params FILE] [--profiles FILE] [--course FILE] [--no-reset]
#                                               [--est-speed M/S]
#
#   robot      container name = namespace (j100_0921)
#   --params   Nav2 params file used instead of the robot's (nav2_robots.yaml still adds its param_overrides);
#              default: the package's own, i.e. what the robot runs
#   --profiles nav2_robots.yaml to use instead of the package's
#   --course   course JSON (nav2_bench.py's format; course.py makes one from the coordinator's graph); default
#              course_lavender_farm.json
#   --no-reset keep the sim and the running stack as they are (the robot must stand at the course start)
#   --est-speed speed of nav2_bench's time estimate (default 0.5 m/s; a leg over 3x its estimate fails): use the
#              robot's own speed limit, e.g. 0.2 for an a300 capped at 0.2 m/s
#
# Results: sim/generated/nav2_tune/<label>/ (results.json, params.yaml, nav2.log); summary: summarize.py <label>...
# A reset restarts every robot container (scripts/fleet_ctl.py reset). If the robot isn't upright at its spawn pose
# after it (a run that went chaotic survives a timeline reset), the sim is restarted. Sim only.
# FLEET_ROOT=<checkout>: the checkout whose scripts/fleet.sh started the sim (fleet_ctl.py talks to the sim through
# that checkout's sim/generated/fleet/); default this one.
set -u
cd "$(dirname "$0")/../.."
[ $# -ge 2 ] || { sed -n '2,20p' "$0"; exit 1; }
R=$1; L=$2; shift 2
PARAMS=""; PROFILES=""; COURSE=scripts/nav2_tune/course_lavender_farm.json; RESET=1; EST=0.5
while [ $# -gt 0 ]; do
  case $1 in
    --params) PARAMS=$2; shift 2;; --profiles) PROFILES=$2; shift 2;; --course) COURSE=$2; shift 2;;
    --no-reset) RESET=0; shift;; --est-speed) EST=$2; shift 2;; *) echo "unknown option $1"; exit 1;;
  esac
done
OUT=sim/generated/nav2_tune/$L; mkdir -p "$OUT"
FLEET_CTL="${FLEET_ROOT:-.}/scripts/fleet_ctl.py"
ros() { timeout -k 5 "$1" docker exec "$R" bash -c "source /home/robot/colcon_ws/install/setup.bash; $2"; }
# this checkout's benchmark (the container's /scripts is the main checkout's)
bench() { docker exec "$R" mkdir -p /tmp/nav2tune && docker cp scripts/nav2_tune/nav2_bench.py "$R":/tmp/nav2tune/nav2_bench.py >/dev/null; }

if [ $RESET = 1 ]; then
  python3 "$FLEET_CTL" reset 2>&1 | tail -1
  # the boot regenerates robot.srdf: the copy from before the reset already exists, and a stack started on it dies
  # when the boot replaces it ("No such file or directory: /etc/clearpath/robot.srdf"), so wait for a newer one
  start=$(date -d "$(docker inspect -f '{{.State.StartedAt}}' "$R")" +%s)
  for i in $(seq 90); do
    [ "$(docker exec "$R" stat -c %Y /etc/clearpath/robot.srdf 2>/dev/null || echo 0)" -gt "$start" ] && break; sleep 2
  done
  docker exec -d "$R" bash -c "source /home/robot/colcon_ws/install/setup.bash && exec ros2 launch mtu32_bringup \
    sim_robot_upstart.launch.py use_nav2:=false ops_manager:=false > /tmp/sim_robot_upstart.log 2>&1"
  T0=$(date +%s)
  until ros 40 "timeout -k 2 10 ros2 topic echo --once /$R/ref_pose --field header.frame_id >/dev/null 2>&1 &&
                timeout -k 2 10 ros2 run tf2_ros tf2_echo map base_link --ros-args -r __ns:=/$R -r /tf:=/$R/tf \
                -r /tf_static:=/$R/tf_static -p use_sim_time:=true 2>&1 | grep -q Translation"; do
    sleep 3; [ $(( $(date +%s) - T0 )) -gt 300 ] && { echo "$R: stack not up after 300 s"; exit 1; }
  done
  echo "$R: stack up after $(( $(date +%s) - T0 )) s"
  bench
  if ! ros 40 "python3 /tmp/nav2tune/nav2_bench.py --upright"; then
    echo "$R: not upright after the reset: restarting the sim"
    docker restart a300-isaac-sim >/dev/null
    timeout 300 python3 "$FLEET_CTL" wait-scene | tail -1
    exec "$0" "$R" "$L" ${PARAMS:+--params "$PARAMS"} ${PROFILES:+--profiles "$PROFILES"} --course "$COURSE"
  fi
  sleep 20  # a stack's start keeps the shared zenoh router busy for a while (CLAUDE.md)
fi

# Nav2, as bringup_main starts it (map, the sim's chargers as docks), optionally with other params
docker exec "$R" mkdir -p /tmp/nav2tune
ARGS="use_sim_time:=true charger_docks:=/generated/fleet/state.json"
if [ -n "$PARAMS" ]; then docker cp "$PARAMS" "$R":/tmp/nav2tune/params.yaml >/dev/null; ARGS="$ARGS params_file:=/tmp/nav2tune/params.yaml"; fi
if [ -n "$PROFILES" ]; then docker cp "$PROFILES" "$R":/tmp/nav2tune/profiles.yaml >/dev/null; ARGS="$ARGS profiles_file:=/tmp/nav2tune/profiles.yaml"; fi
docker exec "$R" bash -c 'pkill -INT -f bringup_nav2_map.launch.py; for i in $(seq 30); do pgrep -f "nav2_container|bringup_nav2_map" >/dev/null || break; sleep 1; done; pkill -9 -f "nav2_container|bringup_nav2_map"; rm -f /tmp/nav2tune/nav2.log; true'
docker exec -d "$R" bash -c "source /home/robot/colcon_ws/install/setup.bash && exec ros2 launch mtu32_bringup bringup_nav2_map.launch.py $ARGS > /tmp/nav2tune/nav2.log 2>&1"
T0=$(date +%s)
until [ "$(docker exec "$R" grep -c 'Managed nodes are active' /tmp/nav2tune/nav2.log 2>/dev/null)" -ge 2 ] 2>/dev/null; do
  sleep 2; [ $(( $(date +%s) - T0 )) -gt 240 ] && { echo "$R: Nav2 not active after 240 s"; docker exec "$R" tail -20 /tmp/nav2tune/nav2.log; exit 1; }
done
echo "$R: Nav2 active after $(( $(date +%s) - T0 )) s"
docker exec "$R" grep "bringup_nav2_map:" /tmp/nav2tune/nav2.log | head -2

docker cp "$COURSE" "$R":/tmp/nav2tune/course.json >/dev/null
bench
ros 7200 "python3 /tmp/nav2tune/nav2_bench.py /tmp/nav2tune/course.json --label $L --out /tmp/nav2tune/results.json --est-speed $EST" 2>&1 \
  | grep --line-buffered -v -E "^\[|^ +|Traceback|InvalidHandle"
docker cp "$R":/tmp/nav2tune/results.json "$OUT/results.json" >/dev/null
docker cp "$R":/tmp/"$R"_nav2_map.yaml "$OUT/params.yaml" >/dev/null  # what Nav2 really ran with (overrides merged)
docker cp "$R":/tmp/nav2tune/nav2.log "$OUT/nav2.log" >/dev/null
python3 scripts/nav2_tune/summarize.py "$OUT/results.json" | tail -1
