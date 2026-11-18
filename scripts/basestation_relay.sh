#!/bin/bash
# Supervisor of the base station's robot relays (scripts/robot_relay.py), one per entry of BASESTATION_RELAYS
# ("<id>=tcp/<ip>:7447 ..."): the operation coordinator's topics to and from each real robot, as a client of that
# robot's own router instead of a router-to-router link (see robot_relay.py for why). Started by
# basestation/entrypoint.sh (log /tmp/relay.log; each relay's own log /tmp/relay_<id>.log); a relay that exits is
# started again. The list comes from the container's environment: change it with the setting, then recreate the
# container (docker compose -f basestation.compose.yml up -d).
set -u
log() { echo "[relay] $(date +%T) $*"; }
[ "${RMW_IMPLEMENTATION:-}" = rmw_zenoh_cpp ] || { log "not on zenoh: no relays"; exit 0; }
[ -n "${BASESTATION_RELAYS:-}" ] || { log "BASESTATION_RELAYS is empty: no relays"; exit 0; }
set +u; source /etc/ros_env.sh; source /home/robot/colcon_ws/install/setup.bash 2>/dev/null; set -u
python3 -c 'import status_interfaces.msg' 2>/dev/null \
    || log "status_interfaces is not built (scripts/colcon_build.sh): status/robot and status/task can't be relayed"
for entry in $BASESTATION_RELAYS; do
    id=${entry%%=*} endpoint=${entry#*=}
    if [[ ! "$id" =~ ^[a-z0-9]+_[0-9]+$ || ! "$endpoint" =~ ^tcp/[A-Za-z0-9.-]+:[0-9]+$ ]]; then
        log "skipping '$entry': not <id>=tcp/<ip>:<port>"
        continue
    fi
    log "$id via $endpoint (log /tmp/relay_$id.log)"
    (while true; do
        python3 -u /scripts/robot_relay.py "$id" "$endpoint" >> "/tmp/relay_$id.log" 2>&1
        echo "[relay] $(date +%T) robot_relay.py exited ($?), starting again in 5 s" >> "/tmp/relay_$id.log"
        sleep 5
    done) &
done
wait
