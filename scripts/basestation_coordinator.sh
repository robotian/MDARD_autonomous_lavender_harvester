#!/bin/bash
# Supervisor of the harvesting operation's coordinator (colcon_ws/src/status_server: the task_manager and
# status_server nodes) in the base station container. Started by basestation/entrypoint.sh (log /tmp/coordinator.log).
# Runs `ros2 launch status_server status_server.launch.py` while public.operation.coordinator_enabled is true
# (the web UI's /operation page flips it; Start sets it too), restarts it when it exits, stops it (SIGINT) when the
# flag goes false. The flag lives in the database, so a recreated base station picks the operation up again.
# Needs colcon_ws built (scripts/colcon_build.sh from a robot container) and migration 002 (the operation table).
# status_server's log: /tmp/status_server.log. psql here uses the container's trusted local socket.
set -u
LOG=/tmp/status_server.log
log() { echo "[coordinator] $(date +%T) $*"; }
flag() { psql -tAq -c "SELECT coordinator_enabled FROM public.operation WHERE id = 1" 2>/dev/null; }
event() {
    psql -q -c "INSERT INTO public.operation_event (source, kind, message) VALUES ('supervisor', 'coordinator', \$\$$1\$\$)" \
        2>/dev/null || true
}
said=""
while true; do
    if [ "$(flag)" != t ]; then
        sleep 2
        continue
    fi
    if [ ! -d /home/robot/colcon_ws/install/status_server ]; then
        [ "$said" = build ] || { log "status_server is not built: run scripts/colcon_build.sh (from a robot container)"; said=build; }
        sleep 10
        continue
    fi
    said=""
    log "starting status_server (use_sim_time ${USE_SIM_TIME:-false})"
    : > "$LOG"
    (set +u; source /etc/ros_env.sh; exec ros2 launch status_server status_server.launch.py \
        use_sim_time:="${USE_SIM_TIME:-false}") >> "$LOG" 2>&1 &
    pid=$!
    event "status_server started (pid $pid)"
    stopped=""
    while kill -0 "$pid" 2>/dev/null; do
        if [ "$(flag)" != t ]; then
            log "coordinator disabled: stopping status_server"
            kill -INT "$pid" 2>/dev/null
            for _ in $(seq 60); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
            kill -TERM "$pid" 2>/dev/null
            stopped=1
            break
        fi
        sleep 2
    done
    wait "$pid"
    rc=$?
    # the launcher gone, its nodes must be too: two task_managers would assign plants twice
    sleep 1
    pkill -TERM -f 'lib/status_server/(status_server|task_manager)' 2>/dev/null && log "stopped node processes the launcher left behind"
    if [ -n "$stopped" ]; then
        event "status_server stopped"
    else
        log "status_server exited (rc $rc); restarting in 3 s"
        event "status_server exited (rc $rc), restarting"
    fi
    sleep 3
done
