#!/bin/bash
# Base station: starts PostgreSQL (creating the cluster and database on first start) and, with rmw_zenoh_cpp, the
# base station's own zenoh router, then runs the container command. PostgreSQL is stopped cleanly (fast shutdown)
# on `docker stop`.
set -Eeuo pipefail  # -E: the ERR trap below also fires inside as_pg
: "${PGPORT:?}" "${PGUSER:?}" "${PGDATABASE:?}"

as_pg() { setpriv --reuid=postgres --regid=postgres --init-groups "$@"; }
log() { echo "[basestation] $*"; }

mkdir -p "$PGDATA" /run/postgresql
chown postgres:postgres "$PGDATA" /run/postgresql
chmod 700 "$PGDATA"

# First start: a cluster whose superuser is $PGUSER (as the official postgres image does with POSTGRES_USER), so
# dumps of the farm database, owned by that user, restore unchanged. Local socket connections are trusted (only
# processes in this container can use the socket); TCP needs the password.
if [ ! -s "$PGDATA/PG_VERSION" ]; then
    : "${PGPASSWORD:?PGPASSWORD is not set: put PGPASSWORD=... in db.env at the repo root}"
    log "creating the database cluster in $PGDATA (PostgreSQL $PG_MAJOR)"
    # anything created here is half-made if a step fails: remove it so the next start begins again
    trap 'log "first-start setup failed, removing the half-made cluster"
          as_pg pg_ctl -D "$PGDATA" -m immediate -w stop >/dev/null 2>&1 || true
          find "$PGDATA" -mindepth 1 -delete; exit 1' ERR
    pwfile=$(mktemp)
    printf '%s' "$PGPASSWORD" > "$pwfile"
    chown postgres "$pwfile"
    as_pg initdb -D "$PGDATA" -U "$PGUSER" --pwfile="$pwfile" --encoding=UTF8 --locale=C.UTF-8 \
        --auth-local=trust --auth-host=scram-sha-256 >/dev/null
    rm -f "$pwfile"
    printf '%s\n' 'host all all 0.0.0.0/0 scram-sha-256' 'host all all ::/0 scram-sha-256' >> "$PGDATA/pg_hba.conf"

    # socket only while the database is filled, so no client sees a half-restored one
    as_pg pg_ctl -D "$PGDATA" -o "-c listen_addresses='' -c port=$PGPORT" -w start >/dev/null
    as_pg psql -q -v ON_ERROR_STOP=1 -d postgres -v db="$PGDATABASE" <<<'CREATE DATABASE :"db";'
    shopt -s nullglob
    loaded=0
    for f in /initdb/*; do
        case "$f" in
            *.sh) log "running $f"; bash "$f" ;;
            *.sql) log "loading $f"; as_pg psql -q -v ON_ERROR_STOP=1 -f "$f" ;;
            *.sql.gz) log "loading $f"; gunzip -c "$f" | as_pg psql -q -v ON_ERROR_STOP=1 ;;
            *.dump) log "restoring $f"; as_pg pg_restore --exit-on-error --no-owner -d "$PGDATABASE" "$f" ;;
            *.md) continue ;;
            *) log "ignoring $f (not .sh/.sql/.sql.gz/.dump)"; continue ;;
        esac
        loaded=1
    done
    # Nothing of this PC's own: the farm every new checkout starts with (basestation/seed/, made by
    # scripts/make_farm_seed.sh: every plant pending, the operation stopped, only the default sim robot enrolled).
    # Without it the migrations fail on an empty database (they extend the farm's tables).
    if [ "$loaded" = 0 ] && [ -f /seed/farm.sql ]; then
        log "loading the farm seed /seed/farm.sql"
        as_pg psql -q -v ON_ERROR_STOP=1 -f /seed/farm.sql
    fi
    as_pg pg_ctl -D "$PGDATA" -m fast -w stop >/dev/null
    trap - ERR
fi

as_pg postgres -D "$PGDATA" -c listen_addresses='*' -c port="$PGPORT" &
pg_pid=$!
until pg_isready -q; do
    kill -0 "$pg_pid" 2>/dev/null || { log "PostgreSQL failed to start"; exit 1; }
    sleep 0.5
done
log "PostgreSQL $PG_MAJOR ready on port $PGPORT (database $PGDATABASE, user $PGUSER)"

# Schema migrations (basestation/migrations/, mounted at /migrations): at every start, not only the first, so a
# database restored from an older dump or created on another PC catches up. A failing file is logged and the base
# station keeps running (the robots and the zenoh router need it); the file is retried at the next start.
/basestation-migrate.sh /migrations || log "schema migrations failed, see the [migrate] lines above"

# Zenoh: like each real robot, the base station has its own router (tcp/[::]:7447, so robots can also dial in);
# its sessions are clients of it (ZENOH_CONFIG_OVERRIDE from compose). The router dials the routers in
# BASESTATION_ZENOH_CONNECT (space-separated: the sim's zenoh-router, real robots' tcp/<ip>:7447) and keeps
# retrying any that are down, so one list serves the sim, real robots or both. Log: /tmp/zenoh_router.log.
# Known problem (2026-10-06, zenoh 1.8.0 on all three routers): with two real robots in the list, the
# router-to-router links deadlock -- this router stops reading a robot's socket (850 KB unread in its receive
# queue) while it waits to push messages zenoh may not drop (RViz opening / closing sends them), and that robot's
# router does the same; the other robot's data is held meanwhile (225 s once). With one robot: 100 % over 10 min
# with RViz opened and closed twice. Shorter close / lease timeouts (0.5 s or 2 s, 5 s lease) only made it loop:
# each reconnection resends all declarations and deadlocks again. So the web UI's per-robot tools (the
# Communication monitor, a real robot's RViz) don't go through this router: each is a zenoh client of that robot's
# own router. List real robots here only for base station programs that need several robots in one graph.
# Fix (2026-10-07): tx queues of 16 batches per priority instead of 2. With the defaults and two real robots
# (a300_00036 + a200_0284), the links flapped (the "Unable to push non droppable network message ... Closing
# transport!" above, ~5 s apart) and a200's routing stayed broken afterwards: "Received router declaration with
# unknown routing context id 0" on both routers, the base station seeing 0 of a200's 171 topics while a client of
# a200's own router saw them all. With 16: both robots' topics and data, no push errors.
TX_QUEUES="control real_time interactive_high interactive_low data_high data data_low background"
if [ "${RMW_IMPLEMENTATION:-}" = rmw_zenoh_cpp ]; then
    endpoints="" tx=""
    for e in ${BASESTATION_ZENOH_CONNECT:-}; do endpoints+="${endpoints:+,}\"$e\""; done
    for q in $TX_QUEUES; do tx+=" --cfg transport/link/tx/queue/size/$q:16"; done
    # zenohd 1.10.1 (Dockerfile) with rmw_zenohd's router config (docker/zenoh_router.json5, mounted by compose).
    # Jazzy's rmw_zenohd (zenoh 1.8.0) went into a 100 % CPU loop for good when the sim's zenoh-router restarted
    # (2026-10-21: 108 CLOSE_WAIT sockets, no new session served), like the fleet's router before (CLAUDE.md).
    (
        export RUST_LOG="${RUST_LOG:-info}"
        while true; do zenohd -c /etc/zenoh/router.json5 --cfg "connect/endpoints:[$endpoints]" $tx || true; sleep 2; done
    ) > /tmp/zenoh_router.log 2>&1 &
    log "zenoh router on port 7447, connecting to: ${BASESTATION_ZENOH_CONNECT:-(none)}"
fi

# Real robots relayed instead of linked (BASESTATION_RELAYS): the coordinator's topics only, each a client of the
# robot's own router (scripts/basestation_relay.sh, robot_relay.py), so a robot's restart can't clog this router's
# links. From the /scripts mount, so it needs no image rebuild; log /tmp/relay.log.
if [ -f /scripts/basestation_relay.sh ]; then
    bash /scripts/basestation_relay.sh > /tmp/relay.log 2>&1 &
fi

# The harvesting operation's coordinator (status_server), run while public.operation.coordinator_enabled is true:
# scripts/basestation_coordinator.sh (the /scripts mount, so it needs no image rebuild), log /tmp/coordinator.log.
coord_pid=""
if [ -f /scripts/basestation_coordinator.sh ]; then
    bash /scripts/basestation_coordinator.sh > /tmp/coordinator.log 2>&1 &
    coord_pid=$!
    log "coordinator supervisor started (status_server while operation.coordinator_enabled; /tmp/coordinator.log)"
fi

"$@" &
cmd_pid=$!
stop() {
    [ -n "$coord_pid" ] && kill -TERM "$coord_pid" 2>/dev/null || true
    pkill -INT -f "[s]tatus_server.launch.py" 2>/dev/null || true
    kill -TERM "$cmd_pid" 2>/dev/null || true
    as_pg pg_ctl -D "$PGDATA" -m fast -w stop >/dev/null 2>&1 || true
    exit 0
}
trap stop TERM INT
# Whichever ends first (PostgreSQL dying, or the command) ends the container.
wait -n "$pg_pid" "$cmd_pid" || true
log "$( kill -0 "$pg_pid" 2>/dev/null && echo 'command exited' || echo 'PostgreSQL exited' ), stopping"
stop
