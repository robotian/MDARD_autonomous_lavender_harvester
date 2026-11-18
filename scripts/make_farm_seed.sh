#!/bin/bash
# Make basestation/seed/farm.sql, the farm database a new base station starts with (basestation/entrypoint.sh
# loads it when basestation/initdb/ has no data of its own): a farm dump's layout with every trace of the
# operation that ran on it cleared, so every new checkout starts from the same farm.
#
#   scripts/make_farm_seed.sh [DUMP]   default: the newest basestation/shared/*.dump
#
# Kept: the farm (plants, rows, route graph, farm nodes and edges, assets, chargers, docks), the robot registry
# (robot_info) and the operation's settings (target rows, max attempts). Cleared: every plant pending again, the
# operation stopped with the coordinator off, no jobs, events, live positions or history, docks free, and only
# SEED_ROBOT (j100_0921, the default sim robot) enrolled: the others stay registered, so they can be added on the
# /operation page, but the coordinator gives them nothing.
#
# Runs in a scratch database inside the running base station container; its own database is not touched.
set -euo pipefail
cd "$(dirname "$0")/.."

dump=${1:-$(ls -1 basestation/shared/*.dump 2>/dev/null | sort | tail -1)}
[ -f "$dump" ] || { echo "no dump: pass one, or put one in basestation/shared/" >&2; exit 1; }
head -c 30 "$dump" | grep -q '^version https://git-lfs' && { echo "$dump is a Git LFS pointer: git lfs pull" >&2; exit 1; }
[ "$(docker inspect -f '{{.State.Running}}' basestation 2>/dev/null)" = true ] ||
    { echo "the base station isn't running (docker compose -f basestation.compose.yml up -d)" >&2; exit 1; }
SEED_ROBOT=${SEED_ROBOT:-j100_0921}
tmp=farm_seed_tmp
out=basestation/seed/farm.sql

bs() { docker exec -i -e PGDATABASE=$tmp basestation "$@"; }
docker exec basestation dropdb --if-exists $tmp
docker exec basestation createdb $tmp
trap 'docker exec basestation dropdb --if-exists $tmp' EXIT

echo "restoring $dump into a scratch database"
bs pg_restore --exit-on-error --no-owner -d $tmp < "$dump"
bs /basestation-migrate.sh /migrations

echo "clearing the operation's state (enrolled: $SEED_ROBOT)"
bs psql -q -v ON_ERROR_STOP=1 -v robot="$SEED_ROBOT" <<'SQL'
BEGIN;
TRUNCATE public.farm_harvesting_job, public.operation_event, public.robot_live, public.robot_history;
UPDATE public.harvest_state SET status = 'pending', lane_id = NULL, pickup_node_id = NULL, assigned_robot_id = NULL,
    job_id = NULL, attempts = 0, last_error = NULL, assigned_at = NULL, harvested_at = NULL, updated_at = now();
UPDATE public.operation SET state = 'stopped', coordinator_enabled = false, started_at = NULL, note = NULL,
    updated_at = now();
UPDATE public.dock_station SET status = 0, robot_id = NULL;
UPDATE public.robot_operation SET enrolled = (robot_id = (SELECT id FROM public.robot_info
    WHERE serial_number = :'robot')), state = 'active', request = NULL, updated_at = now();
COMMIT;
SQL
[ -n "$(bs psql -tAq -v robot="$SEED_ROBOT" <<<"SELECT 1 FROM public.robot_info WHERE serial_number = :'robot';")" ] ||
    { echo "$SEED_ROBOT is not in the dump's robot_info: nothing enrolled" >&2; exit 1; }

mkdir -p "$(dirname "$out")"
# --no-owner/--no-privileges: loads as whichever superuser the new base station has
bs pg_dump --no-owner --no-privileges > "$out.tmp"
mv "$out.tmp" "$out"
echo "wrote $out ($(du -h "$out" | cut -f1)) from $(basename "$dump"):"
bs psql -tA <<'SQL'
SELECT ' plants ' || count(*) || ', all pending: ' || bool_and(status = 'pending') FROM public.harvest_state;
SELECT ' operation ' || state || ', coordinator_enabled ' || coordinator_enabled || ', target_rows ' ||
       coalesce(target_rows::text, 'all') FROM public.operation;
SELECT ' enrolled: ' || string_agg(i.serial_number, ' ') FROM public.robot_operation o
       JOIN public.robot_info i ON i.id = o.robot_id WHERE o.enrolled;
SQL
