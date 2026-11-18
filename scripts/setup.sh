#!/bin/bash
# A new checkout, from clone to ready for scripts/fleet.sh: every PC ends up with the same fleet and farm.
#
#   scripts/setup.sh            check the host, then do whatever is still missing (safe to rerun)
#   scripts/setup.sh --check    only report what is missing
#   scripts/setup.sh --rebuild  also rebuild the images that already exist
#
# Host setup (driver, Docker, NVIDIA Container Toolkit, the docker group) needs sudo and is left to you: the
# check names the command for each missing piece (README "Requirements"). The rest it does itself:
#   submodules + LFS files, the images (robot, sim, base station), db.env (random password), the settings
#   (scripts/fleet_default_profile.json: the lavender farm with j100_0921 on Newton), colcon_ws (scripts/colcon_build.sh),
#   the base station (a new database starts from basestation/seed/farm.sql: every plant pending).
set -uo pipefail
cd "$(dirname "$0")/.."

CHECK=0 REBUILD=0
for a in "$@"; do
    case "$a" in
        --check) CHECK=1 ;;
        --rebuild) REBUILD=1 ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $a" >&2; exit 2 ;;
    esac
done

ok()   { echo "  ok    $*"; }
todo() { echo "  todo  $*"; }
bad()  { echo "  FIX   $*"; FAIL=1; }
step() { echo; echo "== $*"; }
run()  { echo "  \$ $*"; "$@"; }
FAIL=0

# ---------------------------------------------------------------- host (only checked)
step "host"
if ! command -v nvidia-smi >/dev/null; then
    bad "no NVIDIA driver: sudo apt install nvidia-driver-580 (or newer), then reboot"
else
    drv=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
    if [ -z "$drv" ]; then
        bad "nvidia-smi fails (a driver update without a reboot?): reboot"
    elif [ "${drv%%.*}" -lt 580 ]; then
        bad "NVIDIA driver $drv: Isaac Sim 6.0 wants 580 or newer"
    else
        ok "NVIDIA driver $drv ($(nvidia-smi --query-gpu=name --format=csv,noheader | head -1))"
    fi
fi
if ! command -v git-lfs >/dev/null; then
    bad "no Git LFS: sudo apt install git-lfs, then git lfs install"
else
    ok "Git LFS"
fi
DOCKER=0
if ! command -v docker >/dev/null; then
    bad "no Docker: install Docker Engine and the NVIDIA Container Toolkit (README \"Requirements\")"
elif ! docker info >/dev/null 2>&1; then
    if id -nG | tr ' ' '\n' | grep -qx docker || getent group docker | grep -qw "$USER"; then
        bad "Docker refuses you: you were added to the docker group, log out and back in"
    else
        bad "Docker refuses you: sudo usermod -aG docker \$USER, then log out and back in"
    fi
else
    DOCKER=1
    cv=$(docker compose version --short 2>/dev/null || true)
    if [ -z "$cv" ]; then
        bad "no Docker Compose plugin: sudo apt install docker-compose-plugin"
    elif [ "$(printf '%s\n' 2.24 "$cv" | sort -V | head -1)" != 2.24 ]; then
        bad "Docker Compose $cv: 2.24 or newer needed"
    else
        ok "Docker $(docker version --format '{{.Server.Version}}' 2>/dev/null), Compose $cv"
    fi
    if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
        ok "NVIDIA Container Toolkit"
    else
        bad "Docker has no nvidia runtime: install nvidia-container-toolkit, then" \
            "sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
    fi
fi
if command -v ufw >/dev/null && systemctl is-active -q ufw 2>/dev/null; then
    echo "  note  ufw is active: after the first scripts/fleet.sh, sudo ufw allow in on br-fleet (sim robots ->" \
         "base station)"
fi
if [ "$FAIL" = 1 ]; then
    echo; echo "Fix the FIX lines above, then run scripts/setup.sh again."
    exit 1
fi

# ---------------------------------------------------------------- the checkout
step "checkout"
missing_sub=$(git submodule status --recursive | grep -c '^-' || true)
if [ "$missing_sub" -gt 0 ]; then
    if [ "$CHECK" = 1 ]; then todo "$missing_sub submodule(s) not checked out"; else
        run git submodule update --init --recursive || exit 1; fi
else
    ok "submodules"
fi
lfs_ptr=$(git lfs ls-files 2>/dev/null | grep -c ' - ' || true)  # " - " = pointer only, "*" = file present
if [ "$lfs_ptr" -gt 0 ]; then
    if [ "$CHECK" = 1 ]; then todo "$lfs_ptr Git LFS file(s) not downloaded"; else
        run git lfs install --local >/dev/null && run git lfs pull || exit 1; fi
else
    ok "Git LFS files"
fi
if [ ! -s db.env ]; then
    if [ "$CHECK" = 1 ]; then todo "db.env (the base station database password)"; else
        echo "PGPASSWORD=$(python3 -c 'import secrets; print(secrets.token_hex(16))')" > db.env
        chmod 600 db.env
        ok "db.env written with a random database password"
    fi
else
    ok "db.env"
fi

# ---------------------------------------------------------------- images
step "images"
image_build() {  # image_build <image> <what> <command...>
    local img=$1 what=$2; shift 2
    if docker image inspect "$img" >/dev/null 2>&1 && [ "$REBUILD" = 0 ]; then
        ok "$img"
    elif [ "$CHECK" = 1 ]; then
        todo "$img ($what)"
    else
        run "$@" || { echo "building $img failed" >&2; exit 1; }
    fi
}
image_build clearpath-robot:jazzy "every robot and the zenoh router" docker compose build robot0
image_build a300-isaac-sim:6.0.0 "the sim; pulls nvcr.io/nvidia/isaac-sim:6.0.0, docker login nvcr.io if refused" \
    docker compose build isaac-sim
# Compose 2.38+ builds through buildx bake, which refuses the base station's build network: host unless granted,
# and `docker compose build` can't pass the grant
image_build basestation:jazzy "the base station" \
    docker buildx bake -f basestation.compose.yml --allow=network.host --load

# ---------------------------------------------------------------- settings and workspace
step "settings"
if [ ! -f fleet_config.sqlite ]; then
    if [ "$CHECK" = 1 ]; then todo "fleet_config.sqlite (from scripts/fleet_default_profile.json)"; else
        run scripts/fleetcfg.py render >/dev/null || exit 1; fi
else
    ok "fleet_config.sqlite (profile $(scripts/fleetcfg.py profile list 2>/dev/null | sed -n 's/^\* *//p' | awk '{print $1}'))"
fi

step "colcon_ws"
if [ ! -f colcon_ws/install/mtu32_description/share/colcon-core/packages/mtu32_description ]; then
    if [ "$CHECK" = 1 ]; then todo "colcon_ws not built"; else
        run scripts/colcon_build.sh || exit 1; fi
else
    ok "colcon_ws built (after editing packages: scripts/colcon_build.sh)"
fi

# ---------------------------------------------------------------- base station
step "base station"
if [ "$(docker inspect -f '{{.State.Running}}' basestation 2>/dev/null)" = true ]; then
    ok "running (docker compose -f basestation.compose.yml up -d applies setting changes)"
elif [ "$CHECK" = 1 ]; then
    todo "not running (its first start loads the farm: basestation/seed/farm.sql)"
else
    run docker compose -f basestation.compose.yml up -d || exit 1
    for _ in $(seq 60); do
        [ "$(docker inspect -f '{{.State.Health.Status}}' basestation 2>/dev/null)" = healthy ] && break
        sleep 2
    done
    docker logs basestation 2>&1 | grep -E '^\[(basestation|migrate)\]' | sed 's/^/  /' | head -8
fi

echo
if [ "$CHECK" = 1 ]; then
    echo "Check done. scripts/setup.sh does the todo lines."
else
    echo "Ready. Next:"
    echo "  scripts/fleet.sh                      start the sim (the lavender farm) and spawn j100_0921"
    echo "  python3 tools/sim_ui/server.py        web UI on http://127.0.0.1:8090 (operation: /operation)"
    echo "  scripts/fleetcfg.py show              the settings (or the web UI's /config page)"
fi
