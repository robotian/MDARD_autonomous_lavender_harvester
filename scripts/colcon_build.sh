#!/bin/bash
# colcon build ~/colcon_ws in every running robot container (a300_0000, j100_0001, ...), as the `robot` user.
#
#   scripts/colcon_build.sh                    # colcon build
#   scripts/colcon_build.sh --symlink-install   # extra args are passed straight to colcon build
#
# colcon_ws is the *same* host folder bind-mounted into every robot container (see docker-compose.yml), so one
# build's output is already visible to all of them; this still builds it once per container (cheap, incremental
# after the first) so a container that only just started, or one running a different image, is never stale.
#
# With no robot container running (a fresh checkout), it builds once in a throwaway container from the robot
# image instead: a real robot's boot needs the built workspace (its robot.yaml's platform.extras.urdf is in
# mtu32_description), so its container stops before colcon_build.sh could run in it.
set -uo pipefail
cd "$(dirname "$0")/.."

# colcon cannot switch an existing build/ + install/ between the copy layout and --symlink-install ("failed to
# create symbolic link ... because existing path cannot be removed: Is a directory"), so the layout used is
# recorded in build/.layout and, if it differs (or is missing while build/install exist from an older copy-style
# build), build/ install/ log/ are cleared once first -- they are generated and gitignored. The marker is
# written before building so a failed build doesn't cause another full clean next time.
BUILD='
    cd ~/colcon_ws
    layout=symlink
    if [ "$(cat build/.layout 2>/dev/null)" != "$layout" ] && { [ -d build ] || [ -d install ]; }; then
        echo "build layout changed to $layout-install: clearing build/ install/ log/ (one-time full rebuild)"
        rm -rf build install log || { echo "could not clear build/ install/ log/ (root-owned files? clear them from the host)" >&2; exit 1; }
    fi
    mkdir -p build && echo "$layout" > build/.layout
    colcon build --symlink-install "$@" && source install/setup.bash
'

mapfile -t robots < <(docker ps --format '{{.Names}}' | grep -E '^[a-z0-9]+_[0-9]{4,}$' | sort)
if [ ${#robots[@]} -eq 0 ]; then
    image=clearpath-robot:jazzy
    docker image inspect "$image" >/dev/null 2>&1 || { echo "no running robot containers and no $image image (docker compose build robot0 first)" >&2; exit 1; }
    # The owner the robot containers give colcon_ws (robot/entrypoint.sh): HOST_UID/HOST_GID, else the checkout's.
    uid=$(sed -n 's/^HOST_UID=//p' .env 2>/dev/null | tr -d '"'); uid=${uid:-$(stat -c %u scripts)}
    gid=$(sed -n 's/^HOST_GID=//p' .env 2>/dev/null | tr -d '"'); gid=${gid:-$(stat -c %g scripts)}
    echo "== no running robot container: building in a throwaway $image as $uid:$gid =="
    # Mounted at the robots' path so --symlink-install's absolute links resolve in them. /home/robot is 750 in the
    # image, and colcon opens absolute paths through it, so root opens it before dropping to the owner.
    docker run --rm -e HOME=/home/robot -v "$PWD/colcon_ws:/home/robot/colcon_ws" --entrypoint bash "$image" -c '
        chmod o+x /home/robot && exec setpriv --reuid="$1" --regid="$2" --clear-groups \
            bash -c "source /opt/ros/jazzy/setup.bash && $3" bash "${@:4}"
    ' bash "$uid" "$gid" "$BUILD" "$@" || { echo "colcon build failed" >&2; exit 1; }
    exit 0
fi

status=0
for r in "${robots[@]}"; do
    echo "== $r =="
    if ! docker exec -u robot "$r" bash -c "$BUILD" bash "$@"; then
        echo "$r: colcon build failed" >&2
        status=1
    fi
done
exit $status
