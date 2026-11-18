# robot/ — robot containers

One image (`Dockerfile`, base `osrf/ros:jazzy-desktop-full`) for every model; only `/etc/clearpath/robot.yaml` differs. Container main process runs as root.

## Boot order (`entrypoint.sh`)

1. Real robot id (`ROBOT_MODEL` contains `_`, e.g. `j100_0921`): `ROBOT_NAMESPACE` = that id (drops the compose slot suffix); written to `/etc/robot_ns_env.sh` so `docker exec` shells get it too. Compose already sets it right (`${ROBOT_MODEL_i}${ROBOT_SUFFIX_i-_000i}`, like the container name), so `printenv` and `sh` agree; this override covers a `docker compose up` without a generated `.env`.
2. Serial: real id with `_`→`-`, else namespace with `_`→`-` (clearpath_config accepts only `<model>-<unit>` or `cpr-<model>-<unit>`).
3. `robot.yaml`: `robot_data/<id>/robot.yaml` copied as is if it exists, except `domain_id` and middleware are rewritten to the fleet's `ROS_DOMAIN_ID`/`FLEET_RMW` (a200_0284's own yaml says domain 1). Otherwise rendered from `/opt/clearpath/robot.<model>.yaml.tmpl` (`__NS__`, `__SERIAL__`, `__RMW__`, `__DOMAIN__`).
4. Creates a placeholder `colcon_ws/install/setup.bash` if nothing is built yet, so sourcing never fails.
5. uid remap of `robot` (below), then `chown -R robot:robot colcon_ws`.
6. `generate_bash` (clearpath_generator_common, same as a real robot's systemd units) → `/etc/clearpath/setup.bash`: ROS, `colcon_ws/install`, `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` from robot.yaml. Sourced, then the `/opt/clearpath_robot_ws` overlay.
7. One-shot `generate_params`, then `generate_srdf`.
8. Exports `ROS_NAMESPACE` (launch files only; `ros2 run` still needs `__ns`).
9. Five background services, each `while true; do X || true; sleep 2; done`, logging to `/tmp/<name>.log`: `robot_state`, `ekf`, `foxglove`, `pruner_stub`, `battery_sim`. `|| true` hides crash loops: check the log.

`docker exec` shells skip the entrypoint: `BASH_ENV`/bashrc source `/etc/ros_env.sh` (`/etc/clearpath/setup.bash` or plain ROS, the overlay, then `/etc/robot_ns_env.sh`).

## `robot` user and colcon_ws

- Image renames Ubuntu 24.04's uid 1000 `ubuntu` account to `robot`.
- At boot `robot` moves to the uid/gid owning the host checkout (`stat /scripts`, read-only and never chowned; `HOST_UID`/`HOST_GID` override; skipped for uid 0).
- `/home/robot/colcon_ws` is one bind mount of host `./colcon_ws`, shared by all robot containers. `robot` is only for `docker exec -u robot` workspace work.
- The workspace reaches `setup.bash` through robot.yaml's `system.ros2.workspaces` (as on a real robot).

## bin/ tools

- `generate_params`: runs `RobotParamGenerator`/`RobotLaunchGenerator` from `clearpath_generator_robot` → `/etc/clearpath/{platform,manipulators,sensors}/{config,launch}`. That package is source-only (clearpath_robot repo, jazzy branch). The Dockerfile builds it plus `clearpath_sensors` (its templates, required) into `/opt/clearpath_robot_ws`. (apt's clearpath_generator_common leaves `generate_sensors()`/`LaunchGenerator` abstract; the unavailable `exec_depend`s are harmless.) Output is for reference only: the launch files expect a real ros2_control hardware interface (the sim drives wheels via OmniGraph), hardcode `use_sim_time=false` and would clash on topics. `ekf` only uses `localization.yaml`.
- `generate_srdf`: `/etc/clearpath/robot.srdf` for MoveIt (`mtu32_bringup` `moveit.launch.py` fails without it). Same steps as `generate_semantic_description` (`SemanticDescriptionGenerator` + `moveit_collision_updater`), but:
  - `--default --always --trials 10000` instead of Clearpath's `--trials 100000`, because ≥ ~30000 always crashes (stack smashing).
  - Trials = random configurations sampled; a pair that never collides in any sample is disabled for good. Too few (`--trials 1`: ~368 pairs) let MoveIt plan the a200_0284 gripper into `rail_link`. 10000 (~1 s, ~265 pairs) keeps those enabled. Stripping all "Never" pairs (~47 left) made `moveit_servo` jerky.
  - Runs on local FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, discovery `LOCALHOST`, no `ZENOH_*`): under zenoh it writes the SRDF and then aborts at exit (zenoh's Tokio runtime panics in atexit, rc -6), which looked like a random crash (~50%) and on 2026-10-09 failed every attempt on every robot. Retries 10000×3, 5000×2, 2000; a failed attempt's file is deleted, and there is no `--trials 1` fallback (unsafe matrix): no SRDF = upstart refuses with a clear message.
  - Runs `generate_description` itself (`robot_state` starts after it) and prunes dangling joints first.
- `robot_state`: `generate_description` + xacro → `robot_state_publisher` (namespaced, `joint_states:=platform/joint_states`). Before publishing it:
  - prunes dangling joints (mtu32_description mounts `camera_1` on `arm_0_end_effector_link` even without an arm, e.g. j100_0922; the strict loader aborts);
  - turns `arm_0_joint*` continuous joints into ±3.12 rad revolute (else MoveIt plans across ±π and the start state goes invalid);
  - if either changed the URDF, overwrites `robot.urdf.xacro` with it too (moveit.launch.py reads the xacro);
  - rewrites `file://.../share/<pkg>/` meshes to `package://` in the published topic only (foxglove_bridge serves only `package://`);
  - with `ee_cutter_link` in the description (the cutter EE), drops the 2F Lite finger links' visuals and makes the EE's fixed `ee_cutter_<body>_joint` prismatic (slide, z) / revolute (x) in the published topic only (RViz/Foxglove would draw the fingers inside the EE, and the linkage and blade follow the sim's `platform/joint_states`; MoveIt reads the files).
- Dangling-joint pruning is copied inline from `scripts/flatten_urdf.py` (host-side, not in the image): keep the copies in sync.
- `ekf`: `robot_localization` `ekf_node` with the generated `localization.yaml`, `publish_tf:=true` → `platform/odom/filtered` + `odom→base_link`, future-dated by `transform_time_offset:=0.1`: a camera image can be stamped a frame (33 ms) after the EKF's latest TF, and Nav2's docking server then can't move the detected dock pose into `odom` ("extrapolation into the future" → "Failed to get control"). The sim's exact pose is only `ground_truth→base_link_ground_truth`, so base_link has one parent.
- `foxglove`: port 8765 (compose maps a host port per robot), topics whitelisted to `^/<ns>/.*` + `/rosout`.
- `pruner_stub`: fake OpenCR firmware for `pruner_action_server`. PTY with `/dev/ttyOpenCR` symlinked to the slave. On any `<int>\n` it cycles the sim's cutter blade (`end_effector: ee_cutter`, `sim/scripts/ee_cutter.py`): `pruner/blade` 1.0 until `ee_cutter_blade_joint` (in `platform/joint_states`) has closed and stopped, 0.0 until it is open, then `STATUS:DONE\n` (each step at most 3 s, sim time). Without that joint it waits 2 s. It never fails.
- `battery_sim`: simulated BMS, `sensor_msgs/BatteryState` on `platform/bms/state` at 1 Hz (sim time), as a real robot's battery publishes it (husky_operations_manager charges on it). Full to empty in `runtime_hours` (j100/a200 1.5 h, a300 2.5 h); full in `charge_hours` (2) while base_link is within `dock_radius` (0.15 m) of a dock pose of its Nav2 docking server (`/tmp/<ns>_nav2_map.yaml`, written by `bringup_nav2_map`) and slower than 5 cm/s, pose from `ref_pose`; before Nav2 has written that file, within `charge_radius` (1.3 m) of a charger in `/generated/fleet/state.json` (`./sim/generated`, read-only). Nav2's docking server (sim docks have `use_battery_status`) ends the dock as soon as this charging current appears, so it must not appear at the staging pose: with the 1.3 m radius the j100 "docked" at its staging pose, 0.8 m short (2026-10-15). Volts/Ah from robot.yaml's `platform.battery` (`BATTERY_MODELS`), else the platform's usual pack. Starts full on every container start; `ros2 param set /<ns>/battery_sim percentage <0-100>` sets it.
- `restart_ros`: `pkill -9` of everything ROS (`ros2 run|launch`, `/opt/ros/*/lib/`, `colcon_ws/install/*/lib/`, pruner_stub, battery_sim). After 5 s it reports whether the five looped services are back. Manually launched stacks stay stopped.
- `teleop` (TwistStamped, `stamped:=true`), `rviz` (from `robot.rviz.tmpl`), `camera_view` (`rqt_image_view` on `sensors/camera_0/<color|depth>/image`). `arm_goto`/`arm_joints` are shims to `moveit_sim_bridge` in colcon_ws.
- GUIs (RViz, `camera_view`) render with the NVIDIA driver's OpenGL (compose's GPU reservation + `NVIDIA_DRIVER_CAPABILITIES=graphics`; RViz logs `OpenGl version: 4.6`). Without it they fall back to Mesa's software renderer (`4.5`), whose shader linker rejects RViz's Map display: two nav2 views crashed both RViz. A container from before 2026-10-09 has to be recreated (`scripts/fleet.sh`).

## Sim time

`USE_SIM_TIME` (a fleet setting) is in every container's environment. `robot_state`, `ekf`, `foxglove`, `teleop` and `rviz` pass `-p use_sim_time:=${USE_SIM_TIME:-false}`. In `ekf` it comes after `--params-file` because the generated `localization.yaml` says `False`.

## Upstream quirks and workarounds

- `generate_bash` (clearpath_generator_common 2.9.15) uses `workspace.strip('setup.bash')`, a character-set strip: keep the template's `/home/robot/colcon_ws/install` exactly (immune). Other spellings get mangled (`colcon_ws` → `colcon_w`).
- `intel_realsense.urdf.xacro` is patched in the Dockerfile with `use_nominal_extrinsics="true"`, because no realsense driver publishes the depth/color frames in the sim (`camera_0_depth_frame`, `camera_0_color_optical_frame`).
- Deleting `colcon_ws/install` under a running container: new `docker exec` shells print one `No such file` line and carry on; a rebuild fixes it.
