# colcon_ws/ — MTU bringup, MoveIt bridge, cut_stem, Nav2

Sim-side arm drives are in `sim/CLAUDE.md`, container services (`robot_state`, `ekf`, `pruner_stub`) in `robot/CLAUDE.md`, deploying to a robot in `scripts/CLAUDE.md`, per-robot findings in skill `add-real-robot`.

## What's in `src/`

- `mtu32_husky/mtu32_bringup` (submodule): MTU's launch files and per-platform config in `config/<platform>/` (`j100`/`a200`/`a300`).
- Copied verbatim from `robot_data/j100_0921/colcon_ws/src/` (not apt): `docking_utils`, `stow_arm_cpp`, `pruner_action_server`, `kinova_game_pad`, `laser_filters`, `depthimage_to_laserscan`, `plant_cutter_msgs`, `serial_interfaces`, `dual_duro_heading`.
- `moveit_servo`: patched copy overlaying apt's 2.12.4 (`MTU_PATCH.md`). With `use_sim_time` its loop steps `publish_period` on the node clock (upstream's wall-rate loop moved the arm ~2x too far per sim second); wall time unchanged.
- `moveit_sim_bridge` (ours): arm execution in the sim, plus the `arm_goto`/`arm_joints` helpers (`arm_goto.joint_goal` builds the move_group goal `arm_task_server` sends too).
- `arm_task_server` (ours): the robot's `manipulator_action_server` (`status_interfaces/action/ManipulatorTask`) that `husky_operations_manager` drives: `GO STOW` / `GO READY` / `GO DROP` = move_group to the SRDF state (`stow`, platform ready state: j100 `cut_init`, a200/a300 `ready`, `drop`), `START HARVEST` = `cut_stem`; `GO STOW` preempts a running goal (a restarted executor takes the arm back). `husky_operations_manager` holds every arm goal until move_group's `move_action` server is up (`arm.move_group_wait` 180 s, then it sends anyway; no `moveit_msgs`, no wait): move_group comes up 15-30 s after the stack. Launched by `bringup_main` next to the grid cutter (robots with an arm), timeouts on the node clock (`harvest_timeout` 900 s).
- `status_server` (ours): the harvesting operation's coordinator, run in the base station (see `tools/sim_ui/CLAUDE.md` "Operation page", its README for the config, tables and the failure contract).
- `husky_operations_manager` (submodule, branch `waypoint_based_navigation`): the per-robot executor of the coordinator's tasks (`status/task` → navigate, dock/charge, harvest; `status/robot` back).
- Sim stand-ins: `swiftnav_ros2_driver` holds only the real `Baseline.msg`; `duro_sim` provides `baseline_node`.

## Launch structure

`mtu32_bringup/launch/sim_robot_upstart.launch.py` is the sim's main launch. It starts:
- `SetParameter use_sim_time` (default `$USE_SIM_TIME`, so false on a real robot);
- the `moveit_sim_bridge` node;
- `bringup_main.launch.py` with `use_natnet:=false` (the sim publishes `ref_pose`) and `moveit:=true`. It forwards `ref_source`, `ref_anchor`, `moveit_delay`, `use_nav2`, `nav2_map` and `scan_topic`;
- `sim_swift_nav_dual.launch.py` when `use_gps_localization` is set (default) and robot.yaml has two or more GPS sensors (generic models have none).

No EKF is started here: the container's `ekf` service publishes `odom→base_link` and `platform/odom/filtered`.

`bringup_main.launch.py` runs on real robots too, via `clearpath-platform-extras`. It starts:
- sensing: `scan_to_scan_filter_chain`, `pcl_filter.launch.py`, `depthimage_to_laserscan` (`config/<platform>/depth2scan.yaml`), `apriltag_node`, `tf2_pose_node`;
- localization: `ref_localizer` (`use_ref_localizer`) and `natnet_ref_pose` (`use_natnet`);
- the cutter stack: `stow_arm_cpp/launch/grid_cutter.launch.py`, `pruner_server` on `/dev/ttyOpenCR` (`pruner_stub` in the sim), and `kinova_game_pad`'s `cut_stem_gamepad.launch.py` (namespace defaults to `j100_0921`);
- `moveit.launch.py` (move_group + servo_node), after `moveit_delay`; move_group respawns (2 s): MoveIt 2.12.4 segfaulted when two goals met at its start (j100_0921, 2026-10-21) and every later arm goal failed "unavailable". A goal running when it dies is lost: `arm_task_server` waits out `move_timeout` (90 s) and its 60 s cancel wait, the executor's arm goal fails at 120 s;
- `bringup_nav2_map.launch.py`, unless `use_nav2:=false`; it writes the Nav2 params it runs with to `/tmp/<ns>_nav2_map.yaml` (in the sim the docks are the scene's chargers, named by `dock_id`);
- `arm_task_server` (with the arm) and, with `ops_manager:=true` (the sim's upstart default; real robots opt in), `husky_operations_manager` after Nav2, reading its docks from that `/tmp/<ns>_nav2_map.yaml`, `dock_for_charging` defaulting to its first dock.

- Nav2 for j100 (2026-10-13, `docs/nav2_tuning_j100.md`): Graceful controller (not MPPI: it couldn't stop within 0.05 m and cut the lane U-turn into the plants when retuned), `NavigateThroughPoses` BT that replans only when the path becomes invalid (`config/behavior_trees/`; a periodic DUBIN replan near a goal returns a loop), and a global costmap from the farm map only (`nav2_robots.yaml` `platforms`: the ZED scan marked the lavender beside the lane as lethal). Since 2026-10-14 (not re-tuned) every platform's local costmap and collision monitor see the forward scan: the front lidar, else the front camera's depth scan (j100_0936 its SICK, j100_0921/0922 the ZED). Retuned 2026-10-14 after the sim drive retune: planner `minimum_turning_radius` 0.4 (0.2 made a ~110° turn right at the Staging goal) and `v_linear_min` 0.10; `slowdown_radius` 1.2 and a `NavigateToPose` tree that replans only when the path breaks (j100, a200, a300: the default replanned every second and the j100 drove figure eights around a single goal pose).
- Nav2 for a300 (2026-10-14, `docs/nav2_tuning_a300.md`): the same recipe (`config/a300/nav2_map.yaml`, `platforms.a300`), with `prefer_final_rotation: false` and `k_delta` 0.5: the 0.99 × 0.70 m robot can't turn in place in a lane and reaches the plants more than ~20° off it, so it lines up with the goal on the approach. 0.5 m/s (was MPPI at 0.2); 1.0 m/s finished fewer legs.
- Nav2 for a200 (2026-10-14, `docs/nav2_tuning_a200.md`): the a300's set in `config/a200/nav2.yaml` with a 2 m lookahead (1.2 weaved ±5 cm in the lanes) and 0.10 m / 0.10 rad tolerance, plus SmacPlannerHybrid DUBIN instead of the NavFn it shipped with; map-only global costmap in `platforms.a200`, local obstacle layer on the front lidar only (generic a200: its camera). Tune with `scripts/nav2_tune/`.
- Since 2026-10-21 every platform runs **MPPI** (it goes round obstacles that appear on the path; Graceful only follows the plan), tuned on Newton after the sim's wheel-damping fix (`sim/CLAUDE.md`): forward only (`vx_min` 0), `ax_max`/`ax_min` = the velocity smoother's ±1.0 (at ±0.1 it crawled; at ±2.5 with the old sim drive it oscillated and tipped j100_0921), j100's critics. a200/a300 also: `vx_max` 0.5, goal tolerance 0.10 m / 0.10 rad (the a300's 0.05 never "reached" a goal). Every platform: progress checker 0.25 m / 15 s (0.5 / 10 aborted the slow final approach, and can't pass a 0.45 m plant-to-plant leg); j100's GoalCritic acts from 1.0 m (lane b4 harvest course 2026-10-21: 11/11, 190 s, 7.6 cm). Farm course on Newton: a200_0284 7/7 (8 cm, 0 recoveries; Graceful 4/7), a300_00036 5-6/7 (Graceful 2/7; its final approach can stall ~11 cm short), j100_0921 4/7 (weaves through U-turns; Graceful 5/7). The Graceful notes above are the previous tuning. `back_0.5` (0.5 m straight reverse) can't pass with DUBIN and no reversing.
- a200 MPPI since 2026-10-28 (`docs/nav2_tuning_a200.md`): `open_loop: true` too; a200_0284 farm course 0/7 -> 7/7 twice (5 cm, 0 recoveries).
- a300 MPPI since 2026-10-27 (`docs/nav2_tuning_a300.md`): `open_loop: true` for the same reason as the j100's below; a300_00036 farm course 0/7 (147 recoveries) -> 7/7 twice (5 cm, 0 recoveries). 0.05 m tolerance: 2.3 cm but the ring leg aborted 6 cm short.
- j100 MPPI since 2026-10-27 (`docs/nav2_tuning_j100.md`): `open_loop: true` (each optimization starts from the last command, not the measured speed: with the EKF odometry ~50-140 ms behind, closed-loop MPPI raised its command ~0.05 m/s per s, overshot goals while the stateful goal checker counted them reached, and weaved; it failed the west end of row 2 overnight), `wz_max` 1.0, `vx_max` 0.5. Keep Nav2's default trees with MPPI: with the replan-only-when-invalid trees it stuck in tight turn-arounds (U-turn in a lane, 180° at the start). Farm course 7/7 (3.2 cm, 0 recoveries), lane-entry course (`scripts/nav2_tune/course_lane_entry.json`) 7/7 (4.4 cm, footprint ≥ 30 cm from the rows), camera `scan_layer` kept.
- Other robots (2026-10-15, every platform): the global costmaps hold the farm map plus `fleet_layer` (`nav2_robots.yaml` `defaults`), fed by `scripts/fleet_obstacles.py` (started by `bringup_nav2_map`): it fills the other robots' `global_costmap/published_footprint` (map frame, 1 Hz; robots in the same ROS graph only) as a marking cloud and sends an all-clear 360° scan for clearing, so the planner routes round a robot and a moved robot leaves no trace. Without it two robots planned through each other and stopped nose to nose for hours. Head-on in the strip north of row 1: passed 1.17 m apart (centres), j100 swerved 2.5 m. It can't free a robot already pushed into the crop margin ("Start occupied").
- j100 global costmap, work in progress (2026-10-15): `scan_layer` (camera scan for marking with `observation_persistence` 5 s, `fleet_obstacles.py`'s all-clear scan for clearing) after the map's 0.8 m `inflation_layer`, with its own `scan_inflation` of 0.3 m (`nav2_robots.yaml` `platforms.j100`). Meant to let the planner go round what the camera sees without keeping it, but it marks nothing yet (global cost unchanged with the robot facing an obstacle 1.6 m away). Measured with a plain permanent `obstacle_layer` in that place (which kept every obstacle once out of view): 8 m into a lane without a planning failure (0.3 m on scanned lavender keeps the lanes open; 0.8 closed them), and round a robot without Nav2 parked in the strip north of row 1. a200/a300 are map-only.
- `moveit:=auto` starts MoveIt unless robot.yaml has `manipulators.moveit.enable: true` (Clearpath's `clearpath-manipulators` then runs move_group); see the "MoveIt: ..." log line.
- Open TODO: the cutter stack starts on every robot, but only j100_0921 has the hardware; gate it per robot.

`use_sim_time` reaches every node through `SetParameter`, but a node's own params file overrides it. That's why `bringup_nav2_map` and `sim_nav2` write the value into every `ros__parameters` of the generated Nav2 yaml: some `nav2*.yaml` hardcode false.

Python nodes of ours don't use `rclpy.spin()` / `executor.spin()`: with `use_sim_time` they never return after SIGTERM/SIGINT (the signal shuts the context down, the timers run on the ROS clock and `/clock` stops with the context, so nothing wakes the wait; the node ignored `kill` and launch's SIGINT until SIGKILL). Spin `while rclpy.ok(): executor.spin_once(timeout_sec=0.5)`, catch `ExternalShutdownException`, and end with `rclpy.try_shutdown()` (an `if rclpy.ok(): rclpy.shutdown()` races the signal handler's own shutdown and exits 1). Done in `husky_operations_manager`, `arm_task_server`, `moveit_sim_bridge`, `status_server`, `robot/bin/battery_sim`.

## Arm execution in the sim: `moveit_sim_bridge`

The sim arm has no ros2_control: Isaac reads `arm_0/joint_command` (JointState) directly. Without the bridge, move_group finds no controller and every execute returns `CONTROL_FAILED`. `bridge_node.py` provides three interfaces.

`manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory` (FollowJointTrajectory):
- Interpolates between waypoints at 50 Hz (`INTERP_PERIOD_S`); raw waypoint steps made the drive overshoot.
- Reports done only once the arm settles: `settle_tolerance` 0.03 rad, `settle_timeout` 10 s.

`manipulators/arm_0_gripper_controller/gripper_cmd` (GripperCommand):
- `position` is the driven joint's value; the mimic joints follow from the URDF multipliers and offsets.
- Joints are read from `robot_description` (prefix `gripper_joint_prefix`) unless `gripper_joint_names`/`gripper_joint_multipliers` are set.
- The target ramps at the driver's URDF velocity limit (2F Lite 0.6 rad/s, 50 Hz): a step made the drive snap (Newton: 6.9 rad/s, past the joint limit).
- Finishes within `gripper_tolerance` (0.02) of the target after the ramp, on a stall (no motion for 0.5 s after the ramp; judged mid-ramp, a data lag at the start of a cut once counted as one), or `gripper_timeout` (3 s) after the ramp.

A subscriber on `.../arm_0_joint_trajectory_controller/joint_trajectory`: servo streams there (`command_out_type: trajectory_msgs/JointTrajectory`), not through the action.

Pitfalls:
- Every publish must carry every joint ever commanded. The sim keeps only the last message's arrays, so an arm-only message drops the gripper target.
- Don't use a fixed sleep for GripperCommand. MoveIt allows planned duration × 1.2 + 0.5 s, so a near-zero plan times out.
- With `use_sim_time`, `moveit.launch.py` raises move_group's `allowed_goal_duration_margin` to 12 s (a `<ns>/move_group` params file after `moveit.yaml`; a parameters dict would lose to its node-specific value): data stalls through the shared zenoh router made moves time out while the arm got there. The bridge's `settle_timeout` decides instead. Real robots keep the generated 0.5 s.
- Don't copy `position` to every gripper joint. The 2F Lite tip mimics use −0.676 with a +0.149 offset, so copying moves them backwards.

## Arm helpers (`moveit_sim_bridge`, deployed to real robots)

- `ros2 run moveit_sim_bridge arm_goto <state> [--group G] [--velocity-scale S] [--direct]`: a group_state from `/etc/clearpath/robot.srdf` via `move_action`; `--direct` bypasses MoveIt for the bridge (sim only).
- `arm_joints [--record S]` compares `arm_0/joint_command` with `platform/joint_states`. A real robot has no `joint_command`.
- `robot/bin/arm_goto`/`arm_joints` are wrappers. The web UI uses them on sim and real robots (real robots: only after a deploy). Namespace: `$ROBOT_NAMESPACE`, else robot.yaml `system.ros2.namespace`.
- Ctrl+C: `arm_goto` exits 130 but the goal keeps running (stop it with the UI's Stop motion); `arm_joints --record` exits 0. Any exception raised while `not rclpy.ok()` counts as the interrupt.
- `--timeout` (60 s) is wall time, so in a slow sim a move can still finish after "no result".
- Each `arm_goto` is a new zenoh session: finding move_action takes seconds (waits 15 s) and the goal's answer arrived 6-11 s after the request with three robots moving at once (waits 30 s, logs "goal accepted after N s" past 5 s). "no answer to the goal request" ≠ rejected: move_group may still run it.

## `cut_stem` / `grid_cutter_action_server` (`stow_arm_cpp`)

Start a cut: `ros2 action send_goal /<ns>/cut_stem plant_cutter_msgs/action/CutStem "{start_cutting: true}"`.

Configuration:
- `config/grid_cutter_params.yaml` holds the shared values, tuned for j100_0921 (Gen3 Lite + 2F Lite).
- `grid_cutter.launch.py` then loads `config/robots/<namespace>.yaml` if it exists. Per-robot differences go there; don't fork the package.
- `j100_0921.yaml` sets the IK seed, `tool_xyz` [0,0,0.05], `moveit_vel_scale` 0.9, the drop pose (below) and `gripper_close` 0.96 (full close, the Kinova joint's limit: the cutter EE's finger tip at -88.7°, ~1 mm from the jaw horn; 0.8 was ~-70°).
- `a200_0284.yaml` (Gen3 7-DOF + Robotiq 2F-85): zone x 0.44–0.56, 7-joint seed, `drop_joint_positions`, vel/acc scale 0.4 (its sim wrists are slower than the URDF limits), `gripper_close` 0.8, tool offset with matching `grasp_orientation`, ground box lowered for its 0.411 m arm base.

Parameters:
- `preferred_joint_positions` is the IK seed: every pose takes the nearest solution to it. Pick one that keeps the whole zone, approach points included, on one branch.
- `drop_joint_positions`: empty means the SRDF `drop` pose. a200_0284 has no `drop` pose; its joint values are a guess above its `basket` box and should be replaced with a real pose.
- `drop_lower_distance` (0.1): how far the tool goes down after the drop. Not re-verified on a200_0284; if it stops at "singularity", set 0 in its robot file.

Tool frame:
- Params: `tool_link`, `tool_xyz`, `tool_rpy` (the tool pose in `arm_0_end_effector_link`; identity by default) and `publish_tool_tf` (static TF on `tf_static`).
- All patch, approach and drop positions, and `grasp_orientation`, are `tool_link` poses. IK gets the matching ee pose; servo feedback reads base→`tool_link`.
- After changing the offset or rpy, re-check the zone, seed, drop pose and `grasp_orientation`.
- Measure fingertips with `tf2_echo arm_0_end_effector_link <fingertip link>`. Robotiq open: x=±0.068, z=0.098.

Clocks:
- Node clock (`deadline_in`/`ros_sleep`): timeouts, retry pauses and TF-settle waits.
- Wall time, on purpose: `servo_timer_`, action discovery waits, `pruner_server`'s serial timeout, `tf2_pose_node`'s timer.

## Sim-side arm facts behind cut_stem

- Drive force = `ARM_EFFORT_SCALE` (3.0) × the URDF effort for each `arm_0_joint_N` (`configure_arm_drives`). At 1×, joint 2 saturates and the arm collapses. Why the sim needs more torque than the real arm is still unknown.
- `drop_mimic_constraints` (default on for every arm) removes the importer's `NewtonMimicAPI`, which fought the drives.
- Debugging: `arm_joints --record`, the effort field of `platform/joint_states` (a spike of hundreds of N·m means a collision), and `ros2 topic pub` on `arm_0/joint_command`. Only trust these on a fresh sim.

## Nav2

`bringup_main` starts `bringup_nav2_map.launch.py` at boot, the same in the sim and on real robots. It runs map_server and the navigation servers, but no AMCL: `map→odom` comes from `ref_localizer`. Its lifecycle managers use a 20 s `bond_timeout` (Nav2's 4 s reset every server, or aborted the bring-up, whenever a stack's start kept the zenoh router busy).

Per-robot settings live in `config/nav2_robots.yaml`, merged in the order defaults → `platforms.<model>` → `robots.<namespace>`:
- `params_file`: default `[nav2_map.yaml, nav2.yaml]`, the first that exists in `config/<platform>/`.
- `scan_topic`: `auto` (the lidar if robot.yaml has one, else `camera_0`) for j100 and a200; a300 `sensors/camera_0/scan` (its Hokuyo faces the right side). Obstacle detection looks forward: a robot whose lidar doesn't face the front sets its camera's scan. `params` keeps the params file's sources.
- `map`: default `zone_end_2.yaml`.
- `param_overrides`: values merged over the params file.

From `bringup_main` or the upstart, override with `nav2_map:=` / `scan_topic:=`. Starting another Nav2 by hand (`sim_nav2`, `bringup_nav2_*`) needs `use_nav2:=false`, or two stacks run.

`sim_nav2.launch.py`, the sim's manual Nav2:
- uses `config/<platform>/nav2.yaml` with `platform/odom`, no static layer and a fixed 100 m global costmap;
- self-filters `sensors/lidar2d_0/scan` with a box. Without it, the sim lidar sees the robot body and the collision monitor won't move. Jackals have no sim lidar: pass `scan_topic:=.../camera_0/scan`;
- publishes a static `map→odom` unless `gps:=true`. Alongside `bringup_main`, that needs `ref_source:=external`.

## Outdoor dual-GPS localization

Real robot: `swift_nav_dual.launch.py`, then `bringup_nav2_mapping_j100.launch.py`. Sim: `sim_swift_nav_dual.launch.py`, then `sim_nav2.launch.py gps:=true` (uses `nav2_mapping.yaml` if the platform has one).

`sim_swift_nav_dual` stands in for the Duro drivers, which need `libsbp`, not packaged for Ubuntu:
- the reference fix comes from the sim's `sensors/gps_<ref>/fix`;
- `baseline_node`, named `att_duro_node`, computes the baseline from the two fixes;
- the real heading filter, `navsat_transform` and `ekf_global_node` then run with `config/<platform>/dual_duro_heading.yaml` (j100's if the platform has none).

GPS details:
- Antennas follow robot.yaml order: the first is attitude (left), the second reference (right). a200/a300 use `gps_0`/`gps_1`, Jackals `gps_1`/`gps_2`.
- The heading frame is `dual_duro_heading`'s `frame_id` parameter (default `gps_2_link`).
- The datum `SIM_DATUM` is the sim's GPS origin, so `map` is Isaac's world frame.
- `ekf_global_node` runs with `publish_tf: false`; use `ekf_publish_tf:=true` only without `bringup_main`.

## Global localization: `ref_localizer` (`mocap_fake_localizer`)

- Frames: `ref_frame → map → odom → base_link`. `ref_frame` is Motive's frame in the lab (`natnet_ref_pose.py`, Motive at 192.168.50.80, rigid body = namespace) or Isaac's world frame in the sim.
- Parameter files, loaded in order: `mtu32_bringup/config/ref_localization.yaml`, then `config/ref_localization/assignments.yaml` (written by the web UI), then `config/ref_localization/<namespace>.yaml`.
- `source` selects what drives `map→odom`: `auto` (the ref pose while it arrives, else GPS), `ref`, `gps` or `external`.
- `anchor` places `map` in `ref_frame`: `fixed` (`map_pose_in_ref`/`anchor_file`), `start` or `external`.
- A stale source holds its last value. Override with `ref_source:=` and `ref_anchor:=`.
- `calibrate_ref_offset.py` measures `base_link_offset` (`--apply`/`--write`). It has only been tested against `test/fake_motive.py`.

## Real robots

- robot.yaml `platform.extras.launch` starts `intel_realsense.launch.py`, `swift_nav_dual.launch.py` and `bringup_main.launch.py`. One failure aborts all of them, GPS and localization included, so each must work on every robot.
- `mtu32_bringup` must keep `<exec_depend>moveit_servo</exec_depend>`; without it, the extras launch fails with "package 'moveit_servo' not found".
- To use Clearpath's move_group: `manipulators.moveit.enable: true` plus `sudo systemctl enable --now clearpath-manipulators`, which provides the Kinova driver, joint_states and TF.
- Foxglove: each robot's robot.yaml needs these under `platform.extras.ros_parameters.foxglove_bridge`, then a restart of clearpath-robot.
  - `asset_uri_allowlist: ['^package://(?:[-A-Za-z0-9_%]+/)*[-A-Za-z0-9_%]+[.](?:dae|DAE|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|STL|tif|tiff|urdf|webp|xacro)$']`, plus the same pattern with `^file:///opt/ros/jazzy/share/` for `file://` meshes. The generator mangles `\\w`, and its default misses `.STL`.
  - `topic_whitelist: ['^/<ns>/.*', '^/rosout$']`. With `.*`, every robot's TF shares one tree.

## Cutter EE in MoveIt (j100_0921)

- `mtu32_description`'s `robot_description_j100.urdf.xacro` adds `ee_cutter_link` (fixed on `arm_0_gripper_gripper_base_link`, no inertial): visual `meshes/ee_cutter.obj` (+ `.mtl`; the static parts, flat colours, from `sim/scripts/make_ee_cutter_urdf_mesh.py`; 60k triangles with the moving ones), collision the EE's envelope as 8 boxes from the STEP (the jaw opening and the finger tip's travel filled), so MoveIt plans and Servo checks with the real EE, not the 2F Lite's fingers. j100_0922 (no arm) prunes it like camera_1. The 2F Lite's finger visuals can't be removed from the description (Clearpath's gripper macro): the sim's `robot_state` drops them from the published `robot_description` (RViz, Foxglove); a real robot's (Clearpath's robot_state_publisher) still draws them inside the EE. The moving bodies (`ee_cutter_<slide|pusher|link_1|link_2|finger|blade|spur>_link`, `meshes/ee_cutter_<body>.obj`, each at its CAD pivot) hang on fixed `ee_cutter_<body>_joint` (named as the sim's): a real robot publishes no state for them and MoveIt waits for every movable joint, so they stay in the open pose there. The sim's `robot_state` makes them prismatic / revolute in the published topic, and RViz follows the sim's linkage and blade.
- The 2F Lite finger links stay in the model inside the envelope. The generated SRDF disables only the pairs its random samples saw touching (it missed `left_finger_dist`, and MoveIt then saw the closed gripper in collision: Servo stopped every cut, 2026-10-30), so `moveit.launch.py` adds `disable_collisions` for the envelope against every `arm_0_gripper_*_finger_*` link when the URDF has the envelope. MoveIt's other launchers (`clearpath-manipulators`) don't: they'd need the same.
- j100_0921's drop (`config/robots/j100_0921.yaml` `drop_joint_positions`) is 3 cm further from the basket's front wall (`unloader_wall_3`) than the SRDF `drop`: the EE's `finger_sweep` box came within 6 mm of it on the way down (`drop_lower_distance`), inside Servo's 10 mm self-collision margin (`servo_config.yaml`), and every drop stopped ~6 mm short ("retract/drop sequence failed").
- `check_state_validity` / any `RobotState` message with a joint MoveIt doesn't know (the sim's `ee_cutter_*` from `platform/joint_states`) aborts move_group (exit -6, "Variable ... is not known to model"); it respawns. Send only `arm_0_*` joints.

## Expected warnings

Harmless: `arm_0_gripper` "is not a chain"; "No 3D sensor plugin(s) defined for octomap updates"; `tf2_pose_node` lookups of `jackal_charger_april` (no AprilTag dock visible).

## Harvesting operation (`status_server` ↔ `husky_operations_manager`)

- One visit per plant: `harvest_state` (migration 002) is the plant's state (`pending` → `assigned` → `harvested` / `failed`, `attempts`); `farm_harvesting_job` stays the per-task log (`progress_status` 4 failed, 5 released). The coordinator sends a robot to the plant's north Pickup node (`harvest.side`), the goal moved forward by `harvest.cutter_offset_x[platform]` (j100 0.41, a200 0.71, a300 0.61 m): the cutter's grid sits behind the robot's right side. Each move is one `NavigateThroughPoses` goal (the robot's pose, lane exit / entry poses, the target pose): Nav2 plans the way on the farm map, no route graph (2026-10-14). Lanes are one-way, so a robot never turns between two rows: it leaves a lane straight out of its end and enters the target's lane from the end facing the target (`harvest.lane_end_margin` past the rows, from `object_data`); the executor keeps each waypoint's heading. A robot keeps its lane while it has pending plants and never enters a lane another robot works in.
- Failure contract: the robot gives a task up with `RobotStatus.status` ERROR and `RobotStatus.task` = `FAILED:<kind>:<task_id>:<detail>` (kind `nav`, `dock`, `undock`, `arm`, `harvest`, `battery`, `estop`; `arm_stow` / `startup_undock` hold until an operator), held `error.report_hold` 5 s, then it stows and goes IDLE, ignoring republishes of that task_id. The coordinator closes the task, counts the attempt (`operation.max_attempts`, then `failed`), frees the dock, and sends a new task_id (a charging task for `battery`). Two `nav` failures in a row from the same spot (0.25 m) pause the robot instead and cost the plant no attempt: stuck in the crop margin ("Start occupied"), j100_0921 failed 16 plants in an hour (2026-10-21).
- Battery: the coordinator decides (`battery.low_threshold` 30 before a new task, `critical_threshold` 15 preempts a MOVING harvest task, charges to `full_threshold` 92 via `CHARGING.data_str charge_to`); the robot only has a 10 % floor (`battery.critical_threshold`), no 50 % ERROR any more. Offline robots (liveness 30 s) keep their task for `operation.offline_grace_s` (180 s), then it is released.
- The operation row (`state` running / paused / stopped / complete, `coordinator_enabled`, `target_rows`) is written by the web UI's `/operation` page; the coordinator reads it every tick. Paused: the current task finishes, nothing new. Stopped: harvest tasks are cancelled (also one restored from the database after a coordinator restart), robots park at their chargers, charging runs on. Per robot, `robot_operation` (Add / Remove, Pause / Resume, Stop / Start, Charge on the Robots card): a STOP task (`task_id` -1, description `STOP`, empty `job_schedule`) makes the executor cancel its task (`FAILED:stopped`, released without an attempt) and stow.
