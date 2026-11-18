# Third-party notices

The repository as a whole is proprietary (see `LICENSE`). The components below
are not: each stays under its own license, which applies to those files instead
of `LICENSE`. License texts the components require to travel with copies are in
`licenses/`; BSD-3-Clause components keep their own copyright lines in their
sources, with `licenses/BSD-3-Clause.txt` as the text.

When adding third-party code or assets, add a row here and, if its license asks
for it, its text to `licenses/`.

## Git submodules (`colcon_ws/src/`)

Separate repositories; their own `LICENSE` files and `package.xml` govern them.

| Path | Upstream | License |
|---|---|---|
| `clearpath_desktop` | clearpathrobotics/clearpath_desktop | BSD-3-Clause, (c) 2023 Clearpath Robotics |
| `depthimage_to_laserscan` | robotian fork of ros-perception/depthimage_to_laserscan | BSD-3-Clause, (c) 2012 Willow Garage |
| `laser_filters` | ros-perception/laser_filters | BSD-3-Clause, (c) 2008 Willow Garage |
| `husky_operations_manager` | robotian/husky_operations_manager | Apache-2.0 |
| `mtu32_husky` (`mtu32_description` meshes are copied into `sim/assets/`) | robotian/mtu32_husky | BSD-3-Clause |
| `docking_utils` | robotian/docking_utils | none declared (`package.xml`: TODO) |
| `mocap_fake_localizer` | robotian/mocap_fake_localizer | none declared (`package.xml`: TODO) |

## Vendored packages (`colcon_ws/src/`)

| Path | Origin | License |
|---|---|---|
| `moveit_servo` | MoveIt 2.12.4 (moveit/moveit2), modified here: servo loop stepped on the node clock under `use_sim_time` | BSD-3-Clause, (c) 2019 Los Alamos National Security, LLC; (c) 2023 PickNik Inc. |
| `swiftnav_ros2_driver` | Swift Navigation | MIT, (c) 2023 Swift Navigation (`swiftnav_ros2_driver/LICENSE`) |
| `status_server`, `status_interfaces` | MTU (maintainer `vupadhye`) | Apache-2.0 (their own `LICENSE`) |
| `*/test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` | `ros2 pkg create` templates | Apache-2.0, (c) 2015/2017 Open Source Robotics Foundation |

The other packages there (`arm_task_server`, `dual_duro_heading`, `duro_sim`,
`kinova_game_pad`, `moveit_sim_bridge`, `plant_cutter_msgs`,
`pruner_action_server`, `serial_interfaces`, `stow_arm_cpp`) are this project's
own and fall under `LICENSE`.

## Robot meshes (`sim/assets/<model>/meshes/<package>/`)

`scripts/gen_urdf.sh` copies these from the ROS packages installed in the robot
image (Collada converted to OBJ, otherwise unmodified) into each model's folder.

| Package | Upstream | License |
|---|---|---|
| `clearpath_platform_description`, `clearpath_sensors_description`, `clearpath_mounts_description` | clearpathrobotics/clearpath_common | BSD-3-Clause, Clearpath Robotics |
| `kortex_description` | Kinovarobotics/ros2_kortex | BSD-3-Clause, Kinova |
| `robotiq_description` | PickNikRobotics/ros2_robotiq_gripper | BSD-3-Clause, PickNik |
| `ur_description` (ur5e, `a300_00037`) | UniversalRobots/Universal_Robots_ROS2_Description | BSD-3-Clause package; Universal Robots ships its newer meshes under its *Terms and Conditions for Use of Graphical Documentation*, whose copy is in `licenses/UR-Graphical-Documentation.txt` and must accompany any shared copy |
| `velodyne_description` | Dataspeed velodyne_simulator | BSD-3-Clause, Dataspeed |
| `realsense2_description` | IntelRealSense/realsense-ros | Apache-2.0, Intel (`licenses/Apache-2.0.txt`) |
| `zed_description` | stereolabs/zed-ros2-wrapper | Apache-2.0, Stereolabs (`licenses/Apache-2.0.txt`) |
| `mtu32_description` | `colcon_ws/src/mtu32_husky` | BSD-3-Clause |

## Scene assets (`sim/assets/`)

| Path | Origin | Terms |
|---|---|---|
| `trees/`, `shrubs/`, `rocks/` (and the `Trees` symlink) | NVIDIA Omniverse library (`Assets/Vegetation`, `omniverse-content-production` S3 bucket) | NVIDIA Omniverse License Agreement ("Content"), shipped with Isaac Sim as `/isaac-sim/LICENSE.txt`: may be modified and distributed only incorporated into a product, never stand-alone, and must not be placed under an open-source license (§2.1, §6.9) |
| NVIDIA digital-human bodies referenced by `farm_workers/*.usda` | NVIDIA Isaac Sim 6.0 assets, loaded from NVIDIA's S3 server at run time (not stored here) | NVIDIA Omniverse License Agreement |
| `Ground_cover/` | Collected from a local Nucleus (`omniverse://localhost/Projects/Props`); presumably NVIDIA content | **Origin to confirm** |
| `lavender/`, `lavender_v2/` | `lavender/`: purchased on Epic Games' Fab marketplace (Nanite mesh exported from Unreal Engine). `lavender_v2/`: the Sim_LavenderFarm pack's Nanite lavenders | **Not included in this repository**: the Fab End User License Agreement does not allow redistributing the purchased mesh, and the pack's terms were not confirmed, so it is left out too. The scenes refer to them by path (README, *Lavender plant meshes*) |
| `wibotic_tr302_edge/` (excluding `textures/apriltag/`) | Model of the WiBotic TR-302 Edge charger | **Origin to confirm** (WiBotic CAD terms apply if it was derived from their CAD) |
| `wibotic_tr302_edge/textures/apriltag/` | AprilTag tag36h11 family images (AprilRobotics) | BSD-2-Clause per the AprilTag project, to confirm |
| `sky/farm_field_puresky_2k.hdr` | HDRI | **Origin to confirm** (the name follows Poly Haven's convention; Poly Haven assets are CC0) |
| `soil/`, `weed_barrier/` | Generated by `sim/scripts/make_farm_textures.py` | This project's own (`LICENSE`) |
| `wiferion_charger/` | Generated by `scripts/make_wiferion_charger.py` | This project's own (`LICENSE`) |
| `wood_arch/`, `farm_workers/` (gear, crate, poses) | Modelled in MTU's `gen_3d_model` workspace | This project's own (`LICENSE`) |

## Not redistributed

The repository uses these but contains none of their files; their licenses
apply where they are installed or pulled:
NVIDIA Isaac Sim 6.0 container (`nvcr.io`, NVIDIA Omniverse License Agreement),
ROS 2 Jazzy and Clearpath packages (mostly Apache-2.0 / BSD-3-Clause),
Eclipse Zenoh `zenohd` 1.10.1 (EPL-2.0 or Apache-2.0), PostgreSQL (PostgreSQL
License), and Python packages installed in the images.
