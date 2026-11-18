# a300 Nav2 tuning (a300_00036, lavender farm sim)

2026-10-14 · Isaac Sim 6.0 + ROS 2 Jazzy (Nav2 1.3.13), sim time. Applies to every a300 (`mtu32_bringup/config/a300/nav2_map.yaml`, `config/nav2_robots.yaml` `platforms.a300`, the j100's `config/behavior_trees/navigate_through_poses_replan_if_invalid.xml`). Tool: `scripts/nav2_tune/`. Per-leg data of every run: `docs/nav2_tuning_a300/results.json`; what each variant changed: `docs/nav2_tuning_a300/variants.yaml`. Method and course as in `docs/nav2_tuning_j100.md`.

The shipped setup (MPPI capped at 0.2 m/s, camera scan in both costmaps) finished none of the coordinator's farm routes: the planner found no path into the first lane and every later leg failed from there (0/14 legs in 2 runs, 94 recoveries per run). The tuned setup, the j100's recipe adapted to the longer robot (Graceful controller at 0.5 m/s, costmaps from the farm map, replanning only when the path breaks, lining up with the goal on the approach instead of turning at the end), finished 17/21 legs in 3 runs, the whole course in 273 s (1 of 3 runs complete), 4.2 cm mean / 10.7 cm max from the goal, heading median 1.6°, forward only.

Not solved: in 4 of 21 legs the robot arrived beside the line inside a lane and its last S-turn to the exact goal pose put a corner into the plants (see "What still fails"). At 1.0 m/s that got worse, not better.

## MPPI `open_loop` (2026-10-27)

Since 2026-10-21 the a300 runs MPPI (`config/a300/nav2_map.yaml`; `colcon_ws/CLAUDE.md`). On a new PC (Newton, same sim settings, `nav2_mppi_controller` 1.3.13) it finished nothing on the farm course: no path into lane 1, then 21 recoveries per leg (0/7, 147 recoveries). The cause is the one found on the j100 the same day (`docs/nav2_tuning_j100.md`, "MPPI `open_loop`"): closed loop, MPPI seeds each optimization with the EKF's speed, which lags ~50-140 ms in the sim, and creeps, overshoots and weaves. `open_loop: true` seeds it with its own last command.

| a300_00036, farm course | Legs | Mean · max error | Recoveries | Time |
|---|---|---|---|---|
| Shipped (closed loop) | 0 / 7 | – | 147 | – |
| **+ `open_loop`** (run 1 · run 2) | **7 / 7 · 7 / 7** | **5.2 · 7.4 cm · 4.8 · 6.2 cm** | **0 · 0** | 300 s · 299 s |
| + `open_loop`, goal tolerance 0.05 m / 0.05 rad | 6 / 7 (ring to lane 3 aborted 6.2 cm short) | 2.3 · 3.4 cm | 1 | – |

Chosen: `open_loop: true` alone; the tolerance stays 0.10 m / 0.10 rad. Results: `sim/generated/nav2_tune/a3_farm_*` on that PC.

## Result

| Measure | Tuned: Graceful 0.5 m/s | Baseline: MPPI as shipped (0.2 m/s) | Graceful 1.0 m/s (A2i) |
|---|---|---|---|
| Legs succeeded | 17 / 21 | 0 / 14 | 14 / 21 |
| Runs with every leg succeeded | 1 / 3 | 0 / 2 | 0 / 3 |
| Course time (complete run) | 273 s | – | – (best run 214 s, 6/7) |
| Time / estimate at 0.2 m/s, median leg | 0.52× | – | 0.43× |
| Position error mean · p90 · max | 4.2 · 9.3 · 10.7 cm | – | 4.5 · 8.4 · 16.7 cm |
| Legs within 10 cm · within 5 cm | 15 / 17 · 9 / 17 | – | 13 / 14 · 10 / 14 |
| Heading error median · max | 1.6° · 12.1° | – | 3.0° · 7.8° |
| Legs within 0.1 rad (5.7°) | 16 / 17 | – | 9 / 14 |
| Driven backwards per run (longest stretch) | 0.25 m (0.36 m) | 1.00 m (0.44 m) | 0.13 m (0.06 m) |
| Rotation while standing, per run | 2.7 rad | 2.2 rad | 2.1 rad |
| Nav2 recoveries per run | 21 (all in the aborted legs; 0 in the complete run) | 94 | 15 |
| Max body tilt · peak braking | 0.5° · 2.3 m/s² | – | 1.0° · 3.0 m/s² |

The errors are against the sim's ground truth 2 s after the result. A leg counts as failed if it was still running at 300 % of its estimate (route length at the shipped 0.2 m/s + 5 s); no tuned leg came close (median 0.52×).

### Per leg (mean time · mean position error of the succeeded ones, succeeded / runs)

| Leg | Tuned: Graceful 0.5 m/s | Baseline | Graceful 1.0 m/s (A2i) |
|---|---|---|---|
| Staging → lane 1 pickup (14 m) | 36.0 s · 3.5 cm (2/3) | failed (0/2): "no valid path" into the lane | 26.5 s · 2.2 cm (1/3) |
| Harvest step 0.45 m | 4.2 s · 3.4 cm (3/3) | failed (0/2) | 3.9 s · 5.0 cm (1/3) |
| Harvest step 0.9 m | 6.0 s · 6.2 cm (3/3) | failed (0/2) | 7.8 s · 6.2 cm (3/3) |
| Rest of lane 1 (9 m) | 22.6 s · 7.0 cm (3/3) | failed (0/2) | 16.1 s · 6.1 cm (3/3) |
| U-turn into the same lane (5 m) | 20.7 s · 10.7 cm (1/3) | failed (0/2) | 16.0 s · 2.9 cm (3/3) |
| Ring to lane 3 (57 m) | 129.9 s · 1.0 cm (2/3) | failed (0/2) | 81.6 s · 4.4 cm (1/3) |
| Lane 3 → Staging (20 m) | 62.4 s · 0.9 cm (3/3) | failed (0/2) | 41.8 s · 3.0 cm (2/3) |

The baseline legs after the first one started wherever the robot had stopped (6-16 m from the route), so they say little on their own; the first leg failed in both runs.

## Parameters

| Setting | Baseline | Tuned |
|---|---|---|
| Controller (FollowPath) | MPPI: 1000 × 72 steps, vx −0.2…0.2 m/s, wz ≤ 0.3 rad/s, critics incl. PathAlign 20, PreferForward 14 | Graceful: k_phi 2.0, k_delta 0.5, beta 0.2, lambda 2.0, lookahead 0.4–1.2 m, slowdown 0.6 m, v 0.05–0.5 m/s, ω ≤ 1.0 rad/s, in-place ≥ 0.25 rad/s, initial_rotation false, prefer_final_rotation false, allow_backward false |
| Goal checker (xy · yaw) | 0.10 m · 0.15 rad | 0.05 m · 0.05 rad |
| NavigateThroughPoses tree | Nav2's default: replans every 3 s | replans only when the path becomes invalid or the goal changes |
| Global costmap layers | static, voxel (RealSense scan), inflation | static, inflation |
| Local costmap layers | static, obstacle (RealSense scan), inflation | static, inflation |
| Velocity smoother accel · decel | 2.5 m/s², 2.5 rad/s² | 1.0 m/s², 2.0 rad/s² |
| Unchanged | SmacPlannerHybrid DUBIN (r 0.2 m), footprint 0.99 × 0.698 m, smoother limits 0.5 m/s · 1.0 rad/s, collision monitor on the scan, progress checker, recoveries | |

Differences from the j100's tuned set: `prefer_final_rotation: false` (the j100 keeps true), `k_delta` 0.5 (1.0), lookahead 0.4–1.2 m (0.25–1.0 m).

## Variants tried

All Graceful, on the j100 recipe (map-only costmaps, replan-if-invalid tree, 1 m/s² smoother, tolerance 0.05/0.05); each row adds to the one it names.

| Variant | Result | What happened |
|---|---|---|
| A2: j100's set, 0.5 m/s | 4/7 legs, 6.8 cm mean, 29 recoveries | The final in-place turn at the lane pickup put a corner into the plants (36° heading, aborted), and again on lane 3 |
| A2b: + prefer_final_rotation false | 6/7, 4.0 cm, 21 recoveries | Lines up on the approach; all lane goals reached. Ring to lane 3 aborted at its lane entry (17°) |
| A2c: A2b + in-place ≥ 0.5 rad/s | 5/7, 3.9 cm, 22 recoveries | Overshot the final turns (yaw mean 7.7°), lane entry aborted |
| **A2e: A2b + k_delta 0.5, lookahead 0.4–1.2 m (chosen)** | 17/21 in 3 runs, 4.2 cm, 21 recoveries/run | Run 1: 7/7, 0 recoveries, 273 s. Runs 2-3: the U-turn and lane entries aborted at the goal (below) |
| A2f: A2e + yaw tolerance 0.1 rad | 6/7, 3.7 cm, 22 recoveries | U-turn aborted the same way; looser yaw didn't help |
| A2f_10: A2f at 1.0 m/s | 1/7, 39 recoveries | Overshot every in-lane goal; circled with 14 rad of in-place rotation |
| A2g: 1.0 m/s, slowdown 1.5 m, lookahead 1.8 m | 5/7, 4.8 cm, 14 recoveries | Fast (lane run 15 s), ring to lane 3 swung wide at the lane entry |
| A2h: A2g + beta 0.6 (curvature slowdown) | 5/7, 4.7 cm, 21 recoveries | Same lane-3 entry failure |
| A2i: A2h + lookahead/slowdown 1.2 m | 14/21 in 3 runs, 4.5 cm, 15 recoveries/run | Best 1.0 m/s set: U-turn 3/3, but lane entries 2/6 and a harvest step failed |
| A2j: A2i + xy tolerance 0.08 | 2/7, 31 recoveries | Worse; one run |
| A2k: A2e + beta 0.6 | 6/14 in 2 runs, 30 recoveries/run | Lane entry and U-turn aborted in both runs |

Also started, no usable result: the j100 recipe at the shipped 0.2 m/s / 0.3 rad/s limits (A1) and at 1.0 m/s with the final rotation (A3); both were dropped once A2 showed the final in-place turn itself was the problem.

## What was wrong with the baseline

1. **The camera scan made the lanes impassable.** The map has 1.34 m between the plant rows, but the scan sees the foliage much closer (the j100 report measured 0.74 m between plants), and the a300 is 0.70 m wide. The RealSense scan marks that foliage lethal in both costmaps: SmacPlannerHybrid failed ("no valid path found", "Start occupied", over 100 times in two runs) and MPPI could not find a trajectory ("Optimizer fail to compute path", 321 times in run 1). The robot never got into lane 1.
2. **The default tree replans every 3 s**, which with a DUBIN planner near a goal returns a loop (j100 report, point 2).
3. **0.2 m/s** made every leg 2.5× slower than the robot needs to be: the tuned set drives 0.5 m/s and finished legs in 0.52× the time of the 0.2 m/s estimate.

## What still fails (4 of 21 legs)

Every tuned failure is the same: in a lane, the robot reaches the last metre ~5 cm beside the pickup line (it weaves ±5° with a ~4 m wavelength while following the path), then turns sharply to reach the exact goal pose. A 5 cm offset within the last 0.3 m needs a ~20° heading swing; at 22-24° the 0.99 × 0.70 m body's corners reach 0.52 m sideways from its centre, and the north plant row on the map is 0.60 m from the pickup line (lane 1, the line 7 cm north of the lane centre): Graceful's trajectory check reports "Collision detected in trajectory", and the goal is aborted. In run 1 the same swing went to +27° and came back; in runs 2-3 it stuck at −22 to −24°. Examples: staging → lane 1 in run 3 (stopped at 13.6 cm / 23.5°), the U-turn in runs 2-3.

Turning at the end instead (`prefer_final_rotation: true`, A2) fails the same way, earlier: the a300 sweeps a 1.21 m circle turning in place, and at the pickup line that circle touches the north plant row on the map. Looser yaw (A2f) didn't help; looser xy at 1.0 m/s (A2j) was worse. What would address it, not yet tried:
- remove the lateral error earlier: a goal-approach phase that tracks the path tighter in the last 1-2 m (a shorter lookahead only near the goal, e.g. a second controller plugin selected by the tree), or a planner tolerance so the last pose is on the robot's line;
- accept the lateral error: xy tolerance 0.10 at 0.5 m/s (your stated target), which ends the leg before most of the swing; the coordinator's 0.13 m pickup offset and the cutter grid would decide whether 10 cm sideways is acceptable;
- a smaller footprint is **not** a fix: the corner really is at the plants at 24°.

## Speed

The A300 is rated 2 m/s; the shipped 0.2 m/s cap was a config choice. 0.5 m/s is kept because faster finished fewer legs in this farm, not because of the robot:
- 1.0 m/s cut the ring (57 m) from 130 s to 82 s and the lane run from 23 s to 16 s, but with the same lookahead it overshot in-lane goals (A2f_10, 1/7). Longer lookahead and slowdown radius (A2g-A2i) made it workable on straight lanes and the U-turn, but lane entries after a tight turn swung wide (2 of 6 succeeded) and in-lane approaches still aborted.
- More speed would pay off on the ring and the staging routes, not in the lanes. A speed limit by zone (Nav2's speed filter with a keepout/speed mask: 1.0+ m/s outside the lanes, 0.5 m/s inside) is the way to get it; not tried here.

## How it was measured

As for the j100 (`docs/nav2_tuning_j100.md`, "How it was measured"): a300_00036 in the sim's lavender farm, the j100 and a200 parked north of the farm, zenoh, sim time; Nav2 started as `bringup_main` starts it (map `zone_end_2`, scene chargers as docks), no operations manager; the course of the coordinator's own routes (`scripts/nav2_tune/course_lavender_farm.json`: staging → lane 1 pickup, 0.45 m and 0.9 m harvest steps, rest of lane 1, U-turn into the same lane, the 57 m ring to lane 3, lane 3 → staging; ~110 m) plus a synthetic 0.5 m goal straight behind (reported separately: 1 of 3 tuned runs reached it, the others circled it until the 300 % limit, forward only); a sim reset and upright check before each run. Estimates at 0.2 m/s (`trial.sh --est-speed 0.2`, the shipped cap) for every configuration.

## Trade-offs and what to check on the real robot

- **No planning around unmapped obstacles**, as for the j100: the RealSense scan now only feeds the collision monitor (FootprintApproach), which stops the robot before an obstacle on its path; it waits instead of driving around. In a lane there is no way around anyway.
- **No reversing.** DUBIN never asks for it and the controller does not reverse; the longest backward stretch (0.36 m) was a BackUp recovery after an aborted goal.
- **1 m/s² acceleration**, as for the j100. The a300 has no arm on top and didn't tip (max 0.5°); it's kept for the stopping precision.
- **Sim-specific behaviour**: the yaw weave and the breakaway overshoot on small in-place turns come from the sim's skid-steer drive model (`sim/config/model_params.yaml` a300_00036, tuned 2026-10-14); the real Husky-class drive may track tighter, which would also remove most of the in-lane failures. Accuracy is against the sim's ground truth with near-perfect localization.
- **Not run on the real robot.** The parameters are deployed to every a300 with `colcon_ws`; try them on a300_00036 under supervision before relying on them.
