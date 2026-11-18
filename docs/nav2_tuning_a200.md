# a200 Nav2 tuning (a200_0284, lavender farm sim)

2026-10-14 · Isaac Sim 6.0 + ROS 2 Jazzy (Nav2 1.3.13), sim time. Applies to every a200 (`mtu32_bringup/config/a200/nav2.yaml`, `config/nav2_robots.yaml` `platforms.a200`, the j100's `config/behavior_trees/navigate_through_poses_replan_if_invalid.xml`). Tool: `scripts/nav2_tune/`. Per-leg data of every run: `docs/nav2_tuning_a200/results.json`; what each variant changed: `docs/nav2_tuning_a200/variants.yaml`. Method and course as in `docs/nav2_tuning_j100.md`; the a300 report (`docs/nav2_tuning_a300.md`) explains the in-lane failure that shapes this one (same 0.99 × 0.70 m footprint).

The shipped setup (NavFn planner, MPPI at 0.5 m/s, lidar + camera scan in the costmaps) finished 2 of 14 legs in two runs: it drove 34 m on the 14 m route into lane 1, reversed 3 m on the way to Staging, and wedged at the lane entry (95 recoveries per run). The tuned setup (DUBIN planner, the a300's Graceful set with a 2 m lookahead, map-only costmaps, 0.10 m / 0.10 rad tolerance) finished 15 of 21 legs in three runs, every lane leg (entry, harvest steps, lane run) in every run, 7.7 cm mean / 10.9 cm max from the goal, heading median 1.9°, forward only.

Not solved: each tuned run lost one leg near its goal (the U-turn, the lane-3 entry, the last turn into Staging), and a robot stuck after a failed leg fails every leg after it, so no run finished the whole course. The a200 stops less precisely than the a300 (mean 7.7 vs 4.2 cm): it uses the full 0.10 m tolerance you set as the target; 0.05 m made it turn hard for the last centimetres inside a lane and wedge.

## MPPI `open_loop` (2026-10-28)

Since 2026-10-21 the a200 runs MPPI (`config/a200/nav2.yaml`; on the PC it was tuned on: farm course 7/7, 8 cm). On a new PC (Newton, same sim settings, `nav2_mppi_controller` 1.3.13) it finished nothing: it aborted into lane 1 and every later leg failed with ~20 recoveries (0/7). Same cause and fix as the j100 and a300 (`docs/nav2_tuning_j100.md`, "MPPI `open_loop`"): closed loop, MPPI seeds each optimization with the EKF's speed, which lags ~50-140 ms in the sim, and creeps, overshoots and weaves; `open_loop: true` seeds it with its own last command.

| a200_0284, farm course | Legs | Mean · max error | Recoveries | Time |
|---|---|---|---|---|
| Shipped (closed loop) | 0 / 7 | – | 103 | – |
| **+ `open_loop`** (run 1 · run 2) | **7 / 7 · 7 / 7** | **5.2 · 7.7 cm · 5.1 · 6.9 cm** | **0 · 0** | 295 s · 294 s |

Run 1 also reached `back_0.5`. Results: `sim/generated/nav2_tune/a2_farm_*` on that PC.

## Result

| Measure | Tuned: Graceful, 2 m lookahead | Baseline: NavFn + MPPI as shipped |
|---|---|---|
| Legs succeeded | 15 / 21 | 2 / 14 |
| Lane legs (entry, 2 harvest steps, lane run) | 12 / 12 | 1 / 8 |
| Runs with every leg succeeded | 0 / 3 | 0 / 2 |
| Time / estimate at 0.5 m/s, median leg | 0.82× | ring 2.2×; lane run 0.28× (it started 1 m from its goal) |
| Position error mean · p90 · max | 7.7 · 10.0 · 10.9 cm | 7.0 · – · 10.0 cm (2 legs) |
| Legs within 10 cm · within 5 cm | 13 / 15 · 2 / 15 | – |
| Heading error median · max | 1.9° · 3.6° | 4.6° · 5.2° (2 legs) |
| Legs within 0.1 rad (5.7°) | 15 / 15 | 2 / 2 |
| Driven backwards per run (longest stretch) | 0.02 m (0.01 m) | 1.54 m (0.60 m; 3.2 m on the warm-up leg) |
| Rotation while standing, per run | 1.2 rad | 3.5 rad |
| Nav2 recoveries per run | 29 (in the aborted legs) | 95 |
| Max body tilt · peak braking | 0.4° · 1.4 m/s² | – |

Errors are against the sim's ground truth 2 s after the result; a leg still running at 300 % of its estimate (route length at 0.5 m/s, the shipped MPPI limit, + 5 s) fails.

### Per leg (mean time · mean position error of the succeeded ones, succeeded / runs)

| Leg | Tuned | Baseline |
|---|---|---|
| Staging → lane 1 pickup (14 m) | 31.9 s · 6.7 cm (3/3) | failed (0/2): 34 m of wandering, then wedged at the entry |
| Harvest step 0.45 m | 1.9 s · 9.8 cm (3/3) | failed (0/2) |
| Harvest step 0.9 m | 3.7 s · 8.5 cm (3/3) | failed (0/2) |
| Rest of lane 1 (9 m) | 19.6 s · 7.2 cm (3/3) | 10.6 s · 4.1 cm (1/2) |
| U-turn into the same lane (5 m) | 16.7 s · 5.7 cm (2/3) | failed (0/2) |
| Ring to lane 3 (57 m) | 119.8 s · 7.9 cm (1/3) | 259.9 s · 10.0 cm (1/2) |
| Lane 3 → Staging (20 m) | failed (0/3) | failed (0/2) |

Legs after a failed one start wherever the robot stopped: of the 6 tuned failures, 3 were the robot's own (one per run) and 3 followed from them.

## Parameters

| Setting | Baseline | Tuned |
|---|---|---|
| Planner | NavFn (grid paths, turns in place) | SmacPlannerHybrid DUBIN (r 0.2 m), the j100/a300 block |
| Controller (FollowPath) | MPPI: 56 steps, vx −0.35…0.5 m/s, wz ≤ 1.9 rad/s | Graceful: k_phi 2.0, k_delta 0.5, beta 0.2, lambda 2.0, lookahead 0.4–2.0 m, slowdown 0.6 m, v 0.05–0.5 m/s, ω ≤ 1.0 rad/s, initial_rotation false, prefer_final_rotation false, allow_backward false |
| Goal checker (xy · yaw) | 0.15 m · 0.15 rad | 0.10 m · 0.10 rad |
| NavigateThroughPoses tree | Nav2's default: replans every 3 s | replans only when the path becomes invalid or the goal changes |
| Global / local costmap layers | static, obstacle (SICK lidar + RealSense scan), inflation | static, inflation |
| Velocity smoother | ±2.0 m/s, ±2.0 rad/s, 4 m/s² | ±0.5 m/s, ±1.0 rad/s, 1.0 m/s², 2.0 rad/s² |
| Unchanged | footprint 0.99 × 0.698 m, collision monitor on the SICK lidar, progress checker, recoveries | |

Differences from the a300's set: lookahead up to 2.0 m (1.2), tolerance 0.10 m / 0.10 rad (0.05 / 0.05).

## Variants tried

| Variant | Result | What happened |
|---|---|---|
| j100's set (Graceful, final rotation) | 3/7 | Final turn at the lane pickup wedged (21°), as on the a300 |
| a300's set (lookahead 1.2, 0.05 tol) | 6/14 in 2 runs, 62 recoveries/run | Lane entry aborted both runs: the robot weaves ±5 cm along the lane (~5 m wavelength, not decaying), then a hard last turn to the exact pose reaches the plants at 22-24° |
| C1 / C2: a300's set, tolerance 0.10/0.10 and 0.10/0.05 | 1/7, 3/7 | Tolerance alone doesn't help: the swing starts 0.3-0.5 m before the goal |
| D1: lookahead 2.0 | 4/7 | Weave gone: lane entry and harvest steps succeeded (2-7 cm); overshot the U-turn exit by 25 cm |
| **D2: D1 + tolerance 0.10 / 0.10 (chosen)** | 15/21 in 3 runs, 29 recoveries/run | All 12 lane legs; one near-goal failure per run (below) |
| D3: lookahead 1.6 | 4/7 | 0.45 m step timed out (33°) |
| D4: D1 + beta 0.6 | 5/7, 4 recoveries | Every lane leg and the U-turn at 3-9 cm, ≤1.3°; lane-3 entry aborted (25°) |
| D5: D3 + tolerance 0.10 | 4/7, 58 recoveries | U-turn aborted |
| D6: D2 + beta 0.6 | 6/14 in 2 runs | Run 1: swung 23 cm wide out of the lane-1 entry turn and stopped on the map's plant row; run 2: 6/7 |

## What still fails

- **The lookahead trade-off.** Graceful has one lookahead. Short (1.2 m): the robot weaves in the lanes (the sim drive's yaw response lags; the loop doesn't settle) and arrives beside the line, then turns hard at the goal and wedges. Long (2.0 m): no weave, but it converges slowly after a turn, so it comes out of tight turns (U-turn, lane entries after the ring, the last ~110° turn into Staging) wide, and either ends beside the goal with too little room to correct (progress checker gives up) or reaches the plants.
- **Each tuned failure was one of those exits**: run 1 the last turn into Staging (stopped 12.9 cm beside it), run 2 the lane-3 entry at the end of the ring (8.6 cm, 24°), run 3 mid-U-turn ("Collision detected in trajectory").
- **What would address it, not tried**: a controller per situation (Nav2's controller selector in the tree: a short-lookahead plugin for the last metre, the long one elsewhere); or MPPI with the map-only costmaps and DUBIN plans (the j100 report found MPPI imprecise at 0.05 m, but 0.10 m may suit it); or tighter yaw tracking in the sim drive (`model_params.yaml` a200_0284 `vel_ctl`), since the weave is the drive's lag as much as the controller.

## Trade-offs and what to check on the real robot

- **No planning around unmapped obstacles**, as for the j100 and a300: the SICK lidar now only feeds the collision monitor, which stops the robot; it waits instead of driving around.
- **DUBIN instead of NavFn**: no more grid paths with turns in place; the a200's other Nav2 launches that read `a200/nav2.yaml` (sim_nav2, mapless) get it too.
- **Speed 0.5 m/s and 1 m/s²** (was 2 m/s and 4 m/s² in the smoother, with MPPI capped at 0.5): the arm is on top; no tilt above 0.4°.
- **Sim-specific**: the weave comes from the sim's skid-steer yaw response (`vel_ctl`, tuned 2026-10-13); a real Husky may track tighter and allow the shorter lookahead and 0.05 m tolerance.
- **Not run on the real robot.** Deployed to every a200 with `colcon_ws`; try it on a200_0284 under supervision first.
- During these runs the host crashed once at a sim restart (CPU core 4 fault, see the host notes); runs after the restart used the same procedure, so results are comparable.
