# j100 Nav2 tuning (j100_0921, lavender farm sim)

## MPPI `open_loop` (2026-10-27)

From 2026-10-20 the j100 runs MPPI (Nav2's default trees, the camera's `scan_layer` in the global costmap). In an overnight harvesting run (Newton, 2026-10-26/27, a new PC) it harvested row 1 and two plants of row 2, then failed every plant at the west end of row 2 (b2_41..b2_43): it entered the row 1-2 lane weaving ±0.25 m / ±35°, passed the goal and, unable to reverse, the replan sent it round the whole row (35 m) and back, until it stopped with a footprint corner in row 1 ("Start occupied") and the coordinator paused it.

On that PC the shipped set scored 0/7 on the farm course where it had scored 4/7 on 2026-10-20 (same settings and sim rate, `nav2_mppi_controller` 1.3.13, whose MPPI code has had no behaviour change since 2025). Measured there:
- **MPPI's own command ramps at ~0.05 m/s per s** (`cmd_vel_nav`; the robot follows it within a few cm/s), far below `ax_max` 1.0: raising `ax_max` to 3.0 lifted it only to 0.2. So it creeps up to speed, can't brake, and overshoots.
- **"Reached" 30-40 cm off.** The goal checker is `stateful`: once within 0.10 m the position counts as reached, and MPPI, still moving, coasted on while it turned to the goal heading. Localization was fine (`map → base_link` vs the sim's ground truth: 4 mm).
- **Closed loop is the cause.** MPPI seeds each optimization with the measured speed (`platform/odom/filtered`, the EKF: 20 Hz, ~46 ms behind, up to 138 ms). `open_loop: true` (Jazzy 1.3.12+, "useful when using low accelerations and when wheel odometry's latency causes issues in initial state estimation") seeds it with its own last command instead: the ramp went to ~0.4 m/s per s and the stops to 3-4 cm, with no other change.

Course `scripts/nav2_tune/course_lane_entry.json`: the coordinator's own routes for the failing plants (Staging → lane entry → b2_39, then 0.45 m steps to b2_42 with the 0.41 m cutter offset, a lane run, the lane back to Staging, and a re-entry to b2_43). "Margin" = the smallest distance from the footprint to the map's row polygons inside the lane (negative = inside a row).

| Variant | Farm course | Lane-entry course | Mean error | Margin | Recoveries |
|---|---|---|---|---|---|
| Shipped (2026-10-20) | 0 / 7 | 0 / 7 | – | – | – |
| + replan-only-when-invalid trees, no `scan_layer`, `wz_max` 1.0, `vx_max` 0.5 | – | 6 / 7 | 12 cm | −27 cm | – |
| + wider sampling, heavier path or cost critics (closed loop) | – | 2-7 / 7 | 18-26 cm | −14 to −21 cm | – |
| Shipped + `open_loop` | 7 / 7 | 7 / 7 | 4.0 / 4.4 cm | 30 cm | 3 / 4 (spinning on the way out of a lane, 17-23 rad) |
| Nav2's stock critics + `open_loop` (+ the trees above) | 6 / 7 (stuck in the lane U-turn) | 7 / 7 | 3.3 cm | 29 cm | 12 / 0 |
| `open_loop` + the trees above + `wz_max` 1.0 / `vx_max` 0.5 | 6 / 7 (stuck in the lane U-turn) | 7 / 7 | 4.0 cm | 30 cm | 8 / 0 |
| **Shipped + `open_loop` + `wz_max` 1.0 / `vx_max` 0.5** | **7 / 7** (and `back_0.5`) | **7 / 7** | **3.2 / 4.4 cm** | **30 cm** | **0 / 0** |
| Graceful (2026-10-14 set) + the trees above, no `scan_layer` | 7 / 7 | 7 / 7 | 0.7 / 0.6 cm | 33 cm | 0 / 0 |

Chosen: the last MPPI row (`config/j100/nav2.yaml`: `open_loop: true`, `wz_max` 1.0, `vx_max` 0.5; everything else as shipped, `scan_layer` kept: with `open_loop` it no longer upset the lane runs). Graceful is more precise but doesn't go round obstacles that appear on the path, why MPPI was chosen on 2026-10-20. The replan-only-when-invalid trees suit Graceful, not MPPI: kept on a looped path it can't follow, MPPI stuck in tight turn-arounds that Nav2's 3 s replan gets it out of. Results: `sim/generated/nav2_tune/{farm,lane}_*` on that PC.

## Retune 2026-10-14 (after the sim drive retune)

The sections below the line are the 2026-10-13 tuning. That evening the sim drive of j100_0921 was retuned (`sim/config/model_params.yaml`: `vel_ctl` gains and wheel friction, `scripts/vel_tune/`), so the set was benchmarked again on the same course and procedure, and two values changed. Per-leg data: `docs/nav2_tuning_j100/retune_2026-10-14_results.json`; variants: `retune_2026-10-14_variants.yaml`.

On the retuned drive the 2026-10-13 set finished 12 of 14 legs in two runs and failed the same leg each time: the last turn into Staging. The new set finished 21 of 21 legs in three runs, every run complete, 0 recoveries, 284 s per course, 3.9 cm mean / 6.7 cm max from the goal, 18 of 21 legs within 5 cm.

| Measure | New: radius 0.4 m, v_min 0.10 (J6) | 2026-10-13 set on the retuned drive |
|---|---|---|
| Legs succeeded · complete runs | 21 / 21 · 3 / 3 | 12 / 14 · 0 / 2 |
| Course time (complete runs) | 281, 287, 284 s | – (279-282 s with Staging failed) |
| Position error mean · p90 · max | 3.9 · 5.5 · 6.7 cm | 4.4 · 5.2 · 6.2 cm |
| Legs within 5 cm | 18 / 21 | 7 / 12 |
| Heading at the result, median · max | 3.1° · 6.3° | 2.6° · 3.9° |
| Heading 2 s later, median · max | 1.6° · 30.4° | 1.8° · 6.2° |
| Recoveries per run · driven backwards per run | 0 · 0.07 m | 1 · 0.11 m |

| Leg (mean time · error, succeeded / runs) | New | 2026-10-13 set |
|---|---|---|
| Staging → lane 1 pickup | 36.8 s · 3.6 cm (3/3) | 36.3 s · 4.6 cm (2/2) |
| Harvest step 0.45 m | 4.0 s · 3.5 cm (3/3) | 4.8 s · 4.0 cm (2/2) |
| Harvest step 0.9 m | 3.9 s · 4.3 cm (3/3) | 4.8 s · 5.9 cm (2/2) |
| Rest of lane 1 | 22.3 s · 4.1 cm (3/3) | 22.4 s · 4.1 cm (2/2) |
| U-turn into the same lane | 26.0 s · 3.0 cm (3/3) | 27.2 s · 4.9 cm (2/2) |
| Ring to lane 3 | 129.1 s · 4.8 cm (3/3) | 127.7 s · 3.0 cm (2/2) |
| Lane 3 → Staging | 61.9 s · 4.2 cm (3/3) | failed (0/2): stuck 7-9 cm beside the goal |

What changed and why:
- **Planner `minimum_turning_radius` 0.2 → 0.4 m** (still SmacPlannerHybrid DUBIN). The route into Staging ends with a ~110° turn; at 0.2 m the plan made it right at the goal, the robot swung 16 cm past the line and ended 6-9 cm beside the goal facing it, where a forward-only controller can only twitch in yaw until the progress checker gives up. With 0.4 m the plan finishes the turn earlier and the robot arrives lined up. The U-turn and lane entries still plan and drive (3/3).
- **Graceful `v_linear_min` 0.05 → 0.10 m/s**: on the retuned drive (wheel friction 0.3) the robot crept the last centimetres; 0.10 cut the mean error from 4.4 to 2.7 cm in its own run (J3), but alone didn't fix Staging.

Tried and dropped: tolerance 0.10 m / 0.10 rad (J1: 7/7 and 253 s, but 9.0 cm mean, every stop at the edge of the tolerance); lookahead 2.0 m, the a200's fix (J2: 3/7, 56 recoveries: the j100 doesn't weave, and the long lookahead swings it wide).

Heading after the stop: on three legs (end of the ring twice, the 0.9 m step once) the robot was within 2-6° when the goal was accepted and turned another 15-25° over the next 2 s while standing still. That is the sim drive's known after-stop rotation (see "Final heading" below; the 2026-10-13 runs saw up to 29°), not Nav2; the 2 s figure is what the table's heading max reports.


### Single goal poses (2026-10-14)

A single goal pose (RViz's 2D Goal Pose, a `NavigateToPose` goal) still ran Nav2's default tree, which replans every second; near the goal a DUBIN replan is a loop and j100_0921 drove figure eights around it. `config/behavior_trees/navigate_to_pose_replan_if_invalid.xml` gives `NavigateToPose` the same replan-only-when-invalid change (j100, a200, a300: `default_nav_to_pose_bt_xml`). Five single goals in open ground (`goal_pose_test.py`, not in the repo):

| j100, same 5 goals | Reached | Loops | Time per goal | Error 2 s later |
|---|---|---|---|---|
| Default tree | 5/5 | 12 | 23-106 s (34 m driven for one) | 8.8-17.8 cm |
| New tree | 4/5 | 0 | 13-17 s | 3.0-5.5 cm |
| New tree + `slowdown_radius` 1.2 (committed) | 5/5 | 0 | 13-31 s | 0.9-8.3 cm |
| New tree + `prefer_final_rotation` false | 4/5 | 0 | 23-36 s, stalls near goals | 3.2-13.3 cm |

`slowdown_radius` is measured to the controller's motion target, so 1.2 m with `max_lookahead` 1.0 also caps the cruise speed at 0.42 m/s: the farm course took 353 s (7/7, 3.9 cm mean, heading ≤ 2.9°) instead of 284 s. `max_lookahead` ≥ 1.2 would lift the cap (not tested).

---

2026-10-13 · Isaac Sim 6.0 + ROS 2 Jazzy (Nav2 1.3.13), sim time. Applies to every j100 (`mtu32_bringup/config/j100/nav2.yaml`, `config/nav2_robots.yaml` `platforms.j100`, `config/behavior_trees/navigate_through_poses_replan_if_invalid.xml`). Tool: `scripts/nav2_tune/`. Per-leg data of every run: `docs/nav2_tuning_j100/results.json`; what each variant changed: `docs/nav2_tuning_j100/variants.yaml`.

The shipped MPPI setup could not finish the coordinator's farm routes: it aborted in the lanes (8/14 legs, 50 recoveries per run) and stopped only within 15 cm. The tuned setup (Graceful controller, costmaps from the farm map, replanning only when the path becomes invalid, 1 m/s² acceleration limit) finished 21/21 legs in 3 runs, the whole course in 298 s, 5.2 cm mean / 9.1 cm max from the goal, forward only.

## Result

| Measure | Tuned: Graceful | Baseline: MPPI as shipped | MPPI, map-only costmaps |
|---|---|---|---|
| Legs succeeded | 21 / 21 | 8 / 14 | 13 / 14 |
| Runs with every leg succeeded | 3 / 3 | 0 / 2 | 1 / 2 |
| Course time, complete runs | 301 s, 296 s, 298 s | – | 382 s |
| Time / estimate, median leg | 1.16× | 1.62× | 1.54× |
| Position error mean · p90 · max | 5.2 · 7.5 · 9.1 cm | 9.2 · 11.6 · 12.1 cm | 10.6 · 13.6 · 18.5 cm |
| Legs within 10 cm | 21 / 21 | 3 / 8 | 6 / 13 |
| Heading error median · max | 3.5° · 29.3° | 3.1° · 6.4° | 4.8° · 8.5° |
| Legs within 0.1 rad (5.7°) | 18 / 21 | 7 / 8 | 8 / 13 |
| Driven backwards per run (longest stretch) | 0.15 m (0.04 m) | 1.46 m (0.39 m) | 0.61 m (0.26 m) |
| Rotation while standing, per run | 1.08 rad | 1.82 rad | 0.80 rad |
| Nav2 recoveries per run | 0.7 | 49.5 | 3.5 |

Heading did not improve: median 3.5° vs 3.1°; 3 of 21 tuned legs ended above 0.1 rad, once at 29° (the sim robot keeps turning 0.3-0.5 s after a stop; the final in-place turn overshoots). The baseline's figure covers only the legs it finished.

### Per leg (mean time · mean position error, succeeded / runs)

| Leg | Tuned: Graceful | Baseline: MPPI as shipped | MPPI, map-only costmaps |
|---|---|---|---|
| Staging → lane 1 pickup | 39.3 s · 3.0 cm (3/3) | 86.8 s · 11.4 cm (1/2) | 56.5 s · 9.8 cm (2/2) |
| Harvest step 0.45 m | 6.4 s · 6.0 cm (3/3) | 13.7 s · 11.8 cm (2/2) | 5.6 s · 12.7 cm (2/2) |
| Harvest step 0.9 m | 5.4 s · 3.2 cm (3/3) | 7.9 s · 7.5 cm (2/2) | 7.7 s · 9.9 cm (2/2) |
| Rest of lane 1 | 24.8 s · 5.9 cm (3/3) | 52.1 s · 9.4 cm (2/2) | 41.1 s · 9.5 cm (2/2) |
| U-turn into the same lane | 27.4 s · 5.2 cm (3/3) | 38.6 s · 5.1 cm (1/2) | 34.7 s · 12.9 cm (1/2) |
| Ring to lane 3 | 137.8 s · 7.5 cm (3/3) | failed (0/2) | 242.2 s · 11.4 cm (2/2) |
| Lane 3 → Staging | 57.3 s · 5.7 cm (3/3) | failed (0/2) | 78.7 s · 9.0 cm (2/2) |

## Parameters

| Setting | Baseline | Tuned |
|---|---|---|
| Controller (FollowPath) | MPPI: 1000 × 72 steps, vx −0.35…0.5, critics incl. PathAlign 20, PreferForward 14 | Graceful: k_phi 2.0, k_delta 1.0, beta 0.2, lambda 2.0, lookahead 0.25–1.0 m, slowdown 0.6 m, v 0.05–0.5 m/s, ω ≤ 1.0 rad/s, in-place ≥ 0.25 rad/s, initial_rotation false, prefer_final_rotation true, allow_backward false |
| Goal checker (xy · yaw) | 0.15 m · 0.15 rad | 0.05 m · 0.05 rad |
| NavigateThroughPoses tree | Nav2's default: replans every 3 s | replans only when the path becomes invalid or the goal changes (config/behavior_trees/) |
| Global costmap layers | static, obstacle (ZED scan), inflation | static, inflation |
| Local costmap layers | static, voxel (ZED scan), inflation | static, inflation |
| Velocity smoother accel · decel | 2.5 m/s², 2.5 rad/s² | 1.0 m/s², 2.0 rad/s² |
| Unchanged | SmacPlannerHybrid DUBIN (r 0.2 m), 0.5 m/s and 1.0 rad/s limits, collision monitor on the ZED scan, progress checker, recoveries |  |

## Variants tried

| Variant | Result | What happened |
|---|---|---|
| MPPI, tolerance 0.05 | 4/7 legs, 3.2 cm mean, 22 recoveries/run | Dithers forward and back near the goal; 1.5 m of reversing in one U-turn |
| MPPI, PathFollow 12 / PathAlign 14 | 4/7 legs, 3.4 cm mean, 64 recoveries/run | Cut the lane U-turn into the plants and stayed stuck there |
| MPPI, earlier goal hand-off | 1/7 legs, 3.3 cm mean, 13 recoveries/run | Stalls ~12 cm from goals until the progress checker gives up |
| RPP, own collision check | 0/5 legs, 89 recoveries/run | Foliage in the costmap: "collision ahead" 623 times |
| RPP, collision check off | 2/7 legs, 13.3 cm mean, 0 recoveries/run | Circled goals; a Dubins replan sent it 42 m through the lane |
| RPP + replan-if-invalid tree | 3/7 legs, 5.0 cm mean, 26 recoveries/run | Can't remove 5 cm sideways error in a 0.45 m step: circles the goal |
| RPP, lookahead 0.15 m | 1/7 legs, 4.2 cm mean, 32 recoveries/run | Oscillates |
| Graceful, first cut | 6/7 legs, 5.6 cm mean, 21 recoveries/run | All real legs but yaw 5–8° (overshoot of the final turn) |
| Graceful, no final rotation | 5/7 legs, 7.7 cm mean, 22 recoveries/run | Worse position and heading, missed one goal |
| Graceful, allow_backward | 3/5 legs, 14.4 cm mean, 3 recoveries/run | Drove 5.2 m backwards (4.8 m in one go): rejected |
| Graceful, faster curves (beta 0.2) | 14/21 legs, 5.6 cm mean, 35 recoveries/run | Fast and accurate, but braked at 2.5 m/s² into a tight final turn and tipped the robot over (run 4) |
| Graceful, yaw tolerance 5° | 5/7 legs, 5.2 cm mean, 2 recoveries/run | Two aborts, one 26° heading error |
| Graceful + 1 m/s² limit (chosen) | 21/21 legs, 5.2 cm mean, 1 recoveries/run | No tip-over, stops more precisely |

## How it was measured

- **Robot and sim**: j100_0921 alone in the sim's lavender farm (`lavender_farm_chargers.usda`), zenoh, sim time (the sim ran at about 0.68x real time; every time below is sim time). Nav2 started the way `bringup_main` starts it (`bringup_nav2_map.launch.py`, map `zone_end_2`, the scene's chargers as docks), the robot's stack without the operations manager so the coordinator couldn't hand it tasks.
- **Course**: the coordinator's own routes. Each leg is the waypoint list `status_server` publishes between two graph nodes (`tools/path_check.py path A B --waypoints`), sent as one `NavigateThroughPoses` goal like `husky_operations_manager` does: Staging to a harvest pickup in lane 1, a 0.45 m and a 0.9 m harvest step, the rest of the lane, the U-turn at the east end into the same lane's other side, the ring to lane 3 (57 m, 79 waypoints), lane 3 back to Staging. About 100 m in all. A last, synthetic leg asks for a goal 0.5 m straight behind the robot; the coordinator never sends one (its graph is one-way), so it is reported separately.
- **Per leg**: time from goal accepted to result; xy and yaw error to the goal 2 s after the result, against the sim's ground truth (`ref_pose`), not Nav2's own estimate (they differ by about 2 cm while driving); distance driven backwards and the longest backward stretch; rotation while standing; Nav2 recoveries. A leg still running at 300 % of its estimated time (route length at 0.5 m/s + 5 s) is cancelled and counts as failed.
- **Protocol**: every run starts from a sim reset (robot back at its spawn pose, upright check, the sim restarted if not), drives to Staging first (not counted), then the course. The same course for every configuration.

## What was wrong with the baseline

1. **The planner failed inside the lanes.** The lanes are 0.74 m between plants for a 0.43 m robot, and the coordinator's pickup lines run 0.13 m off the lane centre, so the robot passes about 3 cm from the foliage. The ZED depth scan marks that foliage in both costmaps; at 5-6 cm resolution the footprint then overlaps lethal cells. SmacPlannerHybrid failed between consecutive lane waypoints ("exceeded maximum iterations", "Start occupied"), the behavior tree ran out of recoveries and aborted the goal. The aborts were systematic: lane entry and the 57 m ring route failed in every baseline run.
2. **Replanning every 3 s with a DUBIN planner near a goal returns a loop.** A few centimetres beside or short of a goal there is no short forward-only path; the planner returns a loop (minimum turning radius 0.2 m). MPPI mostly ignores the path that close to the goal; a path-following controller drives the loop, which in a lane means circling through the plants.
3. **MPPI can't stop precisely.** The baseline stops as soon as it is within 0.15 m (errors 0.09-0.15 m). With 0.05 m tolerance MPPI dithered back and forth near the goal until the progress checker gave up (1 of 7 legs succeeded with a goal-approach retune). Weakening its path alignment to gain speed made it cut the lane U-turn into the plants and get stuck there. This is the sensitivity you described.
4. **Overshoot after the last command.** Braking at Nav2's 2.5 m/s² into a tight final turn tipped the sim robot over (pitch 77°) with an earlier Graceful variant; a 1 m/s² limit stopped that and also cut the overshoot on short harvest steps from ~9 cm to ~4 cm.
5. **Final heading.** The sim robot keeps turning 0.3-0.5 s after a stop command. A final in-place turn at 0.4 rad/s or more overshoots by 4-25° after the goal checker has already said "reached", so a tighter yaw tolerance alone made heading worse, not better.

## What was tried

Three controllers, each on the same course: MPPI (baseline weights, tighter tolerance, retuned critics), Regulated Pure Pursuit (with and without its own collision check, shorter lookahead) and the Graceful controller (Nav2's docking control law used for path following), plus the planning and costmap changes above. RPP was dropped: with a 0.3 m lookahead it can't remove a 5 cm sideways offset within a 0.45 m harvest step and circles the goal; with 0.15 m it oscillates. MPPI was dropped for the precision reasons above. Graceful was kept: smooth arcs, forward only, converges on the goal pose.

## Trade-offs and what to check on the real robot

- **No planning around unmapped obstacles.** The camera now only feeds the collision monitor (FootprintApproach, 1.2 s ahead). Measured in the sim: driving at an unmapped charger box, the robot stopped with its front 4 cm short of the box, waited, and the goal timed out; it does not plan around it. In a 0.74 m lane there is no way around anyway; on the ring an obstacle stops the robot until it is moved. FootprintApproach creeps until very close: consider a stop zone in front of the footprint (not added: in a lane it could also stop on plants). An alternative not tried: keep the obstacle layer in the global costmap outside the lanes only (a costmap filter mask).
- **No reversing.** The planner (DUBIN) never asks for it and the controller does not reverse on its own (`allow_backward: true` drove 5 m backwards). A goal 0.5 m straight behind is reached by a short loop or not at all; the coordinator never sends one. The BT's BackUp recovery (0.3 m) is still there.
- **Acceleration limits.** Braking at Nav2's 2.5 m/s² into a tight final turn tipped the sim robot over (once certainly, once likely, with an earlier Graceful variant). 1 m/s² stopped that and also made the stops more precise. The sim j100_0921 is top-heavy (arm); the real one may tolerate more, but 1 m/s² is a sensible limit with the arm on.
- **Sim-specific behaviour.** The 0.3-0.5 s turn lag and the rotation dead zone come from the sim's drive model; the real Jackal will likely stop turning sooner, which only helps. Accuracy numbers are against the sim's ground truth with near-perfect localization (~2 cm); on the real robot the localization error adds to them.
- **Not run on the real robot.** The parameters are deployed to every j100 with `colcon_ws`; try them on j100_0921 under supervision before relying on them.
