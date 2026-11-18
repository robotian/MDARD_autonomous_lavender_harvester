# status_server

`status_server` is a ROS 2 `ament_python` package that synchronizes robot status into a PostgreSQL farm database and publishes task assignments back to the fleet.

## Overview

The package has two nodes, both started by the launch file:

- **`status_server`** (`robot_status_sync.py`). Subscribes to `<ns>/status/robot` (`status_interfaces/RobotStatus`) for every namespace in `namespace_config`, and writes each update into `robot_live`. A liveness watchdog marks a robot `OFFLINE` once it has been silent for `liveness.timeout`. Robots that never report at all are also marked offline.
- **`task_manager`** (`job_publisher.py`). Every 5 s it scans `robot_info` / `robot_live` for robots that are online. For each new robot it creates:
  - a publisher on `/<serial>/status/task` (`status_interfaces/Task`), ticked at 1 Hz
  - a `BEST_EFFORT` subscription to `/<serial>/platform/bms/state` (`sensor_msgs/BatteryState`), used for [battery feasibility](#battery-feasibility)

  Tasks (harvesting, unloading, charging) are built by `task_generator.py`. A move is one goal, from the robot's reported pose to the target node's pose (a plant's Pickup node); the robot's Nav2 plans the way on the farm map, no route graph is followed. A dock task's move only leaves the lane straight (the lane exit pose), or is a no-op outside a lane: the docking goal (`navigate_to_staging_pose`) drives to the staging pose the robot's docking server computes from its own dock pose and `staging_x_offset`. `dock_station.node_id` is only the dock's place for the battery check's return trip. Dock/undock goals come from `dock_station`. On start-up, `task_manager` releases any dock reservations left behind by a run that ended while docked.

Database tables used: `robot_info`, `robot_live`, `dock_station`, `farm_harvesting_job`, and the target poses: `graph_node` / `object_data`, or the legacy `farm_node` / `farm_asset_on_map`, depending on `farm.graph_source`. The edges (`graph_edge`, `farm_edge`) are not read; only the offline `tools/` use them.

## Package Contents

- `launch/status_server.launch.py` – launches both nodes together
- `config/config.yaml` – namespaces, battery, liveness, farm, clearance, dock and PostgreSQL settings
- `status_server/robot_status_sync.py` – `status_server` node entry point
- `status_server/job_publisher.py` – `task_manager` node entry point
- `status_server/task_generator.py` – task and goal construction
- `status_server/postgres_manager.py` – all database access (psycopg 3)
- `status_server/configuration.py`, `dataclass.py`, `data_utils.py`, `enum.py` – config loading, data types, status/task enums
- `tools/` – offline graph checkers that need no ROS environment (see [Tools](#tools))
- `data/cost_matrix.csv` – reference data only. It is not installed and not read by the nodes
- `test/` – ament flake8 / pep257 / copyright tests

## Dependencies

- ROS 2 (Jazzy): `rclpy`, `launch`, `launch_ros`, `ament_index_python`
- `status_interfaces` (in this workspace)
- `geometry_msgs`, `sensor_msgs`, `tf_transformations`
- Python: `python3-yaml` (`python3-numpy` and `python3-scipy` only for the offline `tools/`)
- **psycopg 3** (`sudo apt install python3-psycopg`). It has no rosdep key, so `rosdep` won't install it. Don't substitute `python3-psycopg2`, which has a different API.
- A PostgreSQL database holding the farm schema. In this repo that is the base station (`basestation.compose.yml`, port 5433).

## Build and Install

From the workspace root:

```bash
colcon build --packages-select status_server
source install/setup.bash
```

In this repo, use `scripts/colcon_build.sh --packages-select status_server` instead.

## Run

```bash
ros2 launch status_server status_server.launch.py
ros2 launch status_server status_server.launch.py log_level:=debug
```

- `log_level` is applied to `task_manager` only.
- The `namespace` argument is declared but not applied to the nodes. Topics are absolute (`/<serial>/...`), and the robots to watch come from `config.yaml`.
- The launch file does not set `use_sim_time`. Both nodes run on wall time, which is consistent within each node: liveness compares message arrival times against the node clock.

The database password is not in `config.yaml`. Export `PGPASSWORD` or use `~/.pgpass` before launching (see [`database`](#database--postgresql-connection)).

## Harvesting operation (2026-10)

The nodes run in the base station container under `scripts/basestation_coordinator.sh` while `public.operation.coordinator_enabled` is true (the web UI's `/operation` page, `tools/sim_ui`). Tables from `basestation/migrations/002_harvest_operation.sql`:

- `harvest_state`: one row per plant (`object_id`), `status` pending / assigned / harvested / failed, `lane_id`, `pickup_node_id`, `assigned_robot_id`, `job_id` (the current `farm_harvesting_job.id`), `attempts`, `last_error`, timestamps. The source of truth for "harvested"; `farm_harvesting_job` is the per-task log (`progress_status` 0 idle, 1 in progress, 2 paused, 3 done, 4 failed, 5 released).
- `operation` (one row): `state` stopped / running / paused / complete, `coordinator_enabled`, `target_rows` (NULL = all), `max_attempts`, `offline_grace_s`.
- `operation_event`: the log the page shows (`log_event`).
- `robot_operation` (migration 004): the operator's per-robot controls, read every tick (`fetch_robot_operation`): `enrolled` false or `state` paused = no new task after the current one; `state` stopped or `request` stop = `stop_robot` (the task released, a STOP task published: `task_id` -1, description `STOP`, no `job_schedule`; the executor cancels everything and reports `FAILED:stopped`, which counts as released, not as an attempt); `request` charge = a charging task now (`handle_low_battery`), deferred while the robot cuts or docks.
- `charging_stations.dock_id` (migration 003): the dock's name in the robots' Nav2 docking server (`dock_station.dock_id`); the sim names its docks the same.

Task flow per robot (`job_publisher.py`): IDLE and no task → `assign_next_task` (operation `running`: `fetch_task_for_robot` → charging at or below `battery.low_threshold` or when `can_complete_task` fails, else `generate_harvest_task`, which claims a pending plant (`claim_plant`) in the robot's lane or a free lane, the last waypoint moved by `harvest.cutter_offset_x`; `paused`: nothing new; `stopped` / `complete`: park at the charger). While the operation is `stopped`, a harvest task still held (assigned before Stop, or restored after a coordinator restart) is cancelled like the robot's own Stop (`stop_robot`: plant pending, job released, STOP republished until the robot is IDLE / ERROR); charging tasks run on. JOB_DONE → `complete_task` (harvest: `mark_plant_harvested`). ERROR with `RobotStatus.task` = `FAILED:<kind>:<task_id>:<detail>` → `handle_task_failure` (close the task, `fail_plant_attempt` up to `operation.max_attempts`, free the dock, a new task_id; for `battery` the plant goes back to pending without an attempt, and a charging task follows; a `nav` failure within 0.25 m of where the robot's previous `nav` failure left it means the robot is stuck, not the plant: the plant goes back to pending without an attempt and the robot is paused (`robot_operation.state`, event `pause`) until an operator frees it and resumes it). OFFLINE longer than `offline_grace_s` → `handle_offline` releases the task; a robot back on it gets STOP until it is IDLE / ERROR. A MOVING harvest task is preempted by a charging task at `battery.critical_threshold` (only once a charger is free: the task is released after the charging task exists). `running` → `complete` once no target plant is pending or assigned. Status ABNORMAL / online_flag 10, 11: nothing is sent until the robot is back. Moves (`_resolve_route`): the robot's reported pose (else its nearest node's), lane exit / entry poses (`_lane_legs`), and the target pose with its stored heading; the executor sends them as one `NavigateThroughPoses` goal (a single waypoint would make the robot's own position the goal), so Nav2 plans every leg on the map, no graph nodes. Lanes (between two crop rows of `object_data`, `fetch_crop_rows`) are one-way: a robot in a lane leaves it straight out of the end it faces, `harvest.lane_end_margin` (1.2 m) past the rows, and a target in a lane is entered from the end its heading comes from, unless it is straight ahead in the robot's own lane (judged on the stop pose, cutter offset included: the next plant's node sits just behind a robot that stopped past the previous one; judged on the node, the robot drove a lap, and Nav2 ended that lap's goal early as it passed over the stop pose). Without them Nav2 U-turned a j100 inside a lane (0.5 m from plant centres); with them no planned path reverses in a lane (8 cases, 2026-10-14). The executor keeps every waypoint's heading (it used to point each one at the next). The battery check's return trip and `harvest.side: nearest` use `|dx| + |dy|` between node poses (`travel_distance`): the lanes run along x, so a lane change goes round a row end.

## Configuration

All runtime settings live in `config/config.yaml`, parsed into the `Config`
dataclass by `status_server/configuration.py`.

### Loading behaviour

- **Every key is mandatory.** `DataUtils.from_dict` has no defaults; a missing
  key raises `KeyError` inside the parser at node start-up.
- **Config is cached at first load** (`Configuration._config`). A running node
  will not pick up edits — **restart the node** to apply changes.
- With `--symlink-install` the installed `config.yaml` points at the source
  file, so no rebuild is needed; only a restart.
- Values are used as-is, no type coercion beyond what YAML infers. Write floats
  as `20.0`, not `20`.

### `namespace_config` — which robots to track

`mode: auto` subscribes to every `robot_info.serial_number` and rescans the table every 30 s (new robots need no config edit).


Consumer: `robot_status_sync.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `mode` | str | `multi` | `single` = subscribe to `namespace` only. `multi` = subscribe to every entry in `namespaces`. |
| `namespace` | str | `/a300_00036` | Namespace used when `mode` is `single`. Ignored in `multi`. |
| `namespaces` | list[str] | `[/j100_0921, /a300_00036]` | Namespaces subscribed when `mode` is `multi`. Ignored in `single`. |

### `battery` — task feasibility

Harvesting operation thresholds (percent, 0-100): `low_threshold` (30) a robot at or below it charges before its next task (replaces `reserve_threshold`, kept as the fallback name); `critical_threshold` (15) preempts a harvest task still on its way; `full_threshold` (92) is sent in the charging task (`CHARGING.data_str` `{"charge_to": N}`) and the robot charges to it.


Consumer: `job_publisher.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `reserve_threshold` | float | `20.0` | Minimum battery level in percent (0–100) to accept a new harvesting or unloading task. At or below this the robot is sent to a charging dock instead. |
| `ewma_alpha` | float | `0.1` | Smoothing factor for the running power and speed estimates, `0 < alpha <= 1`. Higher reacts faster and is noisier. At a 1 Hz BMS feed, `0.1` means the last ~10 samples dominate. |
| `safety_factor` | float | `1.5` | Margin applied to predicted energy before comparing against the charge on board. `1.5` = the robot must carry 50% more than the estimate says it needs. |
| `min_move_distance` | float | `0.05` | Metres a robot must have moved between ticks before the sample counts towards the speed estimate. Stops a parked robot dragging the speed estimate to zero, which would make the return-leg prediction diverge. |
| `fixed_task_seconds` | dict[str, float] | `{harvesting: 480.0, unloading: 300.0}` | Whole-task durations in seconds, including each task's own navigation. The charger return trip is **not** listed — it is derived from distance (`|dx| + |dy|` from the target to the charger's node, `travel_distance`). `charging` has no entry: it is the fallback task and is never vetoed. |

See [Battery feasibility](#battery-feasibility) below for the prediction model.

### `harvest` — how a plant is approached

`side`: `north` (the Pickup node on the +y side, heading +x, the plant on the robot's right, lane `row_id - 1`) or `nearest`. `cutter_offset_x`: metres the stop pose is moved forward along the heading, per platform (the serial's prefix; `offset_default` otherwise): the cutter (`stow_arm_cpp`) works behind the robot's right side, so the plant must end up beside the arm (j100 0.41, a200 0.71, a300 0.61). `lane_end_margin` (1.2): metres past the crop rows' ends where a robot leaves or enters a lane, so it turns out there, never between two rows.

### `liveness` — offline watchdog

Consumer: `robot_status_sync.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `timeout` | float | `30.0` | Seconds without a status message before a robot is marked `OFFLINE`. Must exceed the worst-case gap between messages (including DDS jitter) or a healthy robot flaps offline. |
| `check_period` | float | `1.0` | How often the watchdog scans for stale robots, in seconds. |
| `idle_task_timeout` | float | `60.0` | Seconds a robot may report `IDLE` while `task_manager` holds a task for it before the task is released (plant back to pending, dock freed, job closed as released) and it gets a new one. The robot ignores republishes of a task it already handled, so a `JOB_DONE` the server missed (both loops run at 1 Hz) would otherwise leave both stuck. |

### `farm` — farm-wide settings not held in the database

Consumers: `postgres_manager.py` (`graph_source`), `task_generator.py` (`crop_type`)

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `crop_type` | str | `lavender` | Crop grown on this farm. Written onto generated harvest tasks; the `farm_asset_on_map` / `object_data` tables carry no crop info of their own. |
| `graph_source` | str | `graph` | Which table family supplies the target nodes (Pickup, Staging). `graph` → `graph_node` / `object_data` (current). `farm` → `farm_node` / `farm_asset_on_map` (legacy). Edges are not used. |

### `clearance` — waypoint shift off the crop

Consumer: `task_generator.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `enabled` | bool | `false` | Apply the shift at all. `false` publishes raw node positions — correct when the graph has already been moved so lanes clear the mapped crop. Set `true` if waypoints start landing on the crop again. |
| `robot_half_width` | float | `0.349` | Half the robot footprint width. This is the inscribed radius Nav2's inflation layer treats as definitely-in-collision around an obstacle; it sets how far sideways a bush must be before the robot may sit beside it. |
| `robot_half_length` | float | `0.495` | Half the robot footprint length. Sets how far ahead or behind an object still counts as level with the robot. A long robot's front corner reaches a bush its centre is nowhere near. |
| `obstacle_radius` | float | `0.225` | How far the mapped obstacle extends from the object centre held in `object_data`. Crop rows measure 0.45 m thick in the map, so 0.225 m. |
| `safety_margin` | float | `0.10` | Extra metres on top of the two radii. Measured, not guessed: crop rows in `zone_end_1.pgm` have a ragged edge with stray occupied cells up to a costmap cell beyond the nominal half-thickness. At `0.05` the footprint still clipped 413 of 760 harvest waypoints; at `0.10` it clips none. |
| `max_shift` | float | `0.30` | Largest shift allowed for a single waypoint. A waypoint needing more is published as close as the cap allows and the shortfall is logged. Past this the geometry is wrong and the fix belongs in the map or the graph. |

`ClearanceConfig.standoff` (derived, not configured) = `obstacle_radius + robot_half_width + safety_margin`.

### `docks` — dock IDs per robot

Consumers: `job_publisher.py`, `task_generator.py`

Structure: `graph_source -> namespace -> purpose -> list of dock IDs`, where
`graph_source` is `graph` or `farm` and `purpose` is `charging` or `unloading`.

```yaml
docks:
  graph:
    /a300_00036:
      charging: [wiferion_charger]
      unloading: [unloading_station]
  farm:
    /a300_00036:
      charging: [husky_charger]
      unloading: [unloading_station]
```

- The block is keyed by graph source first, because node IDs are **not** shared
  between the two schemas. Node 12 in `farm_node` is a different place from
  node 12 in `graph_node`, so switching `farm.graph_source` switches the docks
  with it. The live source's block is used; if it is missing, the robot gets no
  docks and an error is logged.
- A flat block (namespaces at the top level, no source keys) is still accepted
  and used for every source. Keys starting with `/` tell the two layouts apart.
- Only **dock IDs** live here. The node the robot drives to, the Nav2 plugin
  type and the reservation state all stay in the `dock_station` table, so
  giving a robot a second dock is a config edit, not a schema change.
- A dock ID is **global**: it is both the `dock_station` key **and** the name
  registered with that robot's Nav2 docking server, so the two must agree.
- Listing the same ID under two robots is allowed. They share the dock, and the
  `dock_station` reservation decides who gets it.
- An empty `unloading` list makes unloading tasks fall back to a charging dock.
- Config keys are written as ROS namespaces (`/j100_0921`); lookup also accepts
  the bare serial number (`j100_0921`).

### `database` — PostgreSQL connection

Consumer: `postgres_manager.py`

| Parameter | Type | Example | Meaning |
|---|---|---|---|
| `host` | str | `host.docker.internal` | Hostname or IP of the PostgreSQL server. `host.docker.internal` reaches the base station from the sim's robot containers and the base station itself. A real robot uses the base station's LAN IP. |
| `port` | int | `5433` | Server port. The base station uses 5433 so it doesn't collide with a PostgreSQL installed on the host on 5432. |
| `connect_timeout` | int | `5` | Seconds to wait for a connection before giving up. Without it psycopg falls back to the OS TCP timeout, which can block a node for minutes if the host is unreachable. |
| `dbname` | str | `test_lavender_farming` | Database name. |
| `user` | str | `admin` | Login user. |
| `password` | str | `''` | Leave empty: the password is not kept in git. With it empty, libpq reads `PGPASSWORD` from the environment or `~/.pgpass` (`host:port:dbname:user:password`, mode 0600). On a robot, set either for the user that runs the node; in the sim, put `PGPASSWORD=...` in the untracked `db.env` at the repo root (loaded into every robot container). |

## Clearance geometry

The graph puts a Pickup node where the manipulator wants it, which is closer to
the bush than the robot's body may legally sit. Nav2 paints a band of the
robot's own inscribed radius around every mapped obstacle as untraversable and
rejects any pose whose footprint touches it, so those nodes are goals no planner
will accept. Shifting the waypoint away from the bush buys that clearance back,
and costs the same distance in arm reach.

Measured against `zone_end_1.pgm`: the crop rows are drawn 0.45 m thick, the
lanes sit 0.45 m off the bush centres, and the robot is 0.698 m wide — so the
footprint overlaps the mapped crop by about 0.12 m and needs roughly 0.17 m of
shift. If the arm cannot reach that far, the fix belongs in the map or the
graph, not in `clearance`.

With `enabled: false`, the graph itself was moved so the lanes clear the mapped
crop, which is the cheaper fix — moving a lane costs nothing, whereas the shift
here buys clearance by spending manipulator reach. Re-enable only if waypoints
start landing on the crop again; verify with:

```bash
python3 tools/graph_audit.py --checks clearance
```

## Battery feasibility

`job_publisher.py` subscribes to `/{namespace}/platform/bms/state`
(`BEST_EFFORT`) and keeps a per-robot EWMA of power draw (discharge samples
only: a charging sample is the charger's current, not the robot's draw) and
travel speed. A task is feasible when:

```
predicted_energy = avg_power_w * (fixed_task_seconds[task] + return_leg_m / avg_speed_mps)
predicted_energy * safety_factor <= energy_on_board_wh
```

- `return_leg_m` is the distance from the task end to the assigned charging
  dock's node, `|dx| + |dy|` (`travel_distance`; no route graph).
- Speed samples only count when the robot moved at least `min_move_distance`
  between ticks; a speed of zero is treated as "not yet measured" so the
  division cannot blow up.
- `charging` is never vetoed — it is the fallback when every other task fails
  the check.

## Tools

Standalone scripts in `tools/`. They read `config/config.yaml` for the
database connection and need psycopg, pyyaml, numpy and scipy, but no ROS
environment and no running nodes. Both accept `--config` and `--dbname`.

- **`path_check.py`**: graph routing checker, from when the coordinator routed over the graph (it no longer
  does: one Nav2 goal per move). Its node snapping still mirrors `RobotStatusSync`; its `path`/`sweep`/`task`
  output is the old routing, not what the coordinator sends. `scripts/nav2_tune/course.py` builds its courses
  from it. `--source
  graph|farm` overrides `farm.graph_source`.

  ```bash
  python3 tools/path_check.py path 926 1 --waypoints   # route + published waypoints
  python3 tools/path_check.py sweep --limit 80         # harvest order, as tasks will be generated
  python3 tools/path_check.py task --at 970            # simulate the next harvest task
  python3 tools/path_check.py snap -5.114 -0.895 1.600 # pose -> nearest node
  python3 tools/path_check.py edges 926                 # edges into and out of a node
  python3 tools/path_check.py audit --samples 20000    # graph health and path validity
  ```

- **`graph_audit.py`**: checks whether a planner can actually drive the graph,
  using geometry only. The checks are `sideways`, `zerolen`, `approach`,
  `turns`, `clearance` and `dubins`. Robot dimensions default to the A300;
  override them with `--robot-width`, `--robot-length`, `--turning-radius`,
  `--inflation` and `--cost-scaling`. `--sql` prints the SQL that adds the missing approach nodes, then exits. The exit code
  is the number of failed checks.

  ```bash
  python3 tools/graph_audit.py
  python3 tools/graph_audit.py --checks sideways,approach --limit 0
  ```

## Entry Points

- `status_server`: starts the status synchronization node
- `task_manager`: starts the task publishing node

## License

Apache-2.0
