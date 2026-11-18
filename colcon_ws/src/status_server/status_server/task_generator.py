"""TaskGenerator — generates typed Task messages for the robot fleet."""

import json
import math
import os

from rclpy.impl.rcutils_logger import RcutilsLogger
from status_interfaces.msg import DockGoal, SubTask, Task, UndockGoal, WayPoint

from status_server.configuration import Configuration
from status_server.data_utils import DataUtils
from status_server.dataclass import DockStation, JobDetails, PlantTarget, TopologyMapPosition
from status_server.enum import DockTypeEnum, NodeTypeEnum, ProgressStatusEnum, TaskEnum
from status_server.postgres_manager import PostgresOperations

logger = RcutilsLogger(os.path.basename(__file__))

# Maximum time Nav2 is allowed to spend undocking before it gives up.
_UNDOCK_TIMEOUT = 30.0

# harvest_ready values written onto generated task rows.
_HARVEST_READY = 1
_NOT_HARVEST = 0


class TaskGenerator:
    """
    Generates Task messages (with sub-tasks and waypoints) for the robot fleet.

    A move is one Nav2 goal: from where the robot is to the target node's pose (a plant's Pickup node; a dock task
    only leaves the lane, the docking server drives to its staging pose), with no route graph. Nav2 plans the way on the farm map, except that a robot never turns
    between two crop rows: it leaves a lane straight out of its end and enters the target's lane from the end
    that faces the target (lane exit / entry poses from the rows' geometry, _lane_legs).
    """

    def __init__(self, db_ops: PostgresOperations | None = None) -> None:
        """
        Args:
            db_ops: Database access to reuse. The task manager passes its own
                so the process holds one connection instead of two; omit it and
                a new connection is opened.
        """
        self.db_ops = db_ops if db_ops is not None else PostgresOperations()
        self.crop_type = Configuration().get_farm_config().crop_type
        self.harvest = Configuration().get_harvest_config()
        self.charge_to = float(Configuration().get_battery_config().full_threshold)

        # Object positions and the standoff rule are read once: the crop does
        # not move, and every published route consults them.
        self.clearance = Configuration().get_clearance_config()
        self.object_positions = self.db_ops.fetch_object_positions() if self.clearance.enabled else []

        if self.clearance.enabled:
            logger.info(
                f'Waypoint clearance on: standoff {self.clearance.standoff:.3f} m '
                f'(obstacle {self.clearance.obstacle_radius:.3f} + half-width '
                f'{self.clearance.robot_half_width:.3f} + margin {self.clearance.safety_margin:.3f}), '
                f'cap {self.clearance.max_shift:.3f} m, {len(self.object_positions)} objects.'
            )
        else:
            logger.warning('Waypoint clearance off: node positions are published unchanged.')

        # (x_min, x_max, mean y) per crop row, north first: the lanes between them are one-way (_lane_legs)
        self.crop_rows = self.db_ops.fetch_crop_rows()
        if not self.crop_rows:
            logger.warning('No crop rows in object_data: robots may turn around inside the lanes.')

        self.validate_dock_config()

    # =========================================================================
    # TASK GENERATORS
    # =========================================================================

    def generate_harvest_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str = '',
    ) -> Task | None:
        """
        Create a harvest task:  MOVING → HARVESTING (cut and load).

        Picks the next unharvested bush side from the sweep order, writes a new
        row to farm_harvesting_job, and uses the generated row ID as task_id.

        Returns None when the whole farm has been harvested or when no path
        can be computed - never a default Task(), which would publish silently
        empty work.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
        """
        target = self._next_harvest_target(robot_id, current_node_id)
        if target is None:
            logger.info('No unharvested plants remain (for this robot).')
            return None

        plant, node_id, lane_id = target
        asset_name = f'b{plant.row_id}_{plant.object_id}'
        location = (plant.x, plant.y)

        # Resolve the route BEFORE writing anything. Inserting first would
        # leave an orphan open row on a path failure, and task_exists would
        # then treat that bush side as done forever.
        route = self._resolve_route(current_node_id, node_id, context='harvest', robot_id=robot_id,
                                    advance=self._cutter_offset(serial_number))
        if route is None:
            return None

        task_id = self.db_ops.insert_task(
            node_id=node_id,
            job_schedule=asset_name,
            crop_type=self.crop_type,
            harvest_ready=_HARVEST_READY,
            assigned_robot_id=robot_id,
            location=location,
            task_type=TaskEnum.HARVESTING_TASK.value,
        )
        if task_id is None:
            return None

        # The plant is the server's unit of work: claim it for this task. Another assignment in the same tick
        # (two robots' timers) could have taken it: then this row is closed again and the caller retries.
        if not self.db_ops.claim_plant(plant.object_id, robot_id, node_id, lane_id, task_id):
            logger.warning(f'Plant {plant.object_id} was claimed by another task; closing task {task_id}.')
            self.db_ops.close_task(task_id, asset_name, ProgressStatusEnum.RELEASED.value)
            return None

        return self._build_harvest_task(
            task_id=task_id,
            robot_id=robot_id,
            route=route,
            job_schedule=asset_name,
            crop_type=self.crop_type,
            serial_number=serial_number,
            plant=plant,
        )

    def generate_charging_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
    ) -> Task | None:
        """
        Create a charging task:  MOVING → DOCKING → CHARGING.

        Reserves a free charging dock from the robot's own list, writes a new
        task row, and uses the generated row ID as task_id.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
            serial_number:   Robot namespace, used to look up its docks.
        """
        return self._generate_dock_task(
            robot_id=robot_id,
            current_node_id=current_node_id,
            serial_number=serial_number,
            dock_type=DockTypeEnum.CHARGING,
            task_type=TaskEnum.CHARGING_TASK,
        )

    def generate_unloading_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
    ) -> Task | None:
        """
        Create an unloading task:  MOVING → DOCKING → UNLOADING.

        Reserves a free unloading dock, writes a new task row, and uses the
        generated row ID as task_id.

        No unloading dock has a node in the graph yet, so a robot with an
        empty `unloading` list falls back to a charging dock. With one charger
        per robot that dock is already reserved, so unloading returns None and
        the basket is never emptied - the fallback exists to keep the code
        path alive until an unloading node exists, not to work in the field.

        Args:
            robot_id:        Database ID of the robot.
            current_node_id: Node the robot currently occupies.
            serial_number:   Robot namespace, used to look up its docks.
        """
        docks = Configuration().get_robot_docks(serial_number)

        if docks.unloading:
            dock_type = DockTypeEnum.UNLOADING
        else:
            logger.warning(f'[{serial_number}] No unloading dock configured; falling back to a charging dock.')
            dock_type = DockTypeEnum.CHARGING

        return self._generate_dock_task(
            robot_id=robot_id,
            current_node_id=current_node_id,
            serial_number=serial_number,
            dock_type=dock_type,
            task_type=TaskEnum.UNLOADING_TASK,
        )

    def next_target_node(self, task_type: TaskEnum, serial_number: str, robot_id: int | None = None,
                         start_node: int | None = None) -> int | None:
        """
        Report where a task of this type would send the robot, without
        creating it.

        Lets the caller judge whether the robot can survive the trip before
        any row is written. Read-only: the dock is looked up but not reserved,
        so this deliberately ignores occupancy.

        Args:
            task_type:     Kind of task being considered.
            serial_number: Robot namespace, used to look up its docks.

        Returns:
            Target node ID, or None if no such task is available.
        """
        if task_type == TaskEnum.HARVESTING_TASK:
            target = self._next_harvest_target(robot_id, start_node)
            return target[1] if target else None

        docks = Configuration().get_robot_docks(serial_number)
        dock_ids = docks.unloading if task_type == TaskEnum.UNLOADING_TASK else docks.charging

        if not dock_ids:
            dock_ids = docks.charging

        for dock_id in dock_ids:
            dock = self.db_ops.fetch_dock_by_id(dock_id)
            if dock is not None:
                return dock.node_id

        logger.warning(f'[{serial_number}] None of its docks {dock_ids} exist in dock_station.')
        return None

    def rebuild_task(
        self,
        job: JobDetails,
        current_node_id: int,
        serial_number: str = '',
    ) -> Task | None:
        """
        Rebuild a Task message from an existing task row.

        Used to rehydrate a robot's in-flight task after a server restart. No
        row is written - the task already exists, only the message (waypoints,
        sub-tasks) has to be reconstructed from the robot's current position.

        Args:
            job:             Task row fetched from farm_harvesting_job.
            current_node_id: Node the robot currently occupies.

        Returns:
            Task, or None if the task type is unknown or no path exists.
        """
        if job.task_type == TaskEnum.HARVESTING_TASK.value:
            route = self._resolve_route(current_node_id, job.node_id, context='harvest',
                                        robot_id=job.assigned_robot_id, advance=self._cutter_offset(serial_number))
            if route is None:
                return None

            return self._build_harvest_task(
                task_id=job.id,
                robot_id=job.assigned_robot_id,
                route=route,
                job_schedule=job.job_schedule,
                crop_type=job.crop_type,
                serial_number=serial_number,
                plant=self.db_ops.fetch_plant_by_job(job.id),
            )

        if job.task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
            # job_schedule holds the dock_id for dock tasks
            dock = self.db_ops.fetch_dock_by_id(job.job_schedule)
            if dock is None:
                logger.error(f"Cannot rebuild task {job.id}: dock '{job.job_schedule}' not found.")
                return None

            task_type = TaskEnum(job.task_type)
            label = 'charging' if task_type == TaskEnum.CHARGING_TASK else 'unloading'

            route = self._resolve_route(current_node_id, dock.node_id, context=label,
                                        robot_id=job.assigned_robot_id)
            if route is None:
                return None

            task = self._build_dock_task(
                task_id=job.id,
                robot_id=job.assigned_robot_id,
                route=route,
                dock=dock,
                task_type=task_type,
            )

            # A resumed dock task must own its dock again. The reservation may
            # have been dropped while the task was paused - release_dock_for_robot
            # frees every dock the robot holds, not just the one being undocked.
            self.db_ops.reserve_dock(dock.dock_id, job.assigned_robot_id)

            return task

        logger.error(f'Cannot rebuild task {job.id}: unknown task_type {job.task_type}.')
        return None

    # =========================================================================
    # TASK BUILDERS
    # =========================================================================

    def _next_harvest_target(self, robot_id: int | None = None, start_node: int | None = None
                             ) -> tuple[PlantTarget, int, int] | None:
        """
        Pick the next pending plant for this robot: (plant, Pickup node, lane).

        One visit per plant (harvest_state, migration 002). The side is harvest.side: 'north' = the Pickup node
        on the +y side (heading +x, lane row_id - 1), 'nearest' = the one closer to start_node. Plants are taken
        in fetch_plant_targets order (fewest attempts, row, x), but a robot keeps its own lane while it has
        pending plants, and never enters a lane another robot is working in. The legacy 'farm' schema keeps the
        old two-sides sweep.
        """
        if not getattr(self.db_ops, '_use_graph_tables', True):
            legacy = self._next_harvest_target_legacy()
            if legacy is None:
                return None
            name, node_id, (x, y) = legacy
            row = int(name.split('_')[0].lstrip('b') or 0)
            obj = int(name.split('_')[-1])
            return PlantTarget(obj, row, x, y, 0, node_id, 0.0, node_id, 0.0), node_id, row

        op = self.db_ops.fetch_operation()
        targets = self.db_ops.fetch_plant_targets(op.target_rows if op else None)
        if not targets:
            return None

        def side_of(plant: PlantTarget) -> tuple[int, int]:
            if self.harvest.side == 'nearest' and start_node is not None:
                dn = self.travel_distance(start_node, plant.north_node)
                ds = self.travel_distance(start_node, plant.south_node)
                if dn is not None and ds is not None and ds < dn:
                    return plant.south_node, plant.row_id
            return plant.north_node, plant.row_id - 1

        my_lane = self.db_ops.fetch_robot_lane(robot_id) if robot_id is not None else None
        leased = self.db_ops.fetch_leased_lanes(robot_id) if robot_id is not None else set()

        chosen = None
        if my_lane is not None:
            # its own lane, unless another robot took it meanwhile (it went charging, its lease lapsed)
            chosen = next(((p, n, lane) for p in targets for n, lane in [side_of(p)]
                           if lane == my_lane and lane not in leased), None)
        if chosen is None:
            chosen = next(((p, n, lane) for p in targets for n, lane in [side_of(p)] if lane not in leased), None)
        if chosen is None:
            logger.info(f'Pending plants remain, but every lane is taken by another robot (leased {sorted(leased)}).')
            return None
        plant, node_id, lane = chosen
        logger.debug(f'Next harvest target: plant {plant.object_id} (row {plant.row_id}) via node {node_id}, lane {lane}')
        return chosen

    def _next_harvest_target_legacy(self) -> tuple[str, int, tuple[float, float]] | None:
        """
        Pick the next unharvested bush side, following the sweep order (legacy farm schema).

        A bush is reached from two Pickup nodes, one on each side of the row,
        and each side is a separate unit of work because the manipulator
        reaches only half the bush from either side.

        The sweep order itself comes from the database. It runs one row at a
        time, down the near lane and back up the far lane, so a row is
        finished before the next one starts:

            b1_1 .. b1_38   then   b1_38 .. b1_1   then row 2, ...

        A side is done once any task row exists for its (asset name, node)
        pair, so the farm is harvested once and then stops.

        Returns:
            (asset name, approach node ID, (x, y)) or None when nothing is left.
        """
        sweep = self.db_ops.fetch_sweep()
        if not sweep:
            logger.error('No harvestable bushes found in the database.')
            return None

        for unit in sweep:
            if not self.db_ops.task_exists(unit.name, unit.node_id):
                logger.debug(f'Next harvest target: {unit.name} via node {unit.node_id}')
                return unit.name, unit.node_id, (unit.x, unit.y)

        return None

    def _resolve_route(
        self,
        current_node_id: int,
        target_node_id: int,
        context: str = '',
        robot_id: int | None = None,
        advance: float = 0.0,
    ) -> tuple[list[int], list[WayPoint]] | None:
        """
        The move to a target as waypoints, worked out before anything is written: the robot's pose, the lane exit
        / entry poses it needs (_lane_legs), then the target node's pose (pushed off the crop, _apply_clearance,
        then moved `advance` m along its stored heading: a harvest stop pose, _cutter_offset) with that heading,
        the approach the arm needs. The lane legs are decided on the final pose: the next plant's node can sit
        just behind a robot that stopped past the previous one by the same offset.

        The robot's executor replaces the first waypoint's position with its own and sends them all as one
        NavigateThroughPoses goal; Nav2 plans every leg on the map. A single waypoint would make its own position
        the goal. The first waypoint is the robot's reported pose, else current_node_id's.

        Returns:
            ([target node ID], waypoints) or None when the target node or a start pose is missing.
        """
        target = self.db_ops.fetch_map_node(target_node_id)
        if target is None:
            logger.warning(f'{context} node {target_node_id} is not in the node table.')
            return None

        start = self._robot_pose(robot_id)
        if start is None:
            node = self.db_ops.fetch_map_node(current_node_id)
            if node is None:
                logger.warning(f'No pose for robot {robot_id} and no node {current_node_id} to start a {context} move.')
                return None
            start = (float(node.x), float(node.y), float(node.theta or 0.0))

        theta = float(target.theta) if target.theta is not None else math.atan2(
            float(target.y) - start[1], float(target.x) - start[0])
        _, tx, ty, _ = self._apply_clearance([(int(target.id), float(target.x), float(target.y), theta)])[0]
        tx, ty = tx + advance * math.cos(theta), ty + advance * math.sin(theta)
        legs = self._lane_legs(start, (tx, ty, theta))
        if legs:
            logger.info(f'{context} move to node {target_node_id} via lane ends '
                        f'{[(round(x, 2), round(y, 2)) for x, y, _ in legs]}')
        return [int(target.id)], [
            WayPoint(node_id=int(current_node_id), x=start[0], y=start[1], theta=start[2]),
            *(WayPoint(node_id=-1, x=x, y=y, theta=h) for x, y, h in legs),
            WayPoint(node_id=int(target.id), x=tx, y=ty, theta=theta),
        ]

    def _cutter_offset(self, serial_number: str) -> float:
        """
        Metres a harvest stop pose lies ahead of the plant's Pickup node along its heading: harvest.cutter_offset_x
        for the robot's platform (its serial's prefix). The cutter's patch grid sits behind the robot's right
        side, so the robot stops with the plant beside its arm, not abeam.
        """
        platform = serial_number.lstrip('/').split('_')[0] if serial_number else ''
        return float(self.harvest.cutter_offset_x.get(platform, self.harvest.offset_default))

    def _lane_of(self, x: float, y: float) -> int | None:
        """
        Index of the lane between crop rows i and i + 1 that (x, y) is in, or None outside every lane. The pickup
        lines outside the outer rows count as lanes too, one row spacing wide (-1 north of the first row,
        len(rows) - 1 south of the last): turning round there sweeps the robot into the row just the same (an a300
        reaching b1_27 from the east turned at the goal and stuck with a corner in row 1, 2026-10-28).
        """
        rows = self.crop_rows
        if not rows or not min(r[0] for r in rows) <= x <= max(r[1] for r in rows):
            return None
        gap = rows[0][2] - rows[1][2] if len(rows) > 1 else 0.0
        bounds = [rows[0][2] + gap, *(r[2] for r in rows), rows[-1][2] - gap]
        return next((i - 1 for i in range(len(bounds) - 1) if bounds[i + 1] < y < bounds[i]), None)

    def _lane_legs(self, start: tuple[float, float, float], target: tuple[float, float, float]
                   ) -> list[tuple[float, float, float]]:
        """
        Lane exit / entry poses (x, y, heading) between start and target, so the robot never turns between rows:
        - in a lane, unless the target is ahead in the same lane and the robot already faces its way: an exit
          pose lane_end_margin past the rows' end the robot faces, heading straight out;
        - a target in a lane, unless reached straight ahead from there: an entry pose lane_end_margin before the
          rows' end its heading comes from, on the target's line, heading along the lane.
        Outside the lanes (headlands, beyond the outer pickup lines) Nav2 plans freely.
        """
        sx, sy, syaw = start
        tx, ty, th = target
        lane_s, lane_t = self._lane_of(sx, sy), self._lane_of(tx, ty)
        if lane_s is None and lane_t is None:
            return []
        x_min = min(r[0] for r in self.crop_rows) - self.harvest.lane_end_margin
        x_max = max(r[1] for r in self.crop_rows) + self.harvest.lane_end_margin
        straight_on = (lane_s == lane_t and math.cos(syaw - th) > 0.0
                       and (tx - sx) * math.cos(th) + (ty - sy) * math.sin(th) > 0.0)
        if straight_on:
            return []
        legs = []
        if lane_s is not None:
            east = math.cos(syaw) >= 0.0
            legs.append((x_max if east else x_min, sy, 0.0 if east else math.pi))
        if lane_t is not None:
            east = math.cos(th) >= 0.0
            legs.append((x_min if east else x_max, ty, 0.0 if east else math.pi))
        return legs

    def _robot_pose(self, robot_id: int | None) -> tuple[float, float, float] | None:
        """The robot's last reported pose (robot_live.topo_map_pos: x, y, yaw), or None."""
        if robot_id is None:
            return None
        try:
            pos = DataUtils.str_to_model(self.db_ops.fetch_robot_status(robot_id).topo_map_pos, TopologyMapPosition)
            return float(pos.x), float(pos.y), float(pos.w)
        except (ValueError, AttributeError, TypeError):
            return None

    def travel_distance(self, from_node_id: int, to_node_id: int) -> float | None:
        """
        Metres between two nodes as the robot would drive them, estimated without a route graph: |dx| + |dy|.
        The lanes run along x, so a trip changes lanes at a row end; the straight line would underestimate it.
        None when a node is missing.
        """
        a = self.db_ops.fetch_map_node(from_node_id)
        b = self.db_ops.fetch_map_node(to_node_id)
        if a is None or b is None:
            return None
        return abs(float(a.x) - float(b.x)) + abs(float(a.y) - float(b.y))

    def _build_harvest_task(
        self,
        task_id: int,
        robot_id: int,
        route: tuple[list[int], list[WayPoint]],
        job_schedule: str,
        crop_type: str,
        serial_number: str = '',
        plant: PlantTarget | None = None,
    ) -> Task:
        """
        Assemble the harvest Task message from an already-resolved route.

        Writes nothing to the database and cannot fail on pathfinding, because
        the route was validated by _resolve_route before the row was inserted.

        The route's last waypoint is already the stop pose (_cutter_offset). HARVESTING.data_str carries the
        plant as JSON.
        """
        node_id_list, waypoint_list = route

        harvest_data = json.dumps({
            'object_id': plant.object_id, 'row_id': plant.row_id, 'x': round(plant.x, 3), 'y': round(plant.y, 3),
            'attempt': plant.attempts + 1}) if plant else 'harvest data'

        logger.debug(f'Harvest waypoints: {[w.node_id for w in waypoint_list]}')

        move_sub_task = SubTask(
            sub_task_id=1,
            type=SubTask.MOVING,
            description=f'Move to node {node_id_list[-1]}',
            data=waypoint_list,
        )

        harvest_sub_task = SubTask(
            sub_task_id=2,
            type=SubTask.HARVESTING,
            description='Harvesting the bush',
            data_str=harvest_data,
        )

        task = Task(
            task_id=task_id,
            task_type=Task.HARVESTING_TASK,
            assigned_robot_id=robot_id,
            target_node_id=node_id_list[-1],
            job_schedule=job_schedule,
            description=f'Harvest Job {job_schedule}',
            crop_type=crop_type,
            sub_tasks=[move_sub_task, harvest_sub_task],
        )

        logger.debug(f'Harvest task built: {task}')
        return task

    def _generate_dock_task(
        self,
        robot_id: int,
        current_node_id: int,
        serial_number: str,
        dock_type: DockTypeEnum,
        task_type: TaskEnum,
    ) -> Task | None:
        """
        Reserve a dock, persist the task row, and build the Task message.

        Shared by charging and unloading, which differ only in which dock type
        they claim and which sub-task closes the sequence.

        The dock is chosen from the robot's own configured list rather than
        from every dock of that type, so one robot cannot take another's
        charger. Robots that list the same dock share it, and the reservation
        decides who gets it.
        """
        docks = Configuration().get_robot_docks(serial_number)
        dock_ids = docks.unloading if dock_type == DockTypeEnum.UNLOADING else docks.charging

        dock = self.db_ops.fetch_free_dock_from(dock_ids, dock_type.value)
        if dock is None:
            return None

        node = self.db_ops.fetch_map_node(dock.node_id)
        if node is None:
            logger.error(f"Dock '{dock.dock_id}' points at node {dock.node_id}, which is not in the node table.")
            return None

        label = 'charging' if task_type == TaskEnum.CHARGING_TASK else 'unloading'

        # Resolve the route BEFORE writing anything, so a path failure cannot
        # leave an open row behind that nothing will ever complete.
        route = self._resolve_route(current_node_id, dock.node_id, context=label, robot_id=robot_id)
        if route is None:
            return None

        task_id = self.db_ops.insert_task(
            node_id=dock.node_id,
            job_schedule=dock.dock_id,
            crop_type='',
            harvest_ready=_NOT_HARVEST,
            assigned_robot_id=robot_id,
            location=(node.x, node.y),
            task_type=task_type.value,
        )
        if task_id is None:
            return None

        task = self._build_dock_task(
            task_id=task_id,
            robot_id=robot_id,
            route=route,
            dock=dock,
            task_type=task_type,
        )

        self.db_ops.reserve_dock(dock.dock_id, robot_id)

        return task

    def _build_dock_task(
        self,
        task_id: int,
        robot_id: int,
        route: tuple[list[int], list[WayPoint]],
        dock: DockStation,
        task_type: TaskEnum,
    ) -> Task:
        """
        Assemble a charging or unloading Task message from a resolved route.

        Writes nothing to the database, so it is safe to call both when
        creating a task and when rehydrating one after a restart. Cannot fail
        on pathfinding - the route was validated by _resolve_route first.
        """
        is_charging = task_type == TaskEnum.CHARGING_TASK
        label = 'charging' if is_charging else 'unloading'

        node_id_list, waypoint_list = route
        # The docking server drives to its own staging pose (navigate_to_staging_pose, from its dock pose and
        # staging_x_offset), so the move only leaves the lane straight (_lane_legs) and never goes to the dock's
        # node. Outside a lane it is a no-op move to the robot's own pose: the sub-task stays, because
        # JobPublisher picks sub-tasks by status at fixed indices (move 0, dock 1, charge / unload 2).
        start, *legs, _ = waypoint_list
        waypoint_list = [start, *legs] if legs else [start, start]

        logger.debug(
            f"{label.capitalize()} dock: id='{dock.dock_id}' "
            f"type='{dock.dock_type}' "
            f'waypoints={[w.node_id for w in waypoint_list]}'
        )

        move_sub_task = SubTask(
            sub_task_id=1,
            type=SubTask.MOVING,
            description='Leave the lane' if legs else 'No move: dock from here',
            data=waypoint_list,
        )

        docking_sub_task = SubTask(
            sub_task_id=2,
            type=SubTask.DOCKING,
            description=f'Dock at {label} station',
            dock_goal=DockGoal(
                use_dock_id=True,
                dock_id=dock.dock_id,
                navigate_to_staging_pose=True,
            ),
        )

        final_sub_task = SubTask(
            sub_task_id=3,
            type=SubTask.CHARGING if is_charging else SubTask.UNLOADING,
            description=f'{"Charge" if is_charging else "Unload"} at {label} station',
            # the robot charges to this level (battery.full_threshold) instead of its own default
            data_str=json.dumps({'charge_to': self.charge_to}) if is_charging else '',
            undock_goal=UndockGoal(
                dock_type=dock.dock_type,
                max_undocking_time=_UNDOCK_TIMEOUT,
            ),
        )

        task = Task(
            task_id=task_id,
            task_type=(Task.CHARGING_TASK if is_charging else Task.UNLOADING_TASK),
            assigned_robot_id=robot_id,
            target_node_id=node_id_list[-1],
            job_schedule=dock.dock_id,
            description=f'{label.capitalize()} Job -> node {node_id_list[-1]}',
            crop_type='',
            sub_tasks=[move_sub_task, docking_sub_task, final_sub_task],
        )

        logger.debug(f'{label.capitalize()} task built: {task}')
        return task

    # =========================================================================
    # DOCK CONFIGURATION
    # =========================================================================

    def validate_dock_config(self) -> bool:
        """
        Check that every configured dock exists and sits on a real node.

        Switching farm.graph_source swaps which node table is live, and node
        IDs mean different places in each. A dock left pointing at the other
        schema's node fails quietly: fetch_map_node returns None, the dock task
        returns None, and the robot simply never gets sent to charge. That is
        indistinguishable from "no work available" in the logs, so it is
        checked once at start-up and reported loudly instead.

        Read-only. Problems are logged, not raised: a bad dock should not stop
        harvesting, which does not need one.

        Returns:
            True when every configured dock is usable.
        """
        source = Configuration().get_graph_source()
        docks = Configuration().get_docks()

        if not docks:
            logger.error(f"No docks configured for graph_source '{source}'. No robot can be sent to charge.")
            return False

        problems: list[str] = []
        checked: set[str] = set()

        for namespace, robot_docks in docks.items():
            dock_ids = list(robot_docks.charging) + list(robot_docks.unloading)

            if not robot_docks.charging:
                problems.append(f'{namespace}: no charging dock configured')

            for dock_id in dock_ids:
                if dock_id in checked:
                    continue
                checked.add(dock_id)

                dock = self.db_ops.fetch_dock_by_id(dock_id)
                if dock is None:
                    problems.append(f"'{dock_id}' is not in dock_station")
                    continue

                node = self.db_ops.fetch_map_node(dock.node_id)
                if node is None:
                    problems.append(
                        f"'{dock_id}' points at node {dock.node_id}, which does not exist "
                        f"under graph_source '{source}'"
                    )
                    continue

                # Existing is not enough. Regenerating the graph renumbers
                # every node, so a dock left on an old ID still resolves - to
                # whatever now happens to hold that number. Docks belong on
                # Staging nodes; anything else means the row is stale and the
                # robot will drive to the wrong place without complaint.
                node_type = getattr(node, 'node_type', None)
                if source != 'farm' and node_type != NodeTypeEnum.STAGING.value:
                    problems.append(
                        f"'{dock_id}' points at node {dock.node_id}, which is a "
                        f"'{node_type}' node, not '{NodeTypeEnum.STAGING.value}'. "
                        'The graph was probably regenerated and dock_station was not updated.'
                    )

        if problems:
            logger.error(
                f"Dock configuration is broken for graph_source '{source}':\n  " + '\n  '.join(problems)
            )
            return False

        logger.info(f"Dock configuration valid for graph_source '{source}': {sorted(checked)}")
        return True

    # =========================================================================
    # PRIVATE HELPERS
    # =========================================================================

    def _apply_clearance(
        self,
        poses: list[tuple[int, float, float, float]],
    ) -> list[tuple[int, float, float, float]]:
        """
        Push waypoints off the crop until the robot's footprint fits.

        The graph puts a Pickup node where the manipulator wants it, about
        0.45 m from the bush centre. The crop is drawn 0.45 m thick in the map
        and the robot is 0.698 m wide, so the footprint at that node overlaps
        the mapped obstacle - a goal every planner rejects. Each waypoint is
        moved directly away from its nearest object until it stands off by
        obstacle_radius + robot_half_width + safety_margin.

        The distance gained here is lost from the manipulator's reach, one for
        one. A waypoint needing more than max_shift is moved as far as the cap
        allows and the shortfall logged, because past that point the map or the
        graph is wrong and hiding it would only move the failure later.

        Args:
            poses: (node id, x, y, theta) before the shift.

        Returns:
            The same list with positions moved. Node IDs and headings are
            untouched.
        """
        if not self.clearance.enabled or not self.object_positions:
            return poses

        standoff = self.clearance.standoff
        reach = self.clearance.robot_half_length + self.clearance.obstacle_radius

        shifted: list[tuple[int, float, float, float]] = []
        moved = 0
        capped: list[tuple[int, float]] = []

        for nid, x, y, theta in poses:
            # Push across the pose's heading (along the lane), not away from the object
            # centre. Pushing away from the centre also drags the waypoint
            # along the row, which shortens the lane and moves it off the
            # bush it is meant to serve. Sideways is the only useful direction.
            heading = theta
            cos_h, sin_h = math.cos(heading), math.sin(heading)

            required = 0.0
            side = 1.0

            for ox, oy in self.object_positions:
                dx, dy = ox - x, oy - y

                along = dx * cos_h + dy * sin_h
                across = -dx * sin_h + dy * cos_h

                # Only objects level with the robot can touch it. The bush two
                # rows ahead is not a reason to move sideways.
                if abs(along) > reach:
                    continue

                gap = standoff - abs(across)
                if gap > required:
                    required = gap
                    # Move away from whichever side the object is on.
                    side = -1.0 if across > 0 else 1.0

            if required <= 0.0:
                shifted.append((nid, x, y, theta))
                continue

            applied = min(required, self.clearance.max_shift)

            # Unit vector across the direction of travel.
            px, py = -sin_h * side, cos_h * side
            shifted.append((nid, x + px * applied, y + py * applied, theta))
            moved += 1

            if applied < required:
                capped.append((nid, required - applied))

        if moved:
            logger.debug(f'Clearance: moved {moved} of {len(poses)} waypoints off the crop.')

        if capped:
            worst = max(capped, key=lambda c: c[1])
            logger.warning(
                f'Clearance capped at {self.clearance.max_shift:.3f} m on {len(capped)} waypoints; '
                f'node {worst[0]} is still {worst[1]:.3f} m short of standoff {standoff:.3f} m. '
                'Fix the map or the lane offset rather than raising the cap.'
            )

        return shifted
