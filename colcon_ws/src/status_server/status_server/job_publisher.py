import math
import os
import traceback
from functools import partial

import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from status_interfaces.msg import SubTask, Task

from status_server.configuration import Configuration
from status_server.data_utils import DataUtils
from status_server.dataclass import JobPublishers, RobotEnergy, RobotLiveStatus, TopologyMapPosition
from status_server.enum import OnlineFlagEnum, ProgressStatusEnum, RobotStatusEnum, TaskEnum
from status_server.postgres_manager import PostgresOperations
from status_server.task_generator import TaskGenerator

logger = RcutilsLogger(os.path.basename(__file__))

# How often to look for robots that have come online since start-up.
_DISCOVERY_PERIOD_SEC = 5.0
# Two navigation failures in a row within this distance (m): the robot is stuck, not the plant
_STUCK_RADIUS_M = 0.25


def stuck_again(last: tuple[float, float] | None, pose: tuple[float, float] | None) -> bool:
    """A navigation failure from (within _STUCK_RADIUS_M of) where the previous one left the robot."""
    return last is not None and pose is not None and math.dist(last, pose) < _STUCK_RADIUS_M


class JobPublisher(Node):
    def __init__(self):
        super().__init__('job_publisher')

        self.current_task: dict[str, Task | None] = {}
        self.previous_status: dict[str, int | None] = {}
        self.last_published_task_id: dict[str, int | None] = {}
        self.previous_online_flag: dict[str, int] = {}

        # Reverse lookup dictionary for RobotStatusEnum
        self.value_to_enum = {robotStatus.value: robotStatus for robotStatus in RobotStatusEnum}

        # Mapping Status Value to the sub-task id
        self.status_to_sub_task = {
            RobotStatusEnum.IDLE.value: 0,
            RobotStatusEnum.JOB_START.value: 0,
            # Navigation
            RobotStatusEnum.START_MOVING.value: 0,
            RobotStatusEnum.MOVING.value: 0,
            RobotStatusEnum.DESTINATION_REACHED.value: 1,
            # Harvesting
            RobotStatusEnum.START_HARVESTING.value: 1,
            RobotStatusEnum.HARVESTING.value: 1,
            RobotStatusEnum.DONE_HARVESTING.value: 2,
            # Docking
            RobotStatusEnum.START_DOCKING.value: 1,
            RobotStatusEnum.DOCKING.value: 1,
            RobotStatusEnum.DONE_DOCKING.value: 2,
            # Loading
            RobotStatusEnum.START_LOADING.value: -1,
            RobotStatusEnum.LOADING.value: -1,
            RobotStatusEnum.DONE_LOADING.value: -1,
            # Unloading
            RobotStatusEnum.START_UNLOADING.value: 2,
            RobotStatusEnum.UNLOADING.value: 2,
            RobotStatusEnum.DONE_UNLOADING.value: -1,
            # Charging
            RobotStatusEnum.START_CHARGING.value: 2,
            RobotStatusEnum.CHARGING.value: 2,
            RobotStatusEnum.DONE_CHARGING.value: -1,
            # ERROR: the robot reports a failed task (FAILED:<kind>:<task_id>); the task that replaces it is
            # published while it recovers, so a low-battery hold sees its charging task
            RobotStatusEnum.ERROR.value: 0,
            # Undocking - don't publish new tasks during undocking
            RobotStatusEnum.START_UNDOCKING.value: -1,
            RobotStatusEnum.UNDOCKING.value: -1,
            RobotStatusEnum.DONE_UNDOCKING.value: -1,
        }

        self.progress_status_map = {
            RobotStatusEnum.IDLE.value: 0,
            RobotStatusEnum.JOB_START.value: 1,
            RobotStatusEnum.PAUSED.value: 2,
            RobotStatusEnum.ERROR.value: 2,
            RobotStatusEnum.EMERGENCY_STOP.value: 2,
            RobotStatusEnum.OFFLINE.value: 2,
            RobotStatusEnum.JOB_DONE.value: 3,
        }

        # Statuses where we should NOT generate new tasks
        self.skip_new_task_statuses = [
            # RobotStatusEnum.START_UNDOCKING.value,
            # RobotStatusEnum.UNDOCKING.value,
            # RobotStatusEnum.DONE_UNDOCKING.value,
            RobotStatusEnum.START_MOVING.value,
            RobotStatusEnum.MOVING.value,
            RobotStatusEnum.DESTINATION_REACHED.value,
            RobotStatusEnum.START_DOCKING.value,
            RobotStatusEnum.DOCKING.value,
            RobotStatusEnum.START_CHARGING.value,
            RobotStatusEnum.CHARGING.value,
            RobotStatusEnum.DONE_CHARGING.value,
            RobotStatusEnum.START_HARVESTING.value,
            RobotStatusEnum.HARVESTING.value,
            RobotStatusEnum.DONE_HARVESTING.value,
            # RobotStatusEnum.START_LOADING.value,
            # RobotStatusEnum.LOADING.value,
            # RobotStatusEnum.DONE_LOADING.value,
            RobotStatusEnum.START_UNLOADING.value,
            RobotStatusEnum.UNLOADING.value,
            RobotStatusEnum.DONE_UNLOADING.value,
        ]

        self.db_ops = PostgresOperations()

        # Share this node's connection rather than letting TaskGenerator open
        # a second one - both run in the same process and the same thread.
        self.task_generator = TaskGenerator(db_ops=self.db_ops)

        self.battery_config = Configuration().get_battery_config()
        self.liveness_config = Configuration().get_liveness_config()

        # Minimum battery level (percent, 0-100) required to accept a new task; below it the robot charges first
        # (battery.low_threshold; reserve_threshold was its old name and is the fallback).
        self.battery_reserve = float(getattr(self.battery_config, 'low_threshold', None)
                                     or self.battery_config.reserve_threshold)
        # A harvest task still on its way (MOVING) is preempted by a charging task at or below this
        self.battery_critical = float(getattr(self.battery_config, 'critical_threshold', 15.0))
        # Failure reports already handled, per robot: the robot repeats FAILED:<kind>:<task_id> for a few seconds
        self.failed_task_handled: dict[str, int | None] = {}
        # Where the robot stood at its last navigation failure (cleared by any other outcome), to tell a stuck robot
        self.nav_failure_pose: dict[str, tuple[float, float]] = {}
        # When a robot went OFFLINE with a task in hand (node clock seconds), for the release grace
        self.offline_since: dict[str, float] = {}
        # Robots reporting ABNORMAL / e-stop: nothing is sent to them until they report ONLINE again
        self.faulted: set[str] = set()
        # Operator controls (robot_operation): False = not enrolled or paused, no new work after the current task
        self.robot_may_take_work: dict[str, bool] = {}
        # Robots whose harvest task the stopped operation cancelled: STOP is republished until they report IDLE / ERROR
        self.operation_stopping: set[str] = set()

        # Live energy picture per robot, rebuilt from the BMS feed. Held here
        # rather than in the database because this process is the one that
        # decides feasibility.
        self.robot_energy: dict[str, RobotEnergy] = {}
        # (task_id, node-clock s) since a robot has reported IDLE while this task is held for it
        self.idle_with_task_since: dict[str, tuple[int, float]] = {}

        self.get_logger().info(
            f'Battery reserve threshold: {self.battery_reserve:.1f}% | '
            f'safety factor: {self.battery_config.safety_factor} | '
            f'ewma alpha: {self.battery_config.ewma_alpha}'
        )

        # Node each robot returns to for charging, keyed by serial. Robots have
        # their own chargers, so a single shared value would cost every robot's
        # return trip against one other robot's dock. Resolved lazily and then
        # cached: the mapping is static, and the feasibility check runs on
        # every task assignment.
        self.charging_node_ids: dict[str, int | None] = {}

        self._init_publishers()

    def charging_node_for(self, serial_number: str) -> int | None:
        """
        Node this robot charges at, cached after the first lookup.

        Occupancy is ignored: this is used to cost the trip home, and who is
        currently parked there does not change how far away it is.

        Args:
            serial_number: Robot serial, also its namespace in the dock config.

        Returns:
            Node ID, or None when the robot has no usable charging dock.
        """
        if serial_number in self.charging_node_ids:
            return self.charging_node_ids[serial_number]

        node_id: int | None = None

        for dock_id in Configuration().get_robot_docks(serial_number).charging:
            dock = self.db_ops.fetch_dock_by_id(dock_id)
            if dock is not None:
                node_id = dock.node_id
                break

        if node_id is None:
            self.get_logger().error(
                f'[{serial_number}] No charging dock found in dock_station; battery estimates disabled.'
            )
        else:
            self.get_logger().info(f'[{serial_number}] Charging node: {node_id}')

        self.charging_node_ids[serial_number] = node_id
        return node_id

    def _init_publishers(self):
        """
        Set up per-robot publishers, and keep looking for robots that appear
        later.

        Discovery cannot be a one-off. A robot that has never published has no
        robot_live row, so starting the task manager before the robot leaves it
        with nothing to publish to - and previously that state was permanent
        until someone restarted the node. A repeating scan picks up robots as
        they come online.
        """
        self.publisher_dict: dict[str, JobPublishers] = {}

        # A dock still reserved now was left that way by a run that ended
        # without undocking. Nothing else would ever free it, and the robot
        # would be unable to charge or unload for good.
        self.db_ops.release_stale_dock_reservations()

        self.discover_robots()

        self.discovery_timer = self.create_timer(
            _DISCOVERY_PERIOD_SEC, self.discover_robots)

    def discover_robots(self) -> None:
        """
        Create publishers for robots that have come online since the last scan.

        Idempotent: robots that already have a publisher are left untouched,
        so their in-flight task and tracking state survive. Robots that go
        offline keep their publisher - the tick returns early on an offline
        status row, and tearing it down would discard the task in hand.
        """
        robot_details = self.db_ops.fetch_all_robot_details()

        if not robot_details:
            self.get_logger().error(
                'No robot details found in robot_info.', throttle_duration_sec=60.0)
            return

        self.robot_details = robot_details

        for robot_detail in robot_details:
            serial_number = robot_detail.serial_number

            # Already set up. Never rebuild: that would drop current_task.
            if serial_number in self.publisher_dict:
                continue

            robot_live_status = self.db_ops.fetch_robot_status(robot_detail.id)

            # Skip if no live status or robot is offline. robot_live.status is
            # a nullable text column, so it is coerced defensively.
            if (not robot_live_status
                    or self._as_int(robot_live_status.status,
                                    RobotStatusEnum.OFFLINE.value)
                    == RobotStatusEnum.OFFLINE.value):
                # Throttled: this repeats every scan for robots that are simply
                # not switched on.
                self.get_logger().warn(
                    f'Robot {serial_number} offline or no live status. Waiting for it to report in.',
                    throttle_duration_sec=60.0,
                )
                continue

            # publisher
            pub_topic = f'/{serial_number}/status/task'
            pub = self.create_publisher(Task, pub_topic, 10)

            # Battery telemetry. BEST_EFFORT to match the robot's own
            # publisher - a RELIABLE subscription here would receive
            # nothing at all, silently.
            self.robot_energy[serial_number] = RobotEnergy()
            self.create_subscription(
                BatteryState,
                f'/{serial_number}/platform/bms/state',
                partial(self.battery_callback, serial_number),
                qos_profile_sensor_data,
            )

            pub_timer = self.create_timer(1.0, partial(self.publish_job_callback, serial_number))

            self.publisher_dict[serial_number] = JobPublishers(
                robot_detail.id, serial_number, pub_topic, pub, pub_timer
            )

            # Initialize tracking dictionaries
            self.current_task[serial_number] = None
            self.previous_status[serial_number] = None
            self.last_published_task_id[serial_number] = None
            # Seed from DB so server startup does not falsely detect a client restart
            # when the robot was already online before the server started
            self.previous_online_flag[serial_number] = self._as_int(
                robot_live_status.online_flag, OnlineFlagEnum.ONLINE.value
            )

            # No task restoration needed here: the first tick hydrates
            # current_task from the robot's open row in the database.

            self.get_logger().info(f'[{serial_number}] Created publisher for topic: {pub.topic_name}')

    # ===================== ENERGY TRACKING ======================

    def _ewma(self, previous: float | None, sample: float) -> float:
        """Blend a new sample into a running average, seeding on first use."""
        if previous is None:
            return sample
        alpha = self.battery_config.ewma_alpha
        return alpha * sample + (1.0 - alpha) * previous

    def battery_callback(self, serial_number: str, msg: BatteryState) -> None:
        """
        Fold a BMS reading into the robot's running energy picture.

        Every sample updates the power average, so idle draw, sensors and
        manipulator work are all represented - not just the cost of driving.

        BatteryState fields are NaN when unmeasured, and `current` is negative
        while discharging, so readings are validated and the sign dropped.

        Args:
            serial_number: Robot the reading belongs to.
            msg:           Battery telemetry from the robot's BMS.
        """
        energy = self.robot_energy.get(serial_number)
        if energy is None:
            return

        voltage = float(msg.voltage)
        current = float(msg.current)
        charge = float(msg.charge)

        if math.isnan(voltage):
            return

        # Remaining energy: charge (Ah) x voltage (V) = watt-hours
        if not math.isnan(charge):
            energy.energy_wh = charge * voltage

        # Draw: volts x amps = watts, from discharge samples only (reported negative). A charging sample is the
        # charger's current, not the robot's draw: folded in, a fast charge read as a 2 kW draw and every harvest
        # task was "not feasible" at 99 % (2026-10-13).
        if not math.isnan(current) and current < 0.0:
            energy.avg_power_w = self._ewma(energy.avg_power_w, voltage * -current)

    def update_speed_estimate(self, serial_number: str, robot_live_status: RobotLiveStatus) -> None:
        """
        Update the robot's travel speed from successive reported positions.

        Only samples where the robot actually moved contribute. A stationary
        robot would otherwise pull the average towards zero, and the return-leg
        prediction divides by it.

        Args:
            serial_number:     Robot the position belongs to.
            robot_live_status: Current live status row, holding topo_map_pos.
        """
        energy = self.robot_energy.get(serial_number)
        if energy is None:
            return

        try:
            pos = DataUtils.str_to_model(robot_live_status.topo_map_pos, TopologyMapPosition)
        except (ValueError, AttributeError, TypeError):
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # All three are written together below, but check each: the invariant
        # is implicit, and a partial state would otherwise fault here.
        if energy.last_time is not None and energy.last_x is not None and energy.last_y is not None:
            elapsed = now - energy.last_time
            moved = math.hypot(pos.x - energy.last_x, pos.y - energy.last_y)

            if elapsed > 0.0 and moved >= self.battery_config.min_move_distance:
                energy.avg_speed_mps = self._ewma(energy.avg_speed_mps, moved / elapsed)

        energy.last_x = pos.x
        energy.last_y = pos.y
        energy.last_time = now

    def can_complete_task(self, serial_number: str, target_node_id: int, task_name: str) -> bool:
        """
        Decide whether the robot can finish a task and still reach a charger.

        Energy is predicted from measured draw and measured speed rather than
        an assumed consumption per metre:

            seconds = whole-task time + return leg distance / measured speed
            watt-hours = seconds x measured power / 3600

        The task's own navigation is already inside its configured duration,
        so only the trip back to the charger is derived from distance. That
        leg belongs to no task, which is why nothing is counted twice.

        Args:
            serial_number:  Robot under consideration.
            target_node_id: Node the task would send it to.
            task_name:      Key into battery.fixed_task_seconds.

        Returns:
            True if the task fits within the charge on board, or if there is
            not yet enough telemetry to judge.
        """
        energy = self.robot_energy.get(serial_number)
        readings = energy.readings() if energy is not None else None

        if readings is None:
            self.get_logger().debug(
                f'[{serial_number}] No battery telemetry yet; accepting {task_name} task.')
            return True

        energy_wh, avg_power_w, avg_speed_mps = readings

        charging_node_id = self.charging_node_for(serial_number)
        if charging_node_id is None:
            return True

        task_seconds = self.battery_config.fixed_task_seconds.get(task_name)
        if task_seconds is None:
            return True

        return_m = self.task_generator.travel_distance(target_node_id, charging_node_id)
        if return_m is None:
            self.get_logger().warning(
                f'[{serial_number}] Node {target_node_id} or the charger node {charging_node_id} is missing.')
            return False

        predicted_s = task_seconds + return_m / avg_speed_mps
        predicted_wh = predicted_s * avg_power_w / 3600.0
        required_wh = predicted_wh * self.battery_config.safety_factor

        feasible = energy_wh > required_wh

        if not feasible:
            self.get_logger().warning(
                f'[{serial_number}] {task_name} task to node {target_node_id} not feasible: '
                f'needs {required_wh:.1f}Wh, have {energy_wh:.1f}Wh '
                f'(predicted {predicted_s / 60.0:.1f}min at {avg_power_w:.0f}W, '
                f'return {return_m:.1f}m at {avg_speed_mps:.2f}m/s)'
            )

        return feasible

    # ==================== JOB/TASK PUBLISHER ====================

    @staticmethod
    def _as_int(value, default: int) -> int:
        """
        Coerce a robot_live column to int, falling back when it is unusable.

        robot_live.status is a nullable text column and online_flag is nullable
        integer, so a row written by anything other than robot_status_sync can
        hold NULL or an empty string. Raising on that would cost the robot
        every tick with nothing but a repeating traceback to show for it.

        Args:
            value:   Raw column value.
            default: Value to use when the column is NULL or not numeric.

        Returns:
            The coerced integer, or default.
        """
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def publish_job_callback(self, serial_number: str):
        try:
            robot = self.publisher_dict.get(serial_number)

            if robot is None:
                return

            # Current robot status from database
            robot_live_status = self.db_ops.fetch_robot_status(robot.id)

            if robot_live_status is None:
                return

            # A row with NULL status is unusable for dispatch, so treat it as
            # offline rather than letting int(None) abort the tick.
            live_status = self._as_int(robot_live_status.status, RobotStatusEnum.OFFLINE.value)
            current_online_flag = self._as_int(robot_live_status.online_flag, OnlineFlagEnum.OFFLINE.value)

            # ============================================================
            # Log status transitions for debugging
            # ============================================================
            prev_status = self.previous_status.get(serial_number)
            if prev_status != live_status:
                prev_status_name = self.value_to_enum[prev_status].name if prev_status in self.value_to_enum else 'None'
                current_status_name = self.value_to_enum[live_status].name
                self.get_logger().info(
                    f'[{serial_number}] Status transition: {prev_status_name} → {current_status_name}'
                )
                self.previous_status[serial_number] = live_status

            # Travel speed is measured from reported positions, so it has to
            # be sampled on every tick, not only when a task is assigned.
            self.update_speed_estimate(serial_number, robot_live_status)

            # ============================================================
            # Hydrate from database when in-memory task is missing.
            # Covers server restart at any robot status - the task already
            # exists as a row, only the message has to be rebuilt.
            # ============================================================
            self.hydrate_task(serial_number, robot, robot_live_status)

            # ============================================================
            # Update task progress immediately on status change
            # ============================================================
            current_task = self.current_task.get(serial_number)
            if (
                prev_status != live_status
                and isinstance(current_task, Task)
                and current_task.task_type == TaskEnum.HARVESTING_TASK.value
            ):
                self.update_task_progress_to_db(live_status, serial_number)

            op = self.db_ops.fetch_operation()

            # ============================================================
            # Harvesting operation: failures, faults, offline robots, critical battery
            # ============================================================
            if live_status == RobotStatusEnum.ERROR.value:
                if str(robot_live_status.task or '').startswith('FAILED:'):
                    self.handle_task_failure(serial_number, robot, robot_live_status, op)
                    current_task = self.current_task.get(serial_number)
            elif self.failed_task_handled.get(serial_number) is not None and live_status != RobotStatusEnum.ERROR.value:
                self.failed_task_handled[serial_number] = None

            if live_status == RobotStatusEnum.ABNORMAL.value or current_online_flag in (
                    OnlineFlagEnum.ABNORMAL.value, OnlineFlagEnum.EMERGENCY_STOP.value):
                if serial_number not in self.faulted:
                    self.faulted.add(serial_number)
                    self.get_logger().warning(f'[{serial_number}] faulted (status {live_status}, online_flag '
                                              f'{current_online_flag}): sending nothing until it is back')
                    self.db_ops.log_event('task_manager', 'faulted', f'status {live_status}, online_flag '
                                          f'{current_online_flag}', robot=serial_number)
                self.previous_online_flag[serial_number] = current_online_flag
                return
            self.faulted.discard(serial_number)

            if live_status == RobotStatusEnum.OFFLINE.value:
                self.handle_offline(serial_number, robot, current_task, op)
                self.previous_online_flag[serial_number] = current_online_flag
                return
            self.offline_since.pop(serial_number, None)

            # Operator controls (robot_operation: the /operation page's Robots card)
            enrolled, robot_state, request = self.db_ops.fetch_robot_operation(robot.id)
            if request == 'stop' or robot_state == 'stopped':
                if request == 'stop':
                    self.db_ops.clear_robot_request(robot.id)
                if isinstance(current_task, Task) or live_status not in (
                        RobotStatusEnum.IDLE.value, RobotStatusEnum.ERROR.value):
                    self.stop_robot(serial_number, robot, current_task, live_status)
                    current_task = None
                self.previous_online_flag[serial_number] = current_online_flag
                return
            if request == 'charge':
                self.db_ops.clear_robot_request(robot.id)
                if isinstance(current_task, Task) and current_task.task_type == TaskEnum.CHARGING_TASK.value:
                    self.get_logger().info(f'[{serial_number}] charge requested: already on a charging task')
                elif live_status in (RobotStatusEnum.IDLE.value, RobotStatusEnum.JOB_START.value,
                                     RobotStatusEnum.START_MOVING.value, RobotStatusEnum.MOVING.value,
                                     RobotStatusEnum.JOB_DONE.value, RobotStatusEnum.DESTINATION_REACHED.value):
                    self.db_ops.log_event('task_manager', 'charge', 'charging requested by the operator'
                                          + (f' (task {current_task.task_id} released)' if isinstance(current_task, Task) else ''),
                                          robot=serial_number)
                    self.handle_low_battery(serial_number, robot, robot_live_status, current_task)
                    current_task = self.current_task.get(serial_number)
                else:
                    # mid-cut / docking: finish that first, the request stays
                    self.db_ops.db.execute_write("UPDATE public.robot_operation SET request = 'charge' WHERE robot_id = %s",
                                                 params=(robot.id,))
            self.robot_may_take_work[serial_number] = enrolled and robot_state == 'active'

            # Operation stopped: no harvesting at all. A harvest task still held (assigned before Stop, or restored
            # from the database after a coordinator restart) is cancelled like the robot's own Stop; the robot then
            # parks at its charger (assign_next_task). Charging / unloading tasks run on. Pause finishes the task.
            if (op is not None and op.state == 'stopped' and isinstance(current_task, Task)
                    and current_task.task_type == TaskEnum.HARVESTING_TASK.value):
                self.get_logger().warning(f'[{serial_number}] operation stopped: cancelling harvest task '
                                          f'{current_task.task_id} ({current_task.description})')
                self.stop_robot(serial_number, robot, current_task, live_status, why='the operation is stopped')
                self.operation_stopping.add(serial_number)
                self.previous_online_flag[serial_number] = current_online_flag
                return
            if serial_number in self.operation_stopping:
                if live_status in (RobotStatusEnum.IDLE.value, RobotStatusEnum.ERROR.value) or isinstance(
                        current_task, Task):
                    self.operation_stopping.discard(serial_number)
                else:
                    # still on the cancelled task: republish STOP (a restarted executor, a lost message)
                    self.stop_robot(serial_number, robot, None, live_status)
                    self.previous_online_flag[serial_number] = current_online_flag
                    return

            if (isinstance(current_task, Task) and current_task.task_type == TaskEnum.HARVESTING_TASK.value
                    and live_status in (RobotStatusEnum.JOB_START.value, RobotStatusEnum.START_MOVING.value,
                                        RobotStatusEnum.MOVING.value)
                    and 0.0 < float(robot_live_status.battery_level) <= self.battery_critical):
                preempted = current_task.task_id
                if self.handle_low_battery(serial_number, robot, robot_live_status, current_task):
                    self.get_logger().warning(
                        f'[{serial_number}] Battery {robot_live_status.battery_level:.1f}% at or below '
                        f'{self.battery_critical:.0f}% on the way to a plant: charging first')
                    self.db_ops.log_event('task_manager', 'preempt',
                                          f'battery {robot_live_status.battery_level:.0f}%: charging task preempts '
                                          f'task {preempted}', robot=serial_number, job_id=preempted)
                current_task = self.current_task.get(serial_number)

            # ============================================================
            # CRITICAL: Skip task generation during certain statuses
            # ============================================================
            if live_status in self.skip_new_task_statuses:
                # Still publish the current task if it exists (for ongoing operations)
                if current_task and isinstance(current_task, Task):
                    sub_task_index = self.status_to_sub_task.get(live_status)
                    if (
                        sub_task_index is not None
                        and sub_task_index != -1
                        and isinstance(current_task.sub_tasks, list)
                        and sub_task_index < len(current_task.sub_tasks)
                    ):
                        sub_task = current_task.sub_tasks[sub_task_index]
                        self.publish_job(
                            robot.topic_publisher, sub_task, serial_number, float(robot_live_status.load_status)
                        )
                self.previous_online_flag[serial_number] = current_online_flag
                return

            # ============================================================
            # CLIENT RESTART HANDLING
            # ============================================================
            previous_flag = self.previous_online_flag.get(serial_number, 0)

            if previous_flag == 0 and current_online_flag == 1:
                self.client_restart_handle(
                    serial_number, robot, robot_live_status, current_task, float(robot_live_status.load_status)
                )

                # Update stored online_flag state
                self.previous_online_flag[serial_number] = current_online_flag
                return

            # ============================================================
            # Task Handler
            #
            # Tasks are only ever created at three points:
            #   ERROR + low battery - interrupt current task, go charge
            #   JOB_DONE            - close the finished task, take the next
            #   IDLE                - no task in hand, take one
            # Every other status just publishes whatever task is already held.
            # ============================================================
            if live_status == RobotStatusEnum.DONE_UNDOCKING.value:
                # Robot has physically left the dock - free it for others
                self.db_ops.release_dock_for_robot(robot.id)

            # Handle JOB_DONE - close the finished task and take the next one
            elif live_status == RobotStatusEnum.JOB_DONE.value:
                # Only act on entry to JOB_DONE, not on every repeated tick
                if prev_status != RobotStatusEnum.JOB_DONE.value:
                    self.complete_task(serial_number, current_task)
                    self.assign_next_task(serial_number, robot, robot_live_status)
                elif not isinstance(current_task, Task):
                    # nothing could be assigned at the entry tick (paused operation, lanes taken): keep trying
                    self.assign_next_task(serial_number, robot, robot_live_status)

            # Take a task when IDLE and holding none. Hydration already ran,
            # so an interrupted task would have been restored by this point.
            elif live_status == RobotStatusEnum.IDLE.value and not isinstance(current_task, Task):
                self.assign_next_task(serial_number, robot, robot_live_status)

            # IDLE while a task is held: normal for the first ticks after an assignment, a dead end past
            # liveness.idle_task_timeout (the robot ignores republishes of a task it already handled)
            elif live_status == RobotStatusEnum.IDLE.value:
                if self.release_idle_task(serial_number, robot, current_task):
                    current_task = None
                    self.assign_next_task(serial_number, robot, robot_live_status)

            if live_status != RobotStatusEnum.IDLE.value:
                self.idle_with_task_since.pop(serial_number, None)

            # ============================================================
            # Task Publishing
            # ============================================================
            current_task = self.current_task.get(serial_number)
            if current_task and isinstance(current_task, Task):
                sub_task_index = self.status_to_sub_task.get(live_status)

                # Skip if index is -1 (undocking states)
                if sub_task_index == -1 or sub_task_index is None:
                    self.previous_online_flag[serial_number] = current_online_flag
                    return

                if isinstance(current_task.sub_tasks, list) and sub_task_index < len(current_task.sub_tasks):
                    sub_task = current_task.sub_tasks[sub_task_index]

                    self.publish_job(
                        robot.topic_publisher, sub_task, serial_number, float(robot_live_status.load_status)
                    )

            # Always update online flag at end of every tick to track
            # offline → online transitions correctly
            self.previous_online_flag[serial_number] = current_online_flag

        except Exception as e:  # noqa: BLE001 - a failed tick must not kill the node
            # Deliberately broad. This runs in a timer callback, so anything
            # escaping here propagates out of the spin loop in main() and terminates the
            # process. A database blip should cost one tick, not the server.
            self.get_logger().error(f'[{serial_number}] Error: {e}\n Traceback: {traceback.format_exc()}')

    # ================== TASK LIFECYCLE HELPERS ==================

    def hydrate_task(self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus) -> None:
        """
        Rebuild the in-memory task from the database when it is missing.

        Tasks live in farm_harvesting_job, so a server restart loses only the
        Task message, never the assignment. Any row still open for this robot
        is rebuilt from the robot's current position. This replaces the old
        status-group restore logic, and works from any robot status.

        Does nothing when a task is already held, or when the robot has no
        open row.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
        """
        if isinstance(self.current_task.get(serial_number), Task):
            return

        open_job = self.db_ops.fetch_open_task(robot.id)
        if open_job is None:
            return

        task = self.task_generator.rebuild_task(
            open_job, robot_live_status.current_node_id, serial_number
        )

        if not isinstance(task, Task):
            self.get_logger().warning(
                f'[{serial_number}] Open task {open_job.id} could not be '
                f'rebuilt. Leaving robot without a task this tick.'
            )
            return

        self.current_task[serial_number] = task
        self.last_published_task_id[serial_number] = task.task_id

        self.get_logger().info(
            f"[{serial_number}] Task restored from database: '{task.description}' (Task ID: {task.task_id})"
        )

    def complete_task(self, serial_number: str, current_task: Task | None) -> None:
        """
        Close a finished task and clear the robot's in-memory state.

        All task types are marked complete, which moves the row out of
        fetch_open_task's range so hydration will not resurrect it.

        Any dock the task held is released here as well. DONE_UNDOCKING is the
        normal release point and fires first, so this is usually a no-op - but
        a dock task that ends without ever undocking would otherwise leave the
        dock reserved forever, and nothing else would ever free it.

        Args:
            serial_number: Robot serial, key into the tracking dicts.
            current_task:  Task the robot just finished, if one is held.
        """
        if isinstance(current_task, Task):
            self.get_logger().info(
                f'[{serial_number}] Job Done: Task ID: {current_task.task_id} | '
                f'Target Node: {current_task.target_node_id} | '
                f'Description: {current_task.description}'
            )

            try:
                self.db_ops.mark_job_complete(current_task.task_id, current_task.job_schedule)
                if current_task.task_type == TaskEnum.HARVESTING_TASK.value:
                    if self.db_ops.mark_plant_harvested(current_task.task_id):
                        plant = self.db_ops.fetch_plant_by_job(current_task.task_id)
                        self.db_ops.log_event(
                            'task_manager', 'harvested', f'{current_task.job_schedule} harvested',
                            robot=serial_number, object_id=plant.object_id if plant else None,
                            job_id=current_task.task_id)
                else:
                    self.db_ops.log_event('task_manager', 'charged' if current_task.task_type ==
                                          TaskEnum.CHARGING_TASK.value else 'unloaded',
                                          current_task.description, robot=serial_number, job_id=current_task.task_id)
            except Exception as e:  # noqa: BLE001 - never let a DB fault stall the tick
                self.get_logger().error(f'[{serial_number}] Failed to mark job complete: {e}')
            self.nav_failure_pose.pop(serial_number, None)

            if current_task.task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
                robot = self.publisher_dict.get(serial_number)
                if robot is not None:
                    self.db_ops.release_dock_for_robot(robot.id)

        self.current_task[serial_number] = None
        self.last_published_task_id[serial_number] = None

    def assign_next_task(self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus) -> None:
        """
        Create and assign the robot's next task.

        Falls back to a charging task when no work remains, so the robot parks
        at the charging station rather than sitting in the field.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
        """
        op = self.db_ops.fetch_operation()
        state = op.state if op else 'running'
        new_task = None

        if not self.robot_may_take_work.get(serial_number, True):
            self.get_logger().info(f'[{serial_number}] not enrolled / paused by the operator: no new task',
                                   throttle_duration_sec=60.0)
            return

        if state == 'running':
            new_task = self.fetch_task_for_robot(robot_live_status, robot)
            if new_task is None:
                self.check_operation_complete(op)
        elif 0.0 < robot_live_status.battery_level <= self.battery_reserve:
            # paused / stopped / complete: no harvesting, but a low robot still charges
            new_task = self.task_generator.generate_charging_task(
                robot.id, robot_live_status.current_node_id, serial_number)

        if new_task is None and state != 'paused':
            # no work (left, or the operation is stopped / complete): park at the charger instead of in the field
            if not self.db_ops.fetch_last_dock_node(robot.id) or state == 'running':
                new_task = self.task_generator.generate_charging_task(
                    robot.id, robot_live_status.current_node_id, serial_number
                )
                if isinstance(new_task, Task):
                    self.get_logger().info(f'[{serial_number}] No work remaining ({state}). Sending robot to charging station.')

        if not isinstance(new_task, Task):
            self.get_logger().info(f'[{serial_number}] No task to assign ({state}).', throttle_duration_sec=30.0)
            return

        self.current_task[serial_number] = new_task
        self.last_published_task_id[serial_number] = new_task.task_id

        self.get_logger().info(
            f'[{serial_number}] New Task: {new_task.description} | '
            f'Task ID: {new_task.task_id} | '
            f'Battery: {robot_live_status.battery_level:.2f}% | '
            f'Load: {robot_live_status.load_status:.2f}%'
        )
        self.db_ops.log_event(
            'task_manager', 'charge' if new_task.task_type == TaskEnum.CHARGING_TASK.value else 'assign',
            f'{new_task.description} (task {new_task.task_id}, battery {robot_live_status.battery_level:.0f}%)',
            robot=serial_number, job_id=new_task.task_id)

    def stop_robot(self, serial_number: str, robot: JobPublishers, current_task: Task | None, live_status: int,
                   why: str = 'stopped by the operator') -> None:
        """The operator's Stop: the robot's task is released (plant pending again, job closed as released, dock
        freed) and it is sent a STOP task (task_id -1, description 'STOP', no job_schedule): the executor cancels
        navigation / docking / the cut, stows the arm and goes IDLE. Published every tick while the robot is
        stopped (a restarted executor stays stopped); Start / Resume make it active again. Also used for a
        harvest task held while the operation is stopped (why = the event's reason)."""
        if isinstance(current_task, Task):
            if current_task.task_type == TaskEnum.HARVESTING_TASK.value:
                self.db_ops.release_plant(current_task.task_id)
            else:
                self.db_ops.release_dock_for_robot(robot.id)
            self.db_ops.close_task(current_task.task_id, current_task.job_schedule, ProgressStatusEnum.RELEASED.value)
            self.db_ops.log_event('task_manager', 'stop', f'{current_task.description} cancelled: {why}',
                                  robot=serial_number, job_id=current_task.task_id)
            self.current_task[serial_number] = None
            self.last_published_task_id[serial_number] = None
        if live_status not in (RobotStatusEnum.IDLE.value, RobotStatusEnum.ERROR.value):
            robot.topic_publisher.publish(Task(task_id=-1, task_type=Task.CHARGING_TASK, assigned_robot_id=robot.id,
                                               description='STOP', job_schedule='', sub_tasks=[]))

    def check_operation_complete(self, op) -> None:
        """running -> complete once no plant in the target rows is pending or assigned and no harvest task is open."""
        if op is None or op.state != 'running':
            return
        if self.db_ops.count_pending_plants(op.target_rows) == 0 and self.db_ops.count_open_harvest_jobs() == 0:
            if self.db_ops.set_operation_state('complete'):
                self.get_logger().info('Harvesting operation complete: every target plant is harvested or failed.')
                self.db_ops.log_event('task_manager', 'complete', 'every target plant harvested or failed')

    def handle_task_failure(self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus,
                            op) -> None:
        """
        The robot reports 'FAILED:<kind>:<task_id>:<detail>' (status ERROR): close that task, count the attempt on
        its plant (pending again, or failed after operation.max_attempts), free its dock, and give the robot a new
        task_id -- a charging task for a battery failure, else whatever assign_next_task decides. Handled once per
        task_id: the robot repeats the report for a few seconds.
        """
        parts = str(robot_live_status.task).split(':', 3)
        kind = parts[1] if len(parts) > 1 else 'other'
        try:
            task_id = int(parts[2]) if len(parts) > 2 else -1
        except ValueError:
            task_id = -1
        detail = parts[3] if len(parts) > 3 else ''
        if self.failed_task_handled.get(serial_number) == task_id:
            return
        self.failed_task_handled[serial_number] = task_id

        current_task = self.current_task.get(serial_number)
        self.get_logger().warning(f'[{serial_number}] task {task_id} failed ({kind}): {detail}')
        if isinstance(current_task, Task) and kind == 'stopped':
            self.stop_robot(serial_number, robot, current_task, RobotStatusEnum.IDLE.value)
            return
        stuck = False
        if kind == 'nav':
            pose = self.task_generator._robot_pose(robot.id)
            stuck = stuck_again(self.nav_failure_pose.get(serial_number), pose and pose[:2])
            if pose is not None:
                self.nav_failure_pose[serial_number] = pose[:2]
        else:
            self.nav_failure_pose.pop(serial_number, None)
        if stuck:
            # Every plant would fail from here (j100_0921 in the crop margin, 2026-10-21: "Start occupied", 16 plants
            # failed in an hour): pause the robot like the Robots card's Pause until an operator frees it.
            self.db_ops.db.execute_write("UPDATE public.robot_operation SET state = 'paused', updated_at = now() "
                                         "WHERE robot_id = %s", params=(robot.id,))
            self.robot_may_take_work[serial_number] = False
            self.get_logger().error(f'[{serial_number}] navigation failed twice from the same spot: robot paused')
            self.db_ops.log_event('task_manager', 'pause', 'paused: navigation failed twice from the same spot '
                                  f'({pose[0]:.2f}, {pose[1]:.2f}); free the robot, then Resume', robot=serial_number)
        if isinstance(current_task, Task) and (task_id in (-1, current_task.task_id)):
            max_attempts = op.max_attempts if op else 2
            if current_task.task_type == TaskEnum.HARVESTING_TASK.value and (kind == 'battery' or stuck):
                # the battery or a stuck robot, not the plant: back to pending without an attempt, like the
                # coordinator's preempt
                self.db_ops.release_plant(current_task.task_id)
                self.db_ops.log_event('task_manager', 'released',
                                      f'{current_task.job_schedule}: {kind} {detail}'.strip(),
                                      robot=serial_number, job_id=current_task.task_id)
            elif current_task.task_type == TaskEnum.HARVESTING_TASK.value:
                plant = self.db_ops.fetch_plant_by_job(current_task.task_id)
                new_status = self.db_ops.fail_plant_attempt(current_task.task_id, f'{kind}: {detail}', max_attempts)
                self.db_ops.log_event(
                    'task_manager', 'plant_failed' if new_status == 'failed' else 'attempt_failed',
                    f'{current_task.job_schedule}: {kind} {detail}'.strip() + (
                        f' (plant failed after {max_attempts} attempts)' if new_status == 'failed' else ''),
                    robot=serial_number, object_id=plant.object_id if plant else None, job_id=current_task.task_id)
            else:
                self.db_ops.release_dock_for_robot(robot.id)
                self.db_ops.log_event('task_manager', 'attempt_failed',
                                      f'{current_task.description}: {kind} {detail}'.strip(),
                                      robot=serial_number, job_id=current_task.task_id)
            self.db_ops.close_task(current_task.task_id, current_task.job_schedule, ProgressStatusEnum.FAILED.value)
            self.current_task[serial_number] = None
            self.last_published_task_id[serial_number] = None
        elif isinstance(current_task, Task):
            self.get_logger().info(f'[{serial_number}] the failed task {task_id} is not the one held '
                                   f'({current_task.task_id}); keeping it')
            return

        if kind == 'stopped':
            return
        if kind == 'battery':
            charging_task = self.task_generator.generate_charging_task(
                robot.id, robot_live_status.current_node_id, serial_number)
            if isinstance(charging_task, Task):
                self.current_task[serial_number] = charging_task
                self.last_published_task_id[serial_number] = charging_task.task_id
                self.get_logger().info(f'[{serial_number}] Charging task {charging_task.task_id} after the '
                                       f'battery failure')
                self.db_ops.log_event('task_manager', 'charge', f'{charging_task.description} (task '
                                      f'{charging_task.task_id}) after FAILED:battery', robot=serial_number,
                                      job_id=charging_task.task_id)
            else:
                self.get_logger().error(f'[{serial_number}] battery failure but no charging task could be created')
        else:
            self.assign_next_task(serial_number, robot, robot_live_status)

    def release_idle_task(self, serial_number: str, robot: JobPublishers, current_task: Task) -> bool:
        """A robot reporting IDLE while a task is held for it: after liveness.idle_task_timeout the task is released
        (plant back to pending, dock freed, job closed as released) so the robot gets a new one. The server
        sees a one-tick JOB_DONE only by luck when both loops run at 1 Hz; the robot then drops to IDLE and
        ignores every republish of that task_id. Returns True once released."""
        now = self.get_clock().now().nanoseconds / 1e9
        task_id, since = self.idle_with_task_since.get(serial_number, (None, None))
        if task_id != current_task.task_id:
            self.idle_with_task_since[serial_number] = (current_task.task_id, now)
            return False
        if now - since < self.liveness_config.idle_task_timeout:
            return False
        self.get_logger().warning(f'[{serial_number}] IDLE for {now - since:.0f} s while task '
                                  f'{current_task.task_id} ({current_task.description}) is held for it: releasing it')
        if current_task.task_type == TaskEnum.HARVESTING_TASK.value:
            self.db_ops.release_plant(current_task.task_id)
        elif current_task.task_type == TaskEnum.CHARGING_TASK.value:
            self.db_ops.release_dock_for_robot(robot.id)
        self.db_ops.close_task(current_task.task_id, current_task.job_schedule, ProgressStatusEnum.RELEASED.value)
        self.db_ops.log_event('task_manager', 'released',
                              f'{current_task.description} released: robot idle {now - since:.0f} s with it',
                              robot=serial_number, job_id=current_task.task_id)
        self.current_task[serial_number] = None
        self.last_published_task_id[serial_number] = None
        self.idle_with_task_since.pop(serial_number, None)
        return True

    def handle_offline(self, serial_number: str, robot: JobPublishers, current_task: Task | None, op) -> None:
        """A robot the liveness watchdog marked OFFLINE: after operation.offline_grace_s its task is released
        (plant back to pending, dock freed, job closed as released) so another robot can take the plant. Until
        then the task is kept: the robot may just have lost the base station while still working."""
        if not isinstance(current_task, Task):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        since = self.offline_since.setdefault(serial_number, now)
        grace = float(op.offline_grace_s) if op else 180.0
        if now - since < grace:
            return
        self.get_logger().warning(f'[{serial_number}] offline for {now - since:.0f} s: releasing task '
                                  f'{current_task.task_id} ({current_task.description})')
        if current_task.task_type == TaskEnum.HARVESTING_TASK.value:
            self.db_ops.release_plant(current_task.task_id)
        else:
            self.db_ops.release_dock_for_robot(robot.id)
        self.db_ops.close_task(current_task.task_id, current_task.job_schedule, ProgressStatusEnum.RELEASED.value)
        self.db_ops.log_event('task_manager', 'released',
                              f'{current_task.description} released: robot offline {now - since:.0f} s',
                              robot=serial_number, job_id=current_task.task_id)
        self.current_task[serial_number] = None
        self.last_published_task_id[serial_number] = None
        self.offline_since.pop(serial_number, None)
        # back online still on that task: STOP until it is IDLE / ERROR, its plant may be someone else's by then
        self.operation_stopping.add(serial_number)

    def handle_low_battery(
        self, serial_number: str, robot: JobPublishers, robot_live_status: RobotLiveStatus, current_task: Task | None
    ) -> bool:
        """
        Interrupt the current task and send the robot to charge.

        Skips when the robot is already on a charging task, which keeps this
        idempotent across repeated ERROR ticks without needing to track the
        status transition.

        The interrupted task is closed as released (a harvest task's plant back to pending, no attempt counted),
        but only once the charging task exists: without a free charger the robot keeps its task, else it would
        work a plant the coordinator had already handed back.

        Args:
            serial_number:     Robot serial, key into the tracking dicts.
            robot:             Publisher record for this robot.
            robot_live_status: Current live status row.
            current_task:      Task being interrupted, if one is held.

        Returns:
            True when the robot now holds a charging task.
        """
        if isinstance(current_task, Task) and current_task.task_type == TaskEnum.CHARGING_TASK.value:
            return True

        # An unloading task's dock goes first: with no unloading dock configured it is the robot's own charger,
        # which the charging task needs (it is not given back if no charging task can be made either)
        if isinstance(current_task, Task) and current_task.task_type == TaskEnum.UNLOADING_TASK.value:
            self.db_ops.release_dock_for_robot(robot.id)

        charging_task = self.task_generator.generate_charging_task(
            robot.id, robot_live_status.current_node_id, serial_number
        )

        if not isinstance(charging_task, Task):
            self.get_logger().error(f'[{serial_number}] Low battery but no charging task could be created: '
                                    'keeping the current task', throttle_duration_sec=30.0)
            return False

        if isinstance(current_task, Task):
            if current_task.task_type == TaskEnum.HARVESTING_TASK.value:
                self.db_ops.release_plant(current_task.task_id)
            self.db_ops.close_task(current_task.task_id, current_task.job_schedule, ProgressStatusEnum.RELEASED.value)
            self.get_logger().warning(
                f'[{serial_number}] Battery {robot_live_status.battery_level:.2f}% '
                f'at or below reserve {self.battery_reserve:.1f}%. '
                f'Released task {current_task.task_id} '
                f"'{current_task.description}'."
            )

        self.current_task[serial_number] = charging_task
        self.last_published_task_id[serial_number] = charging_task.task_id

        self.get_logger().info(
            f'[{serial_number}] New Charging Task: {charging_task.description} | '
            f'Task ID: {charging_task.task_id} | '
            f'Battery: {robot_live_status.battery_level:.2f}%'
        )
        self.db_ops.log_event('task_manager', 'charge',
                              f'{charging_task.description} (task {charging_task.task_id}, battery '
                              f'{robot_live_status.battery_level:.0f}%)', robot=serial_number,
                              job_id=charging_task.task_id)
        return True

    def client_restart_handle(self, serial_number, robot, robot_live_status, current_task, crop_load: float):
        """Handles the event when a client robot node restarts and comes back online."""
        self.get_logger().info(f'[{serial_number}] Client robot node has restarted. Re-publishing current task...')

        if (
            isinstance(current_task, Task)
            and isinstance(current_task.sub_tasks, list)
            and len(current_task.sub_tasks) > 0
        ):
            # Re-publish first sub-task to sync client
            sub_task = current_task.sub_tasks[0]
            self.publish_job(robot.topic_publisher, sub_task, serial_number, crop_load)
            self.get_logger().info(
                f"[{serial_number}] Re-published task '{current_task.description}' "
                f'(Task ID: {current_task.task_id}) after client restart.'
            )
        # no task held: the next tick's assign_next_task decides, with the pause / enrolment gates

    def publish_job(self, publisher: Publisher, sub_task: SubTask, serial_number: str, crop_load: float):
        """
        Publish a job to a specified publisher.
        """

        self.get_logger().debug(f'Current Crop load: {crop_load:.2f} %')
        current_task = self.current_task.get(serial_number)

        if not isinstance(current_task, Task):
            return

        st = SubTask(
            sub_task_id=sub_task.sub_task_id,
            type=sub_task.type,
            description=sub_task.description,
            data=sub_task.data,
            data_str=sub_task.data_str,
            dock_goal=sub_task.dock_goal,
            undock_goal=sub_task.undock_goal,
        )

        task_msg = Task(
            task_id=current_task.task_id,
            task_type=current_task.task_type,
            assigned_robot_id=current_task.assigned_robot_id,
            target_node_id=current_task.target_node_id,
            job_schedule=current_task.job_schedule,
            description=current_task.description,
            crop_type=current_task.crop_type,
            crop_load=crop_load,
            sub_tasks=[st],
        )

        publisher.publish(task_msg)

    def fetch_task_for_robot(self, robot_status: RobotLiveStatus, robot_detail: JobPublishers) -> Task | None:
        """
        Decide and create the appropriate next task for a robot.

        The battery check runs BEFORE any task is generated. Generators now
        write a row to the database, so generating a task and then discarding
        it on a failed battery check would leave an orphaned open row that
        hydration would later resurrect.

        Order of preference:
            battery at or below reserve -> charging task
            crop load full              -> unloading task
            otherwise                   -> harvest task

        Both gates run before generation. Generators write a row, so a task
        rejected afterwards would leave an orphan that hydration resurrects.

        Returns:
            Task: Generated task
            None: No tasks available
        """
        serial = robot_detail.serial_number
        start_node = robot_status.current_node_id
        self.get_logger().debug(f'[{serial}] Current load: {robot_status.load_status}')

        # Reported percentage gate - nothing is worth starting below reserve. 0 is "not known yet": the executor
        # reports 0 % until its first BatteryState (right after a start), which sent restarted robots to charge.
        if 0.0 < robot_status.battery_level <= self.battery_reserve:
            self.get_logger().info(
                f'[{serial}] Battery '
                f'{robot_status.battery_level:.2f}% at or below reserve '
                f'{self.battery_reserve:.1f}%. Charging before new work.'
            )
            return self.task_generator.generate_charging_task(robot_detail.id, start_node, serial)

        if robot_status.load_status >= 100:
            task_type, task_name = TaskEnum.UNLOADING_TASK, 'unloading'
        else:
            task_type, task_name = TaskEnum.HARVESTING_TASK, 'harvesting'

        # Measured-energy gate. Asks where the task would go before creating
        # it, so an unaffordable task is never written.
        target_node_id = self.task_generator.next_target_node(task_type, serial, robot_detail.id, start_node)
        if target_node_id is None:
            return None

        if not self.can_complete_task(serial, target_node_id, task_name):
            self.get_logger().info(f'[{serial}] Charging instead of the {task_name} task it cannot finish.')
            return self.task_generator.generate_charging_task(robot_detail.id, start_node, serial)

        if task_type == TaskEnum.UNLOADING_TASK:
            return self.task_generator.generate_unloading_task(robot_detail.id, start_node, serial)

        return self.task_generator.generate_harvest_task(robot_detail.id, start_node, serial)

    # ================= DATABASE UPDATE METHODS ==================

    def update_task_progress_to_db(self, robot_status: int, serial_number: str):
        """Update task progress in database."""
        progress_status = self.progress_status_map.get(robot_status)
        current_task = self.current_task.get(serial_number)

        if progress_status is not None and isinstance(current_task, Task):
            self.db_ops.update_task_progress(current_task.task_id, current_task.job_schedule, progress_status)

    # ====================== HELPER METHODS ======================

    # ========================= NODE LIFECYCLE =========================

    def destroy_node(self):
        """
        Close the database connection before destroying the node.

        One close suffices: TaskGenerator shares this node's connection rather
        than holding its own.
        """
        self.db_ops.close()
        super().destroy_node()


def main(args=None) -> None:
    """
    Entry point for the Task Manager node.

    Construction happens inside the try block so a failure in __init__ - a bad
    config or an unreachable database - still reaches the cleanup path instead
    of aborting the process with an open connection.

    Teardown lives in finally because spinning raises on Ctrl-C: anything
    placed after it would never run, which previously leaked the database
    connections on every interrupt.
    """
    node = None
    executor = None
    # Not rclpy.spin(): under use_sim_time it never returns after SIGTERM/SIGINT (the signal shuts the context down,
    # the timers run on the ROS clock and /clock stops with the context, so nothing wakes the executor's wait).
    # spin_once with a wall-time timeout re-checks rclpy.ok() at least every 0.5 s.
    try:
        rclpy.init(args=args)
        node = JobPublisher()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info('Task Manager interrupted by user. JobPublisher Node has been shutdown.')
    except Exception as e:  # noqa: BLE001 - top-level guard so shutdown still runs
        logger.error(f'Runtime error: {e}\n Traceback: {traceback.format_exc()}')
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        # try_shutdown, not `if rclpy.ok(): rclpy.shutdown()`: after SIGTERM/SIGINT rclpy's own handler shuts the
        # context down concurrently and the second shutdown raised (exit code 1). Guarded for a failed init().
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
