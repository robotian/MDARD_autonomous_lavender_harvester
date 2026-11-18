"""
This module defines various data classes used in the Robot Status Monitor system.
Classes:
    Point: Represents a geographical point with latitude and longitude.
    TopologyMapPosition: Represents a node in a topological map with coordinates and orientation.
    Time: Represents a time with seconds and nanoseconds.
    DatabaseDetails: Represents details required to connect to a database.
    Config: Represents configuration details, including database details.
    RobotDetails: Represents details about a robot.
    JobDetails: Represents details about a job assigned to a robot.
    RobotLive: Represents live status information of a robot.
    WayPoint: Represents a waypoint in a route.
    SubTasks: Represents a sub-task with a description and data.
    Task: Represents a task assigned to a robot with sub-tasks.
    RobotLiveStatus: Represents live status information of a robot in string format.
    JobPublisher: Represents a job publisher with a timer.
"""

from dataclasses import dataclass, field
from typing import Any

from rclpy.publisher import Publisher
from rclpy.timer import Timer


@dataclass
class DockStation:
    """
    Represents a row of the `dock_station` table.

    The physical dock pose is owned by the robot's Nav2 docking server and is
    addressed by `dock_id`, so no pose is stored here.

    Attributes:
        id (int): Primary key of the dock station.
        dock_id (str): Dock name registered with the Nav2 docking server.
        type (int): Purpose of the dock, see DockTypeEnum.
        status (int): Occupancy state, see ChargingStationStatusEnum.
        robot_id (int | None): Robot currently holding the dock, if any.
        node_id (int): Farm node the robot must reach before docking.
        dock_type (str): Nav2 dock plugin type, e.g. 'simple_charging_dock'.
    """

    id: int
    dock_id: str
    type: int
    status: int
    robot_id: int | None
    node_id: int
    dock_type: str


@dataclass
class Point:
    """
    Represents a geographical point with latitude and longitude.
    Attributes:
        lat (float): The latitude of the point.
        lon (float): The longitude of the point.
    """

    lat: float
    lon: float


@dataclass
class TopologyMapPosition:
    """
    Represents a position in a topology map with six degrees of freedom.
    Attributes:
        x (float): The x-coordinate of the position.
        y (float): The y-coordinate of the position.
        z (float): The z-coordinate of the position.
        r (float): The roll angle of the position in radians.
        p (float): The pitch angle of the position in radians.
        w (float): The yaw angle of the position in radians.
    """

    x: float
    y: float
    z: float
    r: float
    p: float
    w: float


@dataclass
class Time:
    """
    A class representing time with seconds and nanoseconds.
    Attributes:
        sec (int): The number of seconds.
        nanosec (int): The number of nanoseconds.
    """

    sec: int
    nanosec: int


@dataclass
class FarmAssets:
    """
    A data class representing farm assets.
    Attributes:
        id (int): The unique identifier for the farm asset.
        name (str): The name of the farm asset.
        x_map (float): The x-coordinate of the asset on the map.
        y_map (float): The y-coordinate of the asset on the map.
        conn_nodes (list[int]): A list of connected node identifiers.
    """
    id: int
    name: str
    x_map: float
    y_map: float
    conn_nodes: list[int]


@dataclass
class FarmNodes:
    """Class representing a farm edge."""
    id: int
    zone_prefix: str
    x: float
    y: float
    type: float
    theta: float


@dataclass
class ClearanceConfig:
    """
    How far waypoints are pushed off the crop before they are published.

    The graph places a Pickup node at the distance the manipulator wants,
    which is closer to the bush than the robot's body can legally sit: Nav2
    marks a band of the robot's own inscribed radius around every mapped
    obstacle as untraversable, and refuses any pose whose footprint touches
    it. Publishing the node position unchanged gives the planner a goal it
    will always reject.

    Shifting the waypoint away from the bush buys that clearance back, and
    costs exactly the same distance in manipulator reach. max_shift is the
    cap: past it the geometry is wrong and should be fixed in the map or the
    graph rather than papered over here.

    Attributes:
        enabled (bool): Apply the shift at all.
        robot_half_width (float): Half the footprint width, which is the
            inscribed radius Nav2 inflates by. Sets how far sideways an object
            must be.
        robot_half_length (float): Half the footprint length. Sets how far
            ahead or behind an object still counts as level with the robot -
            a long robot is threatened by a bush its centre has not reached.
        obstacle_radius (float): How far the mapped obstacle extends from the
            object centre stored in object_data.
        safety_margin (float): Extra metres on top of the two radii.
        max_shift (float): Largest shift allowed for one waypoint. A waypoint
            needing more is published short of the requirement and logged.
    """

    enabled: bool
    robot_half_width: float
    robot_half_length: float
    obstacle_radius: float
    safety_margin: float
    max_shift: float

    @property
    def standoff(self) -> float:
        """Distance from an object centre at which the robot is legal."""
        return self.obstacle_radius + self.robot_half_width + self.safety_margin


@dataclass
class GraphNode:
    """
    Represents a row of the `graph_node` table.

    Replaces FarmNodes. There is no zone_prefix: `id` is the sole primary key,
    so a node ID identifies exactly one row.

    Attributes:
        id (int): Primary key, and the ID carried on WayPoint messages.
        node_type (str): Role of the node, see NodeTypeEnum.
        x (float): X-coordinate in the map frame.
        y (float): Y-coordinate in the map frame.
        theta (float): Approach heading in radians. Never NULL in this graph;
            on a Pickup node it also selects which side of the bush the
            manipulator can reach.
        obj_id (int | None): Object this node serves, NULL for nodes that are
            not Pickup nodes.
    """
    id: int
    node_type: str
    x: float
    y: float
    theta: float
    obj_id: int | None


@dataclass
class HarvestUnit:
    """
    One side of one bush - the smallest unit of harvesting work.

    A bush is harvested in two passes because the manipulator reaches only
    half of it from either side, so each object yields two HarvestUnits.
    fetch_harvest_units returns them already in sweep order.

    Attributes:
        name (str): Asset name, 'b{row_id}_{object_id}'. Written to
            farm_harvesting_job.job_schedule and used with node_id to decide
            whether this side is already done.
        node_id (int): Pickup node the robot approaches from.
        row_id (int): Row the bush sits in.
        x (float): X-coordinate of the bush.
        y (float): Y-coordinate of the bush.
    """
    name: str
    node_id: int
    row_id: int
    x: float
    y: float


@dataclass
class RobotDocks:
    """
    Dock IDs one robot is allowed to use, read from the `docks` config block.

    dock_id is globally unique and is both the `dock_station` key and the name
    registered with that robot's Nav2 docking server. Two robots may list the
    same dock; the dock_station reservation decides who gets it.

    Attributes:
        charging (list[str]): Dock IDs usable for charging.
        unloading (list[str]): Dock IDs usable for unloading. Empty until an
            unloading dock exists.
    """
    charging: list[str]
    unloading: list[str]


@dataclass
class JobPublishers:
    """
    Represents the publishers and subscribers associated with a robot job.
    Attributes:
        id (int): Unique identifier for the robot job publisher.
        topic_name (str): Name of the topic associated with the publisher.
        topic_publisher (Publisher): Publisher object for the topic.
        topic_subscriber (Subscription): Subscriber object for the topic.
    """

    id: int
    serial_number: str
    topic_name: str
    topic_publisher: Publisher
    topic_timer: Timer


@dataclass
class DatabaseDetails:
    """
    Class to store database connection details.
    Attributes:
        host (str): The hostname or IP address of the database server.
        port (int): The port number on which the database server is listening.
        dbname (str): The name of the database to connect to.
        user (str): The username for authenticating with the database.
        password (str): The password for authenticating with the database.
        connect_timeout (int): Seconds to wait for a connection before giving
            up. Without it psycopg uses the OS TCP timeout, which can block a
            node for minutes when the host is unreachable.
    """

    host: str
    port: int
    dbname: str
    user: str
    password: str
    connect_timeout: int


@dataclass
class NamespaceConfig:
    """
    NamespaceConfig holds configuration for namespace selection modes.
    Attributes:
        mode (str): Specifies the namespace selection mode.
            Expected values are 'single' or 'multi', as provided in the YAML configuration.
        namespace (str): The namespace to use when mode is set to 'single'.
        namespaces (list[str]): A list of namespaces to use when mode is set to 'multi'.
    """

    mode: str
    namespace: str
    namespaces: list[str]


@dataclass
class BatteryConfig:
    """
    Settings for deciding whether a robot can take on and survive a task.

    Attributes:
        reserve_threshold (float): Minimum battery level, in percent (0-100),
            required before a new task is assigned. Robots at or below this
            level are sent to the charging station instead.
        ewma_alpha (float): Smoothing factor for the running power and speed
            estimates. Higher reacts faster and is noisier.
        safety_factor (float): Margin applied to predicted energy before
            comparing it against the charge actually on board.
        min_move_distance (float): Metres of movement required before a sample
            counts towards the speed estimate.
        fixed_task_seconds (dict[str, float]): Whole-task durations keyed by
            task name, including each task's own navigation. Charging is
            absent by design - it is the fallback task and is never vetoed.
    """

    reserve_threshold: float
    ewma_alpha: float
    safety_factor: float
    min_move_distance: float
    fixed_task_seconds: dict[str, float]
    # Harvesting operation: a robot at or below low_threshold is sent to charge before its next task; a harvest
    # task still on its way (MOVING) is preempted at critical_threshold; a charging task charges to full_threshold.
    low_threshold: float = 30.0
    critical_threshold: float = 15.0
    full_threshold: float = 92.0


@dataclass
class RobotEnergy:
    """
    Running energy picture for one robot, rebuilt from the live BMS feed.

    Held in memory by the task manager rather than the database: it is the
    process that decides feasibility, and the estimates are cheap to relearn
    after a restart.

    Attributes:
        energy_wh (float | None): Charge remaining, charge (Ah) x voltage (V).
        avg_power_w (float | None): Smoothed draw, voltage x discharge current.
            Every discharge sample counts, so idle, sensors and manipulator are all in; charging samples don't.
        avg_speed_mps (float | None): Smoothed travel speed. Only samples
            where the robot actually moved contribute.
        last_x (float | None): Last position used for a speed sample.
        last_y (float | None): Last position used for a speed sample.
        last_time (float | None): Timestamp of that position, in seconds.
    """

    energy_wh: float | None = None
    avg_power_w: float | None = None
    avg_speed_mps: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    last_time: float | None = None

    def readings(self) -> tuple[float, float, float] | None:
        """
        Return (energy_wh, avg_power_w, avg_speed_mps) once all three exist.

        Handing back a tuple rather than letting callers read the optional
        fields directly keeps the "are these populated" check and the use of
        the values in one place, so a caller cannot divide by a speed that was
        never measured.

        Returns:
            The three readings, or None while any is still missing. Speed of
            zero counts as missing: the return-leg prediction divides by it.
        """
        if (self.energy_wh is None
                or self.avg_power_w is None
                or self.avg_speed_mps is None
                or self.avg_speed_mps <= 0.0):
            return None
        return self.energy_wh, self.avg_power_w, self.avg_speed_mps


@dataclass
class FarmConfig:
    """
    Farm-wide settings that are not held in the database.

    Attributes:
        crop_type (str): Crop grown on this farm, written onto harvest tasks.
        graph_source (str): Which table family supplies the routing graph.
            'graph' uses graph_node / graph_edge / object_data, 'farm' uses
            the older farm_node / farm_edge / farm_asset_on_map. The rollback
            switch: the old tables are left in place, so flipping this back
            restores the previous behaviour without a code change.
    """

    crop_type: str
    graph_source: str


@dataclass
class HarvestConfig:
    """
    How a plant is approached for cutting.

    Attributes:
        side (str): Which of a plant's two Pickup nodes a robot is sent to: 'north' (the node with the larger y,
            heading +x, lane row_id - 1) or 'nearest' (the one closer to where the robot starts).
        cutter_offset_x (dict[str, float]): Metres the last waypoint is moved forward along the Pickup node's
            heading, per platform (the serial's prefix: j100, a200, a300): the cutter works behind the robot's
            right side, so the robot stops with the plant beside its arm, not abeam.
        offset_default (float): The offset for a platform not in the dict.
        lane_end_margin (float): Metres past the crop rows' ends where a robot leaves or enters a lane: it turns
            there, never between two rows.
    """
    side: str = 'north'
    cutter_offset_x: dict[str, float] = field(default_factory=dict)
    offset_default: float = 0.0
    lane_end_margin: float = 1.2


@dataclass
class PlantTarget:
    """A pending plant (harvest_state + object_data) with its two Pickup nodes."""
    object_id: int
    row_id: int
    x: float
    y: float
    attempts: int
    north_node: int
    north_theta: float
    south_node: int
    south_theta: float


@dataclass
class OperationRow:
    """The operation table's one row."""
    state: str
    coordinator_enabled: bool
    target_rows: list[int] | None
    max_attempts: int
    offline_grace_s: int


@dataclass
class LivenessConfig:
    """
    Watchdog settings for deciding whether a robot is still alive.

    Attributes:
        timeout (float): Seconds without a status message before the robot is
            marked offline. Must exceed the worst-case gap between messages.
        check_period (float): How often the watchdog scans for stale robots.
        idle_task_timeout (float): Seconds a robot may report IDLE while a task is held for it before the task
            is released (plant back to pending, dock freed) and it gets a new one. The robot ignores republishes
            of a task it already handled, so a completion the server missed would otherwise hold both for good.
    """

    timeout: float
    check_period: float
    idle_task_timeout: float = 60.0


@dataclass
class Config:
    """
    A configuration class for storing application settings.
    Attributes:
        namespace_config (NamespaceConfig): Namespace subscription settings.
        battery (BatteryConfig): Battery threshold settings.
        liveness (LivenessConfig): Robot liveness watchdog settings.
        farm (FarmConfig): Farm-wide settings.
        clearance (ClearanceConfig): How far waypoints are pushed off the crop.
        docks (dict): Raw `docks` block, namespace -> purpose -> list of dock
            IDs. Left untyped here because the keys are robot namespaces;
            Configuration.get_docks() converts it to RobotDocks.
        database (DatabaseDetails): Details of the database configuration.
    """
    namespace_config: NamespaceConfig
    battery: BatteryConfig
    liveness: LivenessConfig
    farm: FarmConfig
    clearance: ClearanceConfig
    docks: dict[str, Any]
    database: DatabaseDetails
    harvest: HarvestConfig = field(default_factory=HarvestConfig)


@dataclass
class RobotDetails:
    """
    Class representing the details of a robot.
    Attributes:
        id (int): Unique identifier for the robot.
        robot_name (str): Name of the robot.
        robot_type (str): Type or category of the robot.
        robot_description (str): Description of the robot's functionality or purpose.
        manufacturer (str): Name of the manufacturer of the robot.
        serial_number (str): Serial number of the robot for identification.
    """

    id: int
    robot_name: str
    robot_type:  str
    robot_description: str
    manufacturer: str
    serial_number: str


@dataclass
class JobDetails:
    """
    Represents the details of a job in the Robot Status Monitor system.
    Attributes:
        id (int): Unique identifier for the job.
        node_id (int): Identifier for the node associated with the job.
        job_schedule (str): Scheduled time or identifier for the job.
        crop_type (str): Type of crop associated with the job.
        harvest_ready (int): Indicator of whether the crop is ready for harvest.
        progress_status (float): Progress status of the job as a percentage.
        assigned_robot_id (int): Identifier of the robot assigned to the job.
        gps_location (Point): GPS location of the job site.
        task_type (int): Type of task this row represents, see TaskEnum.

    Note:
        Field order matches the column order of `farm_harvesting_job`, because
        rows are unpacked positionally as JobDetails(*row).
    """

    id: int
    node_id: int
    job_schedule: str
    crop_type: str
    harvest_ready: int
    progress_status: float
    assigned_robot_id: int
    gps_location: Point
    task_type: int


@dataclass
class RobotLive:
    """
    Class representing the live status of a robot.
    Attributes:
        id (int): Unique identifier for the robot.
        time (str): Timestamp of the status update.
        ros_time (Time): ROS time associated with the status update.
        robot_type (str): Type of the robot.
        online_flag (int): Flag indicating whether the robot is online (1) or offline (0).
        status (str): Current status of the robot.
        gps_location (Point): GPS location of the robot.
        task (str): Current task being performed by the robot.
        topo_map_pos (TopologyMapPosition): Topological map position of the robot.
        current_node_id (int): ID of the current node in the topological map.
        target_node_id (int): ID of the target node in the topological map.
        battery_level (float): Current battery level of the robot as a percentage.
        operation_hours_after_charging (str): Hours of operation since the last charge.
        load_status (float): Current load status of the robot.
        crop_type (str): Type of crop being handled by the robot.
    """

    id: int
    time: str
    ros_time: Time
    robot_type: str
    online_flag: int
    status: int
    gps_location: Point
    task: str
    topo_map_pos: TopologyMapPosition
    current_node_id: int
    target_node_id: int
    battery_level: float
    operation_hours_after_charging: str
    load_status: float
    crop_type: str


@dataclass
class RobotLiveStatus:
    """
    RobotLiveStatus class represents the live status of a robot with various attributes.
    Attributes:
        id (int): Unique identifier for the robot.
        time (str): Timestamp of the status update.
        ros_time (str): ROS-specific timestamp of the status update.
        robot_type (str): Type or model of the robot.
        online_flag (int): Flag indicating whether the robot is online (1) or offline (0).
        status (str): Current status of the robot.
        gps_location (str): GPS coordinates of the robot's current location.
        task (str): Current task assigned to the robot.
        topo_map_pos (str): Topological map position of the robot.
        current_node_id (int): Identifier of the current node in the map.
        target_node_id (int): Identifier of the target node in the map.
        battery_level (float): Current battery level of the robot as a percentage.
        operation_hours_after_charging (str): Hours of operation since the last charge.
        load_status (float): Current load status of the robot.
        crop_type (str): Type of crop the robot is handling, if applicable.
    """

    id: int
    time: str
    ros_time: str
    robot_type: str
    online_flag: int
    status: str
    gps_location: str
    task: str
    topo_map_pos: str
    current_node_id: int
    target_node_id: int
    battery_level: float
    operation_hours_after_charging: str
    load_status: float
    crop_type: str
