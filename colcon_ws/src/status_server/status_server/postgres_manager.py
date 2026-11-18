import time
import traceback
from collections.abc import Mapping, Sequence
from typing import Any, Literal, TypeAlias

import psycopg
from psycopg import DatabaseError, Error, InterfaceError, OperationalError, sql
from rclpy.impl.rcutils_logger import RcutilsLogger

from status_server.configuration import Configuration
from status_server.dataclass import (
    DockStation,
    FarmAssets,
    FarmNodes,
    GraphNode,
    HarvestUnit,
    JobDetails,
    RobotDetails,
    RobotLiveStatus,
    OperationRow,
    PlantTarget,
)
from status_server.enum import (
    ChargingStationStatusEnum,
    OnlineFlagEnum,
    ProgressStatusEnum,
    RobotStatusEnum,
    TaskEnum,
)

# Query: TypeAlias = LiteralString | bytes | "sql.SQL" | "sql.Composed"
Params: TypeAlias = Sequence[Any] | Mapping[str, Any]


class PostgresConnectionManager:
    """
    Unified PostgreSQL database operations for ROS2 nodes with persistent connection.
    """

    def __init__(self):
        self.logger = RcutilsLogger(self.__class__.__name__)
        config = Configuration()
        self.db_config = config.get_database()
        self.conn: psycopg.Connection | None = None

        if not self._connect():
            self.logger.error('Failed to establish initial database connection')

    def _connect(self) -> bool:
        """
        Establish a PostgreSQL connection.

        connect_timeout is mandatory: without it psycopg falls back to the OS
        TCP timeout, so an unreachable database host blocks the calling thread
        for minutes. Every node here calls the database from its executor
        thread, so that stalls the whole node.
        """
        try:
            self.conn = psycopg.connect(
                host=self.db_config.host,
                port=self.db_config.port,
                dbname=self.db_config.dbname,
                user=self.db_config.user,
                password=self.db_config.password or None,  # None: libpq falls back to PGPASSWORD / ~/.pgpass
                connect_timeout=self.db_config.connect_timeout,
            )
            self.conn.autocommit = True
            return True
        except Error as e:
            self.logger.error(f'Failed to connect to database: {e}\n{traceback.format_exc()}')
            self.conn = None
            return False

    def _has_connection(self) -> bool:
        """Open a connection if there isn't a usable one already."""
        if self.conn is not None and not self.conn.closed:
            return True
        return self._connect()

    def _reconnect(self, reason: Exception) -> bool:
        """Drop a broken connection and open a fresh one."""
        self.logger.warning(f'Database connection lost ({reason}), reconnecting...')
        try:
            if self.conn is not None and not self.conn.closed:
                self.conn.close()
        except (Error, OSError):
            pass
        return self._connect()

    def close(self):
        """Close the database connection."""
        try:
            if self.conn and not self.conn.closed:
                self.conn.close()
                self.logger.info('Database connection closed.')
        except Error as e:
            self.logger.error(f'Error closing DB connection: {e}\n{traceback.format_exc()}')
        finally:
            self.conn = None

    # ----------------- Unified Execution Methods -----------------

    def execute_write(self, query, params: Params | None = None, many: bool = False) -> int | None:
        """
        Execute INSERT, UPDATE or DELETE and report how many rows changed.

        Distinguishes "the statement ran but matched nothing" from "the
        statement failed", which execute_query cannot express. Callers that
        need to know a row was actually written should use this.

        Args:
            query:  SQL statement.
            params: Query parameters, or a list of tuples when many=True.
            many:   Use executemany.

        Returns:
            Number of rows affected, or None if the statement failed.

        Note:
            The statement is sent straight to the server. A broken connection
            surfaces as an error here, which triggers one reconnect and one
            retry - cheaper than probing the connection before every call.
        """
        for attempt in (1, 2):
            if not self._has_connection() or self.conn is None:
                return None

            try:
                with self.conn.cursor() as cur:
                    if many:
                        if not isinstance(params, list):
                            raise ValueError('params must be a list of tuples when many=True')
                        cur.executemany(query, params)
                    else:
                        cur.execute(query, params)
                    if not self.conn.autocommit:
                        self.conn.commit()
                    return cur.rowcount

            except (OperationalError, InterfaceError) as e:
                # Connection-level failure: worth one reconnect and retry.
                if attempt == 1 and self._reconnect(e):
                    continue
                self.logger.error(f'Query execution failed: {e}')
                return None

            except Error as e:
                # Statement-level failure. Retrying would fail identically.
                self.logger.error(f'Query execution failed: {e}\n{traceback.format_exc()}')
                if self.conn and not self.conn.closed:
                    self.conn.rollback()
                return None

        return None

    def execute_query(self, query, params: Params | None = None, many: bool = False) -> bool:
        """
        Execute INSERT, UPDATE, DELETE, or parameterized queries.

        Reports only whether the statement ran, not whether it matched any
        rows. Use execute_write when that distinction matters.
        """
        return self.execute_write(query, params, many) is not None

    def fetch_data(
        self,
        query,
        params: Params | None = None,
        fetch_type: Literal['one', 'many', 'all'] = 'all',
        num_rows: int | None = None,
    ) -> tuple | list[tuple] | None:
        """
        Execute a SELECT query and fetch data based on specified fetch type.

        Note:
            Sent straight to the server. A broken connection surfaces as an
            error here and triggers one reconnect and one retry, instead of
            probing the connection before every query.
        """
        if fetch_type not in ('one', 'many', 'all'):
            raise ValueError("fetch_type must be 'one', 'many', or 'all'")

        if fetch_type == 'many' and num_rows is None:
            raise ValueError("num_rows must be specified for fetch_type='many'")

        for attempt in (1, 2):
            if not self._has_connection() or self.conn is None:
                return None

            try:
                with self.conn.cursor() as cur:
                    cur.execute(query, params)
                    if fetch_type == 'one':
                        return cur.fetchone()
                    if fetch_type == 'many':
                        return cur.fetchmany(num_rows)
                    return cur.fetchall()

            except (OperationalError, InterfaceError) as e:
                if attempt == 1 and self._reconnect(e):
                    continue
                self.logger.error(f'Data fetch failed: {e}')
                return None

            except Error as e:
                self.logger.error(f'Data fetch failed: {e}\n{traceback.format_exc()}')
                return None

        return None

    # ----------------- Utility Methods -----------------

    def table_exists(self, table_name: str, schema: str = 'public') -> bool:
        """Check if a table exists."""
        query = """
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_schema = %s AND table_name = %s
            )
        """
        result = self.fetch_data(query, (schema, table_name), fetch_type='one')

        self.logger.debug(f'Table exists: {result}')

        return bool(result[0]) if result else False

    def row_exists(self, table_name: str, row_id: int) -> bool:
        """Check if a row exists."""
        query = sql.SQL('SELECT EXISTS (SELECT FROM {table} WHERE id = %s);').format(table=sql.Identifier(table_name))

        result = self.fetch_data(query, (row_id,), fetch_type='one')

        self.logger.debug(f'Row with ID = {row_id} exists in table {table_name}: {result}')

        return bool(result[0]) if result else False


class PostgresOperations:
    """
    Enhanced database operations for task manager using SafePostgresClient.
    Handles robot status, task updates, and node information with safe queries.
    """

    def __init__(self, db: PostgresConnectionManager | None = None):
        """
        Args:
            db: Connection manager to use. Pass an existing one to share a
                single socket across collaborating objects; omit it to open a
                new connection.
        """
        self.db = db if db is not None else PostgresConnectionManager()
        self.logger = RcutilsLogger(self.__class__.__name__)

        # Which table family the map dispatchers read. Resolved once: the
        # config is not reloadable at runtime, and every path lookup would
        # otherwise re-read it.
        self.graph_source = Configuration().get_farm_config().graph_source
        self._use_graph_tables = self.graph_source != 'farm'

        if not self._use_graph_tables:
            self.logger.warning("graph_source is 'farm': routing from the legacy farm_node / farm_edge tables.")

        # Caches for farm geometry only. Nothing in this package writes
        # farm_node or farm_asset_on_map, and farm_edge is already frozen at
        # start-up inside TaskGenerator's distance matrix. Live tables -
        # robot_live, farm_harvesting_job, dock_station - are never cached.
        # Restart the coordinator after editing the farm layout.
        self._nodes_cache: list[FarmNodes] | None = None
        self._navigable_nodes_cache: list[FarmNodes] | None = None
        self._assets_cache: list[FarmAssets] | None = None

        # Same treatment for the graph_* tables that replace them.
        self._graph_nodes_cache: list[GraphNode] | None = None
        self._navigable_graph_nodes_cache: list[GraphNode] | None = None
        self._harvest_units_cache: list[HarvestUnit] | None = None
        self._object_positions_cache: list[tuple[float, float]] | None = None

    # ---------------- Robot Status ----------------

    def fetch_robot_status(self, robot_id: int) -> RobotLiveStatus | None:
        """
        Fetch current robot status from database.

        Args:
            robot_id: ID of the robot

        Returns:
            RobotLiveStatus object or None if not found
        """
        try:
            # explicit columns: RobotLiveStatus is unpacked positionally and the table has gained updated_at
            row = self.db.fetch_data(
                'SELECT id, time, ros_time, robot_type, online_flag, status, gps_location, task, topo_map_pos, '
                'current_node_id, target_node_id, battery_level, operation_hours_after_charging, load_status, '
                'crop_type FROM robot_live WHERE id = %s', params=(robot_id,), fetch_type='one')

            if row and isinstance(row, tuple):
                return RobotLiveStatus(*row)
            elif isinstance(row, list) and len(row) > 0 and isinstance(row[0], tuple):
                self.logger.warning(f'Unexpected data type from database: {type(row)}')
                self.logger.warning(f'Unexpected data from database: {row}')
                return RobotLiveStatus(*row[0])  # use only the first record

            return None

        except DatabaseError as e:
            self.logger.error(f'Database error fetching robot status: {e}')
            return None

    # ---------------- Task Progress ----------------

    def fetch_open_task(self, robot_id: int) -> JobDetails | None:
        """
        Fetch the robot's currently open task, whatever its type.

        An open task is any row not yet marked complete, i.e. progress_status
        in (NOT STARTED, IN PROGRESS, PAUSED). This is the single source used
        to rehydrate a robot's task after a server restart, replacing the old
        status-group reconstruction logic.

        The newest open row wins. A robot interrupted by low battery has both
        its paused task and its newer charging task open at once, and the
        charging task is the one it is actually executing.

        Args:
            robot_id: Database ID of the robot.

        Returns:
            JobDetails for the newest open task, or None if the robot has none.
        """
        try:
            query = """
                SELECT * FROM public.farm_harvesting_job
                WHERE assigned_robot_id = %s
                  AND progress_status IN (%s, %s, %s)
                ORDER BY id DESC
                LIMIT 1
            """
            row = self.db.fetch_data(
                query,
                params=(
                    robot_id,
                    ProgressStatusEnum.IDLE.value,
                    ProgressStatusEnum.IN_PROGRESS.value,
                    ProgressStatusEnum.PAUSED.value,
                ),
                fetch_type='one',
            )

            if row and isinstance(row, tuple):
                return JobDetails(*row)
            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching open task for robot {robot_id}: {e}')
            return None

    def insert_task(
        self,
        node_id: int,
        job_schedule: str,
        crop_type: str,
        harvest_ready: int,
        assigned_robot_id: int,
        location: tuple[float, float],
        task_type: int,
    ) -> int | None:
        """
        Insert a new task row and return its generated ID.

        `farm_harvesting_job.id` is an identity column, so IDs are sequential
        across all task types. The returned ID becomes Task.task_id, which is
        why the insert must report it back rather than returning a bool.

        Args:
            node_id: Farm node the robot must reach for this task.
            job_schedule: Label for the target - bush asset name for harvest
                tasks, dock_id for charging and unloading tasks.
            crop_type: Crop being handled, empty string for dock tasks.
            harvest_ready: 1 for harvest tasks, 0 for dock tasks.
            assigned_robot_id: Robot the task is assigned to.
            location: (x, y) of the target, stored in the job_location point.
            task_type: See TaskEnum.

        Returns:
            The new row's ID, or None if the insert failed.
        """
        try:
            query = """
                INSERT INTO public.farm_harvesting_job (
                    node_id, job_schedule, crop_type, harvest_ready,
                    progress_status, assigned_robot_id, job_location, task_type
                ) VALUES (%s, %s, %s, %s, %s, %s, Point(%s, %s), %s)
                RETURNING id
            """
            row = self.db.fetch_data(
                query,
                params=(
                    node_id,
                    job_schedule,
                    crop_type,
                    harvest_ready,
                    ProgressStatusEnum.IDLE.value,
                    assigned_robot_id,
                    location[0],
                    location[1],
                    task_type,
                ),
                fetch_type='one',
            )

            if row and isinstance(row, tuple):
                task_id = int(row[0])
                self.logger.info(
                    f'Created task id={task_id} type={task_type} '
                    f'schedule={job_schedule} node={node_id} '
                    f'robot={assigned_robot_id}'
                )
                return task_id

            self.logger.error(f'Task insert returned no id: schedule={job_schedule} node={node_id}')
            return None

        except DatabaseError as e:
            self.logger.error(f'Error inserting task: {e}')
            return None

    def task_exists(self, job_schedule: str, node_id: int) -> bool:
        """
        Check whether a task already exists for a target.

        A harvest target is one side of one bush, identified by the pair
        (job_schedule, node_id) - the two conn_nodes of a bush share a name
        but have different approach nodes. Used to skip targets already done.

        Args:
            job_schedule: Target label (bush asset name or dock_id).
            node_id: Approach node for that target.

        Returns:
            True if any row already covers this target.
        """
        try:
            query = """
                SELECT EXISTS (
                    SELECT 1 FROM public.farm_harvesting_job
                    WHERE job_schedule = %s AND node_id = %s
                )
            """
            row = self.db.fetch_data(query, params=(job_schedule, node_id), fetch_type='one')
            return bool(row[0]) if row else False
        except DatabaseError as e:
            self.logger.error(f'Error checking task existence: {e}')
            return False

    # ---------------- Dock Stations ----------------

    def fetch_free_dock_from(self, dock_ids: list[str], dock_type: int) -> DockStation | None:
        """
        Fetch the first free dock of the requested type from a robot's own list.

        Replaces the global "first free dock of this type" search: a robot may
        only use the docks its namespace lists in config, so one robot can no
        longer take another's charger. Two robots listing the same dock ID
        share it, and this reservation check decides who gets it.

        Args:
            dock_ids: Dock IDs this robot is allowed to use.
            dock_type: See DockTypeEnum.

        Returns:
            DockStation, or None when the robot has no dock of that type free.
        """
        if not dock_ids:
            self.logger.warning(f'Robot has no dock of type {dock_type} configured.')
            return None

        try:
            placeholders = sql.SQL(', ').join(sql.Placeholder() * len(dock_ids))
            query = sql.SQL(
                """
                SELECT * FROM public.dock_station
                WHERE dock_id IN ({ids})
                  AND type = %s
                  AND status = %s
                  AND robot_id IS NULL
                ORDER BY id ASC
                LIMIT 1
                """
            ).format(ids=placeholders)

            params = (*dock_ids, dock_type, ChargingStationStatusEnum.IDLE.value)
            row = self.db.fetch_data(query, params=params, fetch_type='one')

            if row and isinstance(row, tuple):
                return DockStation(*row)

            self.logger.warning(f'No free dock of type {dock_type} among {dock_ids}.')
            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching dock from {dock_ids}: {e}')
            return None

    def fetch_dock_by_id(self, dock_id: str) -> DockStation | None:
        """
        Fetch a dock station by name, regardless of its occupancy.

        Used when rebuilding an existing dock task, where the dock is already
        reserved by this robot and so would not match fetch_free_dock_from.

        Args:
            dock_id: Dock name from the dock_station table.

        Returns:
            DockStation, or None if no dock carries that name.
        """
        try:
            row = self.db.fetch_data(
                'SELECT * FROM public.dock_station WHERE dock_id = %s', params=(dock_id,), fetch_type='one'
            )
            return DockStation(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f"Error fetching dock '{dock_id}': {e}")
            return None

    def reserve_dock(self, dock_id: str, robot_id: int) -> bool:
        """
        Reserve a dock for a robot when its docking task is created.

        Args:
            dock_id: Dock name from the dock_station table.
            robot_id: Robot taking the dock.

        Returns:
            True if the reservation was written.
        """
        query = """
            UPDATE public.dock_station
            SET robot_id = %s, status = %s
            WHERE dock_id = %s
        """
        success = self.db.execute_query(query, params=(robot_id, ChargingStationStatusEnum.RESERVED.value, dock_id))
        if success:
            self.logger.info(f"Dock '{dock_id}' reserved for robot {robot_id}")
        else:
            self.logger.error(f"Failed to reserve dock '{dock_id}'")
        return success

    def release_dock_for_robot(self, robot_id: int) -> bool:
        """
        Release whichever dock a robot is holding.

        Called on DONE_UNDOCKING. Keyed on robot rather than dock name because
        the robot's task may already have been cleared by that point.

        Args:
            robot_id: Robot to release docks for.

        Returns:
            True if the update ran (including when the robot held no dock).
        """
        query = """
            UPDATE public.dock_station
            SET robot_id = NULL, status = %s
            WHERE robot_id = %s
        """
        success = self.db.execute_query(query, params=(ChargingStationStatusEnum.IDLE.value, robot_id))
        if not success:
            self.logger.error(f'Failed to release docks held by robot {robot_id}')
        return success

    def fetch_last_dock_node(self, robot_id: int) -> int | None:
        """
        Return the dock node a robot is parked at, or None if it has moved on.

        The robot's most recently created task tells you where it last was. If
        that task was charging or unloading, the robot drove to a dock and has
        not been sent anywhere since, so the dock node is where it still
        stands. Any harvest task after it means the robot left.

        This is task history, not position. The caller checks it against the
        reported pose before trusting it.

        Args:
            robot_id: Database ID of the robot.

        Returns:
            Dock node ID, or None if the latest task was not a dock task.
        """
        try:
            row = self.db.fetch_data(
                """
                SELECT node_id, task_type FROM public.farm_harvesting_job
                WHERE assigned_robot_id = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                params=(robot_id,),
                fetch_type='one',
            )

            if not row:
                return None

            node_id, task_type = int(row[0]), int(row[1])

            if task_type in (TaskEnum.CHARGING_TASK.value, TaskEnum.UNLOADING_TASK.value):
                return node_id

            return None

        except DatabaseError as e:
            self.logger.error(f'Error fetching last dock node for robot {robot_id}: {e}')
            return None

    def release_stale_dock_reservations(self) -> int:
        """
        Free docks reserved by robots that have no dock task left to finish.

        A reservation is written when a dock task is created and cleared when
        the robot undocks. Anything that ends the task in between - the server
        restarting, the robot restarting, a task abandoned before docking -
        leaves the row reserved with nothing left to release it, and that dock
        is then unavailable forever.

        Run at start-up: a dock still held by a robot whose dock tasks are all
        closed cannot be genuinely occupied, because the task that reserved it
        no longer exists.

        Returns:
            Number of reservations cleared.
        """
        query = """
            UPDATE public.dock_station d
            SET robot_id = NULL, status = %s
            WHERE d.robot_id IS NOT NULL
              AND NOT EXISTS (
                  SELECT 1 FROM public.farm_harvesting_job j
                  WHERE j.assigned_robot_id = d.robot_id
                    AND j.task_type IN (%s, %s)
                    AND j.progress_status IN (%s, %s, %s)
              )
        """
        rows = self.db.execute_write(
            query,
            params=(
                ChargingStationStatusEnum.IDLE.value,
                TaskEnum.CHARGING_TASK.value,
                TaskEnum.UNLOADING_TASK.value,
                ProgressStatusEnum.IDLE.value,
                ProgressStatusEnum.IN_PROGRESS.value,
                ProgressStatusEnum.PAUSED.value,
            ),
        )

        if rows is None:
            self.logger.error('Failed to sweep stale dock reservations')
            return 0

        if rows:
            self.logger.warning(
                f'Released {rows} stale dock reservation(s) left by a previous run.')

        return rows

    def update_task_progress(self, task_id: int, job_schedule: str, progress_status: int):
        """
        Update task progress in database.

        Args:
            task_id: ID of the task
            job_schedule: Job schedule identifier
            progress_status: Progress status value (0=not started, 1=in progress,
                             2=paused/error, 3=completed)

        Returns:
            Number of rows affected
        """
        try:
            query = """
                UPDATE public.farm_harvesting_job
                SET progress_status=%s
                WHERE id=%s AND job_schedule=%s
            """
            success = self.db.execute_query(query, params=(progress_status, task_id, job_schedule))

            if success:
                self.logger.debug(
                    f'Updated task progress: task_id={task_id}, schedule={job_schedule}, progress={progress_status}'
                )
            return success
        except DatabaseError as e:
            self.logger.error(f'Error updating task progress: {e}')
            return False

    def mark_job_complete(self, task_id: int, job_schedule: str):
        """
        Mark a task as complete in the database.

        Sets progress_status to COMPLETED, which is what takes the row out of
        fetch_open_task's range. harvest_ready is left alone: it records
        whether the target was ready when the task was created, and is not
        part of the completion check.

        Args:
            task_id: ID of the task
            job_schedule: Job schedule identifier

        Returns:
            bool: True if update succeeded, False otherwise
        """
        try:
            query = """
                UPDATE public.farm_harvesting_job
                SET progress_status = %s
                WHERE id=%s AND job_schedule=%s
            """
            success = self.db.execute_query(query, params=(ProgressStatusEnum.COMPLETED.value, task_id, job_schedule))

            if success:
                self.logger.info(f'Marked job complete: task_id={task_id}, schedule={job_schedule}')
            else:
                self.logger.error(f'Failed to mark job complete: task_id={task_id}, schedule={job_schedule}')
            return success
        except DatabaseError as e:
            self.logger.error(f'Error marking job complete: {e}')
            return False

    # ---------------- Robot Details ------------

    def fetch_robot_details(self, namespace: str) -> tuple | list[tuple] | None:
        """
        Fetch robot details from database by namespace.

        Args:
            namespace (str): Robot namespace to search for.

        Returns:
            list[tuple] | None: List of robot detail tuples or None if not found.
        """
        try:
            result = self.db.fetch_data(
                'SELECT * FROM robot_info WHERE serial_number= %s', params=(namespace,), fetch_type='all'
            )
            self.logger.debug(f'Result: {result}')
            return result
        except Error as e:
            self.logger.error(f'Error fetching robot details: {e}')
            return None

    def mark_robot_offline(self, serial_number: str) -> bool:
        """
        Mark a robot offline in robot_live after the watchdog declares it stale.

        A robot that dies simply stops publishing, so its robot_live row would
        otherwise freeze at its last reported values and the task manager would
        keep assigning work to it. Writing OFFLINE is the only signal the rest
        of the system gets.

        Both columns are written, each from its own enum:
            status      <- RobotStatusEnum.OFFLINE  (97)
            online_flag <- OnlineFlagEnum.OFFLINE   (0)

        No reset is needed on recovery: the robot's next status message
        overwrites both through the normal update path.

        Args:
            serial_number: Robot serial, matching robot_info.serial_number.

        Returns:
            True only if a row was actually updated. A robot with no
            robot_live row yet returns False, so the caller keeps retrying and
            marks it once the row appears, instead of banking a write that
            never happened.
        """
        query = """
            UPDATE public.robot_live
            SET status = %s, online_flag = %s
            WHERE id = (SELECT id FROM public.robot_info WHERE serial_number = %s)
        """
        rows = self.db.execute_write(
            query,
            params=(
                RobotStatusEnum.OFFLINE.value,
                OnlineFlagEnum.OFFLINE.value,
                serial_number,
            ),
        )

        if rows is None:
            self.logger.error(f'Failed to mark robot {serial_number} offline')
            return False

        if rows == 0:
            self.logger.debug(
                f'No robot_live row for {serial_number}; nothing to mark offline')
            return False

        return True

    def fetch_all_robot_details(self) -> list[RobotDetails] | None:
        """
        Fetches the details of all the robots from the database based on the given robot name.

        Args:
            robot_name (str): The serial number of the robot to fetch details for.

        Returns:
            list: A list of rows containing the robot details.

        Raises:
            ProgrammingError: If a programming error occurs while executing the query.
            Exception: If any other unexpected error occurs.
        """
        try:
            result = self.db.fetch_data(query='SELECT * FROM public.robot_info', fetch_type='all')

            if not result:
                raise Error('Robot details not found in the database.')

            return [RobotDetails(*row) for row in result]
        except (Error, TypeError) as e:
            self.logger.error(f'Error fetching robot details: {e}')
            return None

    # ---------------- Farm Edges ----------------

    def fetch_farm_assets(self) -> list[FarmAssets]:
        """
        Fetch all harvestable farm assets (bushes), ordered by ID.

        Each asset carries conn_nodes: the approach nodes on either side of
        the bush. One task is generated per (asset, approach node) pair.

        Returns:
            List of FarmAssets, empty if none found or on error.
        """
        if self._assets_cache is not None:
            return self._assets_cache

        try:
            rows = self.db.fetch_data('SELECT * FROM public.farm_asset_on_map ORDER BY id ASC', fetch_type='all')
            self._assets_cache = [FarmAssets(*row) for row in rows] if rows else []
            return self._assets_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching farm assets: {e}')
            return []

    def fetch_node(self, node_id: int) -> FarmNodes | None:
        """
        Fetch a single farm node by ID.

        Note:
            farm_node's primary key is (id, zone_prefix), so an ID could in
            principle appear in more than one zone. The first row is used.

        Args:
            node_id: ID of the node.

        Returns:
            FarmNodes, or None if not found.
        """
        try:
            row = self.db.fetch_data(
                'SELECT * FROM public.farm_node WHERE id = %s ORDER BY zone_prefix ASC',
                params=(node_id,),
                fetch_type='one',
            )
            return FarmNodes(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching node {node_id}: {e}')
            return None

    def fetch_navigable_nodes(self) -> list[FarmNodes]:
        """
        Fetch farm nodes the robot can actually drive to or from.

        A node is navigable only if it appears in farm_edge. Nodes with no
        edges (the 'map' origin marker, for example) sit outside the routing
        graph: the distance matrix is sized from farm_edge, so treating one as
        a robot position makes every path lookup fail and the robot silently
        stops receiving tasks.

        Returns:
            List of FarmNodes with at least one edge, empty on error.
        """
        if self._navigable_nodes_cache is not None:
            return self._navigable_nodes_cache

        try:
            rows = self.db.fetch_data(
                """
                SELECT n.* FROM public.farm_node n
                WHERE EXISTS (
                    SELECT 1 FROM public.farm_edge e
                    WHERE e.from_node = n.id OR e.to_node = n.id
                )
                ORDER BY n.id ASC, n.zone_prefix ASC
                """,
                fetch_type='all'
            )
            self._navigable_nodes_cache = [FarmNodes(*row) for row in rows] if rows else []
            return self._navigable_nodes_cache
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching navigable nodes: {e}')
            return []

    def fetch_all_nodes(self) -> list[FarmNodes]:
        """
        Fetch all farm nodes from database.

        Returns:
            List of FarmNodes objects
        """
        if self._nodes_cache is not None:
            return self._nodes_cache

        try:
            rows = self.db.fetch_data(
                'SELECT * FROM public.farm_node ORDER BY id ASC, zone_prefix ASC', fetch_type='all'
            )
            self._nodes_cache = [FarmNodes(*row) for row in rows] if rows else []
            return self._nodes_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching nodes: {e}')
            return []

    # ---------------- Graph Edges ----------------

    def fetch_graph_node(self, node_id: int) -> GraphNode | None:
        """
        Fetch a single graph node by ID.

        Replaces fetch_node. graph_node.id is the whole primary key, so unlike
        farm_node there is no zone to disambiguate and exactly one row can
        match.

        Args:
            node_id: ID of the node.

        Returns:
            GraphNode, or None if not found.
        """
        try:
            row = self.db.fetch_data(
                'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node WHERE id = %s',
                params=(node_id,),
                fetch_type='one',
            )
            return GraphNode(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching graph node {node_id}: {e}')
            return None

    def fetch_all_graph_nodes(self) -> list[GraphNode]:
        """
        Fetch every graph node, ordered by ID.

        Replaces fetch_all_nodes. TaskGenerator needs the full set at start-up
        to build the euclidean distance matrix.

        Returns:
            List of GraphNode, empty on error.
        """
        if self._graph_nodes_cache is not None:
            return self._graph_nodes_cache

        try:
            rows = self.db.fetch_data(
                'SELECT id, node_type, x, y, theta, obj_id FROM public.graph_node ORDER BY id ASC',
                fetch_type='all',
            )
            self._graph_nodes_cache = [GraphNode(*row) for row in rows] if rows else []
            return self._graph_nodes_cache
        except DatabaseError as e:
            self.logger.error(f'Error fetching graph nodes: {e}')
            return []

    def fetch_navigable_graph_nodes(self) -> list[GraphNode]:
        """
        Fetch graph nodes the robot can actually drive to or from.

        Replaces fetch_navigable_nodes. Every node in the current graph has at
        least one edge, so the EXISTS clause filters nothing today. It is kept
        because the cost matrix is sized from graph_edge: an edgeless node
        added later would make every path lookup from it fail, and the robot
        would silently stop receiving tasks.

        Returns:
            List of GraphNode with at least one edge, empty on error.
        """
        if self._navigable_graph_nodes_cache is not None:
            return self._navigable_graph_nodes_cache

        try:
            rows = self.db.fetch_data(
                """
                SELECT n.id, n.node_type, n.x, n.y, n.theta, n.obj_id
                FROM public.graph_node n
                WHERE EXISTS (
                    SELECT 1 FROM public.graph_edge e
                    WHERE e.source_id = n.id OR e.target_id = n.id
                )
                ORDER BY n.id ASC
                """,
                fetch_type='all',
            )
            self._navigable_graph_nodes_cache = [GraphNode(*row) for row in rows] if rows else []
            return self._navigable_graph_nodes_cache
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching navigable graph nodes: {e}')
            return []

    # ---------------- Harvest Units ----------------

    def fetch_harvest_units(self) -> list[HarvestUnit]:
        """
        Fetch every side of every bush, already in sweep order.

        Replaces fetch_farm_assets. Each object is served by two Pickup nodes,
        one per side of the row, and each side is a separate unit of work
        because the manipulator reaches only half the bush from either side.

        The sweep is ordered row by row, and within a row one lane at a time:

            row 1 near lane, node IDs ascending   -> b1_1 .. b1_38
            row 1 far lane,  node IDs ascending   -> b1_38 .. b1_1
            row 2 near lane, ...

        Node IDs run opposite to object IDs on the far lane, so ordering both
        lanes by node ID drives down one side of the row and back up the other
        as a single loop, and finishes each row before starting the next.

        Objects that do not have exactly two Pickup nodes are dropped rather
        than half-harvested; the count is logged so a broken map is visible.

        Returns:
            Ordered list of HarvestUnit, empty if none found or on error.
        """
        if self._harvest_units_cache is not None:
            return self._harvest_units_cache

        try:
            rows = self.db.fetch_data(
                """
                WITH sides AS (
                    SELECT o.object_id, o.row_id, o.x_coord, o.y_coord,
                           MIN(n.id) AS near_node, MAX(n.id) AS far_node
                    FROM public.object_data o
                    JOIN public.graph_node n ON n.obj_id = o.object_id
                    WHERE n.node_type = 'Pickup'
                    GROUP BY o.object_id, o.row_id, o.x_coord, o.y_coord
                    HAVING COUNT(*) = 2
                )
                SELECT row_id, object_id, near_node AS node_id, x_coord, y_coord, 0 AS lane FROM sides
                UNION ALL
                SELECT row_id, object_id, far_node AS node_id, x_coord, y_coord, 1 AS lane FROM sides
                ORDER BY row_id ASC, lane ASC, node_id ASC
                """,
                fetch_type='all',
            )

            if not rows:
                self.logger.error('No harvest units found: object_data or graph_node is empty.')
                self._harvest_units_cache = []
                return self._harvest_units_cache

            units = [
                HarvestUnit(
                    name=f'b{row_id}_{object_id}',
                    node_id=node_id,
                    row_id=row_id,
                    x=float(x_coord),
                    y=float(y_coord),
                )
                for row_id, object_id, node_id, x_coord, y_coord, _lane in rows
            ]

            expected = self.count_objects()
            if expected is not None and len(units) != expected * 2:
                self.logger.warning(
                    f'Harvest sweep has {len(units)} units but object_data holds {expected} objects; '
                    f'{expected * 2 - len(units)} sides dropped for not having exactly two Pickup nodes.'
                )

            self._harvest_units_cache = units
            return units
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching harvest units: {e}')
            return []

    def fetch_object_positions(self) -> list[tuple[float, float]]:
        """
        Fetch every object's map position.

        Used to push published waypoints off the crop: the graph places nodes
        where the manipulator wants them, which is inside the band Nav2 treats
        as untraversable around a mapped obstacle.

        Returns:
            (x, y) per object, empty on error or when the schema has none.
        """
        if self._object_positions_cache is not None:
            return self._object_positions_cache

        if not self._use_graph_tables:
            # farm_asset_on_map is the legacy equivalent; keeping the shift
            # available on rollback avoids a behaviour change between sources.
            assets = self.fetch_farm_assets()
            self._object_positions_cache = [(float(a.x_map), float(a.y_map)) for a in assets]
            return self._object_positions_cache

        try:
            rows = self.db.fetch_data('SELECT x_coord, y_coord FROM public.object_data', fetch_type='all')
            self._object_positions_cache = [(float(r[0]), float(r[1])) for r in rows] if rows else []
            return self._object_positions_cache
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching object positions: {e}')
            return []

    def fetch_crop_rows(self) -> list[tuple[float, float, float]]:
        """
        Each crop row's extent from object_data: (x_min, x_max, mean y), sorted north (+y) first. The lanes run
        between consecutive rows. Empty on error or on the legacy schema (no rows there).
        """
        if not self._use_graph_tables:
            return []
        try:
            rows = self.db.fetch_data(
                'SELECT min(x_coord), max(x_coord), avg(y_coord) FROM public.object_data '
                'WHERE row_id IS NOT NULL GROUP BY row_id ORDER BY avg(y_coord) DESC', fetch_type='all')
            return [(float(r[0]), float(r[1]), float(r[2])) for r in rows] if rows else []
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching crop rows: {e}')
            return []

    def count_objects(self) -> int | None:
        """
        Count rows in object_data, used to detect bushes dropped from the sweep.

        Returns:
            Row count, or None on error.
        """
        try:
            row = self.db.fetch_data('SELECT COUNT(*) FROM public.object_data', fetch_type='one')
            return int(row[0]) if row and isinstance(row, tuple) else None
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error counting objects: {e}')
            return None

    # ---------------- Map Dispatch ----------------
    #
    # Everything that routes calls these rather than the table-specific
    # readers above, so `farm.graph_source` is the only place that knows which
    # schema is live. Both families return the same shapes: normalised edge
    # triples, node objects exposing id / x / y / theta, and HarvestUnits.

    def fetch_map_node(self, node_id: int) -> GraphNode | FarmNodes | None:
        """
        Fetch one node by ID from whichever schema is live.

        Args:
            node_id: ID of the node.
        """
        return self.fetch_graph_node(node_id) if self._use_graph_tables else self.fetch_node(node_id)

    def fetch_navigable_map_nodes(self) -> list[GraphNode] | list[FarmNodes]:
        """Fetch every node with at least one edge, from whichever schema is live."""
        return self.fetch_navigable_graph_nodes() if self._use_graph_tables else self.fetch_navigable_nodes()

    def fetch_sweep(self) -> list[HarvestUnit]:
        """
        Fetch every side of every bush, in the order they should be harvested.

        On the graph schema the ordering is done in SQL, row by row. On the
        legacy schema it is rebuilt here from farm_asset_on_map so the old
        behaviour survives a rollback: one pass over every bush ascending
        taking the lower conn_node, then one descending taking the higher.
        That sweep crosses the whole farm twice instead of finishing a row at
        a time, which is the behaviour the graph schema replaces.

        Returns:
            Ordered HarvestUnits, empty if none found or on error.
        """
        if self._use_graph_tables:
            return self.fetch_harvest_units()

        assets = self.fetch_farm_assets()
        if not assets:
            self.logger.error('No farm assets found in the database.')
            return []

        sweep: list[HarvestUnit] = []

        for asset in assets:
            if asset.conn_nodes:
                sweep.append(
                    HarvestUnit(
                        name=asset.name,
                        node_id=min(asset.conn_nodes),
                        row_id=0,
                        x=float(asset.x_map),
                        y=float(asset.y_map),
                    )
                )

        for asset in reversed(assets):
            if len(asset.conn_nodes or []) > 1:
                sweep.append(
                    HarvestUnit(
                        name=asset.name,
                        node_id=max(asset.conn_nodes),
                        row_id=0,
                        x=float(asset.x_map),
                        y=float(asset.y_map),
                    )
                )

        return sweep

    # ---------------- Harvest operation (migration 002: harvest_state, operation, operation_event) ----------------

    def fetch_operation(self) -> OperationRow | None:
        """The operation row (state, coordinator flag, target rows, limits), cached for a second: the publish tick
        runs per robot at 1 Hz."""
        cache = getattr(self, '_operation_cache', None)
        now = time.monotonic()
        if cache and now - cache[0] < 1.0:
            return cache[1]
        try:
            row = self.db.fetch_data(
                'SELECT state, coordinator_enabled, target_rows, max_attempts, offline_grace_s '
                'FROM public.operation WHERE id = 1', fetch_type='one')
            op = OperationRow(*row) if row and isinstance(row, tuple) else None
        except DatabaseError as e:
            self.logger.error(f'Error fetching the operation row: {e}')
            op = None
        self._operation_cache = (now, op)
        return op

    def set_operation_state(self, state: str) -> bool:
        rows = self.db.execute_write(
            'UPDATE public.operation SET state = %s, updated_at = now() WHERE id = 1', params=(state,))
        self._operation_cache = None
        return bool(rows)

    def log_event(self, source: str, kind: str, message: str, robot: str | None = None,
                  object_id: int | None = None, job_id: int | None = None) -> None:
        """Append to operation_event (the /operation page's log). Never raises: an event is not worth a tick."""
        try:
            self.db.execute_write(
                'INSERT INTO public.operation_event (source, robot, kind, object_id, job_id, message) '
                'VALUES (%s, %s, %s, %s, %s, %s)',
                params=(source, robot, kind, object_id, job_id, message[:500]))
        except Exception as e:  # noqa: BLE001
            self.logger.warning(f'operation_event insert failed: {e}')

    _PLANT_TARGET_SQL = """
        SELECT h.object_id, h.row_id, o.x_coord, o.y_coord, h.attempts,
               nn.id, nn.theta, sn.id, sn.theta
        FROM public.harvest_state h
        JOIN public.object_data o USING (object_id)
        JOIN LATERAL (SELECT id, theta FROM public.graph_node
                      WHERE obj_id = h.object_id AND node_type = 'Pickup' AND y > o.y_coord
                      ORDER BY id LIMIT 1) nn ON true
        JOIN LATERAL (SELECT id, theta FROM public.graph_node
                      WHERE obj_id = h.object_id AND node_type = 'Pickup' AND y < o.y_coord
                      ORDER BY id LIMIT 1) sn ON true
    """

    def fetch_plant_targets(self, target_rows: list[int] | None) -> list[PlantTarget]:
        """Every pending plant (in target_rows, or all rows when None) with its north and south Pickup nodes:
        fewest attempts first, then row by row along x. Not cached: claims change it."""
        try:
            rows = self.db.fetch_data(
                self._PLANT_TARGET_SQL + """
                WHERE h.status = 'pending' AND (%s::int[] IS NULL OR h.row_id = ANY(%s::int[]))
                ORDER BY h.attempts, h.row_id, o.x_coord
                """, params=(target_rows, target_rows), fetch_type='all')
            return [PlantTarget(int(r[0]), int(r[1]), float(r[2]), float(r[3]), int(r[4]),
                                int(r[5]), float(r[6] or 0.0), int(r[7]), float(r[8] or 0.0)) for r in rows or []]
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching plant targets: {e}')
            return []

    def fetch_plant_by_job(self, job_id: int) -> PlantTarget | None:
        """The plant a harvest task (farm_harvesting_job.id) is for, whatever its state."""
        try:
            r = self.db.fetch_data(self._PLANT_TARGET_SQL + ' WHERE h.job_id = %s', params=(job_id,), fetch_type='one')
            if not r:
                return None
            return PlantTarget(int(r[0]), int(r[1]), float(r[2]), float(r[3]), int(r[4]),
                               int(r[5]), float(r[6] or 0.0), int(r[7]), float(r[8] or 0.0))
        except (DatabaseError, TypeError, ValueError) as e:
            self.logger.error(f'Error fetching the plant of job {job_id}: {e}')
            return None

    def fetch_leased_lanes(self, exclude_robot_id: int) -> set[int]:
        """Lanes other robots are working in (their assigned plants' lane_id)."""
        try:
            rows = self.db.fetch_data(
                "SELECT DISTINCT lane_id FROM public.harvest_state "
                "WHERE status = 'assigned' AND lane_id IS NOT NULL AND assigned_robot_id <> %s",
                params=(exclude_robot_id,), fetch_type='all')
            return {int(r[0]) for r in rows or []}
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching leased lanes: {e}')
            return set()

    def fetch_robot_lane(self, robot_id: int) -> int | None:
        """The lane of the robot's latest assigned or harvested plant, so it keeps working its lane."""
        try:
            row = self.db.fetch_data(
                "SELECT lane_id FROM public.harvest_state WHERE assigned_robot_id = %s "
                "AND status IN ('assigned', 'harvested') AND lane_id IS NOT NULL "
                "ORDER BY COALESCE(harvested_at, assigned_at) DESC NULLS LAST LIMIT 1",
                params=(robot_id,), fetch_type='one')
            return int(row[0]) if row else None
        except (DatabaseError, TypeError) as e:
            self.logger.error(f'Error fetching the lane of robot {robot_id}: {e}')
            return None

    def claim_plant(self, object_id: int, robot_id: int, pickup_node_id: int, lane_id: int, job_id: int) -> bool:
        """pending -> assigned for this robot and task; False when another assignment got it first."""
        rows = self.db.execute_write(
            "UPDATE public.harvest_state SET status = 'assigned', assigned_robot_id = %s, pickup_node_id = %s, "
            "lane_id = %s, job_id = %s, last_error = NULL, assigned_at = now(), updated_at = now() "
            "WHERE object_id = %s AND status = 'pending'",
            params=(robot_id, pickup_node_id, lane_id, job_id, object_id))
        return bool(rows)

    def mark_plant_harvested(self, job_id: int) -> bool:
        rows = self.db.execute_write(
            "UPDATE public.harvest_state SET status = 'harvested', harvested_at = now(), updated_at = now() "
            "WHERE job_id = %s AND status = 'assigned'", params=(job_id,))
        return bool(rows)

    def fail_plant_attempt(self, job_id: int, error: str, max_attempts: int) -> str | None:
        """Count a failed attempt on the task's plant: back to pending, or failed after max_attempts. Returns the
        plant's new status, or None when the task had no assigned plant."""
        try:
            row = self.db.fetch_data(
                "UPDATE public.harvest_state SET attempts = attempts + 1, last_error = %s, "
                "status = CASE WHEN attempts + 1 >= %s THEN 'failed' ELSE 'pending' END, "
                "assigned_robot_id = CASE WHEN attempts + 1 >= %s THEN assigned_robot_id ELSE NULL END, "
                "job_id = NULL, updated_at = now() "
                "WHERE job_id = %s AND status = 'assigned' RETURNING status",
                params=(error[:500], max_attempts, max_attempts, job_id), fetch_type='one')
            return str(row[0]) if row else None
        except DatabaseError as e:
            self.logger.error(f'Error failing the plant of job {job_id}: {e}')
            return None

    def release_plant(self, job_id: int) -> bool:
        """The task's plant back to pending without counting an attempt (offline robot, charging preemption)."""
        rows = self.db.execute_write(
            "UPDATE public.harvest_state SET status = 'pending', assigned_robot_id = NULL, job_id = NULL, "
            "updated_at = now() WHERE job_id = %s AND status = 'assigned'", params=(job_id,))
        return bool(rows)

    def close_task(self, task_id: int, job_schedule: str, status: int) -> bool:
        """farm_harvesting_job row -> status (3 done, 4 failed, 5 released): out of fetch_open_task's range."""
        rows = self.db.execute_write(
            'UPDATE public.farm_harvesting_job SET progress_status = %s WHERE id = %s AND job_schedule = %s',
            params=(status, task_id, job_schedule))
        if rows:
            self.logger.info(f'Closed task {task_id} ({job_schedule}) with status {status}')
        return bool(rows)

    def fetch_robot_operation(self, robot_id: int) -> tuple[bool, str, str | None]:
        """(enrolled, state, request) of the robot's operator controls (robot_operation, migration 004); a robot
        without a row gets one (enrolled, active)."""
        try:
            row = self.db.fetch_data('SELECT enrolled, state, request FROM public.robot_operation WHERE robot_id = %s',
                                     params=(robot_id,), fetch_type='one')
            if row:
                return bool(row[0]), str(row[1]), row[2]
            self.db.execute_write('INSERT INTO public.robot_operation (robot_id) VALUES (%s) ON CONFLICT DO NOTHING',
                                  params=(robot_id,))
        except DatabaseError as e:
            self.logger.error(f'Error fetching robot_operation for {robot_id}: {e}')
        return True, 'active', None

    def clear_robot_request(self, robot_id: int) -> None:
        self.db.execute_write('UPDATE public.robot_operation SET request = NULL, updated_at = now() WHERE robot_id = %s',
                              params=(robot_id,))

    def count_pending_plants(self, target_rows: list[int] | None) -> int:
        try:
            row = self.db.fetch_data(
                "SELECT count(*) FROM public.harvest_state WHERE status IN ('pending', 'assigned') "
                "AND (%s::int[] IS NULL OR row_id = ANY(%s::int[]))", params=(target_rows, target_rows),
                fetch_type='one')
            return int(row[0]) if row else 0
        except DatabaseError as e:
            self.logger.error(f'Error counting pending plants: {e}')
            return 0

    def count_open_harvest_jobs(self) -> int:
        try:
            row = self.db.fetch_data(
                'SELECT count(*) FROM public.farm_harvesting_job WHERE task_type = %s AND progress_status IN (0, 1, 2)',
                params=(TaskEnum.HARVESTING_TASK.value,), fetch_type='one')
            return int(row[0]) if row else 0
        except DatabaseError as e:
            self.logger.error(f'Error counting open harvest jobs: {e}')
            return 0

    # ---------------- Cleanup ----------------

    def close(self):
        """Close the database connection"""
        self.db.close()
        self.logger.info('Database connection closed')
