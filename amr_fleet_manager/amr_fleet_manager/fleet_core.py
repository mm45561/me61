"""
Fleet task allocation without ROS: decides which AMR goes where, and when.

The ROS node (fleet_manager_node.py) feeds FleetCore with robot states and
action results, calls tick() periodically and sends every task that FleetCore
puts in its outbox. Keeping the decisions here, with no rclpy import, means
they can be unit tested with plain pytest.

Rules that keep the robots out of each other's way around the GTP station:

1. Every dock, drop-off and queue slot is an exclusive reservation. A robot is
   only sent somewhere after it holds that place; until then its task waits
   as PENDING.
2. A robot keeps the reservation of the place it is leaving until it is more
   than `clearance_m` away, so the next robot cannot drive in too early.
3. There are at least as many queue slots as robots, so a robot that finished
   a delivery always has a free place to return to.
4. A robot that goes offline or fails a task keeps every reservation it holds:
   nobody is sent into a place that it may still be blocking.
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import logging
import uuid

from amr_fleet_manager.geometry import Pose2D
from amr_fleet_manager.reservations import ReservationTable
from amr_fleet_manager.site_map import dest_key, dock_key, resource_kind, Site, slot_key


class RobotStateCode(enum.IntEnum):
    """Operational state of one AMR; values match amr_interfaces/msg/RobotState."""

    UNKNOWN = 0
    IDLE = 1
    TO_GTP = 2
    DOCKED_AT_GTP = 3
    LOADING = 4
    IN_TRANSIT = 5
    UNLOADING = 6
    RETURNING = 7
    ERROR = 8


class TaskType(enum.IntEnum):
    """One leg of the GTP cycle; values match amr_interfaces/action/ExecuteTask."""

    GO_TO_STATION = 1
    DELIVER = 2
    RETURN_TO_QUEUE = 3


class TaskStatus(enum.Enum):
    """Where a task is in its life, as seen by the fleet manager."""

    PENDING = 'pending'   # waiting for its target reservation
    SENT = 'sent'         # goal sent, the robot has not answered yet
    ACTIVE = 'active'     # the robot accepted the goal and is working on it


@dataclass
class Task:
    """One leg of work for one robot: drive to `target_pose` and do the job there."""

    task_id: str
    task_type: TaskType
    robot_id: str
    target_id: str          # station, destination or slot id
    target_resource: str    # reservation the robot must hold before it is sent
    target_pose: Pose2D
    status: TaskStatus = TaskStatus.PENDING


@dataclass
class RobotRecord:
    """What the fleet manager knows about one robot."""

    robot_id: str
    state: RobotStateCode = RobotStateCode.UNKNOWN
    pose: Pose2D | None = None
    battery_percent: float = 0.0
    last_seen: float = 0.0
    online: bool = False
    faulted: bool = False           # failed a task: needs a human, gets no more work
    task: Task | None = None        # the task the fleet manager gave it, if any
    anchor: str | None = None       # reservation of the place it is parked at / heading to
    idle_since: float = 0.0
    cooldown_until: float = 0.0     # offer no work before this time (after a rejection)
    warned_unmanaged: bool = False


@dataclass(frozen=True)
class FleetConfig:
    """Tuning knobs. The ROS node declares one parameter per field."""

    selection_policy: str = 'nearest'   # 'nearest' or 'longest_idle'
    min_battery_percent: float = 25.0   # below this a robot gets no new work
    heartbeat_timeout_s: float = 3.0    # no RobotState for this long = offline
    clearance_m: float = 1.0            # distance that frees the place a robot left
    match_tolerance_m: float = 0.5      # how close counts as 'parked at' a slot or dock
    retry_cooldown_s: float = 5.0       # pause before offering work to a robot that refused


@dataclass(frozen=True)
class DispatchResult:
    """Answer to a dispatch request, copied into the DispatchAmr response."""

    accepted: bool
    message: str
    task_id: str = ''
    robot_id: str = ''


class FleetCore:
    """Allocate GTP tasks to robots and keep the reservation table consistent."""

    POLICIES = ('nearest', 'longest_idle')
    AT_DOCK_STATES = (RobotStateCode.DOCKED_AT_GTP, RobotStateCode.LOADING)

    def __init__(self, site: Site, robot_ids: list[str], config: FleetConfig | None = None,
                 logger=None) -> None:
        self._cfg = config or FleetConfig()
        if self._cfg.selection_policy not in self.POLICIES:
            raise ValueError(
                f'selection_policy must be one of {self.POLICIES}, '
                f'got {self._cfg.selection_policy!r}')
        if not robot_ids or len(set(robot_ids)) != len(robot_ids):
            raise ValueError(f'robot_ids must be non-empty and unique, got {robot_ids}')
        if len(site.queue_slots) < len(robot_ids):
            raise ValueError(
                f'{len(robot_ids)} robots but only {len(site.queue_slots)} queue slots: '
                'every robot needs a slot to return to, or returns can deadlock')
        self._site = site
        self._log = logger or logging.getLogger(__name__)
        self._robots = {robot_id: RobotRecord(robot_id) for robot_id in robot_ids}
        self._reservations = ReservationTable()
        self._tasks: dict[str, Task] = {}
        self._pending: list[Task] = []
        self._outbox: list[Task] = []

    # ------------------------------------------------------------------ views

    @property
    def reservations(self) -> ReservationTable:
        """Return the reservation table (read it, do not modify it)."""
        return self._reservations

    def robot(self, robot_id: str) -> RobotRecord:
        """Return the record of one robot."""
        return self._robots[robot_id]

    def pop_outbox(self) -> list[Task]:
        """Return the tasks that must be sent to robots now, and empty the outbox."""
        tasks, self._outbox = self._outbox, []
        return tasks

    # ----------------------------------------------------------------- inputs

    def update_robot(self, robot_id: str, state: int, pose: Pose2D, battery_percent: float,
                     now: float) -> None:
        """Record one RobotState message; it doubles as the robot's heartbeat."""
        robot = self._robots.get(robot_id)
        if robot is None:
            return
        try:
            code = RobotStateCode(state)
        except ValueError:
            code = RobotStateCode.UNKNOWN
        if not robot.online:
            self._log.info(f'{robot_id} is online ({code.name})')
        if code == RobotStateCode.IDLE and robot.state != RobotStateCode.IDLE:
            robot.idle_since = now
        robot.state = code
        robot.pose = pose
        robot.battery_percent = battery_percent
        robot.last_seen = now
        robot.online = True

    def dispatch(self, station_id: str, destination_id: str = '') -> DispatchResult:
        """Send the AMR docked at `station_id` to a drop-off (the GTP 'dispatch' button)."""
        station = self._site.stations.get(station_id)
        if station is None:
            return DispatchResult(False, f'unknown station {station_id!r}')
        destination_id = destination_id or station.default_destination
        if destination_id not in self._site.destinations:
            return DispatchResult(False, f'unknown destination {destination_id!r}')

        robot_id = self._reservations.holder(dock_key(station_id))
        if robot_id is None:
            return DispatchResult(False, f'no AMR is assigned to {station_id}')
        robot = self._robots[robot_id]
        if robot.task is not None:
            return DispatchResult(
                False, f'{robot_id} is busy with {robot.task.task_type.name} '
                f'({robot.task.task_id})', robot_id=robot_id)
        if not robot.online or robot.faulted or robot.state not in self.AT_DOCK_STATES:
            return DispatchResult(
                False, f'{robot_id} is not ready at {station_id} (state {robot.state.name}, '
                f'online={robot.online}, faulted={robot.faulted})', robot_id=robot_id)

        task = self._new_task(TaskType.DELIVER, robot, dest_key(destination_id))
        if self._submit(task):
            message = f'{robot_id} is on its way to {destination_id}'
        else:
            holder = self._reservations.holder(task.target_resource)
            message = f'{robot_id} will leave as soon as {holder} has left {destination_id}'
        return DispatchResult(True, message, task.task_id, robot_id)

    def tick(self, now: float) -> None:
        """Run one allocation cycle; the node calls this periodically (e.g. at 2 Hz)."""
        self._check_heartbeats(now)
        self._release_cleared_places()
        self._adopt_unplaced_robots()
        self._send_finished_robots_home(now)
        self._start_pending_tasks()
        self._replenish_stations(now)

    def on_task_accepted(self, task_id: str) -> None:
        """Handle the robot accepting a goal: it now owns its target place."""
        task = self._tasks.get(task_id)
        if task is None:
            return
        task.status = TaskStatus.ACTIVE
        self._robots[task.robot_id].anchor = task.target_resource

    def on_task_rejected(self, task_id: str, reason: str, now: float) -> None:
        """Handle a goal that never reached or was refused by the robot; retry later."""
        task = self._tasks.pop(task_id, None)
        if task is None:
            return
        robot = self._robots[task.robot_id]
        robot.task = None
        robot.cooldown_until = now + self._cfg.retry_cooldown_s
        if task.target_resource != robot.anchor:
            self._reservations.release(task.target_resource, robot.robot_id)
        self._log.warning(f'{task.task_id}: {robot.robot_id} rejected '
                          f'{task.task_type.name}: {reason}')

    def on_task_finished(self, task_id: str, success: bool, message: str) -> None:
        """Handle the final result of a goal the robot had accepted."""
        task = self._tasks.pop(task_id, None)
        if task is None:
            return
        robot = self._robots[task.robot_id]
        robot.task = None
        if success:
            robot.anchor = task.target_resource
            self._log.info(f'{task.task_id}: {robot.robot_id} finished '
                           f'{task.task_type.name} at {task.target_id}')
            return
        robot.faulted = True
        self._log.error(
            f'{task.task_id}: {robot.robot_id} failed {task.task_type.name}: {message}. '
            f'It keeps {self._reservations.held_by(robot.robot_id)} and gets no more work; '
            'clear the problem on the floor, then restart the fleet manager.')

    # ---------------------------------------------------------- tick stages

    def _check_heartbeats(self, now: float) -> None:
        for robot in self._robots.values():
            if robot.online and now - robot.last_seen > self._cfg.heartbeat_timeout_s:
                robot.online = False
                self._log.warning(f'{robot.robot_id} went offline; it keeps '
                                  f'{self._reservations.held_by(robot.robot_id)}')

    def _release_cleared_places(self) -> None:
        """Free the places robots have driven away from (rule 2)."""
        for robot in self._robots.values():
            if not robot.online or robot.faulted:
                continue
            keep = {robot.anchor, robot.task.target_resource if robot.task else None}
            for resource in self._reservations.held_by(robot.robot_id):
                if resource in keep:
                    continue
                distance = robot.pose.distance_to(self._site.resource_pose(resource))
                if distance > self._cfg.clearance_m:
                    self._reservations.release(resource, robot.robot_id)
                    self._log.info(f'{robot.robot_id} has cleared {resource}')

    def _adopt_unplaced_robots(self) -> None:
        """Claim the slot or dock a robot is already at (e.g. after a fleet manager restart)."""
        for robot in self._robots.values():
            if (not robot.online or robot.faulted or robot.task is not None
                    or robot.anchor is not None):
                continue
            if robot.state == RobotStateCode.IDLE:
                candidates = [slot_key(slot_id) for slot_id in self._site.queue_slots]
            elif robot.state in self.AT_DOCK_STATES:
                candidates = [dock_key(station_id) for station_id in self._site.stations]
            else:
                candidates = []
            place = self._nearest_free(robot.pose, candidates)
            if (place is not None and robot.pose.distance_to(self._site.resource_pose(place))
                    <= self._cfg.match_tolerance_m):
                self._reservations.try_acquire(place, robot.robot_id)
                robot.anchor = place
                robot.warned_unmanaged = False
                self._log.info(f'{robot.robot_id} found at {place}')
            elif robot.state != RobotStateCode.IDLE and not robot.warned_unmanaged:
                # IDLE robots away from a slot are sent home by the next stage.
                robot.warned_unmanaged = True
                self._log.warning(f'{robot.robot_id} is {robot.state.name} away from any free '
                                  'dock or slot; waiting until it reports IDLE')

    def _send_finished_robots_home(self, now: float) -> None:
        """Send IDLE robots that are at a drop-off, or nowhere known, back to the queue."""
        slots = [slot_key(slot_id) for slot_id in self._site.queue_slots]
        for robot in self._robots.values():
            if (not robot.online or robot.faulted or robot.task is not None
                    or robot.state != RobotStateCode.IDLE or now < robot.cooldown_until
                    or resource_kind(robot.anchor) not in ('', 'dest')):
                continue
            slot = self._nearest_free(robot.pose, slots)
            if slot is not None:
                self._submit(self._new_task(TaskType.RETURN_TO_QUEUE, robot, slot))

    def _start_pending_tasks(self) -> None:
        for task in list(self._pending):
            if not self._robots[task.robot_id].online:
                continue
            if self._reservations.try_acquire(task.target_resource, task.robot_id):
                self._pending.remove(task)
                self._start(task)

    def _replenish_stations(self, now: float) -> None:
        """Call an idle AMR to every GTP dock that has none (step 1 of the cycle)."""
        for station in self._site.stations.values():
            dock = dock_key(station.station_id)
            if not station.auto_replenish or self._reservations.holder(dock) is not None:
                continue
            candidates = [r for r in self._robots.values() if self._is_available(r, now)]
            if not candidates:
                continue
            robot = self._select(candidates, station.dock_pose)
            self._submit(self._new_task(TaskType.GO_TO_STATION, robot, dock))

    # --------------------------------------------------------------- helpers

    def _is_available(self, robot: RobotRecord, now: float) -> bool:
        """Return True if `robot` is parked in the queue and fit for new work."""
        return (robot.online and not robot.faulted and robot.task is None
                and robot.state == RobotStateCode.IDLE
                and resource_kind(robot.anchor) == 'slot'
                and robot.battery_percent >= self._cfg.min_battery_percent
                and now >= robot.cooldown_until)

    def _select(self, candidates: list[RobotRecord], target: Pose2D) -> RobotRecord:
        """Pick one robot according to the selection policy."""
        if self._cfg.selection_policy == 'nearest':
            # Straight-line distance; Nav2's ComputePathToPose gives the real path length.
            return min(candidates, key=lambda r: (r.pose.distance_to(target), r.idle_since))
        return min(candidates, key=lambda r: r.idle_since)

    def _nearest_free(self, pose: Pose2D, keys: list[str]) -> str | None:
        free = [key for key in keys if self._reservations.holder(key) is None]
        if not free:
            return None
        return min(free, key=lambda key: pose.distance_to(self._site.resource_pose(key)))

    def _new_task(self, task_type: TaskType, robot: RobotRecord, resource: str) -> Task:
        task_id = f'{task_type.name.lower()}-{uuid.uuid4().hex[:8]}'
        target_id = resource.partition(':')[2]   # 'dock:gtp_1' -> 'gtp_1'
        return Task(task_id, task_type, robot.robot_id, target_id, resource,
                    self._site.resource_pose(resource))

    def _submit(self, task: Task) -> bool:
        """Give `task` to its robot; start it now if the target is free. Return True if started."""
        self._robots[task.robot_id].task = task
        self._tasks[task.task_id] = task
        if self._reservations.try_acquire(task.target_resource, task.robot_id):
            self._start(task)
            return True
        self._pending.append(task)
        self._log.info(f'{task.task_id}: {task.robot_id} waits for {task.target_resource} '
                       f'(held by {self._reservations.holder(task.target_resource)})')
        return False

    def _start(self, task: Task) -> None:
        task.status = TaskStatus.SENT
        self._outbox.append(task)
        self._log.info(f'{task.task_id}: {task.robot_id} -> {task.task_type.name} '
                       f'{task.target_id}')
