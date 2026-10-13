"""Unit tests for FleetCore: no ROS needed, run with `python3 -m pytest test/`."""

from amr_fleet_manager.fleet_core import FleetConfig, FleetCore, RobotStateCode, TaskType
from amr_fleet_manager.geometry import Pose2D
from amr_fleet_manager.site_map import parse_site
import pytest

S = RobotStateCode

SITE = parse_site({
    'frame_id': 'map',
    'stations': {'gtp_1': {'dock_pose': [0.0, 2.0, 0.0], 'default_destination': 'sort_1'}},
    'destinations': {
        'sort_1': {'pose': [2.0, -2.0, 0.0]},
        'sort_2': {'pose': [0.0, -2.0, 0.0]},
    },
    'queue_slots': {
        'q1': {'pose': [-2.0, 1.0, 0.0]},
        'q2': {'pose': [-2.0, 0.0, 0.0]},
        'q3': {'pose': [-2.0, -1.0, 0.0]},
    },
})
SLOTS = {'amr_1': (-2.0, 1.0), 'amr_2': (-2.0, 0.0), 'amr_3': (-2.0, -1.0)}


class FleetSim:
    """Drive FleetCore like the ROS node does; robots re-send their last state every tick."""

    def __init__(self, robot_ids=('amr_1', 'amr_2', 'amr_3'), **config):
        self.core = FleetCore(SITE, list(robot_ids), FleetConfig(**config))
        self.now = 0.0
        self._last = {}

    def report(self, robot_id, state, x, y, battery=100.0):
        self._last[robot_id] = (state, Pose2D(x, y), battery)
        self.core.update_robot(robot_id, state, Pose2D(x, y), battery, self.now)

    def silence(self, robot_id):
        del self._last[robot_id]

    def tick(self, dt=0.5):
        self.now += dt
        for robot_id, (state, pose, battery) in self._last.items():
            self.core.update_robot(robot_id, state, pose, battery, self.now)
        self.core.tick(self.now)
        return self.core.pop_outbox()

    def park_all(self, **battery):
        for robot_id, (x, y) in SLOTS.items():
            self.report(robot_id, S.IDLE, x, y, battery.get(robot_id, 100.0))

    def holder(self, resource):
        return self.core.reservations.holder(resource)


def only(tasks):
    assert len(tasks) == 1, tasks
    return tasks[0]


def dock_first_robot(sim):
    """Park all robots, let amr_1 reach the dock, and return its GO_TO_STATION task."""
    sim.park_all()
    task = only(sim.tick())
    sim.core.on_task_accepted(task.task_id)
    sim.core.on_task_finished(task.task_id, True, '')
    sim.report('amr_1', S.DOCKED_AT_GTP, 0.0, 2.0)
    return task


def test_nearest_parked_robot_is_called_to_the_free_dock():
    sim = FleetSim()
    sim.park_all()
    task = only(sim.tick())
    assert (task.robot_id, task.task_type, task.target_id) == (
        'amr_1', TaskType.GO_TO_STATION, 'gtp_1')
    assert sim.holder('dock:gtp_1') == 'amr_1'
    assert sim.holder('slot:q2') == 'amr_2'
    assert sim.tick() == []   # one dock, one robot: nobody else is sent


def test_longest_idle_policy():
    sim = FleetSim(selection_policy='longest_idle')
    sim.report('amr_3', S.IDLE, *SLOTS['amr_3'])
    sim.now = 1.0
    sim.report('amr_1', S.IDLE, *SLOTS['amr_1'])
    assert only(sim.tick()).robot_id == 'amr_3'


def test_dispatch_is_rejected_without_a_ready_robot():
    sim = FleetSim()
    assert not sim.core.dispatch('gtp_9').accepted
    assert not sim.core.dispatch('gtp_1').accepted   # nothing docked yet
    sim.park_all()
    sim.tick()                                        # amr_1 is on its way
    result = sim.core.dispatch('gtp_1')
    assert not result.accepted and 'busy' in result.message
    assert not sim.core.dispatch('gtp_1', 'nowhere').accepted


def test_full_gtp_cycle():
    sim = FleetSim()
    dock_first_robot(sim)

    # Operator presses "dispatch": amr_1 leaves for the default drop-off.
    result = sim.core.dispatch('gtp_1')
    assert result.accepted and result.robot_id == 'amr_1'
    deliver = only(sim.core.pop_outbox())
    assert (deliver.task_type, deliver.target_id) == (TaskType.DELIVER, 'sort_1')
    sim.core.on_task_accepted(deliver.task_id)

    # Still beside the station: the dock stays reserved, nobody else is called.
    assert sim.tick() == []
    assert sim.holder('dock:gtp_1') == 'amr_1'

    # Once amr_1 is clear of the dock, the next nearest robot is called.
    sim.report('amr_1', S.IN_TRANSIT, 1.5, 0.5)
    call = only(sim.tick())
    assert (call.robot_id, call.task_type) == ('amr_2', TaskType.GO_TO_STATION)
    assert sim.holder('dock:gtp_1') == 'amr_2'

    # amr_1 unloads, reports IDLE at the drop-off and is sent to a free slot.
    sim.core.on_task_finished(deliver.task_id, True, '')
    sim.report('amr_1', S.IDLE, 2.0, -2.0)
    home = only(sim.tick())
    assert (home.robot_id, home.task_type, home.target_id) == (
        'amr_1', TaskType.RETURN_TO_QUEUE, 'q1')


def test_delivery_waits_while_the_drop_off_is_occupied():
    sim = FleetSim()
    first = dock_first_robot(sim)
    assert first.robot_id == 'amr_1'
    sim.core.dispatch('gtp_1')
    deliver_1 = only(sim.core.pop_outbox())
    sim.core.on_task_accepted(deliver_1.task_id)
    sim.report('amr_1', S.UNLOADING, 2.0, -2.0)       # amr_1 is unloading at sort_1

    call = only(sim.tick())                            # amr_2 replaces it at the dock
    sim.core.on_task_accepted(call.task_id)
    sim.core.on_task_finished(call.task_id, True, '')
    sim.report('amr_2', S.LOADING, 0.0, 2.0)

    result = sim.core.dispatch('gtp_1', 'sort_1')
    assert result.accepted and 'amr_1' in result.message
    assert sim.core.pop_outbox() == []                 # queued, not sent

    sim.core.on_task_finished(deliver_1.task_id, True, '')
    sim.report('amr_1', S.IDLE, 2.0, -2.0)
    home = only(sim.tick())
    sim.core.on_task_accepted(home.task_id)
    sim.report('amr_1', S.RETURNING, 0.0, -0.5)        # 2.5 m away from sort_1
    deliver_2 = only(sim.tick())
    assert (deliver_2.robot_id, deliver_2.target_id) == ('amr_2', 'sort_1')


def test_offline_robot_gets_no_work():
    sim = FleetSim()
    sim.park_all()
    sim.silence('amr_1')
    assert only(sim.tick(dt=4.0)).robot_id == 'amr_2'


def test_offline_robot_keeps_its_reservations():
    sim = FleetSim()
    sim.park_all()
    task = only(sim.tick())
    sim.core.on_task_accepted(task.task_id)
    sim.silence('amr_1')
    assert sim.tick(dt=4.0) == []
    assert not sim.core.robot('amr_1').online
    assert sim.holder('dock:gtp_1') == 'amr_1'


def test_low_battery_robot_gets_no_work():
    sim = FleetSim()
    sim.park_all(amr_1=10.0)
    assert only(sim.tick()).robot_id == 'amr_2'


def test_robot_found_away_from_the_queue_is_sent_home():
    sim = FleetSim(robot_ids=('amr_1',))
    sim.report('amr_1', S.IDLE, 0.0, 0.0)
    home = only(sim.tick())
    assert (home.task_type, home.target_id) == (TaskType.RETURN_TO_QUEUE, 'q2')


def test_robot_found_at_the_dock_can_be_dispatched():
    sim = FleetSim(robot_ids=('amr_1',))
    sim.report('amr_1', S.LOADING, 0.1, 2.0)
    assert sim.tick() == []
    assert sim.holder('dock:gtp_1') == 'amr_1'
    assert sim.core.dispatch('gtp_1').accepted


def test_rejected_goal_frees_the_dock_and_backs_off():
    sim = FleetSim()
    sim.park_all()
    task = only(sim.tick())
    sim.core.on_task_rejected(task.task_id, 'busy', sim.now)
    assert sim.core.robot('amr_1').task is None
    assert only(sim.tick()).robot_id == 'amr_2'


def test_failed_task_faults_the_robot_and_blocks_the_dock():
    sim = FleetSim()
    sim.park_all()
    task = only(sim.tick())
    sim.core.on_task_accepted(task.task_id)
    sim.core.on_task_finished(task.task_id, False, 'path blocked')
    assert sim.core.robot('amr_1').faulted
    assert sim.tick() == []
    assert sim.holder('dock:gtp_1') == 'amr_1'


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match='queue slots'):
        FleetCore(SITE, ['a', 'b', 'c', 'd'])
    with pytest.raises(ValueError, match='selection_policy'):
        FleetCore(SITE, ['a'], FleetConfig(selection_policy='random'))
    with pytest.raises(ValueError, match='unique'):
        FleetCore(SITE, ['a', 'a'])


def test_site_validation():
    with pytest.raises(ValueError, match='unknown destination'):
        parse_site({'stations': {'gtp_1': {'dock_pose': [0, 0, 0],
                                           'default_destination': 'nope'}}})
    with pytest.raises(ValueError, match=r'\[x, y, yaw\]'):
        parse_site({'destinations': {'d': {'pose': [1, 2]}}})
