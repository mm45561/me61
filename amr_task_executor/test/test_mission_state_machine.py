"""Tests for MissionStateMachine (no ROS needed)."""

from amr_task_executor.mission_state_machine import MissionStateMachine, State


def walk(sm, *triggers):
    """Fire the triggers in order and check that each one is accepted."""
    for trigger in triggers:
        assert sm.fire(trigger), f'{trigger!r} refused in state {sm.state.name}'


def test_starts_idle():
    assert MissionStateMachine().state == State.IDLE


def test_full_gtp_cycle():
    sm = MissionStateMachine()
    walk(sm, 'go_to_station')
    assert sm.state == State.TO_GTP
    walk(sm, 'arrived')
    assert sm.state == State.DOCKED_AT_GTP
    walk(sm, 'start_loading')
    assert sm.state == State.LOADING
    walk(sm, 'deliver')
    assert sm.state == State.IN_TRANSIT
    walk(sm, 'arrived')
    assert sm.state == State.UNLOADING
    walk(sm, 'unloaded')
    assert sm.state == State.IDLE
    walk(sm, 'return_to_queue')
    assert sm.state == State.RETURNING
    walk(sm, 'arrived')
    assert sm.state == State.IDLE


def test_dispatch_before_loading_was_started():
    sm = MissionStateMachine()
    walk(sm, 'go_to_station', 'arrived', 'deliver')
    assert sm.state == State.IN_TRANSIT


def test_refused_trigger_keeps_the_state():
    sm = MissionStateMachine()
    assert not sm.can('deliver')          # an empty robot in the queue cannot deliver
    assert not sm.fire('deliver')
    assert not sm.fire('arrived')         # it is not driving anywhere
    assert not sm.fire('no_such_trigger')
    assert sm.state == State.IDLE


def test_can_does_not_change_the_state():
    sm = MissionStateMachine()
    assert sm.can('go_to_station')
    assert sm.state == State.IDLE


def test_failure_on_every_driving_leg_then_reset():
    legs = [
        ['go_to_station'],
        ['go_to_station', 'arrived', 'deliver'],
        ['return_to_queue'],
    ]
    for leg in legs:
        sm = MissionStateMachine()
        walk(sm, *leg, 'fail')
        assert sm.state == State.ERROR
        assert not sm.fire('go_to_station')   # no new work until a human resets it
        walk(sm, 'reset')
        assert sm.state == State.IDLE


def test_cannot_fail_while_parked():
    assert not MissionStateMachine().fire('fail')
