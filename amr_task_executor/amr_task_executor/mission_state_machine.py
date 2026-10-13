"""
Mission state machine of one AMR, without ROS.

TRANSITIONS lists every allowed move: (current state, trigger) -> next state.
Anything that is not in the table is not allowed, and fire() refuses it.
"""

import enum


class State(enum.IntEnum):
    """Robot states; the values match amr_interfaces/msg/RobotState."""

    UNKNOWN = 0
    IDLE = 1
    TO_GTP = 2
    DOCKED_AT_GTP = 3
    LOADING = 4
    IN_TRANSIT = 5
    UNLOADING = 6
    RETURNING = 7
    ERROR = 8


# (current state, trigger) -> next state
TRANSITIONS: dict[tuple[State, str], State] = {
    (State.IDLE, 'go_to_station'): State.TO_GTP,
    (State.TO_GTP, 'arrived'): State.DOCKED_AT_GTP,
    (State.DOCKED_AT_GTP, 'start_loading'): State.LOADING,
    (State.DOCKED_AT_GTP, 'deliver'): State.IN_TRANSIT,
    (State.LOADING, 'deliver'): State.IN_TRANSIT,
    (State.IN_TRANSIT, 'arrived'): State.UNLOADING,
    (State.UNLOADING, 'unloaded'): State.IDLE,
    (State.IDLE, 'return_to_queue'): State.RETURNING,
    (State.RETURNING, 'arrived'): State.IDLE,
    (State.TO_GTP, 'fail'): State.ERROR,
    (State.IN_TRANSIT, 'fail'): State.ERROR,
    (State.RETURNING, 'fail'): State.ERROR,
    (State.ERROR, 'reset'): State.IDLE,
}


class MissionStateMachine:
    """Track the state of one robot and allow only the moves in TRANSITIONS."""

    def __init__(self) -> None:
        self.state = State.IDLE

    def can(self, trigger: str) -> bool:
        """Return True if `trigger` is allowed in the current state."""
        return (self.state, trigger) in TRANSITIONS

    def fire(self, trigger: str) -> bool:
        """Apply `trigger` if it is allowed; otherwise return False and keep the state."""
        next_state = TRANSITIONS.get((self.state, trigger))
        if next_state is None:
            return False
        self.state = next_state
        return True