"""
Exclusive reservations of shared floor space.

Every place where only one AMR fits at a time (a GTP dock, a drop-off point,
a parking slot) is a named resource. A robot is only sent to a place after it
holds that place's reservation, so two robots are never sent to the same spot.
The same table can later hold traffic zones, e.g. the single-lane aisle in
front of a GTP station.
"""

from __future__ import annotations


class ReservationTable:
    """Map each resource name to the robot that currently holds it."""

    def __init__(self) -> None:
        self._holders: dict[str, str] = {}

    def holder(self, resource: str) -> str | None:
        """Return the robot holding `resource`, or None if it is free."""
        return self._holders.get(resource)

    def try_acquire(self, resource: str, robot_id: str) -> bool:
        """Reserve `resource` for `robot_id`; return False if another robot holds it."""
        current = self._holders.get(resource)
        if current is not None and current != robot_id:
            return False
        self._holders[resource] = robot_id
        return True

    def release(self, resource: str, robot_id: str) -> None:
        """Free `resource` if `robot_id` holds it; do nothing otherwise."""
        if self._holders.get(resource) == robot_id:
            del self._holders[resource]

    def held_by(self, robot_id: str) -> list[str]:
        """Return the resources held by `robot_id`, sorted by name."""
        return sorted(res for res, holder in self._holders.items() if holder == robot_id)
