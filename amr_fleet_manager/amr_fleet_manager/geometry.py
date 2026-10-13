"""Small 2D geometry helpers shared by the fleet manager and the mock robot."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the map frame: x and y in metres, yaw in radians."""

    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, other: Pose2D) -> float:
        """Return the straight-line distance to another pose [m]."""
        return math.hypot(self.x - other.x, self.y - other.y)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) of a rotation of `yaw` radians about +Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the rotation about +Z of a quaternion [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
