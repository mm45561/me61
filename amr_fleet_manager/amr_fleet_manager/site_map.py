"""
Floor layout: GTP stations, drop-off destinations and ready-queue slots.

The layout is read from its own YAML file (config/site.yaml) because ROS 2
parameters cannot hold dictionaries of dictionaries; the node only receives
the file path as a parameter.

Every place where only one AMR fits at a time is also a reservation key:
'dock:<station_id>', 'dest:<destination_id>' and 'slot:<slot_id>'.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from amr_fleet_manager.geometry import Pose2D
import yaml


def dock_key(station_id: str) -> str:
    """Return the reservation key of a GTP station's dock."""
    return f'dock:{station_id}'


def dest_key(destination_id: str) -> str:
    """Return the reservation key of a drop-off destination."""
    return f'dest:{destination_id}'


def slot_key(slot_id: str) -> str:
    """Return the reservation key of a ready-queue slot."""
    return f'slot:{slot_id}'


def resource_kind(key: str | None) -> str:
    """Return 'dock', 'dest' or 'slot' for a reservation key ('' for None)."""
    return key.partition(':')[0] if key else ''


@dataclass(frozen=True)
class Station:
    """A GTP pick station with one AMR dock beside it."""

    station_id: str
    dock_pose: Pose2D
    default_destination: str
    auto_replenish: bool = True   # keep an AMR waiting at this dock at all times


@dataclass(frozen=True)
class Destination:
    """A drop-off point in the sorting / consolidation area."""

    destination_id: str
    pose: Pose2D


@dataclass(frozen=True)
class QueueSlot:
    """A parking bay in the ready / idle queue."""

    slot_id: str
    pose: Pose2D


@dataclass(frozen=True)
class Site:
    """Everything the fleet manager knows about the floor layout."""

    frame_id: str
    stations: dict[str, Station]
    destinations: dict[str, Destination]
    queue_slots: dict[str, QueueSlot]

    def resource_pose(self, key: str) -> Pose2D:
        """Return the pose of the place behind a reservation key."""
        kind, _, name = key.partition(':')
        if kind == 'dock':
            return self.stations[name].dock_pose
        if kind == 'dest':
            return self.destinations[name].pose
        if kind == 'slot':
            return self.queue_slots[name].pose
        raise KeyError(f'unknown reservation key {key!r}')


def _mapping(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f'{where}: expected a mapping, got {value!r}')
    return value


def _pose(value: Any, where: str) -> Pose2D:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{where}: expected [x, y, yaw], got {value!r}')
    x, y, yaw = (float(v) for v in value)
    return Pose2D(x, y, yaw)


def parse_site(data: Any) -> Site:
    """Build a Site from an already-parsed YAML document and validate it."""
    data = _mapping(data, 'site file')

    destinations = {}
    for name, entry in _mapping(data.get('destinations', {}), 'destinations').items():
        entry = _mapping(entry, f'destinations.{name}')
        destinations[name] = Destination(
            name, _pose(entry.get('pose'), f'destinations.{name}.pose'))

    stations = {}
    for name, entry in _mapping(data.get('stations', {}), 'stations').items():
        entry = _mapping(entry, f'stations.{name}')
        default_destination = str(entry.get('default_destination', ''))
        if default_destination not in destinations:
            raise ValueError(
                f'stations.{name}.default_destination: unknown destination '
                f'{default_destination!r}')
        stations[name] = Station(
            name,
            _pose(entry.get('dock_pose'), f'stations.{name}.dock_pose'),
            default_destination,
            bool(entry.get('auto_replenish', True)))

    slots = {}
    for name, entry in _mapping(data.get('queue_slots', {}), 'queue_slots').items():
        entry = _mapping(entry, f'queue_slots.{name}')
        slots[name] = QueueSlot(name, _pose(entry.get('pose'), f'queue_slots.{name}.pose'))

    if not stations:
        raise ValueError('the site needs at least one station')
    if not slots:
        raise ValueError('the site needs at least one queue slot')
    return Site(str(data.get('frame_id', 'map')), stations, destinations, slots)


def load_site(path: str) -> Site:
    """Read and validate a site YAML file."""
    with open(path, encoding='utf-8') as stream:
        return parse_site(yaml.safe_load(stream))
