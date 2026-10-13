# GTP AMR Fleet (ROS 2)

A prototype fleet of 1–3 autonomous mobile robots (designed to grow to 10+) that carry
picked items from a Goods-to-Person (GTP) pick station to a sorting / consolidation area.

```
 ready queue ──► GTP dock ──► operator loads ──► DISPATCH ──► drop-off ──► worker unloads ──┐
      ▲                                                                                     │
      └─────────────────────────────────── return ───────────────────────────────────────────┘
```

Design details, the robot state list and the deadlock rules are in
[docs/architecture.md](docs/architecture.md).

## Workspace layout

This repository is the `src/` folder of a colcon workspace (for example `~/amr_ws/src`).

| Package | Build type | Runs on | Status | Responsibility |
|---|---|---|---|---|
| `amr_interfaces` | ament_cmake | everywhere | ✅ done | The contract: messages, services and actions shared by all other packages |
| `amr_fleet_manager` | ament_python | fleet server | ✅ done | Task allocation, dock / drop-off / queue reservations, mock AMR for testing |
| `amr_task_executor` | ament_python | each robot | ⏳ next | Mission state machine; serves `execute_task` using Nav2 and the docking controller |
| `amr_docking` | ament_cmake (C++) | each robot | ⏳ planned | Millimetre-level alignment beside the GTP station |
| `amr_navigation` | ament_cmake (config only) | each robot | ⏳ planned | Nav2 parameters, maps, behavior trees |
| `amr_description` | ament_cmake | each robot | ⏳ planned | URDF / xacro, sensors, Gazebo plugins |
| `amr_hardware_interface` | ament_cmake (C++) | each robot | ⏳ planned | ros2_control diff-drive, battery, e-stop, bin sensors |
| `amr_bringup` | ament_cmake | both | ⏳ planned | Launch files: one namespaced robot, the fleet server, a multi-robot Gazebo world |
| `amr_station_hmi` | ament_python | GTP station PC | ⏳ planned | Touchscreen / button that calls `/fleet/dispatch_amr` |

Dependency rule: every package may depend on `amr_interfaces`, and no robot-side
package depends on `amr_fleet_manager`. Robots and the fleet manager only talk through
the interfaces below, so either side can be replaced later (for example by Open-RMF).

## Interfaces

| Name | Type | Direction |
|---|---|---|
| `/fleet/dispatch_amr` | `amr_interfaces/srv/DispatchAmr` | station HMI → fleet manager |
| `/<robot_id>/robot_state` | `amr_interfaces/msg/RobotState` | robot → fleet manager, ≥ 1 Hz (also the heartbeat) |
| `/<robot_id>/execute_task` | `amr_interfaces/action/ExecuteTask` | fleet manager → robot, one goal per leg |

## Quick start: simulated fleet, no Gazebo needed

Tested on ROS 2 Jazzy. The code only uses APIs available from Humble to Lyrical.

```bash
mkdir -p ~/amr_ws
git clone https://github.com/mm45561/me61.git ~/amr_ws/src
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_fleet_manager fleet_sim.launch.py
```

The fleet manager immediately calls the nearest parked robot (`amr_1`) to `gtp_1`.
After about 10 s it reports `DOCKED_AT_GTP`. Then, in a second terminal:

```bash
source ~/amr_ws/install/setup.bash
ros2 topic echo /amr_1/robot_state --field state      # 3 = DOCKED_AT_GTP
ros2 service call /fleet/dispatch_amr amr_interfaces/srv/DispatchAmr "{station_id: gtp_1}"
```

`amr_1` drives to `sort_1`. As soon as it is 1 m clear of the dock, `amr_2` is called
to replace it. `amr_1` unloads and then returns to a free queue slot.

## Tests

```bash
cd ~/amr_ws
colcon test --packages-select amr_fleet_manager && colcon test-result --verbose
```

The allocation logic does not import ROS, so its tests also run on their own:
`cd ~/amr_ws/src/amr_fleet_manager && python3 -m pytest test/test_fleet_core.py`.
