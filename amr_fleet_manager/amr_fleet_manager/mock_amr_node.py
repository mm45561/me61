"""
Simulated AMR for testing the fleet manager without Gazebo or hardware.

It serves /<namespace>/execute_task, "drives" in a straight line at a fixed
speed and publishes /<namespace>/robot_state. It plays the part of the
on-robot task executor (the state machine) that will later drive Nav2 and the
docking controller instead of faking the motion.
"""

import math
import threading
import time

from amr_fleet_manager.fleet_core import RobotStateCode, TaskType
from amr_fleet_manager.geometry import Pose2D, quaternion_to_yaw, yaw_to_quaternion
from amr_interfaces.action import ExecuteTask
from amr_interfaces.msg import RobotState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class _Canceled(Exception):
    """Raised inside a task when the fleet manager cancels the goal."""


class MockAmr(Node):
    """Pretend to be one AMR: accept one task at a time and fake the motion."""

    STEP_S = 0.1

    def __init__(self) -> None:
        super().__init__('mock_amr')
        self._robot_id = (self.declare_parameter('robot_id', '').value
                          or self.get_namespace().strip('/') or 'amr')
        x, y, yaw = self.declare_parameter('initial_pose', [0.0, 0.0, 0.0]).value
        self._speed = self.declare_parameter('speed_mps', 0.5).value
        self._dock_time = self.declare_parameter('dock_time_s', 2.0).value
        self._unload_time = self.declare_parameter('unload_time_s', 4.0).value
        rate = self.declare_parameter('publish_rate_hz', 2.0).value

        # The action runs in its own thread (MultiThreadedExecutor), the state timer in
        # another one, so everything they share is guarded by this lock.
        self._lock = threading.Lock()
        self._pose = Pose2D(x, y, yaw)
        self._state = RobotStateCode.IDLE
        self._task_id = ''
        self._battery = 100.0
        self._busy = False

        self._state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.create_timer(1.0 / rate, self._publish_state)
        self._server = ActionServer(
            self, ExecuteTask, 'execute_task',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'mock {self._robot_id} ready at {self._pose}')

    def _on_goal(self, goal: ExecuteTask.Goal) -> GoalResponse:
        if goal.task_type not in {int(t) for t in TaskType}:
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> ExecuteTask.Result:
        goal = goal_handle.request
        task_type = TaskType(goal.task_type)
        q = goal.target_pose.pose.orientation
        target = Pose2D(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
                        quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self.get_logger().info(f'{goal.task_id}: {task_type.name} -> {goal.target_id}')
        result = ExecuteTask.Result()
        with self._lock:
            self._task_id = goal.task_id
        try:
            if task_type == TaskType.GO_TO_STATION:
                self._drive_to(goal_handle, target, RobotStateCode.TO_GTP)
                self._wait(goal_handle, self._dock_time)    # stands in for precision docking
                self._set_state(RobotStateCode.DOCKED_AT_GTP)
            elif task_type == TaskType.DELIVER:
                self._drive_to(goal_handle, target, RobotStateCode.IN_TRANSIT)
                self._set_state(RobotStateCode.UNLOADING)
                self._wait(goal_handle, self._unload_time)  # stands in for the worker
                self._set_state(RobotStateCode.IDLE)
            else:
                self._drive_to(goal_handle, target, RobotStateCode.RETURNING)
                self._set_state(RobotStateCode.IDLE)
        except _Canceled:
            self._set_state(RobotStateCode.IDLE)
            goal_handle.canceled()
            result.message = 'canceled'
            return result
        finally:
            with self._lock:
                self._task_id = ''
                self._busy = False
        goal_handle.succeed()
        result.success = True
        result.message = f'{task_type.name} done at {goal.target_id}'
        return result

    def _drive_to(self, goal_handle, target: Pose2D, state: RobotStateCode) -> None:
        self._set_state(state)
        feedback = ExecuteTask.Feedback()
        feedback.state = int(state)
        step = self._speed * self.STEP_S
        while True:
            if goal_handle.is_cancel_requested:
                raise _Canceled()
            with self._lock:
                pose = self._pose
                remaining = pose.distance_to(target)
                if remaining <= step:
                    self._pose = target
                    return
                ratio = step / remaining
                heading = math.atan2(target.y - pose.y, target.x - pose.x)
                self._pose = Pose2D(pose.x + (target.x - pose.x) * ratio,
                                    pose.y + (target.y - pose.y) * ratio, heading)
                self._battery = max(0.0, self._battery - 0.02 * step)
            feedback.distance_remaining = float(remaining)
            goal_handle.publish_feedback(feedback)
            time.sleep(self.STEP_S)

    def _wait(self, goal_handle, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                raise _Canceled()
            time.sleep(self.STEP_S)

    def _set_state(self, state: RobotStateCode) -> None:
        with self._lock:
            self._state = state
        self._publish_state()   # publish changes right away, not only on the timer

    def _publish_state(self) -> None:
        with self._lock:
            pose, state, task_id, battery = self._pose, self._state, self._task_id, self._battery
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_id = self._robot_id
        msg.state = int(state)
        msg.task_id = task_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quaternion(pose.yaw)
        msg.battery_percent = battery
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockAmr()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
