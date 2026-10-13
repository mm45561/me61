"""
On-robot task executor: runs ExecuteTask goals through the MissionStateMachine.

Motion is still simulated (straight line at a fixed speed). The next step replaces
_drive_to() with a Nav2 NavigateToPose call; everything else stays the same.
"""

import math
import threading
import time

from amr_interfaces.action import ExecuteTask
from amr_interfaces.msg import RobotState
from amr_task_executor.mission_state_machine import MissionStateMachine
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

# ExecuteTask.task_type -> trigger of the state machine
TASK_TRIGGERS = {
    ExecuteTask.Goal.GO_TO_STATION: 'go_to_station',
    ExecuteTask.Goal.DELIVER: 'deliver',
    ExecuteTask.Goal.RETURN_TO_QUEUE: 'return_to_queue',
}


class ExecutorNode(Node):
    """Accept only the tasks the state machine allows, and report the state."""

    def __init__(self) -> None:
        super().__init__('task_executor')
        self._robot_id = (self.declare_parameter('robot_id', '').value
                          or self.get_namespace().strip('/') or 'amr')
        x, y, yaw = self.declare_parameter('initial_pose', [0.0, 0.0, 0.0]).value
        self._speed = self.declare_parameter('speed_mps', 0.5).value
        self._unload_time = self.declare_parameter('unload_time_s', 4.0).value

        self._lock = threading.Lock()   # the action thread and the timer share the data
        self._sm = MissionStateMachine()
        self._x, self._y, self._yaw = x, y, yaw
        self._task_id = ''
        self._busy = False

        self._state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.create_timer(0.5, self._publish_state)
        self._server = ActionServer(
            self, ExecuteTask, 'execute_task',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'{self._robot_id} executor ready ({self._sm.state.name})')

    def _on_goal(self, goal: ExecuteTask.Goal) -> GoalResponse:
        trigger = TASK_TRIGGERS.get(goal.task_type)
        with self._lock:
            if trigger is None or self._busy or not self._sm.can(trigger):
                self.get_logger().warning(
                    f'rejected {goal.task_id}: not allowed in {self._sm.state.name}')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> ExecuteTask.Result:
        goal = goal_handle.request
        trigger = TASK_TRIGGERS[goal.task_type]
        result = ExecuteTask.Result()
        self.get_logger().info(f'{goal.task_id}: {trigger} -> {goal.target_id}')
        self._fire(trigger, goal.task_id)
        try:
            ok = self._drive_to(goal.target_pose.pose.position.x,
                                goal.target_pose.pose.position.y)
            if not ok:
                self._fire('fail')
                goal_handle.abort()
                result.message = 'driving failed'
                return result
            self._fire('arrived')
            if trigger == 'deliver':
                time.sleep(self._unload_time)   # stands in for the worker unloading
                self._fire('unloaded')
        finally:
            with self._lock:
                self._busy = False
                self._task_id = ''
        goal_handle.succeed()
        result.success = True
        result.message = f'{trigger} done, now {self._sm.state.name}'
        return result

    def _drive_to(self, x: float, y: float) -> bool:
        """Drive in a straight line (simulated); return False on failure."""
        step = self._speed * 0.1
        while True:
            with self._lock:
                dx, dy = x - self._x, y - self._y
                distance = math.hypot(dx, dy)
                if distance <= step:
                    self._x, self._y = x, y
                    return True
                self._x += dx / distance * step
                self._y += dy / distance * step
                self._yaw = math.atan2(dy, dx)
            time.sleep(0.1)

    def _fire(self, trigger: str, task_id: str | None = None) -> None:
        with self._lock:
            before = self._sm.state.name
            self._sm.fire(trigger)
            if task_id is not None:
                self._task_id = task_id
        self.get_logger().info(f'{before} --{trigger}--> {self._sm.state.name}')
        self._publish_state()

    def _publish_state(self) -> None:
        msg = RobotState()
        with self._lock:
            msg.state = int(self._sm.state)
            msg.task_id = self._task_id
            msg.pose.position.x = self._x
            msg.pose.position.y = self._y
            msg.pose.orientation.z = math.sin(self._yaw / 2.0)
            msg.pose.orientation.w = math.cos(self._yaw / 2.0)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_id = self._robot_id
        msg.battery_percent = 100.0
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
