"""
ROS 2 front end of FleetCore.

Interfaces, for robots that each run in their own namespace (/amr_1, /amr_2, ...):

  service  /fleet/dispatch_amr       amr_interfaces/srv/DispatchAmr      (from the GTP HMI)
  sub      /<robot_id>/robot_state   amr_interfaces/msg/RobotState       (state + heartbeat)
  action   /<robot_id>/execute_task  amr_interfaces/action/ExecuteTask   (client, one per robot)

Everything runs in one single-threaded executor, so FleetCore never sees two
callbacks at once and needs no locks. No callback blocks: goals are sent with
send_goal_async() and their answers come back as callbacks.
"""

import dataclasses
from functools import partial

from action_msgs.msg import GoalStatus
from amr_fleet_manager.fleet_core import FleetConfig, FleetCore, Task
from amr_fleet_manager.geometry import Pose2D, quaternion_to_yaw, yaw_to_quaternion
from amr_fleet_manager.site_map import load_site
from amr_interfaces.action import ExecuteTask
from amr_interfaces.msg import RobotState
from amr_interfaces.srv import DispatchAmr
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FleetManagerNode(Node):
    """Connect FleetCore to ROS: one service, N subscriptions, N action clients."""

    def __init__(self) -> None:
        super().__init__('fleet_manager')
        robot_ids = list(self.declare_parameter('robot_ids', ['amr_1']).value)
        site_file = self.declare_parameter('site_config_file', '').value
        tick_period = self.declare_parameter('tick_period_s', 0.5).value
        # One ROS parameter per FleetConfig field, with the same name and default.
        config = FleetConfig(**{
            field.name: self.declare_parameter(field.name, field.default).value
            for field in dataclasses.fields(FleetConfig)
        })
        if not site_file:
            raise RuntimeError('parameter "site_config_file" is required (path to site.yaml)')

        self._site = load_site(site_file)
        self._core = FleetCore(self._site, robot_ids, config, logger=self.get_logger())

        self._goal_clients = {}
        for robot_id in robot_ids:
            self.create_subscription(RobotState, f'/{robot_id}/robot_state',
                                     partial(self._on_robot_state, robot_id), 10)
            self._goal_clients[robot_id] = ActionClient(self, ExecuteTask,
                                                        f'/{robot_id}/execute_task')
        self.create_service(DispatchAmr, 'dispatch_amr', self._on_dispatch)
        self.create_timer(tick_period, self._on_tick)
        self.get_logger().info(
            f'fleet manager up: robots={robot_ids}, stations={list(self._site.stations)}, '
            f'policy={config.selection_policy}')

    # --------------------------------------------------------------- inputs

    def _on_robot_state(self, robot_id: str, msg: RobotState) -> None:
        if msg.robot_id and msg.robot_id != robot_id:
            self.get_logger().warning(
                f'/{robot_id}/robot_state carries robot_id={msg.robot_id!r}; check namespaces',
                throttle_duration_sec=10.0)
        q = msg.pose.orientation
        pose = Pose2D(msg.pose.position.x, msg.pose.position.y,
                      quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self._core.update_robot(robot_id, msg.state, pose, msg.battery_percent, self._now())

    def _on_dispatch(self, request: DispatchAmr.Request,
                     response: DispatchAmr.Response) -> DispatchAmr.Response:
        result = self._core.dispatch(request.station_id, request.destination_id)
        response.accepted = result.accepted
        response.task_id = result.task_id
        response.robot_id = result.robot_id
        response.message = result.message
        # Two separate calls on purpose: an rclpy logger call site must always use the
        # same severity, otherwise rclpy raises "Logger severity cannot be changed".
        if result.accepted:
            self.get_logger().info(f'dispatch_amr({request.station_id!r}): {result.message}')
        else:
            self.get_logger().warning(
                f'dispatch_amr({request.station_id!r}) rejected: {result.message}')
        self._send_outbox()
        return response

    def _on_tick(self) -> None:
        self._core.tick(self._now())
        self._send_outbox()

    # ---------------------------------------------------------- goal sending

    def _send_outbox(self) -> None:
        for task in self._core.pop_outbox():
            self._send_goal(task)

    def _send_goal(self, task: Task) -> None:
        client = self._goal_clients[task.robot_id]
        if not client.server_is_ready():
            self._core.on_task_rejected(task.task_id, 'execute_task server not available',
                                        self._now())
            return
        goal = ExecuteTask.Goal()
        goal.task_id = task.task_id
        goal.task_type = int(task.task_type)
        goal.target_id = task.target_id
        # Leave the stamp at zero ("latest transform"): the server's clock and the
        # robots' clocks are never perfectly in sync.
        goal.target_pose.header.frame_id = self._site.frame_id
        goal.target_pose.pose.position.x = task.target_pose.x
        goal.target_pose.pose.position.y = task.target_pose.y
        o = goal.target_pose.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quaternion(task.target_pose.yaw)

        future = client.send_goal_async(goal)
        future.add_done_callback(partial(self._on_goal_response, task.task_id))

    def _on_goal_response(self, task_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._core.on_task_rejected(task_id, f'send_goal failed: {exc}', self._now())
            return
        if not goal_handle.accepted:
            self._core.on_task_rejected(task_id, 'goal rejected by the robot', self._now())
            return
        self._core.on_task_accepted(task_id)
        goal_handle.get_result_async().add_done_callback(
            partial(self._on_goal_result, task_id))

    def _on_goal_result(self, task_id: str, future) -> None:
        response = future.result()
        success = response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
        message = response.result.message or f'goal status {response.status}'
        self._core.on_task_finished(task_id, success, message)
        self._send_outbox()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FleetManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
