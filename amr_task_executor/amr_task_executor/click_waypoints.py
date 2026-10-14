"""
Click waypoints in RViz with "Publish Point" and send them to Nav2.

Workaround for the RViz Nav2 panel, which sends an empty waypoint list in Lyrical.
Every click on the map (Publish Point tool) adds one waypoint; after `count`
clicks the list goes to Nav2's /follow_waypoints action and a new round starts.
"""

from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ClickWaypoints(Node):
    """Collect clicked points and send them as one FollowWaypoints goal."""

    def __init__(self) -> None:
        super().__init__('click_waypoints')
        self._count = self.declare_parameter('count', 3).value
        self._points: list[PointStamped] = []
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.create_subscription(PointStamped, 'clicked_point', self._on_click, 10)
        self.get_logger().info(
            f'Click {self._count} points with "Publish Point" in RViz '
            '(press the tool before every click).')

    def _on_click(self, msg: PointStamped) -> None:
        self._points.append(msg)
        self.get_logger().info(
            f'point {len(self._points)}/{self._count}: '
            f'x={msg.point.x:.2f} y={msg.point.y:.2f}')
        if len(self._points) == self._count:
            self._send()

    def _send(self) -> None:
        goal = FollowWaypoints.Goal()
        for point in self._points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'      # stamp 0 = use the latest transform
            pose.pose.position = point.point
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)
        self._points = []
        # Check without waiting: a callback must never block the executor.
        if not self._client.server_is_ready():
            self.get_logger().error('follow_waypoints is not available; is Nav2 active?')
            return
        self.get_logger().info(f'sending {len(goal.poses)} waypoints to Nav2')
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, msg) -> None:
        self.get_logger().info(f'heading to waypoint {msg.feedback.current_waypoint + 1}',
                               throttle_duration_sec=2.0)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the waypoints')
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        missed = len(result.missed_waypoints)
        self.get_logger().info(
            f'done: {missed} waypoint(s) missed. Click {self._count} new points to go again.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClickWaypoints()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
