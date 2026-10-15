"""
Pass the pose of the AprilTag the camera sees to Nav2's docking server.

apriltag_ros publishes WHICH tags it sees on /detections, and WHERE each one is as a
TF frame (camera_optical_link -> tag36h11:<id>). The docking server reads neither:
it waits for one PoseStamped on detected_dock_pose. This node joins the two.
"""

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TagDockPose(Node):
    """Republish the pose of the clearest visible tag on detected_dock_pose."""

    def __init__(self) -> None:
        super().__init__('tag_dock_pose')
        # below this decision margin a read is doubtful (ours read about 120)
        self._min_margin = self.declare_parameter('min_decision_margin', 50.0).value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(AprilTagDetectionArray, 'detections', self._on_detections, 10)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        good = [d for d in msg.detections if d.decision_margin >= self._min_margin]
        if not good:
            return                      # no clear tag in this image: publish nothing
        best = max(good, key=lambda d: d.decision_margin)
        tag_frame = f'{best.family}:{best.id}'
        try:
            # Time() = the newest transform we have for this tag
            tf = self._tf_buffer.lookup_transform(msg.header.frame_id, tag_frame, Time())
        except TransformException:
            return                      # the tag's first TF has not arrived yet

        pose = PoseStamped()
        # Keep the TF's own time: the docking server uses it to place the tag in
        # the odom frame as it was when that image was taken, even if the robot moved.
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self._pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagDockPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()