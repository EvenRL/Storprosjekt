from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros

_buffer = None  
_listener = None


def _ensure(node):
    global _buffer, _listener
    if _buffer is None:
        _buffer = tf2_ros.Buffer()
        _listener = tf2_ros.TransformListener(_buffer, node, spin_thread=False)


def pose(node, pose_cam: PoseStamped,
         target_frame: str = "base_link",
         timeout: float = 0.5) -> PoseStamped:

    _ensure(node)
    return _buffer.transform(
        pose_cam, target_frame, timeout=Duration(seconds=timeout))


def point(node, point_cam: PointStamped,
          target_frame: str = "base_link",
          timeout: float = 0.5) -> PointStamped:

    _ensure(node)
    return _buffer.transform(
        point_cam, target_frame, timeout=Duration(seconds=timeout))
