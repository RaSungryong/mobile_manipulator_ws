"""
ros_image.py
============
Helpers to grab a single ``sensor_msgs/Image`` + ``sensor_msgs/CameraInfo``
synchronously via ``rospy.wait_for_message``.

Returns a BGR numpy array (HxWx3) and a 3x3 intrinsic matrix K.
"""
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


_BRIDGE = CvBridge()


def grab_image(topic: str, timeout: float = 3.0) -> np.ndarray:
    """Block until one Image arrives on ``topic`` and return BGR ndarray."""
    msg = rospy.wait_for_message(topic, Image, timeout=timeout)
    return _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def grab_K(info_topic: str, timeout: float = 3.0) -> np.ndarray:
    """Block until one CameraInfo arrives on ``info_topic`` and return 3x3 K."""
    msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=timeout)
    K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
    return K


def grab_image_and_K(image_topic: str,
                     info_topic: str,
                     timeout: float = 3.0):
    """Convenience: grab Image then CameraInfo from given topics."""
    img = grab_image(image_topic, timeout=timeout)
    K = grab_K(info_topic, timeout=timeout)
    return img, K
