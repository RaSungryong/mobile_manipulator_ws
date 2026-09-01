# Per-camera-frame AprilTag detection result, published by robot_camera_node.
# One message per processed frame from ONE camera (front_cam or side_cam) --
# never merged across cameras. Detection-only: the node that publishes this
# message never acts on it (see robot_camera_node.py module docstring); any
# stop/motion decision is made by the subscriber (robot_controller.py).
std_msgs/Header header       # stamp = source image stamp; frame_id = camera_name
string camera_name           # "front_cam" | "side_cam"
int32 image_width            # source image width, px
int32 image_height           # source image height, px
AprilTagDetection[] detections
