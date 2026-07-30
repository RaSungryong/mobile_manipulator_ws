"""
apriltag_nav — AprilTag navigation, arm scanning and Ra prediction library.

Layout:
  src/apriltag_nav/   this package — importable library code
  scripts/            ROS node entry points (thin; they import from here)
  tools/              standalone one-off scripts, not part of the runtime

Import as `from apriltag_nav.map_manager import MapManager`. Deliberately no
re-exports here: the modules pull in rospy, pypylon, onnxruntime and the Fairino
SDK, so eagerly importing them would make `import apriltag_nav` fail on any
machine missing a piece of hardware tooling.

Use apriltag_nav.paths for every on-disk location instead of computing paths
from __file__.
"""
