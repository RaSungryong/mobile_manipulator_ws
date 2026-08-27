# Single AprilTag detection within one camera frame (see AprilTagDetectionArray).
int32 id             # AprilTag ID
float64 center_x      # detected tag center, image column (px)
float64 center_y      # detected tag center, image row (px)
float64 pose_x        # dt_apriltags pose_t[0] -- lateral offset from camera (m)
float64 pose_y        # dt_apriltags pose_t[1] -- vertical offset from camera (m)
float64 pose_z        # dt_apriltags pose_t[2] -- distance from camera, forward (m)

# Tag orientation, from dt_apriltags pose_R. Degrees, ZYX-intrinsic euler of
# the tag frame expressed in the camera frame -- same convention as the scan
# CSVs (see CLAUDE.md "Coordinate Frames"), so the numbers read the same way.
# NOTE: a tag facing the camera dead-on does NOT read 0,0,0 -- one of the
# three angles sits at +/-180. That is correct, not a bug: the tag's own +Z
# points out of its face, i.e. back at the camera, a 180 deg flip from the
# camera's +Z. Which angle carries the flip depends on how the euler
# decomposition splits it, so do not test "squareness" on any single one of
# them -- that is what tilt_from_normal below is for.
float64 roll          # rotation about the camera X axis (deg)
float64 pitch         # rotation about the camera Y axis (deg)
float64 yaw           # rotation about the camera Z axis (deg)

# The four detected corners, flattened: [x0,y0, x1,y1, x2,y2, x3,y3] in px,
# in dt_apriltags' own order. robot_controller derives its alignment angle
# from the corner0->corner1 edge, so the raw corners travel with the message
# rather than each consumer re-detecting the frame to get them back.
float64[8] corners

# Angle between the tag's surface normal and the camera's optical axis (deg).
# 0 = the tag faces the camera dead square; this is the single number for
# "how far off square is it", independent of how the roll/pitch/yaw split up.
float64 tilt_from_normal
