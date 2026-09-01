# Everything a consumer needs to know about the Fairino arm, in one message.
#
# Published by arm_node at ~10 Hz. It replaces the plain std_msgs/String that
# /arm/status used to carry, which held a state word and nothing else — a UI
# wanting the live TCP pose had no source for it and would have had to open its
# own Fairino RPC connection, which is exactly the device double-ownership this
# workspace forbids (arm_node is the sole owner of the arm).
#
# COMPLETION IS motion_seq-BASED, not pose-based. Same rule as mobile_node:
# "did the arm arrive" is a judgement made inside ArmController (it may retry a
# MoveJ, clamp a jog, or refuse while busy), so a client cannot decide it by
# comparing poses. arm_node stamps every FINISHED motion with an incrementing
# motion_seq plus a result string, and a client waits for motion_seq to pass the
# value it read BEFORE publishing its command. A stale pre-command state cannot
# satisfy that wait, because its motion_seq is by definition not newer.

Header header

# "starting" | "idle" | "busy" | "error"
string state

# True while a scan / home / move_cart / jog is executing. Commands arriving in
# this state are REFUSED, not queued — see result_message on the next bump.
bool busy

# Current TCP pose in the Fairino convention: x, y, z [mm], rx, ry, rz [deg],
# read with TOOL_ID=1 (vision_tip), NOT tool 0 (flange). Only meaningful when
# pose_valid is true; the RPC read can fail transiently without the node dying.
float64[6] tcp_pose
bool pose_valid

# Current joint angles J1..J6 [deg]. Same validity flag as the pose — both come
# from the same poll cycle.
float64[6] joints

# Incremented once per COMPLETED motion, whatever kind. Never decremented, never
# reset while the node lives. Starts at 0, so a client that reads 0 and waits for
# ">= 1" is correct on the very first command.
uint32 motion_seq

# What the motion that produced the current motion_seq did. Free text, e.g.
# "move_cart ok", "jog z +1.0 ok", "scan 169 pts ok", "refused: busy".
string result_message

# Whether that motion succeeded. Meaningless until motion_seq >= 1.
bool result_success
