#!/usr/bin/env python3
"""
End-to-end validation: feed paired CSV rows through the *actual* rewritten
process_transforms (replicated here), compare vs FK(q0) in arm frame.

For each group we first Procrustes-fit the body pose (x_base, y_base, yaw_world)
against all its rows — this acts as ground-truth msg (no logs available).
Then we run the transform forward with that msg and compare to FK(vision).
"""
import os, numpy as np, pandas as pd
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation as R

# Paths resolved via the package, not os.getcwd() — these scripts used to
# live at the workspace root and silently required being run from there.
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src'))
from apriltag_nav.paths import PKG_DIR, SRC_SPACE, TASK_DIR

URDF = os.path.join(SRC_SPACE, 'frcobot_ros', 'frcobot_description',
                    'urdf', 'fr10v6_vision.urdf')
robot = rtb.Robot.URDF(URDF)
j = pd.read_csv(os.path.join(TASK_DIR, 'optimized_joints_line1.csv'))
p = pd.read_csv(os.path.join(TASK_DIR, 'grid_path_line1.csv'))

MOUNT_YAW = np.pi
BODY_OFF_X = 0.0
BODY_OFF_Y = 0.0
BODY_OFF_Z = 1.000   # average of per-group fit


def process_transforms_v2(goals, msg):
    """Rewritten process_transforms (replicated here, no ROS)."""
    x_base = -msg['y']
    y_base = -msg['x']
    theta  = np.radians(msg['theta'])
    c, s = np.cos(theta), np.sin(theta)
    p_A_W = np.array([
        x_base + c*BODY_OFF_X - s*BODY_OFF_Y,
        y_base + s*BODY_OFF_X + c*BODY_OFF_Y,
        BODY_OFF_Z
    ])
    alpha = theta + MOUNT_YAW
    R_AW = R.from_euler('z', -alpha).as_dcm()
    out = []
    for g in goals:
        p_W = np.array([g['x'], g['y'], g['z']])
        p_A = R_AW @ (p_W - p_A_W)
        out.append(p_A)
    return np.array(out)


def full_procrustes(A, B):
    cA = A.mean(0); cB = B.mean(0)
    H = (A-cA).T @ (B-cB)
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, np.sign(d)])
    Rm = Vt.T @ D @ U.T
    t = cB - Rm @ cA
    return Rm, t


print(f"Model: BODY_OFF_Z={BODY_OFF_Z}, MOUNT_YAW=π, body_off_xy=0\n")
print(f"{'group':>5} {'n':>3}  {'msg.x':>8} {'msg.y':>8} {'msg.θ':>6}  "
      f"{'pos RMS(mm)':>11} {'pos max(mm)':>11}")
print('-' * 75)

overall = []
for gid in sorted(j.group_id.unique()):
    jg = j[j.group_id==gid]; pg = p[p.group_id==gid]
    Q = jg[['q1','q2','q3','q4','q5','q6']].values
    Pa_fk = np.array([robot.fkine(q).t for q in Q])   # default EE = vision
    Pw = pg[['x','y','z']].values

    # Ground-truth body pose from full rigid fit
    Rm, t_fit = full_procrustes(Pa_fk, Pw)
    # arm-world yaw from fit (Z rotation)
    euler = R.from_dcm(Rm).as_euler('zyx', degrees=True)
    arm_yaw = euler[0]                    # world yaw of arm base
    body_yaw_world = arm_yaw - np.degrees(MOUNT_YAW)  # undo mount_yaw
    # Body world xy = t_fit xy (since body_off_xy=0)
    body_x_world, body_y_world = t_fit[0], t_fit[1]
    # Convert to msg convention
    msg = {
        'x':     -body_y_world,
        'y':     -body_x_world,
        'theta': body_yaw_world,   # degrees
    }

    goals = pg[['x','y','z']].rename(columns=str).to_dict('records')
    Pa_model = process_transforms_v2(goals, msg)

    res = np.linalg.norm(Pa_model - Pa_fk, axis=1)
    overall.append(res)
    print(f"{gid:>5} {len(Q):>3}  {msg['x']:>+8.3f} {msg['y']:>+8.3f} "
          f"{msg['theta']:>+6.2f}  "
          f"{res.std()*1000:>11.2f} {res.max()*1000:>11.2f}")

overall = np.concatenate(overall)
print('-' * 75)
print(f"OVERALL  pos RMS={overall.std()*1000:.2f}mm  max={overall.max()*1000:.2f}mm  "
      f"mean={overall.mean()*1000:.2f}mm\n")

# Also compare to USD geometric body_off_z = 1.0255
print(f"Sensitivity: swap BODY_OFF_Z to 1.0255 (USD value) and check…")
BODY_OFF_Z = 1.0255
overall2 = []
for gid in sorted(j.group_id.unique()):
    jg = j[j.group_id==gid]; pg = p[p.group_id==gid]
    Q = jg[['q1','q2','q3','q4','q5','q6']].values
    Pa_fk = np.array([robot.fkine(q).t for q in Q])
    Pw = pg[['x','y','z']].values
    Rm, t_fit = full_procrustes(Pa_fk, Pw)
    arm_yaw = R.from_dcm(Rm).as_euler('zyx', degrees=True)[0]
    body_yaw_world = arm_yaw - np.degrees(MOUNT_YAW)
    body_x_world, body_y_world = t_fit[0], t_fit[1]
    msg = {'x':-body_y_world, 'y':-body_x_world, 'theta':body_yaw_world}
    goals = pg[['x','y','z']].to_dict('records')
    Pa_model = process_transforms_v2(goals, msg)
    overall2.append(np.linalg.norm(Pa_model - Pa_fk, axis=1))
overall2 = np.concatenate(overall2)
print(f"  With BODY_OFF_Z=1.0255: pos RMS={overall2.std()*1000:.2f}mm "
      f"max={overall2.max()*1000:.2f}mm mean={overall2.mean()*1000:.2f}mm")
