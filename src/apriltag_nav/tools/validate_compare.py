#!/usr/bin/env python3
"""Compare 9-DOF calibrated model vs my 2-param physical model."""
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


def full_procrustes(A, B):
    cA = A.mean(0); cB = B.mean(0)
    H = (A-cA).T @ (B-cB)
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, np.sign(d)])
    Rm = Vt.T @ D @ U.T
    t = cB - Rm @ cA
    return Rm, t


def model_v2(goals, msg, body_off_x=0.0, body_off_y=0.0, body_off_z=1.0,
             mount_yaw=np.pi, tilt_x=0.0, tilt_y=0.0):
    x_base = -msg['y']; y_base = -msg['x']; theta = np.radians(msg['theta'])
    c, s = np.cos(theta), np.sin(theta)
    p_A_W = np.array([
        x_base + c*body_off_x - s*body_off_y,
        y_base + s*body_off_x + c*body_off_y,
        body_off_z])
    alpha = theta + mount_yaw
    # Mount tilt applied in body frame after yaw
    R_WA = (R.from_euler('z', alpha)
            * R.from_euler('y', tilt_y)
            * R.from_euler('x', tilt_x)).as_dcm()
    R_AW = R_WA.T
    return np.array([R_AW @ (np.array([g['x'], g['y'], g['z']]) - p_A_W)
                     for g in goals])


def model_9dof(goals, msg):
    """Old calibrated 9-DOF model."""
    body_off_x=-0.166715; body_off_y=-0.254772; arm_base_z=0.974167
    tilt_x=0.054898; tilt_y=0.017894; h_bias=0.014368
    ori_corr=np.array([-0.095520,-0.052944,-0.008688])
    x_base = -msg['y']; y_base = -msg['x']
    theta = np.radians(msg['theta']) + h_bias
    c, s = np.cos(theta), np.sin(theta)
    arm_world = np.array([x_base + c*body_off_x - s*body_off_y,
                          y_base + s*body_off_x + c*body_off_y, arm_base_z])
    R_aw = (R.from_euler('z', theta) * R.from_euler('y', tilt_y)
            * R.from_euler('x', tilt_x)).as_dcm()
    return np.array([R_aw @ (np.array([g['x'], g['y'], g['z']]) - arm_world)
                     for g in goals])


def body_pose_from_fit(gid, MOUNT_YAW=np.pi):
    jg = j[j.group_id==gid]; pg = p[p.group_id==gid]
    Q = jg[['q1','q2','q3','q4','q5','q6']].values
    Pa = np.array([robot.fkine(q).t for q in Q])
    Pw = pg[['x','y','z']].values
    Rm, t_fit = full_procrustes(Pa, Pw)
    arm_yaw = R.from_dcm(Rm).as_euler('zyx', degrees=True)[0]
    body_yaw = arm_yaw - np.degrees(MOUNT_YAW)
    return {'x': -t_fit[1], 'y': -t_fit[0], 'theta': body_yaw}, Pa, Pw


print(f"{'model':<28} {'RMS(mm)':>8} {'max(mm)':>8} {'mean(mm)':>9}")
print('-'*60)

def test(name, fn, **kwargs):
    all_res = []
    for gid in sorted(j.group_id.unique()):
        msg, Pa_fk, Pw = body_pose_from_fit(gid)
        jg = j[j.group_id==gid]; pg = p[p.group_id==gid]
        goals = pg[['x','y','z']].to_dict('records')
        Pa_pred = fn(goals, msg, **kwargs) if kwargs else fn(goals, msg)
        all_res.append(np.linalg.norm(Pa_pred - Pa_fk, axis=1))
    r = np.concatenate(all_res)
    print(f"{name:<28} {r.std()*1000:>8.2f} {r.max()*1000:>8.2f} {r.mean()*1000:>9.2f}")

test("9-DOF calibrated", model_9dof)
test("2-param (z=0.974, π)", lambda g, m: model_v2(g, m, body_off_z=0.974))
test("2-param (z=1.000, π)", lambda g, m: model_v2(g, m, body_off_z=1.000))
test("2-param (z=1.0255, π)", lambda g, m: model_v2(g, m, body_off_z=1.0255))
test("2-param + 2° tilt_y",  lambda g, m: model_v2(g, m, body_off_z=1.000,
                                                   tilt_y=np.radians(1.5)))
test("2-param + 3° tilt_x",  lambda g, m: model_v2(g, m, body_off_z=1.000,
                                                   tilt_x=np.radians(2.5)))
