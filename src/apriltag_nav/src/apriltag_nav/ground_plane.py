#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Ground-plane correction for a downward-looking camera (front_cam).

The navigation camera is neither distortion-free nor exactly vertical
(2026-09-08, measured with a precisely laid tag pair: roll +1.23 deg,
pitch -0.50 deg, lens 302 mm above the floor). Both effects put a
position-dependent error on what `robot_camera_node` publishes and
`mobile_controller` acts on — the worst being the tag's EDGE ANGLE, which a
tilt rotates in proportion to the tag's fore-aft position (+0.67 deg for a
square-laid tag 0.2 m ahead, -0.21 deg on the crosshair), and which the
aim / align logic multiplies by a 0.55-0.75 m lever.

`GroundPlane.correct()` takes the RAW detected corners, undistorts them
(plumb_bob D from CameraInfo), casts each through the tilted camera onto
the floor, and re-images the resulting ground points with a LEVEL virtual
camera of the same intrinsics at the same height. Consumers therefore keep
working in pixels with `z / fx` scaling, exactly as before, but the pixels
now describe a flat, distortion-free, vertical view — and `pose_x / pose_y`
are the ground coordinates relative to the lens NADIR (the point straight
below the lens), in the robot frame (x forward, y right).

Frame conventions (must match tools/fit_front_cam_ground.py, which produced
the numbers in robot.yaml): ground frame X = the robot's travel axis
(forward), Y = right, Z down, floor at Z = 0, lens at Z = -height. Camera
axes = ground axes rotated by R = Rz(yaw) Ry(pitch) Rx(roll); a ground
direction d maps to camera coordinates R d. roll / pitch are FIT
parameters in this convention, not mechanical readings. `yaw` is the
camera's rotation about its optical axis relative to the travel axis: the
edge angle a tag laid parallel to the travel axis reads on the crosshair
with yaw = 0 (front_cam: -0.38 deg, from a straight-drive test — the tag
pair cannot see it, only motion can). With it set, the virtual camera's
x axis IS the travel axis, so "edge 0" means the body is parallel to the
lane and the aim's base-offset estimate is made in the body frame.
"""
import math

import numpy as np

try:
    import cv2
except ImportError:  # the geometry still works without undistortion
    cv2 = None


def rot_xyz(roll, pitch, yaw=0.0):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T_tilted_to_level(roll, pitch, yaw=0.0):
    """4x4 pose of the LEVEL virtual camera expressed in the TILTED
    (physical) camera's frame — the ground-plane fit as a rigid transform
    (radians; translation zero, both cameras share the lens centre).

    In path_tag_locator's T_X2Y convention (pose of Y in X):

        T_mb2fc_level    = T_mb2fc_physical @ T_tilted_to_level(...)
        T_mb2fc_physical = T_mb2fc_level    @ inv(T_tilted_to_level(...))

    Why: `to_ground` maps a camera ray r to ground coordinates R^T r, i.e.
    v_physical = R v_level with R = rot_xyz(roll, pitch, yaw), and the
    level virtual camera's axes ARE the ground axes (x forward = image
    right, y right = image down, z down). So the level camera's axes,
    written in the physical camera's frame, are the columns of R.

    Consumers: `robot_camera_node` publishes detections in the LEVEL
    frame whenever the correction is enabled, so a chain that consumes
    those detections needs T_mb2fc_level, while a consumer re-detecting
    RAW frames needs the physical matrix. `extrinsics.yaml` carries the
    physical one (2026-09-15); `path_tag_locator.constants.
    load_extrinsics_full` derives the level one with this function.
    """
    T = np.eye(4)
    T[:3, :3] = rot_xyz(float(roll), float(pitch), float(yaw))
    return T


class GroundPlane(object):
    """Pixel <-> floor mapping for one camera.

    fx, fy, cx, cy : intrinsics (CameraInfo K)
    dist           : plumb_bob coefficients (CameraInfo D), any length
                     (None / empty = no distortion)
    roll, pitch    : radians, see the module docstring
    height         : lens height above the floor (m)
    yaw            : radians, camera rotation about the optical axis vs
                     the travel axis (see the module docstring); 0 = none
    """

    def __init__(self, fx, fy, cx, cy, dist, roll, pitch, height, yaw=0.0):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.K = np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])
        d = np.asarray(dist if dist is not None else [], dtype=float).ravel()
        self.D = d if d.size and np.any(d != 0.0) else None
        self.roll, self.pitch, self.h = float(roll), float(pitch), float(height)
        self.yaw = float(yaw)
        self.R = rot_xyz(self.roll, self.pitch, self.yaw)

    # ---- raw pixels -> floor ----
    def undistort(self, px):
        """raw pixels (N,2) -> normalised undistorted image coordinates (N,2)."""
        px = np.asarray(px, dtype=float).reshape(-1, 2)
        if self.D is not None and cv2 is not None:
            und = cv2.undistortPoints(px.reshape(-1, 1, 2), self.K, self.D)
            return und.reshape(-1, 2)
        return np.column_stack([(px[:, 0] - self.cx) / self.fx, (px[:, 1] - self.cy) / self.fy])

    def to_ground(self, px):
        """raw pixels (N,2) -> ground (X forward, Y right) in metres, relative
        to the lens nadir."""
        und = self.undistort(px)
        rays_cam = np.column_stack([und[:, 0], und[:, 1], np.ones(len(und))])
        rays = rays_cam @ self.R          # = R^T ray, camera -> ground
        t = self.h / rays[:, 2]
        return np.column_stack([rays[:, 0] * t, rays[:, 1] * t])

    # ---- floor -> level virtual camera ----
    def to_virtual_px(self, ground):
        g = np.asarray(ground, dtype=float).reshape(-1, 2)
        return np.column_stack([self.cx + self.fx * g[:, 0] / self.h,
                                self.cy + self.fy * g[:, 1] / self.h])

    def correct(self, corners_px, center_px):
        """Corrected (corners (4,2) px, center (2,) px, ground center (X, Y) m)
        for one detection: what a level, distortion-free camera at the same
        height and intrinsics would have seen."""
        g = self.to_ground(np.vstack([np.asarray(corners_px, dtype=float).reshape(4, 2),
                                      np.asarray(center_px, dtype=float).reshape(1, 2)]))
        v = self.to_virtual_px(g)
        return v[:4], v[4], g[4]

    # ---- forward model (for tests / fitting) ----
    def project(self, ground):
        """ground (N,2) -> RAW pixels through the tilted, distorting camera."""
        g = np.asarray(ground, dtype=float).reshape(-1, 2)
        d = np.column_stack([g[:, 0], g[:, 1], np.full(len(g), self.h)])
        pc = d @ self.R.T
        xn, yn = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
        if self.D is not None:
            k1, k2, p1, p2 = (list(self.D) + [0.0] * 4)[:4]
            k3 = self.D[4] if self.D.size > 4 else 0.0
            r2 = xn * xn + yn * yn
            k = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
            xd = xn * k + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
            yd = yn * k + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
            xn, yn = xd, yd
        return np.column_stack([self.fx * xn + self.cx, self.fy * yn + self.cy])

    # ---- whole-frame rectification (overlay only) ----
    def rectify_maps(self, width, height):
        """cv2.remap maps that turn a raw frame into the level virtual
        camera's view (same K, nadir at (cx, cy)). Built once per size."""
        cache = getattr(self, '_maps', None)
        if cache is not None and cache[0] == (width, height):
            return cache[1], cache[2]
        u, v = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
        ground = np.column_stack([((u - self.cx) * self.h / self.fx).ravel(),
                                  ((v - self.cy) * self.h / self.fy).ravel()])
        raw = self.project(ground)
        mx = raw[:, 0].reshape(height, width).astype(np.float32)
        my = raw[:, 1].reshape(height, width).astype(np.float32)
        self._maps = ((width, height), mx, my)
        return mx, my

    def rectify(self, img):
        """The raw frame re-imaged by the level virtual camera (what the
        corrected detections' pixels refer to)."""
        if cv2 is None:
            return img
        h, w = img.shape[:2]
        mx, my = self.rectify_maps(w, h)
        return cv2.remap(img, mx, my, cv2.INTER_LINEAR)

    def axis_offset_m(self):
        """Where the optical axis meets the floor, relative to the nadir (m)."""
        return self.to_ground(np.array([[self.cx, self.cy]]))[0]
