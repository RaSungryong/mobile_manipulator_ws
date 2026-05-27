#!/usr/bin/env python3
"""
save_npz.py
===========
Direct-input bridge for T_hc2ee. Reads a human-editable yaml file
(default ``config/hand_eye/T_hc2ee.yaml``) and writes the 4x4 numpy
``.npz`` that the locator + map_calibrator load at startup.

This is the manual alternative to running ``handeye_calib_node`` —
use it when you have measured T_hc2ee externally (CAD, datasheet, or
another tool) and want to plug the value in without a fresh capture
session.

Usage::

    # write T_hc2ee.npz from the yaml interface file (default paths)
    rosrun path_tag_locator save_npz.py

    # override the yaml input file
    rosrun path_tag_locator save_npz.py --from-yaml /path/to/my_T.yaml

    # override the output npz path
    rosrun path_tag_locator save_npz.py --out /tmp/T_hc2ee.npz

    # overwrite existing npz
    rosrun path_tag_locator save_npz.py --force

    # legacy: write the hard-coded fallback matrix (skips yaml read)
    rosrun path_tag_locator save_npz.py --hardcoded
"""
import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml


_DEFAULT_OUT_REL = "config/hand_eye/T_hc2ee.npz"
_DEFAULT_YAML_REL = "config/hand_eye/T_hc2ee.yaml"

# Last-resort fallback matrix (= nominal hardware design, vision_tip).
# Kept for `--hardcoded` so the script still works when neither yaml nor
# CLI args are available.
_FALLBACK = np.array([[-1.0,  0.0, 0.0,  0.03250],
                      [ 0.0, -1.0, 0.0, -0.16246],
                      [ 0.0,  0.0, 1.0, -0.16775],
                      [ 0.0,  0.0, 0.0,  1.0]], dtype=np.float64)


def _pkg_root() -> Path:
    """Resolve path_tag_locator's package root via rospkg, else source tree."""
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path("path_tag_locator"))
    except Exception:
        here = Path(__file__).resolve().parent
        return here.parent


def _default_yaml_path() -> Path:
    return _pkg_root() / _DEFAULT_YAML_REL


def _default_out_path() -> Path:
    return _pkg_root() / _DEFAULT_OUT_REL


def _rpy_to_R(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """ZYX intrinsic euler (deg) -> 3x3 rotation. Mirrors geometry.rpy_deg_to_R
    but kept local so save_npz can run standalone without importing the
    package (helpful before catkin_make has produced the dist-package)."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy_, sy = math.cos(rz), math.sin(rz)
    return np.array([
        [cy_ * cp, cy_ * sp * sr - sy * cr, cy_ * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy_ * cr, sy * sp * cr - cy_ * sr],
        [-sp,      cp * sr,                 cp * cr],
    ], dtype=np.float64)


def load_from_yaml(yaml_path: Path) -> np.ndarray:
    """Read T_hc2ee.yaml and return a validated 4x4 matrix."""
    with open(yaml_path, "r") as fh:
        data = yaml.safe_load(fh) or {}
    fmt = str(data.get("format", "pose"))
    if fmt == "matrix":
        flat = data.get("matrix_4x4")
        if flat is None:
            raise ValueError(
                f"{yaml_path}: format=matrix requires 'matrix_4x4' field")
        T = np.asarray(flat, dtype=np.float64).reshape(4, 4)
    elif fmt == "pose":
        pos = data.get("position_m")
        rpy = data.get("rpy_deg")
        if pos is None or rpy is None:
            raise ValueError(
                f"{yaml_path}: format=pose requires 'position_m' and 'rpy_deg'")
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = _rpy_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        T[:3, 3] = np.asarray(pos, dtype=np.float64)
    else:
        raise ValueError(f"{yaml_path}: unknown format {fmt!r}")

    # Light validation; loud failure beats silently shipping a bogus matrix.
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError(
            f"{yaml_path}: rotation is not orthogonal (R Rᵀ != I)")
    det = float(np.linalg.det(R))
    if not math.isclose(det, 1.0, abs_tol=1e-4):
        raise ValueError(
            f"{yaml_path}: det(R) = {det:.6f} (expected +1)")
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=1e-9):
        raise ValueError(
            f"{yaml_path}: last row != [0, 0, 0, 1]")
    return T


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--from-yaml", type=str, default=None,
        help=f"Input yaml file (default: {_DEFAULT_YAML_REL})")
    ap.add_argument(
        "--out", type=str, default=None,
        help=f"Output .npz path (default: {_DEFAULT_OUT_REL})")
    ap.add_argument(
        "--hardcoded", action="store_true",
        help="Skip yaml, write the built-in nominal matrix instead")
    ap.add_argument(
        "--force", action="store_true",
        help="Overwrite an existing .npz file")
    args = ap.parse_args()

    out_path = Path(args.out) if args.out else _default_out_path()
    if out_path.exists() and not args.force:
        print(f"refuse to overwrite existing {out_path} (use --force).",
              file=sys.stderr)
        sys.exit(2)

    if args.hardcoded:
        T = _FALLBACK.copy()
        source = "hard-coded fallback"
    else:
        yaml_path = (Path(args.from_yaml) if args.from_yaml
                     else _default_yaml_path())
        if not yaml_path.exists():
            print(f"yaml not found: {yaml_path}\n"
                  f"  fill in {_DEFAULT_YAML_REL}, or pass --hardcoded.",
                  file=sys.stderr)
            sys.exit(2)
        T = load_from_yaml(yaml_path)
        source = f"yaml {yaml_path}"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_path), T)
    print(f"wrote T_hc2ee to {out_path}  (from: {source})")
    print("matrix:")
    print(T)


if __name__ == "__main__":
    main()
