"""
hand_eye.py
===========
Load T_hc2ee (pose of EE expressed in the hand camera frame) from a .npz file.

The npz file produced by ``cv2.calibrateHandEye`` stores the gripper-to-cam
transform under the user notation ``T_hc2ee`` (= pose of EE in cam frame).
"""
from pathlib import Path
import numpy as np


def load_T_hc2ee(npz_path) -> np.ndarray:
    """Load and validate a 4x4 hand-eye transform from a .npz file."""
    p = Path(npz_path)
    if not p.exists():
        raise FileNotFoundError(f"Hand-eye file not found: {p}")
    data = np.load(p)
    if len(data.files) == 0:
        raise ValueError(f"{p}: empty npz")
    key = data.files[0]
    T = np.asarray(data[key], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{p}: expected 4x4 matrix, got shape {T.shape}")
    return T
