#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Single source of truth for on-disk locations inside the apriltag_nav package.

Why this exists
---------------
Paths used to be recomputed from `__file__` in a dozen modules, each assuming
"this file lives in <pkg>/scripts/" and walking up one level. That assumption
broke the moment library modules moved into a proper Python package (they now
sit two levels deeper), and it silently resolves to the WRONG directory rather
than failing — `<pkg>/src/config` instead of `<pkg>/config`. Resolve once, here.

Resolution order:
  1. rospkg — the ROS-correct answer, works in devel and install spaces
  2. __file__ walk-up via realpath — fallback for plain `python3 foo.py` runs
     outside a sourced workspace (tests, offline tools). realpath matters
     because catkin symlinks the package into the devel space.
"""

import os

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
# <pkg>/src/apriltag_nav/paths.py  ->  up two  ->  <pkg>
_FALLBACK_PKG_DIR = os.path.abspath(os.path.join(_THIS_DIR, os.pardir, os.pardir))


def _resolve_pkg_dir():
    try:
        import rospkg
        return rospkg.RosPack().get_path('apriltag_nav')
    except Exception:
        return _FALLBACK_PKG_DIR


PKG_DIR = _resolve_pkg_dir()

CONFIG_DIR = os.path.join(PKG_DIR, 'config')
CONFIG_PATH = os.path.join(CONFIG_DIR, 'robot.yaml')
MAP_PATH = os.path.join(CONFIG_DIR, 'map.yaml')

TASK_DIR = os.path.join(PKG_DIR, 'task', 'csv')
MODEL_DIR = os.path.join(PKG_DIR, 'model')
EXPORTED_MODEL_DIR = os.path.join(MODEL_DIR, 'exported')

# The graph the scan pipeline runs, and inference_node's primary slot — the same
# file on purpose, so an on-demand Ra from the UI is comparable with the numbers
# in the scan CSVs rather than being a second opinion nobody can reconcile.
MODEL_PATH = os.path.join(EXPORTED_MODEL_DIR, 'resnet3D.onnx')

# inference_node's secondary slot: a different architecture over the same input,
# so it is a cross-check rather than a replicate. Optional — a missing file
# costs the comparison, not the primary prediction.
RA_MODEL_SECONDARY = os.path.join(EXPORTED_MODEL_DIR, 'resnet3D_gray.onnx')

# Both are gitignored (*.onnx), so a fresh clone has to copy them in.

# The Fairino SDK is a sibling package in the catkin source space, not part of
# apriltag_nav, so it is resolved relative to the source space rather than the
# package. Kept here so there is exactly one place to fix if it ever moves.
SRC_SPACE = os.path.dirname(PKG_DIR)
FAIRINO_SDK_PATH = os.path.join(
    SRC_SPACE, 'fairino_sdk', 'fairino-python-sdk', 'Linux')


def add_fairino_sdk_to_path():
    """Put the Fairino SDK on sys.path. Idempotent; returns True if present."""
    import sys
    if not os.path.isdir(FAIRINO_SDK_PATH):
        return False
    if FAIRINO_SDK_PATH not in sys.path:
        sys.path.append(FAIRINO_SDK_PATH)
    return True


def load_config(path=None):
    """Load robot.yaml (or another yaml). Returns {} when unreadable."""
    import yaml
    try:
        with open(path or CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def load_yaml_block(block_name, path=None):
    """Load one top-level block from robot.yaml; {} if absent."""
    return (load_config(path).get(block_name) or {})
