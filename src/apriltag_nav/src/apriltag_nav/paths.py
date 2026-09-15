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


# ---------------------------------------------------------------------------
# Run output lives INSIDE the workspace (user rule, 2026-09-14): results and
# records under <ws>/results and <ws>/log, never ~/.ros or /tmp or $HOME.
#
# The workspace root is the parent of the catkin source space (PKG_DIR is
# <ws>/src/apriltag_nav in a devel-space build). `MM_WS` overrides it and is
# also what the yaml configs use (`${MM_WS}/log/...`), so it is exported into
# this process's environment here when the shell did not set it — the catkin
# env hook (env-hooks/50.apriltag_nav.sh.in) sets it, plus ROS_LOG_DIR, for
# a sourced shell; an unsourced `python3 tool.py` still gets the same answer.
# ---------------------------------------------------------------------------
WS_DIR = os.path.abspath(os.environ.get('MM_WS') or os.path.dirname(SRC_SPACE))
os.environ.setdefault('MM_WS', WS_DIR)

LOG_DIR = os.path.join(WS_DIR, 'log')
RESULTS_DIR = os.path.join(WS_DIR, 'results')

# mobile_controller: one yaml per TASK / GOTO command, <day>/<ts>_<cmd>.yaml
NAV_LOG_DIR = os.path.join(LOG_DIR, 'apriltag_nav', 'nav_log')
# path_tag_locator's calibrate/ locate/ handeye_calib/ map_world_*.yaml root
# (that package derives the same path itself; both must agree).
PTL_LOG_DIR = os.path.join(LOG_DIR, 'path_tag_locator')
# roslaunch / node logs (ROS_LOG_DIR, set by the env hook): <run_id>/*.log
ROS_LOG_DIR = os.path.join(LOG_DIR, 'ros')
# task_manager: <task>_ra_map_<ts>.csv (versioned — the deliverable).
# Under log/, not results/, since 2026-09-15 (user request) — same file,
# same content, just alongside the other per-run RECORDS (nav_log,
# path_tag_locator) rather than in results/ next to the large, unversioned
# scan_images bulk. `results/README.md` and CLAUDE.md's "Where run output
# lives" table both name this path; keep them in step with any change here.
RA_MAP_DIR = os.path.join(LOG_DIR, 'apriltag_nav', 'ra_maps')
# arm_node save_images: <ra_map stem>/point_<id>_sample_<n>_ra_<x>.png —
# unaffected: the bulk frames stay in results/ (unversioned, large).
SCAN_IMAGE_DIR = os.path.join(RESULTS_DIR, 'scan_images')


def expand_path(path):
    """`~` and `${VAR}` expansion with MM_WS guaranteed to resolve."""
    return os.path.expandvars(os.path.expanduser(str(path)))


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
