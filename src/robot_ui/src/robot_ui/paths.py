#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
On-disk locations inside robot_ui. Same rule as apriltag_nav.paths: resolve
once, here, never recompute from __file__ at a call site.

rospkg first so this works from both the devel and the install space; the
__file__ walk is the fallback for running the window straight out of the source
tree without a sourced workspace.
"""

import os

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
# <pkg>/src/robot_ui/paths.py -> up two -> <pkg>
_FALLBACK_PKG_DIR = os.path.abspath(os.path.join(_THIS_DIR, os.pardir, os.pardir))


def _resolve_pkg_dir():
    try:
        import rospkg
        return rospkg.RosPack().get_path('robot_ui')
    except Exception:
        return _FALLBACK_PKG_DIR


PKG_DIR = _resolve_pkg_dir()

UI_DIR = os.path.join(PKG_DIR, 'ui')

# Where hot-reloadable operator scripts live. Overridable with ~plugin_dir so a
# site can keep its own collection outside the package.
PLUGIN_DIR = os.path.join(PKG_DIR, 'plugins')

# Default place captures land. /tmp on purpose: an operator who never sets a
# path should not silently fill the package directory, and losing an unnamed
# throwaway capture on reboot is the lesser harm.
DEFAULT_SAVE_DIR = '/tmp/robot_ui_captures'
