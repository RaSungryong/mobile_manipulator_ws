"""path_tag_locator — see README.md.

Importing the package settles where run output goes (2026-09-14, user rule:
every record lives INSIDE the workspace, never ~/.ros):

    WS_DIR    the workspace root — `MM_WS` from the environment (exported by
              apriltag_nav's catkin env hook into every sourced shell), else
              derived from this file: <ws>/src/path_tag_locator/src/... .
              Exported back into os.environ so `${MM_WS}` in the yaml configs
              (locator.yaml default_save_dir, handeye_calib.yaml run_root)
              resolves in an unsourced `python3 tool.py` too.
    LOG_ROOT  <ws>/log/path_tag_locator — calibrate/<session>/, locate/<day>/,
              handeye_calib/run_<ts>/, map_world_<ts>.yaml. Same value
              apriltag_nav.paths.PTL_LOG_DIR holds; mobile_node reads the
              newest map_world from it.
"""
import os as _os

_THIS_DIR = _os.path.dirname(_os.path.realpath(__file__))
# <ws>/src/path_tag_locator/src/path_tag_locator -> up four -> <ws>
_DERIVED_WS = _os.path.abspath(_os.path.join(_THIS_DIR, *([_os.pardir] * 4)))

WS_DIR = _os.path.abspath(_os.environ.get('MM_WS') or _DERIVED_WS)
_os.environ.setdefault('MM_WS', WS_DIR)
LOG_ROOT = _os.path.join(WS_DIR, 'log', 'path_tag_locator')
