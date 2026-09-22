# -*- coding: utf-8 -*-
"""Re-export: the FR10v6 URDF forward kinematics moved to
``apriltag_nav.arm_fk`` on 2026-09-22 so the locator chain (path_tag_locator)
can APPLY the joint offsets this package fits. Same class, same API."""
from apriltag_nav.arm_fk import ArmChain, T_of, default_urdf_path, _rot_xyz, _axis_angle_T   # noqa: F401
