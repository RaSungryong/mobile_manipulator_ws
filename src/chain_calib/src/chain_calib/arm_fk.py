# -*- coding: utf-8 -*-
"""Re-export: the FR10v6 URDF forward kinematics moved to
``apriltag_nav.arm_fk`` on 2026-09-22 so the locator chain (path_tag_locator)
can APPLY the joint offsets this package fits. Same class, same API."""
from apriltag_nav.arm_fk import ArmChain

__all__ = ['ArmChain']
