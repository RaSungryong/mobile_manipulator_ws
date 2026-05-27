#!/usr/bin/env python3
"""
save_npz.py (dev-only)
======================
Writes a hand-coded 4x4 T_hc2ee matrix to a .npz file. Intended ONLY for
bootstrapping / quick tests; for real use run ``handeye_calib_node`` which
produces the same npz from real captures.

Safety: refuses to overwrite an existing file unless ``--force`` is given.
Default output path is resolved via rospkg if available, else falls back
to the package's source-tree config directory.
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np


_DEFAULT_REL = "config/hand_eye/T_hc2ee.npz"


def _default_output_path() -> Path:
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path("path_tag_locator")) / _DEFAULT_REL
    except Exception:
        # fallback: source tree layout
        here = Path(__file__).resolve().parent
        return here.parent / _DEFAULT_REL


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=str, default=None,
                    help="Output .npz path (default: package config/hand_eye/T_hc2ee.npz)")
    ap.add_argument("--force", action="store_true",
                    help="Overwrite existing file")
    args = ap.parse_args()

    out_path = Path(args.out) if args.out else _default_output_path()
    if out_path.exists() and not args.force:
        print(f"refuse to overwrite existing {out_path} (use --force).",
              file=sys.stderr)
        sys.exit(2)

    arr = np.array([[-1.0,  0.0, 0.0,  0.03250],
                    [ 0.0, -1.0, 0.0, -0.16246],
                    [ 0.0,  0.0, 1.0, -0.16775],
                    [ 0.0,  0.0, 0.0,  1.0]])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_path), arr)
    print(f"wrote dev T_hc2ee to {out_path}")


if __name__ == "__main__":
    main()
