"""chain_calib — front_cam <-> hand_cam chain calibration against the
printed A0 tag sheet (sheet/).

``sheet``    layout JSON + print scale, frame accumulation, multi-tag PnP
``solver``   pure numpy / scipy: the AX = YB fit (hand / base / joint),
             hold-out evaluation, the tag-pair T_A2B metric
``session``  sample persistence, per-sample diagnostics, view-coverage
             advice — no ROS
``scripts/chain_calib.py``  the operator tool (ROS)
"""
