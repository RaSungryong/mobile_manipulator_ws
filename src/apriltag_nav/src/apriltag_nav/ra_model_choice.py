#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Which Ra model scores a frame — the operator's choice (2026-11-13).

Three choices, by SURFACE GEOMETRY:

  auto    the scan model of paths.MODEL_PATH (the v3 split manifest: model A's
          stage head picks ra_finishing / ra_roughing per frame). The default.
  flat    ra_flat_only.onnx   — trained on the 2026-11-11/12 FLAT tiles only
  curved  ra_curved_only.onnx — trained on the CURVED tiles only

Why a choice and not a classifier: nothing in the frame is used to tell flat
from curved (the UPR_Ra_inference_v3_geo package takes `--geometry` from the
caller), and a geometry model on the other geometry reads nonsense — on
2026-11-13 ra_curved_only read ~0.6 um on flat roughing frames measured at
1.2, ra_flat_only ~0.7 on curved ones measured at 2.1. So the operator, who
knows which surface the tool is over, says so; the state is published so the
UI can never show a number without the model that produced it.

Pure: no ROS. arm_controller (the scan path and the step evaluation),
inference_node (robot_ui's on-demand Ra) and robot_ui share it.
"""

import os

from apriltag_nav import paths

CHOICES = ('auto', 'flat', 'curved')

# The slot / display name of each choice's value. `auto` keeps the names the
# code already uses ('scan' in the scan pipeline, finishing / roughing / A in
# the step evaluation).
SLOT_NAME = {'flat': 'flat_only', 'curved': 'curved_only'}

LABEL = {
    'auto': 'auto (split: finishing / roughing by image)',
    'flat': 'flat only (ra_flat_only)',
    'curved': 'curved only (ra_curved_only)',
}

_ALIASES = {
    'auto': 'auto', 'split': 'auto', 'ra_split': 'auto', 'default': 'auto', '': 'auto',
    'flat': 'flat', 'flat_only': 'flat', 'ra_flat_only': 'flat', 'ra_flat': 'flat',
    '평면': 'flat',
    'curved': 'curved', 'curved_only': 'curved', 'ra_curved_only': 'curved',
    'ra_curved': 'curved', 'curve': 'curved', '곡면': 'curved',
}


def normalize(text):
    """'flat' / 'Flat_only' / '평면' ... -> the choice, or None when unknown."""
    if text is None:
        return 'auto'
    return _ALIASES.get(str(text).strip().lower())


def model_path(choice, auto_path=None):
    """The file a choice loads. `auto_path` is the node's own scan model
    (arm_node's model_path argument); paths.MODEL_PATH when not given."""
    if choice == 'flat':
        return paths.RA_FLAT_ONLY_MODEL_PATH
    if choice == 'curved':
        return paths.RA_CURVED_ONLY_MODEL_PATH
    if choice == 'auto':
        return auto_path or paths.MODEL_PATH
    raise ValueError(f"unknown Ra model choice {choice!r} (one of {', '.join(CHOICES)})")


def model_name(choice, auto_path=None):
    """The file's stem — what a log line, a CSV cell and the UI call it."""
    return os.path.splitext(os.path.basename(model_path(choice, auto_path)))[0]


def describe(choice, auto_path=None):
    """The dict a node publishes as its Ra-model state."""
    path = model_path(choice, auto_path)
    return {'choice': choice, 'model': model_name(choice, auto_path),
            'label': LABEL[choice], 'path': path,
            'present': os.path.isfile(path), 'choices': list(CHOICES)}
