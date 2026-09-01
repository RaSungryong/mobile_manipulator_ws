#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
InferenceInterface — the scan pipeline's Ra predictor.

Thin adapter over RaPredictor, which is also what inference_node serves. There
is deliberately only ONE implementation of model loading, preprocessing and
result validation in this workspace: the two paths previously had their own
copies and had already drifted apart on the thing that matters most (see
below). Keeping this class is about `scan_pipeline` not needing to change —
`load_model(path) -> bool` and `infer(image) -> Optional[float]` are unchanged.

⚠️ PREPROCESSING CHANGED 2026-08-12: Resize -> CenterCrop.
    RA VALUES PRODUCED BEFORE THAT DATE ARE NOT COMPARABLE WITH THESE.

This file used to apply `Resize((900, 900))`, squashing the Basler's full
5472x3648 frame into the model's input. That is wrong for these graphs, and
structurally so: they cut their 900x900 input into a 3x3 grid of 300x300 tiles
fed to Conv3d (visible in model/exported/*.onnx as Slice/Transpose into 3D
convolutions), and those tiles are meant to be NATIVE camera pixels. Resizing
made each tile cover ~6x the surface area at ~1/6 the detail, so the texture
scale a roughness model reads was destroyed — while it still returned a
confident-looking number, which is why this went unnoticed.

Centre-cropping keeps native pixels over the middle of the frame, which is the
region the operator UI's guide box aims the tool at.

Consequence, stated plainly because it cannot be undone by re-running anything:
every Ra value in every result CSV written before this change came out of the
resize path. New scans are not comparable with them. Old CSVs are not wrong as
records of what was measured; they are measurements of a different transform.
Do not mix them in one analysis.
"""

from typing import Optional

import rospy

from apriltag_nav.ra_predictor import RaPredictor


class InferenceInterface:
    """ONNX Runtime (CPU) inference for Ra surface roughness prediction."""

    def __init__(self):
        self._predictor = None

    def load_model(self, model_path: str) -> bool:
        """Load one ONNX graph. Returns True on success.

        ONNX only — RaPredictor refuses a .pt path rather than pulling in torch
        and unpickling a whole nn.Module. Convert checkpoints before deploying.
        """
        rospy.loginfo("[Inference] Loading ONNX model...")
        self._predictor = RaPredictor(
            slots=[('scan', model_path)], log=rospy.loginfo)
        if self._predictor.load() == 1:
            rospy.loginfo("[Inference] ONNX model loaded successfully")
            return True
        rospy.logerr(f"[Inference] Failed to load: "
                     f"{self._predictor.failures().get('scan')}")
        self._predictor = None
        return False

    def infer(self, image) -> Optional[float]:
        """Score one BGR frame. None when there is no usable value.

        None covers a missing model, a failed run, and a non-finite result.
        RaPredictor discards NaN/Inf for us, which matters here: the caller
        records success=False on None, so a garbage value would otherwise land
        in the Ra CSV as a real measurement.
        """
        if self._predictor is None:
            return None
        try:
            results, _elapsed = self._predictor.predict(image)
        except Exception as e:
            rospy.logerr(f"[Inference] Error: {e}")
            return None
        return results[0][1] if results else None
