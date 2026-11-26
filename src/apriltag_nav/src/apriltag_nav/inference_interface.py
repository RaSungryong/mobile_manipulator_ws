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

import threading
from typing import Optional

import rospy

from apriltag_nav.ra_predictor import RaPredictor


class InferenceInterface:
    """ONNX Runtime (CPU) inference for Ra surface roughness prediction.

    Holds every model it has loaded, keyed by path, and scores with the
    SELECTED one (2026-11-13: the operator picks auto / flat-only /
    curved-only per scan — ra_model_choice.py). `load_model(path)` is the old
    call and selects what it loads; `preload_model` loads without switching
    (a scan's worker may be scoring); `select_model` switches, loading first
    when needed, and keeps the current model when the new one fails.
    """

    def __init__(self):
        self._predictor = None
        self._loaded = {}               # path -> RaPredictor
        self._lock = threading.Lock()
        self.model_path = None          # the selected model's path

    def _load(self, model_path):
        """The predictor of a path, loading it on first use. None on failure."""
        with self._lock:
            pred = self._loaded.get(model_path)
            if pred is not None:
                return pred
            rospy.loginfo(f"[Inference] Loading ONNX model {model_path} ...")
            pred = RaPredictor(slots=[('scan', model_path)], log=rospy.loginfo)
            if pred.load() != 1:
                rospy.logerr(f"[Inference] Failed to load: "
                             f"{pred.failures().get('scan')}")
                return None
            rospy.loginfo("[Inference] ONNX model loaded successfully")
            self._loaded[model_path] = pred
            return pred

    def preload_model(self, model_path: str) -> bool:
        """Load a model into the cache WITHOUT selecting it."""
        return self._load(model_path) is not None

    def select_model(self, model_path: str) -> bool:
        """Score with this model from now on. False (and the current model
        kept) when it cannot be loaded."""
        pred = self._load(model_path)
        if pred is None:
            return False
        self._predictor, self.model_path = pred, model_path
        return True

    def load_model(self, model_path: str) -> bool:
        """Load one ONNX graph (or a split .json manifest) and select it.
        Returns True on success.

        ONNX only — RaPredictor refuses a .pt path rather than pulling in torch
        and unpickling a whole nn.Module. Convert checkpoints before deploying.
        """
        if self.select_model(model_path):
            return True
        if self.model_path is None:
            self._predictor = None
        return False

    def infer(self, image) -> Optional[float]:
        """Score one BGR frame. None when there is no usable value.

        None covers a missing model, a failed run, and a non-finite result.
        RaPredictor discards NaN/Inf for us, which matters here: the caller
        records success=False on None, so a garbage value would otherwise land
        in the Ra CSV as a real measurement.
        """
        predictor = self._predictor
        if predictor is None:
            return None
        try:
            results, _elapsed = predictor.predict(image)
        except Exception as e:
            rospy.logerr(f"[Inference] Error: {e}")
            return None
        return results[0][1] if results else None
