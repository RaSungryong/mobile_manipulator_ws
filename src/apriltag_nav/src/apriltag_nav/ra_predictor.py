#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RaPredictor — surface-roughness inference over one or more named ONNX models.

Pure logic: no rospy, no ROS types, no device. inference_node.py wraps it, the
same way arm_node wraps ArmController. That split is what lets this be tested
without a master.

ONNX only, deliberately. The .onnx graphs in model/exported/ are self-describing,
load in ~1 s, need no network-definition source alongside them, and let the
robot run without torch installed at all. A .pt checkpoint pickles a whole
nn.Module, so loading one drags in torch plus the exact class definitions it was
saved with — a version coupling that breaks silently years later. Convert
weights to ONNX before deploying them; do not add a torch backend here.

Several slots exist so a frame can be scored by more than one model in the same
pass. The models in this workspace are two different architectures
(resnet3D vs resnet3D_gray) rather than two training runs of one, so the second
slot is a cross-check, not a replicate — worth having at collection time, when
the operator can still re-take a shot that the two disagree wildly on.

⚠️ PREPROCESSING IS CenterCrop(900, 900), NOT Resize — and the reason is
structural, not stylistic. Both exported graphs take a fixed 900x900 input and
immediately cut it into a 3x3 grid of 300x300 tiles fed to Conv3d (Slice /
Transpose into 3D convolutions, visible in the graph). Those tiles are meant to
be NATIVE Basler pixels. The camera delivers 5472x3648, so resizing to 900x900
makes every tile cover ~6x the surface area at ~1/6 the detail; the texture
scale the network learned roughness from is gone, and it still returns a
confident-looking number. Centre-cropping keeps native pixels over the region
the live preview's centre box aims the operator at.

Both consumers go through this one implementation: inference_node serves it
over /inference/predict, and inference_interface.py (the scan pipeline) is a
thin adapter over it. That is on purpose — they used to carry separate copies
of this transform and had drifted apart exactly here, the scan path resizing
while the on-demand path cropped.

⚠️ The scan pipeline was switched from Resize to CenterCrop on 2026-08-12, so
Ra values recorded before that date came out of a different transform and are
not comparable with new ones. See CLAUDE.md.
"""

import os
import time

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - cv2 is a hard dep of the ROS node
    cv2 = None


CROP_SIZE = 900


class ModelSlot:
    """One named ONNX graph plus its runtime session."""

    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.session = None
        self.input_name = None
        self.error = None

    @property
    def loaded(self):
        return self.session is not None

    def load(self, log=print):
        if not self.path:
            self.error = 'no path configured'
            return False
        if os.path.splitext(self.path)[1].lower() != '.onnx':
            self.error = (f'not an .onnx file: {self.path} — convert the '
                          'checkpoint before deploying it')
            log(f"[RaPredictor] slot '{self.name}': {self.error}")
            return False
        if not os.path.exists(self.path):
            self.error = f'file not found: {self.path}'
            log(f"[RaPredictor] slot '{self.name}': {self.error}")
            return False
        try:
            import onnxruntime as ort

            opts = ort.SessionOptions()
            opts.intra_op_num_threads = max(1, (os.cpu_count() or 2) // 2)
            opts.graph_optimization_level = \
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(
                self.path, sess_options=opts,
                providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name

            # Warm up. The first run pays graph-optimisation cost that would
            # otherwise land on the operator's first capture and read as a hang.
            dummy = np.zeros((1, 3, CROP_SIZE, CROP_SIZE), dtype=np.float32)
            for _ in range(2):
                self.session.run(None, {self.input_name: dummy})

            log(f"[RaPredictor] slot '{self.name}' loaded: {self.path}")
            return True
        except Exception as e:
            self.session = None
            self.error = f'{type(e).__name__}: {e}'
            log(f"[RaPredictor] slot '{self.name}' failed to load: {self.error}")
            return False

    def run(self, chw_float32):
        """chw_float32: (1, 3, 900, 900) normalised float32. Returns a float."""
        out = self.session.run(None, {self.input_name: chw_float32})[0]
        return float(np.squeeze(out))


class RaPredictor:
    """Loads named ONNX slots and scores BGR frames against all of them."""

    def __init__(self, slots, log=print):
        """slots: ordered list of (name, path). The FIRST is the primary."""
        self.log = log
        self.slots = [ModelSlot(name, path) for name, path in slots if path]

    def load(self):
        """Load every configured slot. Returns the number that came up.

        Deliberately partial: one missing file should not cost the operator the
        slot that is present. Callers report which names are live.
        """
        return sum(1 for slot in self.slots if slot.load(self.log))

    def names(self):
        return [s.name for s in self.slots]

    def loaded_names(self):
        return [s.name for s in self.slots if s.loaded]

    def failures(self):
        return {s.name: s.error for s in self.slots if not s.loaded}

    @staticmethod
    def preprocess(bgr):
        """BGR uint8 (any size) -> (1, 3, 900, 900) float32 in [-1, 1].

        Mirrors torchvision's
            ToTensor -> CenterCrop(900) -> Normalize(0.5, 0.5)
        done with numpy so the ROS node needs neither torch nor torchvision.
        Verified bit-exact against that chain (max |diff| = 0.0) for full-frame,
        720p, straddling and undersized inputs.
        """
        if bgr is None:
            raise ValueError('no image')
        if cv2 is None:
            raise RuntimeError('cv2 unavailable')
        if bgr.ndim == 2:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        h, w = rgb.shape[:2]
        if h < CROP_SIZE or w < CROP_SIZE:
            # torchvision's CenterCrop pads with zeros here. Pad the same way
            # rather than upscaling, which would reintroduce the exact
            # texture-rescaling error this crop exists to avoid.
            pad_y = max(0, CROP_SIZE - h)
            pad_x = max(0, CROP_SIZE - w)
            rgb = cv2.copyMakeBorder(
                rgb, pad_y // 2, pad_y - pad_y // 2,
                pad_x // 2, pad_x - pad_x // 2,
                cv2.BORDER_CONSTANT, value=(0, 0, 0))
            h, w = rgb.shape[:2]

        top = (h - CROP_SIZE) // 2
        left = (w - CROP_SIZE) // 2
        crop = rgb[top:top + CROP_SIZE, left:left + CROP_SIZE]

        arr = crop.astype(np.float32) / 255.0          # ToTensor scaling
        arr = (arr - 0.5) / 0.5                        # Normalize(0.5, 0.5)
        arr = np.transpose(arr, (2, 0, 1))             # HWC -> CHW
        return np.ascontiguousarray(arr[None, ...], dtype=np.float32)

    def predict(self, bgr):
        """Score one frame. Returns (results, elapsed_s).

        results is an ordered list of (name, value); value is None when that
        slot is not loaded or produced a non-finite number. A non-finite Ra is
        discarded rather than passed on — it would otherwise land in a CSV as a
        real measurement.
        """
        started = time.time()
        chw = self.preprocess(bgr)
        results = []
        for slot in self.slots:
            if not slot.loaded:
                results.append((slot.name, None))
                continue
            try:
                value = slot.run(chw)
            except Exception as e:
                self.log(f"[RaPredictor] slot '{slot.name}' inference failed: {e}")
                results.append((slot.name, None))
                continue
            results.append((slot.name, value if np.isfinite(value) else None))
        return results, time.time() - started
