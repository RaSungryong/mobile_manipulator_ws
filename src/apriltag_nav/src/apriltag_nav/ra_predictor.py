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

⚠️ CROP POSITION IS PER MODEL since 2026-10-27, read from the model's sidecar
`<model>.json` (written next to the .onnx by machine_vision's export_onnx.py):
`crop_center_xy` [x, y] px and `frame_size` [w, h]. The scan_plate1 models
(resnet3D_multi onwards) were trained on a 900x900 window centred at
(2200, 1834) of the 5472x3648 Basler frame — LEFT of the standoff laser spot at
~(2971, 1834), whose halo reaches ~200 px — while the frame centre (2736, 1824)
puts the spot inside the window. The window is taken there only when the input
IS a full frame of `frame_size` (a robot_ui ROI crop has its own coordinates
and falls back to the centre crop, logged once); a model without the keys (the
2025 resnet3D / resnet3D_gray) keeps the centre crop, bit for bit as before.
"""

import json
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
        self.output_index = 0
        self.error = None
        self.crop_center = None     # (x, y) px of the 900 window in a full frame; None = centre
        self.frame_size = None      # (w, h) the crop_center refers to
        self._warned_fallback = False

    @property
    def loaded(self):
        return self.session is not None

    def _read_sidecar(self, log):
        """crop_center_xy / frame_size from `<model>.json`, if it has them."""
        side = os.path.splitext(self.path)[0] + '.json'
        if not os.path.exists(side):
            return
        with open(side) as f:
            meta = json.load(f)
        cc, fs = meta.get('crop_center_xy'), meta.get('frame_size')
        if cc is None:
            return
        if fs is None or len(cc) != 2 or len(fs) != 2:
            raise ValueError(f'{side}: crop_center_xy needs frame_size, both [x, y]')
        self.crop_center = (int(cc[0]), int(cc[1]))
        self.frame_size = (int(fs[0]), int(fs[1]))
        half = CROP_SIZE // 2
        x, y = self.crop_center
        w, h = self.frame_size
        if not (half <= x <= w - half and half <= y <= h - half):
            raise ValueError(f'{side}: a {CROP_SIZE} window at {self.crop_center} '
                             f'does not fit a {w}x{h} frame')
        log(f"[RaPredictor] slot '{self.name}': crop at {self.crop_center} "
            f"of a {w}x{h} frame ({side})")

    def crop_for(self, shape, log=print):
        """The crop centre to use for an image of `shape`, or None = centre."""
        if self.crop_center is None:
            return None
        h, w = shape[:2]
        if (w, h) == self.frame_size:
            return self.crop_center
        if not self._warned_fallback:
            log(f"[RaPredictor] slot '{self.name}': input {w}x{h} is not a "
                f"{self.frame_size[0]}x{self.frame_size[1]} frame — centre crop "
                f"instead of {self.crop_center} (logged once)")
            self._warned_fallback = True
        return None

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
        if os.path.getsize(self.path) < 1024:
            with open(self.path, 'rb') as f:
                head = f.read(64)
            if head.startswith(b'version https://git-lfs'):
                self.error = (f'{self.path} is a Git LFS pointer, not the model — '
                              'run `git lfs install && git lfs pull` in the workspace')
                log(f"[RaPredictor] slot '{self.name}': {self.error}")
                return False
        try:
            self._read_sidecar(log)
            import onnxruntime as ort

            opts = ort.SessionOptions()
            opts.intra_op_num_threads = max(1, (os.cpu_count() or 2) // 2)
            opts.graph_optimization_level = \
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(
                self.path, sess_options=opts,
                providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            # Stage models (resnet3D_multi) also return `stage_logits`; Ra is
            # the output named `output` (the first one in every graph so far).
            outs = [o.name for o in self.session.get_outputs()]
            self.output_index = outs.index('output') if 'output' in outs else 0
            if 'stage_logits' in outs and self.crop_center is None:
                # every stage model so far is a scan_plate1 model, trained left
                # of the laser spot: without the sidecar it scores the wrong place
                log(f"[RaPredictor] slot '{self.name}': WARNING stage model "
                    f"without crop_center_xy in its .json — centre crop, which "
                    f"is NOT where it was trained")

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
        out = self.session.run(None, {self.input_name: chw_float32})[self.output_index]
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
    def preprocess(bgr, crop_center=None):
        """BGR uint8 (any size) -> (1, 3, 900, 900) float32 in [-1, 1].

        Mirrors torchvision's
            ToTensor -> CenterCrop(900) -> Normalize(0.5, 0.5)
        done with numpy so the ROS node needs neither torch nor torchvision.
        Verified bit-exact against that chain (max |diff| = 0.0) for full-frame,
        720p, straddling and undersized inputs.

        crop_center (x, y): take the 900 window centred there instead of at the
        image centre (the caller has checked it fits — ModelSlot.crop_for).
        That equals machine_vision's training path for a full frame: tile
        1100 at (x, y) (prepare_scan_dataset.py) -> CenterCrop(900).
        """
        if crop_center is not None:
            if bgr is None:
                raise ValueError('no image')
            x, y = crop_center
            half = CROP_SIZE // 2
            bgr = bgr[y - half:y - half + CROP_SIZE, x - half:x - half + CROP_SIZE]
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
        if bgr is None:
            raise ValueError('no image')
        chw_by_crop = {}            # slots sharing a crop share one preprocessing
        results = []
        for slot in self.slots:
            if not slot.loaded:
                results.append((slot.name, None))
                continue
            crop = slot.crop_for(bgr.shape, self.log)
            if crop not in chw_by_crop:
                chw_by_crop[crop] = self.preprocess(bgr, crop)
            try:
                value = slot.run(chw_by_crop[crop])
            except Exception as e:
                self.log(f"[RaPredictor] slot '{slot.name}' inference failed: {e}")
                results.append((slot.name, None))
                continue
            results.append((slot.name, value if np.isfinite(value) else None))
        return results, time.time() - started
