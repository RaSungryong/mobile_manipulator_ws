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

⚠️ SPOT-TILE MODELS since 2026-11-11 (the UPR_Ra_inference package of
2026-11-10: ra_standard.onnx / ra_new.onnx, a 2D ResNet-18 over 9 patches).
Their sidecar says `"preprocess": "spot_tile"` with `nominal_spot_xy` and
`frame_size`, and the input is built the way their training tiles were
(`spot_crop` below, copied from the package's predict.py): find the standoff
laser spot within 300 px of the nominal position of a full GREY frame, cut a
1100 px square centred on it, fill the spot with the median of a ring around
it, take the centre 900. So the window FOLLOWS the spot and holds it (filled
in) — the opposite of resnet3D_multi's fixed window beside it. Do not change
`spot_crop`: any change changes the model input. A frame is also checked the
way the package checks it (`spot_flags`: spot not found / weak / far from
nominal, saturated, underexposed, Ra outside the stage's training range); a
flagged frame is LOGGED and its value still returned — in a scan the frame is
the record (Ra data collection), so a flag must not cost the point. An input
that is not a full frame (robot_ui ROI) falls back to the centre crop, logged
once, as for a fixed-window model.
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
SPOT_TILE = 1100        # spot-centred square a spot-tile model's 900 window is cut from
SPOT_SEARCH = 300       # the spot is looked for within this many px of its nominal position


def spot_crop(im, nominal, tile=SPOT_TILE, search=SPOT_SEARCH, thresh=200, dilate=25, max_r=90):
    """``(tile x tile crop centred on the laser spot with the spot filled in, info)`` of a grey frame.

    Copied from UPR_Ra_inference/predict.py (2026-11-10), which copied it
    unchanged from machine_vision's prepare_spot_tiles.py (the training
    tiles): any change here changes the model input. The one difference is
    that the nominal spot (x, y) is an argument (the model's sidecar) instead
    of a module constant.
    """
    nx, ny = nominal
    x0, y0 = nx - search, ny - search
    roi = im[y0:ny + search, x0:nx + search].astype(np.float32)
    blur = cv2.GaussianBlur(roi, (0, 0), 8)
    py, px = np.unravel_index(blur.argmax(), blur.shape)
    peak = float(blur.max())
    cx, cy = x0 + int(px), y0 + int(py)
    half = tile // 2
    crop = im[cy - half:cy - half + tile, cx - half:cx - half + tile].copy()
    bright = (cv2.GaussianBlur(crop, (0, 0), 2) > thresh).astype(np.uint8)
    n, lab = cv2.connectedComponents(bright)
    centre_lab = lab[half, half]
    if centre_lab == 0:
        ys, xs = np.nonzero(lab)
        if len(ys):
            i = np.argmin((ys - half) ** 2 + (xs - half) ** 2)
            if (ys[i] - half) ** 2 + (xs[i] - half) ** 2 < 40 ** 2:
                centre_lab = lab[ys[i], xs[i]]
    mask = (lab == centre_lab) if centre_lab else np.zeros_like(bright, bool)
    yy, xx = np.ogrid[:tile, :tile]
    mask &= (yy - half) ** 2 + (xx - half) ** 2 <= max_r ** 2
    mask = cv2.dilate(mask.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilate + 1,) * 2)) > 0
    if mask.any():
        ring = cv2.dilate(mask.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (61, 61))) > 0
        ring &= ~mask
        crop[mask] = np.uint8(np.median(crop[ring]))
    return crop, {"spot_x": cx, "spot_y": cy, "peak": round(peak, 1), "masked_px": int(mask.sum())}


def spot_flags(info, nominal):
    """Reasons not to trust a spot-tile frame (empty = ok) — the package's
    flags_of(), thresholds in its README_ko.md; the Ra-range flag is added
    per slot in RaPredictor.predict_detail."""
    f = []
    if info['masked_px'] == 0:
        f.append('spot_not_found')
    elif info['peak'] < 200:
        f.append('spot_weak')
    if abs(info['spot_x'] - nominal[0]) > 150 or abs(info['spot_y'] - nominal[1]) > 150:
        f.append('spot_far_from_nominal')
    if info['sat'] > 0.3:
        f.append('saturated')
    if info['mean'] < 10:
        f.append('underexposed')
    return f


def sidecar_meta(model_path):
    """The model's `<model>.json` as a dict; {} when there is none."""
    side = os.path.splitext(model_path or '')[0] + '.json'
    if not model_path or not os.path.exists(side):
        return {}
    with open(side, encoding='utf-8') as f:
        return json.load(f)


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
        self.frame_size = None      # (w, h) the crop_center / spot_nominal refers to
        self.spot_nominal = None    # (x, y) nominal laser spot of a spot-tile model; None = not one
        self.stages = None          # names of the stage_logits entries, from the sidecar
        self.ra_range = None        # {stage: [lo, hi]} training Ra range, from the sidecar
        self.has_stage = False      # the graph returns stage_logits
        self._warned_fallback = False

    @property
    def loaded(self):
        return self.session is not None

    def _read_sidecar(self, log):
        """crop_center_xy / frame_size from `<model>.json`, if it has them."""
        side = os.path.splitext(self.path)[0] + '.json'
        if not os.path.exists(side):
            return
        with open(side, encoding='utf-8') as f:
            meta = json.load(f)
        cc, fs = meta.get('crop_center_xy'), meta.get('frame_size')
        self.stages = meta.get('stages')
        self.ra_range = meta.get('train_ra_range')
        prep = meta.get('preprocess')
        if prep == 'spot_tile':
            ns = meta.get('nominal_spot_xy')
            if ns is None or fs is None or len(ns) != 2 or len(fs) != 2:
                raise ValueError(f'{side}: spot_tile needs nominal_spot_xy and '
                                 f'frame_size, both [x, y]')
            self.spot_nominal = (int(ns[0]), int(ns[1]))
            self.frame_size = (int(fs[0]), int(fs[1]))
            reach = SPOT_SEARCH + SPOT_TILE // 2    # the farthest a tile can extend
            x, y = self.spot_nominal
            w, h = self.frame_size
            if not (reach <= x <= w - reach and reach <= y <= h - reach):
                raise ValueError(f'{side}: a {SPOT_TILE} tile within {SPOT_SEARCH} px '
                                 f'of {self.spot_nominal} does not fit a {w}x{h} frame')
            log(f"[RaPredictor] slot '{self.name}': spot tile, laser spot near "
                f"{self.spot_nominal} of a {w}x{h} frame ({side})")
            return
        if prep not in (None, 'crop'):
            raise ValueError(f'{side}: unknown preprocess {prep!r}')
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
        self._warn_fallback(w, h, self.crop_center, log)
        return None

    def prep_for(self, shape, log=print):
        """How to build this slot's input from an image of `shape`:
        ('spot', (x, y)) = spot tile around that nominal spot, (x, y) = the
        fixed 900 window centred there, None = centre crop."""
        if self.spot_nominal is None:
            return self.crop_for(shape, log)
        h, w = shape[:2]
        if (w, h) == self.frame_size:
            return ('spot', self.spot_nominal)
        self._warn_fallback(w, h, f'the spot tile near {self.spot_nominal}', log)
        return None

    def _warn_fallback(self, w, h, instead_of, log):
        if not self._warned_fallback:
            log(f"[RaPredictor] slot '{self.name}': input {w}x{h} is not a "
                f"{self.frame_size[0]}x{self.frame_size[1]} frame — centre crop "
                f"instead of {instead_of} (logged once)")
            self._warned_fallback = True

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
            self.has_stage = 'stage_logits' in outs
            self._stage_index = outs.index('stage_logits') if self.has_stage else None
            if self.has_stage and self.crop_center is None and self.spot_nominal is None:
                # every stage model so far is a scan_plate1 model, trained left
                # of the laser spot: without the sidecar it scores the wrong place
                log(f"[RaPredictor] slot '{self.name}': WARNING stage model "
                    f"without crop_center_xy / spot_tile in its .json — centre crop, which "
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
        return self.run_all(chw_float32)[0]

    def run_all(self, chw_float32):
        """(Ra, stage_logits) — stage_logits is None for a graph without them."""
        outs = self.session.run(None, {self.input_name: chw_float32})
        logits = None
        if self.has_stage:
            logits = np.asarray(outs[self._stage_index], dtype=np.float64).reshape(-1)
        return float(np.squeeze(outs[self.output_index])), logits


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

    @staticmethod
    def preprocess_spot(img, nominal):
        """Full frame (mono, or BGR) -> ((1, 3, 900, 900) float32, info).

        The UPR_Ra_inference package's load_tile + to_input: grey frame ->
        spot_crop (1100 tile on the laser spot, spot filled in) -> centre 900
        -> (x / 255 - 0.5) / 0.5, grey copied to 3 channels. info = spot_x,
        spot_y, peak, masked_px, mean, sat (of the 900 window) and flags.
        The caller has checked the frame size (ModelSlot.prep_for).
        """
        if img is None:
            raise ValueError('no image')
        if cv2 is None:
            raise RuntimeError('cv2 unavailable')
        grey = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tile, info = spot_crop(grey, nominal)
        if tile.shape != (SPOT_TILE, SPOT_TILE):
            raise ValueError(f"spot at ({info['spot_x']}, {info['spot_y']}) "
                             f"too close to the frame border")
        o = (SPOT_TILE - CROP_SIZE) // 2
        win = tile[o:o + CROP_SIZE, o:o + CROP_SIZE]
        info['mean'] = round(float(win.mean()), 1)
        info['sat'] = round(float((win >= 250).mean()), 4)
        info['flags'] = spot_flags(info, nominal)
        x = win.astype(np.float32) / 255.0
        x = (x - 0.5) / 0.5
        chw = np.ascontiguousarray(
            np.broadcast_to(x, (1, 3, CROP_SIZE, CROP_SIZE)), dtype=np.float32)
        return chw, info

    def predict(self, bgr):
        """Score one frame. Returns (results, elapsed_s).

        results is an ordered list of (name, value); value is None when that
        slot is not loaded or produced a non-finite number. A non-finite Ra is
        discarded rather than passed on — it would otherwise land in a CSV as a
        real measurement.
        """
        results, elapsed, _detail = self.predict_detail(bgr)
        return results, elapsed

    def predict_detail(self, bgr):
        """predict() plus what the frame itself said: (results, elapsed_s, detail).

        detail: `spot` (spot_crop's info of a spot-tile frame, or None),
        `stage` / `stage_probs` (from the first loaded slot with a stage head
        and stage names — only the trained finishing / roughing pair is
        compared, as in the package) and `flags` (the frame's spot_flags plus
        `<slot>_ra_out_of_range` when a value is outside that slot's training
        range for the stage, +-10 %). Flags are logged, never acted on here.
        """
        started = time.time()
        if bgr is None:
            raise ValueError('no image')
        prepared = {}               # slots sharing a preprocessing share its result
        results = []
        detail = {'spot': None, 'stage': None, 'stage_probs': None, 'flags': []}
        for slot in self.slots:
            if not slot.loaded:
                results.append((slot.name, None))
                continue
            key = slot.prep_for(bgr.shape, self.log)
            try:
                if key not in prepared:
                    if key is not None and key[0] == 'spot':
                        prepared[key] = self.preprocess_spot(bgr, key[1])
                    else:
                        prepared[key] = (self.preprocess(bgr, key), None)
                chw, info = prepared[key]
                value, logits = slot.run_all(chw)
            except Exception as e:
                self.log(f"[RaPredictor] slot '{slot.name}' inference failed: {e}")
                results.append((slot.name, None))
                continue
            value = value if np.isfinite(value) else None
            results.append((slot.name, value))
            if info is not None and detail['spot'] is None:
                detail['spot'] = info
                detail['flags'] += info['flags']
            if detail['stage'] is None and logits is not None and slot.stages:
                e = np.exp(logits - logits.max())
                prob = e / e.sum()
                k = int(np.argmax(prob[:2]))    # nc is not trained by every model
                detail['stage'] = slot.stages[k]
                detail['stage_probs'] = {n: round(float(p), 4)
                                         for n, p in zip(slot.stages, prob)}
            rng = (slot.ra_range or {}).get(detail['stage'])
            if value is not None and rng:
                lo, hi = rng
                margin = 0.1 * (hi - lo)
                if not (lo - margin <= value <= hi + margin):
                    detail['flags'].append(f'{slot.name}_ra_out_of_range')
        if detail['flags']:
            sp = detail['spot'] or {}
            self.log(f"[RaPredictor] WARNING frame flagged: {';'.join(detail['flags'])}"
                     + (f" (spot ({sp['spot_x']}, {sp['spot_y']}) peak {sp['peak']} "
                        f"masked {sp['masked_px']} px, mean {sp['mean']} sat {sp['sat']})"
                        if sp else '')
                     + ' — value kept, see UPR_Ra_inference README_ko.md §4')
        return results, time.time() - started, detail
