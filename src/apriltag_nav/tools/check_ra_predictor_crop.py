#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check of RaPredictor's per-model crop (2026-10-27).

    python3 tools/check_ra_predictor_crop.py

No model file and no onnxruntime needed: the ONNX session is replaced by a fake
that returns the mean of the input, so WHICH 900x900 window was scored can be
read back from the value. Checks: a sidecar's crop_center_xy is used for a full
frame of frame_size, a smaller input (robot_ui ROI) falls back to the centre
crop once-logged, a model without the keys keeps the centre crop, slots sharing
a crop share the preprocessing, `output` is picked by name, bad sidecars are
refused, and the deployed resnet3D_multi.json says (2200, 1834) of 5472x3648.
"""
import json
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from apriltag_nav import paths                               # noqa: E402
from apriltag_nav import ra_predictor as rp                  # noqa: E402

n_ok = n_fail = 0


def check(cond, text):
    global n_ok, n_fail
    n_ok, n_fail = n_ok + bool(cond), n_fail + (not cond)
    print('  %s %s' % ('ok  ' if cond else 'FAIL', text))


class _Out:
    def __init__(self, name):
        self.name = name


class FakeSession:
    """Mean of the input as 'Ra'; optionally a stage_logits output first."""
    calls = 0

    def __init__(self, outputs):
        self._outs = outputs

    def get_inputs(self):
        return [_Out('input')]

    def get_outputs(self):
        return [_Out(n) for n in self._outs]

    def run(self, _names, feed):
        FakeSession.calls += 1
        x = feed['input']
        res = {'output': np.array([[x.mean()]], np.float32),
               'stage_logits': np.zeros((1, 3), np.float32)}
        return [res[n] for n in self._outs]


def make_slot(tmp, name, sidecar, outputs=('output',)):
    path = os.path.join(tmp, name + '.onnx')
    with open(path, 'wb') as f:
        f.write(b'\0' * 2048)          # not an LFS pointer (those are < 1 KB of text)
    if sidecar is not None:
        json.dump(sidecar, open(os.path.join(tmp, name + '.json'), 'w'))
    return path


def install_fake(outputs_by_path):
    import types
    ort = types.ModuleType('onnxruntime')

    class SessionOptions:
        intra_op_num_threads = 1
        graph_optimization_level = None
    ort.SessionOptions = SessionOptions
    ort.GraphOptimizationLevel = types.SimpleNamespace(ORT_ENABLE_ALL=None)
    ort.InferenceSession = lambda path, **kw: FakeSession(outputs_by_path[path])
    sys.modules['onnxruntime'] = ort


def window_value(img, cx, cy):
    """What the fake returns for the 900 window at (cx, cy) of a mono image."""
    h = rp.CROP_SIZE // 2
    w = img[cy - h:cy + h, cx - h:cx + h].astype(np.float32) / 255.0
    return float(((w - 0.5) / 0.5).mean())


def main():
    if rp.cv2 is None:
        sys.exit('cv2 is needed (RaPredictor.preprocess uses it)')
    tmp = tempfile.mkdtemp()
    try:
        side = {'crop_center_xy': [2200, 1834], 'frame_size': [5472, 3648]}
        p_new = make_slot(tmp, 'new', side)
        p_old = make_slot(tmp, 'old', None)
        p_new2 = make_slot(tmp, 'new2', side)
        install_fake({p_new: ['stage_logits', 'output'], p_old: ['output'], p_new2: ['output']})

        # a frame whose brightness encodes x, so every window has its own mean
        frame = np.tile((np.arange(5472) * 255 // 5471).astype(np.uint8), (3648, 1))
        logs = []
        pred = rp.RaPredictor([('new', p_new), ('old', p_old), ('new2', p_new2)], log=logs.append)
        check(pred.load() == 3, 'three fake slots load')
        new, old, _ = pred.slots
        check(new.crop_center == (2200, 1834) and new.frame_size == (5472, 3648),
              'crop_center / frame_size read from the sidecar')
        check(new.output_index == 1, '`output` found by name behind stage_logits')
        check(old.crop_center is None, 'no sidecar -> centre crop')

        FakeSession.calls = 0
        res, _ = pred.predict(frame)
        r = dict(res)
        check(abs(r['new'] - window_value(frame, 2200, 1834)) < 1e-6,
              'full frame scored at (2200, 1834): %.4f' % r['new'])
        check(abs(r['old'] - window_value(frame, 2736, 1824)) < 1e-6,
              'old slot scored at the frame centre (2736, 1824): %.4f' % r['old'])
        check(abs(r['new2'] - r['new']) < 1e-9, 'second slot with the same crop gives the same value')
        # warm-up runs happened in load(); predict() runs one per slot
        check(FakeSession.calls == 3, 'one run per slot (%d)' % FakeSession.calls)

        # bit-exact against slicing the window by hand and centre-cropping it
        win = frame[1834 - 450:1834 + 450, 2200 - 450:2200 + 450]
        check(np.array_equal(rp.RaPredictor.preprocess(frame, (2200, 1834)),
                             rp.RaPredictor.preprocess(win)),
              'preprocess(frame, crop) == preprocess(window), bit for bit')
        tile = frame[1834 - 550:1834 + 550, 2200 - 550:2200 + 550]
        check(np.array_equal(rp.RaPredictor.preprocess(frame, (2200, 1834)),
                             rp.RaPredictor.preprocess(tile)),
              '== the 1100 training tile centre-cropped (machine_vision test path)')

        roi = frame[1000:2400, 1500:3100]
        logs.clear()
        r1 = dict(pred.predict(roi)[0])
        pred.predict(roi)
        check(abs(r1['new'] - window_value(roi, 800, 700)) < 1e-6, 'ROI input -> centre crop')
        check(sum('not a 5472x3648 frame' in s for s in logs) == 2,
              'fallback logged once per slot (2 slots with a crop)')

        lg = []
        q = rp.RaPredictor([('s', make_slot(tmp, 'nosc', None))], log=lg.append)
        install_fake({os.path.join(tmp, 'nosc.onnx'): ['output', 'stage_logits']})
        q.load()
        check(any('WARNING stage model' in s for s in lg), 'stage model without sidecar warns')

        for bad, why in (({'crop_center_xy': [5300, 1834], 'frame_size': [5472, 3648]}, 'does not fit'),
                         ({'crop_center_xy': [2200, 1834]}, 'frame_size')):
            pth = make_slot(tmp, 'bad', bad)
            install_fake({pth: ['output']})
            q = rp.RaPredictor([('b', pth)], log=lambda s: None)
            check(q.load() == 0 and why in q.failures()['b'], 'refused: %s' % why)

        # a clone without git-lfs checks the model out as a pointer file
        ptr = os.path.join(tmp, 'ptr.onnx')
        with open(ptr, 'w') as f:
            f.write('version https://git-lfs.github.com/spec/v1\noid sha256:7440d5ca\nsize 132830111\n')
        q = rp.RaPredictor([('p', ptr)], log=lambda s: None)
        check(q.load() == 0 and 'Git LFS pointer' in q.failures()['p'], 'LFS pointer refused with the fix in the message')

        deployed = os.path.splitext(paths.MODEL_PATH)[0] + '.json'
        if os.path.exists(deployed):
            meta = json.load(open(deployed))
            check(meta.get('crop_center_xy') == [2200, 1834] and meta.get('frame_size') == [5472, 3648],
                  '%s: crop (2200, 1834) of 5472x3648' % os.path.basename(deployed))
        else:
            check(False, 'sidecar of paths.MODEL_PATH exists: %s' % deployed)
        print('  [info] paths.MODEL_PATH = %s (%s)' % (
            os.path.relpath(paths.MODEL_PATH, paths.WS_DIR),
            'present' if os.path.exists(paths.MODEL_PATH) else 'NOT on disk — git lfs install && git lfs pull'))
    finally:
        shutil.rmtree(tmp)
    print('%d ok, %d failed' % (n_ok, n_fail))
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
