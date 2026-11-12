#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check of RaPredictor's per-model preprocessing (2026-10-27, spot
tiles 2026-11-11).

    python3 tools/check_ra_predictor_crop.py

No model file and no onnxruntime needed: the ONNX session is replaced by a fake
that returns the mean of the input, so WHICH 900x900 window was scored can be
read back from the value. Checks: a sidecar's crop_center_xy is used for a full
frame of frame_size, a smaller input (robot_ui ROI) falls back to the centre
crop once-logged, a model without the keys keeps the centre crop, slots sharing
a crop share the preprocessing, `output` is picked by name, bad sidecars are
refused, an LFS pointer is refused. Spot-tile models (ra_standard / ra_new,
2026-11-11): the window follows a synthetic laser spot and the spot is filled
in, two spot slots share one preprocessing, the stage and the flags come out
of predict_detail, bad spot sidecars are refused, and the deployed sidecar of
paths.MODEL_PATH is a spot_tile one. FIRST, when the delivery zip
(model/UPR_Ra_inference_ko.zip) and the real models are on disk, the REAL
models score the zip's two sample frames and must match its
expected_output.csv within 0.001 um (skipped with a note otherwise).
"""
import csv
import io
import json
import os
import shutil
import sys
import tempfile
import zipfile

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
    return bool(cond)


class _Out:
    def __init__(self, name):
        self.name = name


class FakeSession:
    """Mean of the input as 'Ra'; optionally a stage_logits output first."""
    calls = 0
    logits = (0.0, 0.0, 0.0)

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
               'stage_logits': np.array([FakeSession.logits], np.float32)}
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


def check_real_samples():
    """The deployed models on the delivery's own sample frames (real onnxruntime)."""
    zpath = os.path.join(paths.MODEL_DIR, 'UPR_Ra_inference_ko.zip')
    models = [('standard', paths.MODEL_PATH), ('new', paths.RA_NEW_MODEL_PATH)]
    if not os.path.exists(zpath) or not all(p and os.path.exists(p) for _, p in models):
        print('  [skip] real-model check: needs %s and both models on disk' % os.path.relpath(zpath, paths.WS_DIR))
        return
    pred = rp.RaPredictor(models, log=lambda s: None)
    if not check(pred.load() == 2, 'real ra_standard / ra_new load (%s)' % (pred.failures() or 'ok')):
        return
    with zipfile.ZipFile(zpath) as z:
        root = 'UPR_Ra_inference/samples/'
        rows = list(csv.DictReader(io.TextIOWrapper(z.open(root + 'expected_output.csv'), encoding='utf-8-sig')))
        for e in rows:
            name = os.path.basename(e['image'])
            im = rp.cv2.imdecode(np.frombuffer(z.read(root + name), np.uint8), rp.cv2.IMREAD_UNCHANGED)
            res, _dt, d = pred.predict_detail(im)
            r, sp = dict(res), d['spot']
            err = max(abs(r['standard'] - float(e['ra_standard'])), abs(r['new'] - float(e['ra_new'])))
            check(err < 1e-3, '%s: standard %.4f / new %.4f, %.5f um from expected_output.csv'
                  % (name, r['standard'], r['new'], err))
            check((sp['spot_x'], sp['spot_y'], sp['masked_px']) == (int(e['spot_x']), int(e['spot_y']), int(e['masked_px']))
                  and d['stage'] == e['stage'] and not d['flags'],
                  '  spot (%d, %d), %d px filled, stage %s, no flags — as expected'
                  % (sp['spot_x'], sp['spot_y'], sp['masked_px'], d['stage']))


def check_spot_tiles(tmp):
    """Spot-tile slots against the fake session (the scored window is read back)."""
    side = {'preprocess': 'spot_tile', 'nominal_spot_xy': [2813, 1906], 'frame_size': [5472, 3648],
            'stages': ['finishing', 'roughing', 'nc'],
            'train_ra_range': {'finishing': [-0.9, -0.2], 'roughing': [5.0, 6.0]}}
    p_a, p_b = make_slot(tmp, 'spot_a', side), make_slot(tmp, 'spot_b', side)
    p_fix = make_slot(tmp, 'fixed', {'crop_center_xy': [2200, 1834], 'frame_size': [5472, 3648]})
    install_fake({p_a: ['output', 'stage_logits'], p_b: ['output', 'stage_logits'], p_fix: ['output']})
    logs = []
    pred = rp.RaPredictor([('a', p_a), ('b', p_b), ('fixed', p_fix)], log=logs.append)
    check(pred.load() == 3, 'two spot-tile slots + a fixed-window slot load')
    a = pred.slots[0]
    check(a.spot_nominal == (2813, 1906) and a.crop_center is None and a.frame_size == (5472, 3648),
          'spot_tile sidecar read: nominal spot (2813, 1906)')
    check(not any('WARNING stage model' in s for s in logs), 'a spot-tile stage model does not warn about its crop')

    # dim surface whose brightness encodes x, plus a bright disc 87 px right / 56 px up of nominal
    clean = np.tile((20 + np.arange(5472) * 100 // 5471).astype(np.uint8), (3648, 1))
    frame = clean.copy()
    rp.cv2.circle(frame, (2900, 1850), 30, 255, -1)
    calls = []
    real_spot_crop = rp.spot_crop
    rp.spot_crop = lambda *a, **k: (calls.append(1), real_spot_crop(*a, **k))[1]
    FakeSession.logits = (2.0, 0.0, -3.0)
    try:
        res, _dt, d = pred.predict_detail(frame)
    finally:
        rp.spot_crop = real_spot_crop
    r, sp = dict(res), d['spot']
    check(abs(sp['spot_x'] - 2900) <= 2 and abs(sp['spot_y'] - 1850) <= 2,
          'spot found at (%d, %d) for a disc at (2900, 1850)' % (sp['spot_x'], sp['spot_y']))
    check(sp['masked_px'] > 3000, 'spot filled in (%d px)' % sp['masked_px'])
    want = window_value(clean, sp['spot_x'], sp['spot_y'])
    check(abs(r['a'] - want) < 2e-3, 'window follows the spot, disc gone: %.4f vs the clean surface there %.4f' % (r['a'], want))
    check(abs(r['a'] - window_value(clean, 2813, 1906)) > 0.01, '... and is not the window at the nominal spot')
    check(r['b'] == r['a'] and len(calls) == 1, 'two spot slots share one spot_crop')
    check(abs(r['fixed'] - window_value(frame, 2200, 1834)) < 1e-6, 'the fixed-window slot beside them is unchanged')
    check(d['stage'] == 'finishing' and abs(d['stage_probs']['finishing'] - 0.8756) < 1e-3,
          'stage from the logits: %s %s' % (d['stage'], d['stage_probs']))
    check(d['flags'] == [], 'in-range Ra, clean frame: no flags')
    check(dict(pred.predict(frame)[0]) == r, 'predict() == predict_detail() values')
    bgr = rp.cv2.cvtColor(frame, rp.cv2.COLOR_GRAY2BGR)
    check(dict(pred.predict(bgr)[0]) == r, 'a bgr8 copy of the mono frame scores the same, bit for bit')

    logs.clear()
    FakeSession.logits = (0.0, 2.0, -3.0)           # roughing: its range [5, 6] cannot hold a mean in [-1, 1]
    _res, _dt, d = pred.predict_detail(frame)
    check(d['stage'] == 'roughing' and d['flags'] == ['a_ra_out_of_range', 'b_ra_out_of_range'],
          'Ra outside the stage range flagged per slot: %s' % d['flags'])
    check(sum('WARNING frame flagged' in s for s in logs) == 1 and dict(_res)['a'] is not None,
          'flagged frame logged once, value kept')
    FakeSession.logits = (2.0, 0.0, -3.0)
    _res, _dt, d = pred.predict_detail(np.zeros((3648, 5472), np.uint8))
    check('spot_not_found' in d['flags'] and 'underexposed' in d['flags'], 'black frame: %s' % d['flags'])
    _res, _dt, d = pred.predict_detail(np.full((3648, 5472), 255, np.uint8))
    check('saturated' in d['flags'], 'white frame: %s' % d['flags'])
    far = clean.copy()
    rp.cv2.circle(far, (2813 + 250, 1906), 30, 255, -1)
    _res, _dt, d = pred.predict_detail(far)
    check('spot_far_from_nominal' in d['flags'], 'spot 250 px from nominal: %s' % d['flags'])

    roi = frame[1000:2400, 1500:3100]
    logs.clear()
    r1 = dict(pred.predict(roi)[0])
    pred.predict(roi)
    check(abs(r1['a'] - window_value(roi, 800, 700)) < 1e-6 and sum('instead of the spot tile' in s for s in logs) == 2,
          'ROI input -> centre crop, logged once per spot slot')

    for bad, why in (({'preprocess': 'spot_tile', 'frame_size': [5472, 3648]}, 'nominal_spot_xy'),
                     ({'preprocess': 'spot_tile', 'nominal_spot_xy': [5000, 1906], 'frame_size': [5472, 3648]}, 'does not fit'),
                     ({'preprocess': 'resize'}, 'unknown preprocess')):
        pth = make_slot(tmp, 'badspot', bad)
        install_fake({pth: ['output']})
        q = rp.RaPredictor([('b', pth)], log=lambda s: None)
        check(q.load() == 0 and why in q.failures()['b'], 'refused: %s' % why)


def main():
    if rp.cv2 is None:
        sys.exit('cv2 is needed (RaPredictor.preprocess uses it)')
    check_real_samples()            # before the fake onnxruntime is installed
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

        check_spot_tiles(tmp)

        for label, mpath in (('MODEL_PATH', paths.MODEL_PATH), ('RA_NEW_MODEL_PATH', paths.RA_NEW_MODEL_PATH)):
            if not mpath:
                continue
            meta = rp.sidecar_meta(mpath)
            check(meta.get('preprocess') == 'spot_tile' and meta.get('nominal_spot_xy') == [2813, 1906]
                  and meta.get('frame_size') == [5472, 3648],
                  '%s sidecar: spot_tile near (2813, 1906) of 5472x3648' % os.path.basename(mpath))
            print('  [info] paths.%s = %s (%s)' % (
                label, os.path.relpath(mpath, paths.WS_DIR),
                'present' if os.path.exists(mpath) else 'NOT on disk'))
    finally:
        shutil.rmtree(tmp)
    print('%d ok, %d failed' % (n_ok, n_fail))
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
