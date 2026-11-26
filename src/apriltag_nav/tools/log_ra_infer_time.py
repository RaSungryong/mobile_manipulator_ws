#!/usr/bin/env python3
"""Write the Ra model inference time of every step estimate to a log file.

A passive listener: it subscribes to /arm/step_state (the latched state
arm_node publishes after each Estimate) and appends one line per estimate to

    <run folder>/<run>_infer_time.log        (results/scan_images/<run>/)

so it can be started while a scan is already waiting at a stop — no node is
restarted and nothing is commanded. The numbers are arm_node's own: per-model
`infer_s` (the ONNX run alone), `prep_s` (frame -> model input), `infer_s`
total (= prep + every model) and `elapsed_s` (the whole Estimate call). The
same values are in <run>_step_eval.csv; this is the readable log.

/arm/scan_progress `finished` appends a summary (mean / min / max per model).
The process exits when the ROS master goes away.

    python3 src/apriltag_nav/tools/log_ra_infer_time.py
"""
import json
import os
import threading
import time

import rosgraph
import rospy
from std_msgs.msg import String


class InferTimeLog:
    def __init__(self):
        self.lock = threading.Lock()
        self.seen = set()          # (run, index, group, point, estimate no)
        self.stats = {}            # log path -> {name: [seconds]}
        self.current = None        # log path of the run being scanned

    @staticmethod
    def _path(st):
        out_dir = st.get('image_dir') or ''
        if not out_dir or not os.path.isdir(out_dir):
            return None
        run = st.get('run') or os.path.basename(out_dir.rstrip('/'))
        return os.path.join(out_dir, f'{run}_infer_time.log')

    def _append(self, path, lines):
        new = not os.path.isfile(path)
        with open(path, 'a') as f:
            if new:
                f.write('# Ra model inference time, one line per step estimate '
                        '(* = the model whose Ra is used)\n'
                        '# model times = the ONNX run alone; prep = frame -> model input; '
                        'total = prep + every model; call = the whole Estimate\n')
            for ln in lines:
                f.write(ln + '\n')

    def on_state(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if not st or st.get('kind') != 'point':
            return
        path = self._path(st)
        if path is None:
            return
        sp = st.get('source_point_id')
        pt = (f"g{st.get('group_id')} p{st.get('point_id')}"
              + (f' sp{sp}' if sp is not None else '')
              + f" ({st.get('index')}/{st.get('total')})")
        lines = []
        with self.lock:
            self.current = path
            for e in st.get('estimates') or []:
                key = (path, st.get('index'), st.get('group_id'), st.get('point_id'), e.get('n'))
                if key in self.seen:
                    continue
                self.seen.add(key)
                stamp = time.strftime('%Y-%m-%d %H:%M:%S')
                if not e.get('ok'):
                    lines.append(f"{stamp}  {pt}  #{e.get('n')}  FAILED: {e.get('message', '')}"
                                 f"  call {e.get('elapsed_s')} s")
                    continue
                acc = self.stats.setdefault(path, {})
                parts = []
                for m in e.get('models') or []:
                    t = m.get('infer_s')
                    if t is None:
                        continue
                    acc.setdefault(m['name'], []).append(float(t))
                    parts.append(f"{m['name']}{'*' if m.get('selected') else ''} {float(t):.3f} s")
                for name, key_ in (('prep', 'prep_s'), ('total', 'infer_s'), ('call', 'elapsed_s')):
                    v = e.get(key_)
                    if v is not None:
                        acc.setdefault(name, []).append(float(v))
                        parts.append(f'{name} {float(v):.3f} s')
                lines.append(f"{stamp}  {pt}  #{e.get('n')}  "
                             f"Ra {float(e['ra_mean']):.4f}"
                             + (f" ({e['stage']})" if e.get('stage') else '')
                             + '  |  ' + '  '.join(parts)
                             + '  |  ' + ' '.join(e.get('images') or []))
            if lines:
                try:
                    self._append(path, lines)
                except OSError as ex:
                    rospy.logerr(f'[infer-log] cannot write {path}: {ex}')
                    return
        for ln in lines:
            rospy.loginfo(f'[infer-log] {ln}')

    def on_progress(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev.get('phase') != 'finished':
            return
        with self.lock:
            path = self.current
            acc = self.stats.get(path) or {}
            if not path or not acc:
                return
            n = max(len(v) for v in acc.values())
            lines = [f"# {time.strftime('%Y-%m-%d %H:%M:%S')}  summary of {n} estimate(s)"
                     + (' (scan cancelled)' if ev.get('cancelled') else '')]
            for name, v in acc.items():
                lines.append(f'#   {name:<12} mean {sum(v) / len(v):.3f} s   '
                             f'min {min(v):.3f}   max {max(v):.3f}   n {len(v)}')
            try:
                self._append(path, lines)
            except OSError as ex:
                rospy.logerr(f'[infer-log] cannot write {path}: {ex}')


def main():
    rospy.init_node('ra_infer_time_log', anonymous=True, disable_signals=False)
    log = InferTimeLog()
    rospy.Subscriber('/arm/step_state', String, log.on_state, queue_size=64)
    rospy.Subscriber('/arm/scan_progress', String, log.on_progress, queue_size=64)
    rospy.loginfo('[infer-log] listening on /arm/step_state — '
                  'log: <run folder>/<run>_infer_time.log')
    while not rospy.is_shutdown():
        time.sleep(5.0)
        if not rosgraph.is_master_online():
            break


if __name__ == '__main__':
    main()
