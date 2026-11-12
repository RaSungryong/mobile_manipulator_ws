#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Inference node — on-demand Ra (surface roughness) prediction.

Owns the ONNX sessions the way basler_camera_node owns the camera: loaded and
warmed up once at startup, reached by everyone else through a service. A UI must
NOT load a model of its own — two resident copies is ~250 MB of duplicated RAM
and two answers that drift apart the moment the configured paths do.

ONNX only. See ra_predictor.py for why no torch/.pt backend exists.

This node is the ON-DEMAND path. The scan pipeline (scan_pipeline.py ->
inference_interface.py) runs its own session inside arm_node, but both now go
through the same RaPredictor, so the two cannot disagree about preprocessing —
which they previously did.

Interface
---------
Services:
  /inference/predict   robot_msgs/PredictRa   image in, Ra out
Publishes:
  /inference/state     std_msgs/String  (latched) which slots came up

⚠️ PREPROCESSING IS PER MODEL (its sidecar .json) AND NEVER A RESIZE — native
Basler pixels in a 900x900 window: around the laser spot for the spot-tile
models (ra_standard / ra_new, 2026-11-11), a fixed window for resnet3D_multi,
the centre for a model without a sidecar. See ra_predictor.py. Ra values from
different models / windows are not comparable with each other — see CLAUDE.md.

ROS parameters
--------------
  ~model_primary    .onnx for the primary slot   (default paths.MODEL_PATH,
                    i.e. the same graph the scan pipeline uses)
  ~model_secondary  .onnx for the second slot    (default
                    paths.RA_MODEL_SECONDARY = '' since 2026-11-12: primary
                    (ra_standard) only, so the reply and the UI carry Ra
                    alone and elapsed_s is Ra's inference time;
                    paths.RA_NEW_MODEL_PATH brings ra_new back)
  ~require_all      refuse to start unless every configured slot loads
                    (default false)

A slot whose file is missing is skipped with a warning rather than taken as
fatal: losing the second model should not cost the operator the primary one.
Its entry still appears in the reply with NaN, so a caller can see it was tried.

A slot is NAMED after its sidecar's `model` (ra_standard / ra_new), so the
reply and the UI say which quantity a number is; without a sidecar the names
are primary / secondary as before. The reply's `message` also carries what a
spot-tile frame said about itself: the stage seen in the image and any flags
(spot not found, saturated, ... — ra_predictor.spot_flags).
"""

import math

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge

from robot_msgs.srv import PredictRa, PredictRaResponse

from apriltag_nav import paths
from apriltag_nav.ra_predictor import RaPredictor, sidecar_meta


class InferenceNode:
    def __init__(self):
        rospy.init_node('inference_node', anonymous=False)

        primary = rospy.get_param('~model_primary', paths.MODEL_PATH)
        secondary = rospy.get_param('~model_secondary', paths.RA_MODEL_SECONDARY)
        require_all = bool(rospy.get_param('~require_all', False))

        self._bridge = CvBridge()
        self._state_pub = rospy.Publisher('/inference/state', String,
                                          queue_size=1, latch=True)
        self._state_pub.publish(String('loading'))

        # Order matters: the first slot is the one the reply's `ra` field
        # carries, and it defaults to the scan pipeline's graph so an on-demand
        # number is comparable with the scan CSVs.
        slots = [(self._slot_name(primary, 'primary'), primary)]
        if secondary:
            name = self._slot_name(secondary, 'secondary')
            slots.append((name if name != slots[0][0] else 'secondary', secondary))

        self._predictor = RaPredictor(slots=slots, log=rospy.loginfo)
        loaded = self._predictor.load()
        failures = self._predictor.failures()

        for name, err in failures.items():
            rospy.logwarn(f"[Inference] slot '{name}' unavailable: {err}")

        if require_all and failures:
            rospy.logfatal(f"[Inference] ~require_all is set and these slots "
                           f"failed: {sorted(failures)}. Shutting down.")
            rospy.signal_shutdown('required model missing')
            return

        if loaded == 0:
            # Not fatal by default: the node stays up and answers
            # success=false, which is a far clearer symptom for an operator
            # than a service that does not exist at all.
            rospy.logerr("[Inference] No model loaded — /inference/predict will "
                         "answer success=false for every request.")

        rospy.Service('/inference/predict', PredictRa, self._srv_predict)

        state = f"ready slots={self._predictor.loaded_names() or 'none'}"
        self._state_pub.publish(String(state))
        rospy.loginfo(f"[Inference] {state} — /inference/predict")

    @staticmethod
    def _slot_name(path, fallback):
        """The sidecar's `model` name, so a reply says WHICH Ra a value is."""
        try:
            return str(sidecar_meta(path).get('model') or fallback)
        except Exception:
            return fallback         # a broken sidecar fails the slot's load, visibly

    def _srv_predict(self, req):
        resp = PredictRaResponse()
        resp.tag = req.tag
        resp.ra = float('nan')
        resp.model_names = self._predictor.names()
        resp.ra_values = [float('nan')] * len(resp.model_names)
        resp.elapsed_s = 0.0

        if not self._predictor.loaded_names():
            resp.success = False
            resp.message = 'no model loaded'
            return resp

        try:
            # desired_encoding='bgr8' converts mono8 and rgb8 inputs too, so a
            # caller that hands over whatever its camera published still gets
            # the channel order preprocess() expects.
            bgr = self._bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"[Inference] cv_bridge conversion failed: {e}")
            resp.success = False
            resp.message = f'bad image: {e}'
            return resp

        try:
            results, elapsed, detail = self._predictor.predict_detail(bgr)
        except Exception as e:
            rospy.logerr(f"[Inference] predict failed: {e}")
            resp.success = False
            resp.message = f'inference failed: {e}'
            return resp

        resp.elapsed_s = float(elapsed)
        resp.model_names = [name for name, _ in results]
        resp.ra_values = [float('nan') if v is None else float(v)
                          for _, v in results]
        if results and results[0][1] is not None:
            resp.ra = float(results[0][1])

        produced = [n for n, v in results if v is not None]
        missing = [n for n, v in results if v is None]

        # Success means at least one real number came back. A caller reading
        # only `ra` must still check for NaN — hence naming what is missing.
        resp.success = bool(produced)
        if not produced:
            resp.message = 'every slot failed or returned a non-finite value'
        elif missing:
            resp.message = (f"ok ({', '.join(produced)}); "
                            f"NaN from: {', '.join(missing)}")
        else:
            resp.message = 'ok'

        extra = ''
        if detail.get('stage'):
            p = (detail.get('stage_probs') or {}).get(detail['stage'])
            extra += f"; stage {detail['stage']}" + (f' (p {p:.2f})' if p is not None else '')
        if detail.get('flags'):
            extra += f"; FLAGS: {';'.join(detail['flags'])}"
        if resp.success:
            resp.message += extra
            pairs = ' '.join(f'{n}={_fmt(v)}' for n, v in
                             zip(resp.model_names, resp.ra_values))
            rospy.loginfo(f"[Inference] tag='{req.tag}' {pairs}{extra} "
                          f"in {elapsed:.3f}s")
        return resp


def _fmt(value):
    return 'NaN' if math.isnan(value) else f'{value:.4f}'


if __name__ == '__main__':
    InferenceNode()
    rospy.spin()
