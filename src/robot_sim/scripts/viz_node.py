#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_node — live top-down view of the simulated cell.

    rosrun robot_sim viz_node.py                 # interactive window
    rosrun robot_sim viz_node.py --snapshot F.png  # render one frame, exit

Shows: floor tags (colored by zone), cross tags (magenta +), the plates,
the robot body with heading, front/hand camera positions, rings around
the tags each camera currently detects, and the driven trail. Everything
is drawn from the sim's own topics — the viewer holds no transform
knowledge of its own.
"""
import argparse
import json
import math
import sys
import threading

import matplotlib

_ARGS = None


def _parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--snapshot', default=None,
                    help='render one frame to this PNG and exit')
    ap.add_argument('--watch', type=float, default=0.0,
                    help='with --snapshot: record this many seconds '
                         'of trail before saving')
    ap.add_argument('--rate', type=float, default=10.0)
    return ap.parse_known_args()[0]


_ARGS = _parse()
matplotlib.use('Agg' if _ARGS.snapshot else 'Qt5Agg')

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import yaml  # noqa: E402
import rospy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from robot_msgs.msg import AprilTagDetectionArray  # noqa: E402

from apriltag_nav.paths import MAP_PATH  # noqa: E402
import rospkg  # noqa: E402

ZONE_COLOR = {'B': '#2e7d32', 'C': '#1565c0', 'D': '#6d4c41',
              'E': '#00838f', 'A': '#7986cb', 'DOCK': '#8e24aa'}
BODY_L, BODY_W = 0.90, 0.70
TRAIL_N = 4000


class Viz:
    def __init__(self):
        rospy.init_node('sim_viz', anonymous=True)
        self.lock = threading.Lock()
        self.truth = None
        self.trail = []
        self.seen = {'front_cam': set(), 'hand_cam': set()}

        rospy.Subscriber('/robot_sim/ground_truth', String, self._cb_truth,
                         queue_size=1)
        rospy.Subscriber('/front_cam/tag_detections',
                         AprilTagDetectionArray, self._cb_det,
                         callback_args='front_cam', queue_size=1)
        rospy.Subscriber('/hand_cam/tag_detections',
                         AprilTagDetectionArray, self._cb_det,
                         callback_args='hand_cam', queue_size=1)

        # static world
        self.floor = yaml.safe_load(open(MAP_PATH))['tags']
        ptl = rospkg.RosPack().get_path('path_tag_locator')
        self.cross = []
        for fn in ('reference_tags.yaml', 'reference_tags_plate2.yaml'):
            try:
                for r in yaml.safe_load(open(ptl + '/config/' + fn))[
                        'reference_tags']:
                    self.cross.append((r['id'], r['position_m'][0],
                                       r['position_m'][1]))
            except Exception:
                pass

        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        self.fig.canvas.manager.set_window_title('robot_sim') \
            if not _ARGS.snapshot else None

    def _cb_truth(self, msg):
        with self.lock:
            self.truth = json.loads(msg.data)
            self.trail.append((self.truth['x'], self.truth['y']))
            if len(self.trail) > TRAIL_N:
                self.trail = self.trail[-TRAIL_N:]

    def _cb_det(self, msg, cam):
        with self.lock:
            self.seen[cam] = {d.id for d in msg.detections}

    # ------------------------------------------------------------------
    def draw(self):
        with self.lock:
            truth = dict(self.truth) if self.truth else None
            trail = list(self.trail)
            seen = {k: set(v) for k, v in self.seen.items()}

        ax = self.ax
        ax.clear()
        ax.set_aspect('equal')
        ax.set_facecolor('#fafafa')
        ax.grid(True, lw=0.3, alpha=0.5)
        ax.set_xlabel('world x [m] (east)')
        ax.set_ylabel('world y [m] (north)')

        # plates (estimated outlines, centres 0 / 3.89)
        for cx in (0.0, 3.89):
            ax.add_patch(Rectangle((cx - 1.26, -1.6), 2.52, 3.2,
                                   fill=False, ls='--', ec='#909090'))

        # floor tags
        for tid, info in self.floor.items():
            c = ZONE_COLOR.get(info.get('zone', 'A'), '#555555')
            ring = (tid in seen['front_cam'])
            ax.plot(info['x'], info['y'], 's', ms=7 if ring else 5,
                    color=c, mec='#ff6f00' if ring else c,
                    mew=2.0 if ring else 0.5, zorder=3)
            if info.get('type') in ('DOCK', 'PIVOT'):
                ax.annotate(str(tid), (info['x'], info['y']),
                            textcoords='offset points', xytext=(0, 7),
                            fontsize=6, ha='center', color='#555555')

        # cross tags
        for cid, x, y in self.cross:
            ring = (cid in seen['hand_cam'])
            ax.plot(x, y, '+', ms=12 if ring else 9,
                    mew=3 if ring else 2,
                    color='#d500f9' if not ring else '#ff1744', zorder=4)
            ax.annotate(str(cid), (x, y), textcoords='offset points',
                        xytext=(6, 4), fontsize=6, color='#8e24aa')

        # trail
        if len(trail) > 1:
            xs, ys = zip(*trail)
            ax.plot(xs, ys, '-', lw=1.0, color='#e53935', alpha=0.6,
                    zorder=2)

        # robot
        if truth:
            x, y = truth['x'], truth['y']
            th = math.radians(truth['theta_deg'])
            c, s = math.cos(th), math.sin(th)
            corners = []
            for dx, dy in ((BODY_L / 2, BODY_W / 2),
                           (BODY_L / 2, -BODY_W / 2),
                           (-BODY_L / 2, -BODY_W / 2),
                           (-BODY_L / 2, BODY_W / 2)):
                corners.append((x + c * dx - s * dy, y + s * dx + c * dy))
            corners.append(corners[0])
            bx, by = zip(*corners)
            ax.plot(bx, by, '-', lw=1.5, color='#212121', zorder=5)
            ax.annotate('', xy=(x + 0.45 * c, y + 0.45 * s),
                        xytext=(x, y),
                        arrowprops=dict(arrowstyle='->', lw=2,
                                        color='#212121'), zorder=6)
            fc = truth.get('fc')
            if fc:
                ax.plot(fc[0], fc[1], 'o', ms=6, color='#ff6f00',
                        zorder=6)
            hc = truth.get('hc')
            if hc:
                ax.plot(hc[0], hc[1], 'o', ms=6, color='#ff1744',
                        zorder=6)
                ax.annotate('hand cam z=%.2f' % hc[2], (hc[0], hc[1]),
                            textcoords='offset points', xytext=(8, -10),
                            fontsize=7, color='#ff1744')
            ax.set_title(
                'robot (%.3f, %.3f, %.1f°)  v=%.3f m/s  lift %.0f mm   '
                '— front cam ●  hand cam ●  detected = ring'
                % (x, y, truth['theta_deg'], truth.get('v', 0.0),
                   truth.get('lift_mm', 0.0)), fontsize=10)
        else:
            ax.set_title('waiting for /robot_sim/ground_truth ...')

        ax.set_xlim(-2.6, 6.7)
        ax.set_ylim(-3.5, 4.2)

    def run(self):
        if _ARGS.snapshot:
            deadline = rospy.Time.now() + rospy.Duration(5.0)
            while (self.truth is None and not rospy.is_shutdown()
                   and rospy.Time.now() < deadline):
                rospy.sleep(0.1)
            if _ARGS.watch > 0:
                rospy.sleep(_ARGS.watch)
            self.draw()
            self.fig.savefig(_ARGS.snapshot, dpi=110,
                             bbox_inches='tight')
            print('snapshot ->', _ARGS.snapshot)
            return
        plt.ion()
        self.fig.show()
        rate = rospy.Rate(_ARGS.rate)
        while not rospy.is_shutdown():
            self.draw()
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == '__main__':
    Viz().run()
