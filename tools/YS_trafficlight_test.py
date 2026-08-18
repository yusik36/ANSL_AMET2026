"""Offline check of traffic_light_node's real classify()/best_lamp_area().

Imports the actual node module with the ROS pieces stubbed out, so the code
under test is the code that ships -- not a reimplementation.

Scenes match the contest setup as described 2026-08-18: a small signal box
standing on the ground ahead and to the RIGHT of the start position, about
four o'clock in the camera frame. Distractors are the ones that actually
fooled the old whole-frame argmax -- large saturated surfaces (grass beside
the track, a green mat, wood panelling) that dwarf the lamp in area.
"""
import math
import os
import sys
import types

import cv2
import numpy as np

# ---- stub the ROS surface so the module imports on a machine without ROS ----
for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs',
             'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'cv_bridge'):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy.node'].Node = type('Node', (), {})
sys.modules['rclpy.qos'].qos_profile_sensor_data = None
sys.modules['sensor_msgs.msg'].Image = type('Image', (), {})
sys.modules['std_msgs.msg'].Bool = type('Bool', (), {})
sys.modules['std_msgs.msg'].String = type('String', (), {})
sys.modules['cv_bridge'].CvBridge = type('CvBridge', (), {})
sys.modules['cv_bridge'].CvBridgeError = type('CvBridgeError', (Exception,), {})

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'physicar_vision', 'physicar_vision'))
import traffic_light_node as T

FAILURES = []
W, H = 480, 360

# Where the signal box sits in frame: lower right, about four o'clock.
LAMP_XY = (395, 250)
LAMP_R = 12                       # ~450 px, the measured order of magnitude


def check(name, ok, detail=''):
    print('  %-50s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))
    if not ok:
        FAILURES.append(name)


class Fake(T.TrafficLightNode):
    """Bypass ROS __init__, keep the real algorithm and the real defaults."""
    def __init__(self):
        self.min_blob_area_px = T.MIN_BLOB_AREA_PX
        self.max_blob_area_px = T.MAX_BLOB_AREA_PX
        self.min_circularity = T.MIN_CIRCULARITY
        self.roi_x_min_frac = T.ROI_X_MIN_FRAC
        self.roi_x_max_frac = T.ROI_X_MAX_FRAC
        self.roi_y_min_frac = T.ROI_Y_MIN_FRAC
        self.roi_y_max_frac = T.ROI_Y_MAX_FRAC
        self.sat_min = T.SAT_MIN
        self.val_min = T.VAL_MIN
        self.debug_candidates = False


def scene(lamp=None, grass=False, panel=False, lamp_xy=LAMP_XY, lamp_r=LAMP_R):
    """Build a BGR frame. lamp is 'red' or 'green'."""
    f = np.full((H, W, 3), 60, np.uint8)           # dull grey road
    if grass:
        # Green verges down both sides, thousands of px each, overlapping the
        # search window on the right. Saturation is set to ~140, inside the
        # range measured for real background surfaces on the practice rig
        # (never above ~160) rather than the lamp's own 255 -- a distractor
        # brighter than anything real would only prove the test wrong.
        cv2.rectangle(f, (0, int(H * 0.45)), (int(W * 0.16), H), (68, 150, 68), -1)
        cv2.rectangle(f, (int(W * 0.84), int(H * 0.45)), (W, H), (68, 150, 68), -1)
    if panel:
        cv2.rectangle(f, (int(W * 0.5), 20), (W - 10, 150), (95, 130, 165), -1)
    if lamp:
        bgr = (40, 40, 250) if lamp == 'red' else (60, 240, 60)
        cv2.circle(f, lamp_xy, lamp_r, bgr, -1)
    return f


def classify(node, frame):
    """Run the same crop + classify the node does per frame."""
    return node.classify(cv2.cvtColor(node.crop(frame), cv2.COLOR_BGR2HSV))


def main():
    n = Fake()
    print('defaults: area %d-%d px, circularity>%.2f, window x %.0f-%.0f%% y %.0f-%.0f%%\n'
          % (n.min_blob_area_px, n.max_blob_area_px, n.min_circularity,
             n.roi_x_min_frac * 100, n.roi_x_max_frac * 100,
             n.roi_y_min_frac * 100, n.roi_y_max_frac * 100))
    print('signal box at %s in a %dx%d frame, lamp ~%.0f px'
          % (LAMP_XY, W, H, math.pi * LAMP_R ** 2))

    # Print what the synthetic colours actually are, so a passing run cannot
    # quietly be passing against distractors that are easier than reality.
    def hsv_of(bgr):
        return cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
    print('colours (H,S,V): lamp red %s  lamp green %s  grass %s  panel %s'
          % (hsv_of((40, 40, 250)), hsv_of((60, 240, 60)),
             hsv_of((68, 150, 68)), hsv_of((95, 130, 165))))
    print('gates: sat>=%d val>=%d  (real background measured S<=160, lamp S->255)\n'
          % (n.sat_min, n.val_min))

    print('the two readings that actually matter')
    check('red lamp alone -> RED', classify(n, scene('red')) == 'RED')
    check('green lamp alone -> GREEN', classify(n, scene('green')) == 'GREEN')

    print('\nthe distractors that beat the old whole-frame argmax')
    check('grass verges alone -> NONE', classify(n, scene(grass=True)) == 'NONE')
    check('wood panel alone -> NONE', classify(n, scene(panel=True)) == 'NONE')
    check('red lamp still wins over grass',
          classify(n, scene('red', grass=True)) == 'RED')
    check('green lamp not drowned by grass',
          classify(n, scene('green', grass=True)) == 'GREEN')
    check('green lamp survives grass and panel together',
          classify(n, scene('green', grass=True, panel=True)) == 'GREEN')

    print('\nsafety bias')
    both = scene('red', lamp_xy=(395, 225))
    cv2.circle(both, (395, 275), LAMP_R, (60, 240, 60), -1)
    check('red and green together -> RED (cautious)', classify(n, both) == 'RED')

    print('\nnothing there')
    check('empty scene -> NONE', classify(n, scene()) == 'NONE')

    print('\nsize gates')
    check('speckle below min area -> NONE',
          classify(n, scene('red', lamp_r=3)) == 'NONE', '(~28 px)')
    check('blob above max area -> NONE',
          classify(n, scene('red', lamp_r=50, lamp_xy=(330, 230))) == 'NONE',
          '(~7854 px)')

    print('\nsearch window')
    check('same lamp on the far LEFT is out of window -> NONE',
          classify(n, scene('green', lamp_xy=(60, 250))) == 'NONE')
    check('same lamp high in the sky is out of window -> NONE',
          classify(n, scene('green', lamp_xy=(395, 30))) == 'NONE')

    print()
    if FAILURES:
        print('FAILED: %s' % ', '.join(FAILURES))
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
