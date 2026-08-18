"""Offline check of traffic_light_node's real classify()/best_lamp_area().

Imports the actual node module with the ROS pieces stubbed out, so the code
under test is the code that ships -- not a reimplementation. Scenes are
synthesised to match what was measured on the practice rig 2026-08-18:
a genuine lamp of a few hundred px against background surfaces of several
thousand px that the old whole-frame argmax lost to.
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


def check(name, ok, detail=''):
    print('  %-46s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))
    if not ok:
        FAILURES.append(name)


class Fake(T.TrafficLightNode):
    """Bypass ROS __init__, keep the real algorithm and the real defaults."""
    def __init__(self):
        self.min_blob_area_px = T.MIN_BLOB_AREA_PX
        self.max_blob_area_px = T.MAX_BLOB_AREA_PX
        self.min_circularity = T.MIN_CIRCULARITY
        self.roi_bottom_frac = T.ROI_BOTTOM_FRAC
        self.sat_min = T.SAT_MIN
        self.val_min = T.VAL_MIN


def scene(lamp=None, mat=False, door=False):
    """Build a BGR frame. lamp=('red'|'green', radius, (cx, cy))."""
    f = np.full((H, W, 3), 60, np.uint8)          # dull grey road/sky
    if mat:
        # Big saturated green mat on the FLOOR (lower part of the frame) --
        # this is the ~4900 px false positive from the practice rig.
        cv2.rectangle(f, (0, int(H * 0.72)), (W, H), (60, 180, 60), -1)
    if door:
        # Tan wood panel, large and rectangular, up on a wall.
        cv2.rectangle(f, (20, 30), (150, 250), (90, 150, 190), -1)
    if lamp:
        colour, r, (cx, cy) = lamp
        bgr = (40, 40, 250) if colour == 'red' else (60, 240, 60)
        cv2.circle(f, (cx, cy), r, bgr, -1)
    return f


def classify(node, frame):
    """Run the same ROI crop + classify the node does per frame."""
    roi = frame[0:max(1, int(frame.shape[0] * node.roi_bottom_frac)), :]
    return node.classify(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV))


def main():
    n = Fake()
    print('defaults: area %d-%d px, circularity>%.2f, ROI top %.0f%%\n'
          % (n.min_blob_area_px, n.max_blob_area_px,
             n.min_circularity, n.roi_bottom_frac * 100))

    lamp_r = 12                      # ~450 px, matches the measured range
    lamp_px = math.pi * lamp_r ** 2
    print('synthetic lamp area ~%.0f px\n' % lamp_px)

    print('the two readings that actually matter')
    check("red lamp alone -> RED",
          classify(n, scene(lamp=('red', lamp_r, (330, 90)))) == 'RED')
    check("green lamp alone -> GREEN",
          classify(n, scene(lamp=('green', lamp_r, (330, 90)))) == 'GREEN')

    print('\nthe false positives that beat the old whole-frame argmax')
    check("green floor mat alone -> NONE",
          classify(n, scene(mat=True)) == 'NONE')
    check("tan door panel alone -> NONE",
          classify(n, scene(door=True)) == 'NONE')
    check("red lamp still wins over green mat",
          classify(n, scene(lamp=('red', lamp_r, (330, 90)), mat=True)) == 'RED')
    check("red lamp still wins over door panel",
          classify(n, scene(lamp=('red', lamp_r, (330, 90)), door=True)) == 'RED')
    check("green lamp not drowned by green mat",
          classify(n, scene(lamp=('green', lamp_r, (330, 90)), mat=True)) == 'GREEN')

    print('\nsafety bias')
    both = scene(lamp=('red', lamp_r, (330, 70)))
    cv2.circle(both, (330, 130), lamp_r, (60, 240, 60), -1)
    check("red and green together -> RED (cautious)",
          classify(n, both) == 'RED')

    print('\nnothing there')
    check("empty scene -> NONE", classify(n, scene()) == 'NONE')

    print('\nsize gates')
    check("speckle below min area -> NONE",
          classify(n, scene(lamp=('red', 3, (330, 90)))) == 'NONE',
          '(~28 px)')
    check("huge round red blob above max area -> NONE",
          classify(n, scene(lamp=('red', 40, (240, 100)))) == 'NONE',
          '(~5027 px)')

    print('\nROI')
    check("lamp-sized green low in frame (on the floor) -> NONE",
          classify(n, scene(lamp=('green', lamp_r, (330, 330)))) == 'NONE')

    print()
    if FAILURES:
        print('FAILED: %s' % ', '.join(FAILURES))
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
