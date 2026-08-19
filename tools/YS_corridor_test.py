"""Offline check of the corridor planner, against scenes built from the
colours and geometry measured in the simulator on 2026-08-18.

The point is not that the planner produces some output -- it is that the
output is right for a situation whose answer is known independently: a
straight track should steer nearly zero, a track shifted left should steer
left, a cone should push the target past it on the side with more room, and
a blocked view should stop the car rather than guess.
"""
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'physicar_planner'))
from physicar_planner import corridor as C   # noqa: E402

W, H = 480, 360
FAILURES = []

# Measured HSV, as BGR for drawing.
ROAD = cv2.cvtColor(np.uint8([[[107, 112, 76]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
GRASS = cv2.cvtColor(np.uint8([[[35, 118, 167]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
CONE = cv2.cvtColor(np.uint8([[[60, 245, 165]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
WHITE = cv2.cvtColor(np.uint8([[[24, 8, 231]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
ORANGE = cv2.cvtColor(np.uint8([[[17, 236, 216]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
WALL = cv2.cvtColor(np.uint8([[[141, 95, 70]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()


def check(name, ok, detail=''):
    print('  %-46s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))
    if not ok:
        FAILURES.append(name)


def scene(track_centre=0.0, half_width=0.40, cone=None, curve=0.0):
    """A track of the measured width, painted by projecting ground metres
    into the image, so the scene obeys the same geometry the planner uses.

    track_centre shifts the track sideways; curve bends it (metres of lateral
    offset per metre of distance squared); cone is (distance, lateral).
    """
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = GRASS
    # Painted row by row, not distance by distance: perspective compresses
    # far ground into few rows and spreads near ground over many, so an even
    # sweep in distance leaves near rows unpainted and the corridor scan
    # reads those gaps as grass.
    _fx, fy, _cx, cy = C.intrinsics(W, H)
    for row in range(int(cy) + 2, H):
        d = C.CAM_HEIGHT * fy / (row - cy) + C.CAM_FORWARD
        if d > 8.0:
            continue
        centre = track_centre + curve * d * d
        left = C.lateral_to_column(centre + half_width, d, W, H)
        right = C.lateral_to_column(centre - half_width, d, W, H)
        c0, c1 = int(round(min(left, right))), int(round(max(left, right)))
        c0, c1 = max(0, c0), min(W, c1)
        if c1 > c0:
            img[row, c0:c1] = ROAD
            img[row, c0:min(c0 + 3, c1)] = WHITE          # edge lines
            img[row, max(c0, c1 - 3):c1] = WHITE
            mid = int(round(C.lateral_to_column(centre, d, W, H)))
            if c0 <= mid < c1 and int(d * 6) % 2 == 0:
                img[row, max(c0, mid - 1):min(c1, mid + 2)] = ORANGE

    if cone is not None:
        cd, clat = cone
        row = int(round(C.distance_to_row(cd, W, H)))
        col = int(round(C.lateral_to_column(clat, cd, W, H)))
        # 0.18 m wide, 0.38 m tall, projected at that distance
        fx, fy, _cx, _cy = C.intrinsics(W, H)
        z = max(cd - C.CAM_FORWARD, 1e-6)
        half_px = int(round(0.09 * fx / z))
        tall_px = int(round(0.38 * fy / z))
        cv2.rectangle(img, (col - half_px, row - tall_px), (col + half_px, row),
                      CONE, -1)
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def run(hsv, speed=1.5, a_lat=4.0, gain=0.35, base=0.70):
    # base matches planner_node's default on purpose: a suite tuned to
    # different numbers than the ones that ship tests a planner nobody runs.
    return C.plan(hsv, speed, a_lat, gain, base)


def main():
    print('drivable rule: road H %d-%d | paint S<=%d V>=%d | dashes H %d-%d S>=%d'
          % (C.ROAD_H_MIN, C.ROAD_H_MAX, C.PAINT_S_MAX, C.PAINT_V_MIN,
             C.MARK_H_MIN, C.MARK_H_MAX, C.MARK_S_MIN))
    print('corridor sampled %.2f-%.2f m\n' % (C.DEFAULT_MIN_RANGE,
                                              C.DEFAULT_MAX_RANGE))

    print('the measured colours are classified as intended')
    for name, hsvpx, want_blocked in (
            ('road   H107 S112', (107, 112, 76), False),
            ('white  H24  S8  ', (24, 8, 231), False),
            ('orange H17  S236', (17, 236, 216), False),
            ('grass  H35  S118', (35, 118, 167), True),
            ('cone   H60  S245', (60, 245, 165), True),
            ('wall   H141 S95 ', (141, 95, 70), True),
            ('unseen H160 S200', (160, 200, 200), True)):
        px = np.uint8([[list(hsvpx)]])
        got = bool(C.blocked_mask(px)[0][0])
        check('%s -> %s' % (name, 'blocked' if want_blocked else 'drivable'),
              got == want_blocked)

    print('\nstraight track')
    sp, st, dbg = run(scene())
    check('steers nearly straight', abs(st) < math.radians(2.0),
          '%.1f deg' % math.degrees(st))
    check('moves', sp > 0.5, '%.2f m/s' % sp)
    check('found a target', dbg['target'] is not None)
    if dbg['target']:
        check('target near the centre line', abs(dbg['target'][1]) < 0.05,
              '%.3f m' % dbg['target'][1])

    print('\ntrack shifted 0.25 m to the left of the car')
    sp, st, dbg = run(scene(track_centre=0.25))
    check('steers left', st > math.radians(2.0), '%.1f deg' % math.degrees(st))

    print('\ntrack shifted 0.25 m to the right')
    sp, st, dbg = run(scene(track_centre=-0.25))
    check('steers right', st < -math.radians(2.0), '%.1f deg' % math.degrees(st))

    # lateral = d^2/(2R), so curve = 1/(2R). The real track's tightest
    # optimised corner is R = 0.58 m; 0.40 is a moderate corner.
    print('\ncurving left (R ~ 1.25 m)')
    sp_c, st_c, dbg_c = run(scene(curve=0.40))
    check('steers left into the curve', st_c > math.radians(1.0),
          '%.1f deg' % math.degrees(st_c))
    sp_s, _st_s, _ = run(scene())
    check('slower than on the straight', sp_c < sp_s,
          '%.2f vs %.2f m/s' % (sp_c, sp_s))

    print('\ncone ahead, offset to the left of the track centre')
    base_sp, base_st, _ = run(scene())
    sp, st, dbg = run(scene(cone=(1.2, 0.18)))
    check('target pushed right of the cone',
          dbg['target'] is not None and dbg['target'][1] < -0.02,
          '%.3f m' % (dbg['target'][1] if dbg['target'] else float('nan')))
    check('steers right, away from it', st < base_st,
          '%.1f deg vs %.1f' % (math.degrees(st), math.degrees(base_st)))

    print('\ncone ahead, offset right')
    sp, st, dbg = run(scene(cone=(1.2, -0.18)))
    check('target pushed left of the cone',
          dbg['target'] is not None and dbg['target'][1] > 0.02,
          '%.3f m' % (dbg['target'][1] if dbg['target'] else float('nan')))

    # A single frame's correction is small by design -- the car re-plans
    # every frame and the cone is still a metre off. What matters is that
    # the response grows as it closes, so the avoidance actually completes
    # rather than nibbling at the problem all the way into the cone.
    print('\navoidance strengthens as the cone closes')
    steers = []
    for cd in (1.8, 1.4, 1.0, 0.7):
        _s, stx, _d = run(scene(cone=(cd, 0.18)))
        steers.append((cd, math.degrees(stx)))
    print('        ' + '  '.join('%.1fm:%+.1fdeg' % t for t in steers))
    # 0.05 deg is below what the steering can resolve, let alone act on.
    check('never steers into it', all(s <= 0.05 for _d, s in steers))
    check('strongest when nearest',
          steers[-1][1] < steers[0][1],
          '%+.1f at %.1fm vs %+.1f at %.1fm'
          % (steers[-1][1], steers[-1][0], steers[0][1], steers[0][0]))

    print('\nno track visible at all')
    blank = cv2.cvtColor(np.full((H, W, 3), GRASS, np.uint8), cv2.COLOR_BGR2HSV)
    sp, st, dbg = run(blank)
    check('stops', sp == 0.0 and st == 0.0)
    check('reports no target', dbg['target'] is None)

    print('\nspeed responds to aggression')
    slow = run(scene(curve=0.40), a_lat=2.0)[0]
    fast = run(scene(curve=0.40), a_lat=8.0)[0]
    check('higher a_lat is faster in a curve', fast > slow,
          '%.2f vs %.2f m/s' % (fast, slow))

    # The launch defect: on the first frame that saw a clear corridor the
    # planner commanded 2.02 m/s from a standstill and was at the 3 m/s cap
    # 200 ms later, while asking for 10 degrees of steering. The car left the
    # track sideways after 1.1 m. Both halves are checked here.
    print('\nthe command ramps instead of stepping')
    v, tick = 0.0, 0.05                       # the node's 20 Hz timer
    for _ in range(4):
        v = C.ramp(3.0, v, tick)
    check('0 -> 3.0 m/s takes longer than 200 ms', v < 1.0,
          'reached %.2f m/s in 200 ms' % v)
    check('braking is allowed to be harder than accelerating',
          C.MAX_DECEL > C.MAX_ACCEL,
          '%.1f vs %.1f m/s^2' % (C.MAX_DECEL, C.MAX_ACCEL))
    v = 3.0
    for _ in range(int(1.0 / tick)):
        v = C.ramp(0.0, v, tick)
    check('a stop request still stops within a second', v == 0.0,
          '%.2f m/s' % v)
    v = C.ramp(1.0, 1.0, tick)
    check('holds a steady target', abs(v - 1.0) < 1e-9)
    check('dt of zero cannot move the command',
          C.ramp(3.0, 0.5, 0.0) == 0.5)

    # Pure pursuit's sensitivity to a lateral error goes as 2*L/Ld^2, so the
    # lookahead at a standstill sets how violently the car launches. At the
    # old base of 0.45 m, 10 cm off centre asked for 10 degrees before the
    # car had moved.
    print('\nsteering gain at rest is not violent')
    for base, want in ((0.45, False), (0.70, True)):
        st, _r = C.pure_pursuit((C.lookahead_for(0.0, 0.35, base), 0.10))
        deg = math.degrees(st)
        check('base %.2f m: 10 cm off centre -> %.1f deg%s'
              % (base, deg, '' if want else '  (the old default)'),
              (deg < 6.0) == want, 'gain %.2f rad/m'
              % (2.0 * C.WHEELBASE / C.lookahead_for(0.0, 0.35, base) ** 2))

    print('\nlookahead grows with speed')
    check('Ld(0.5) < Ld(2.5)',
          C.lookahead_for(0.5, 0.35, 0.45) < C.lookahead_for(2.5, 0.35, 0.45),
          '%.2f -> %.2f m' % (C.lookahead_for(0.5, 0.35, 0.45),
                              C.lookahead_for(2.5, 0.35, 0.45)))
    check('Ld stays inside the trusted range at top speed',
          C.lookahead_for(3.0, 0.35, 0.45) <= C.DEFAULT_MAX_RANGE,
          '%.2f m vs %.2f' % (C.lookahead_for(3.0, 0.35, 0.45),
                              C.DEFAULT_MAX_RANGE))

    print()
    if FAILURES:
        print('FAILED: %s' % ', '.join(FAILURES))
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
