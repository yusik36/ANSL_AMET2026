#!/usr/bin/env python3
"""Everything the loaded track is, in one pass, saved so it never has to be
asked again.

    python3 tools/YS_track_dump.py
    python3 tools/YS_track_dump.py --save tools/track_71e69ee9.npz

The competition track is not in the bundle -- the bundle predates it -- so
its geometry exists only in the running simulator, and the simulator is
metered. This pulls the whole thing once and writes it to disk, after which
raceline work, corridor limits and width questions are answerable offline.

Width gets the most space because it is what the planner is actually
constrained by. A mean width says almost nothing: the car is 0.16 m wide and
needs its body plus a margin inside whatever the *narrowest* point allows,
so the profile and the tight end are the numbers that matter. The narrowest
places are printed with coordinates, which is what makes them findable in
the simulator view.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import YS_bench as B

CAR_WIDTH = 0.16            # track_width from the URDF
CAR_HALF = 0.08
WHEELBASE = 0.18
MAX_STEER = math.radians(20.0)


def menger(a, b, c):
    """Curvature through three points. Scale-dependent by nature, which is
    why the caller sweeps the stencil rather than picking one."""
    ab = np.linalg.norm(b - a)
    bc = np.linalg.norm(c - b)
    ca = np.linalg.norm(a - c)
    if ab * bc * ca < 1e-12:
        return 0.0
    area = 0.5 * abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
    return 4.0 * area / (ab * bc * ca)


def radius_profile(centre, stencil_m, spacing):
    """Minimum radius over the lap at a given stencil width, in metres."""
    k = max(1, int(round(stencil_m / max(spacing, 1e-9))))
    n = len(centre)
    worst, where = float('inf'), 0
    for i in range(n):
        c = menger(centre[(i - k) % n], centre[i], centre[(i + k) % n])
        if c > 1e-9 and 1.0 / c < worst:
            worst, where = 1.0 / c, i
    return worst, where


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--api', default=B.DEFAULT_BASE)
    ap.add_argument('--save', default='', help='write geometry to this .npz')
    ap.add_argument('--bins', type=int, default=20,
                    help='width profile resolution around the lap')
    a = ap.parse_args()

    w = B.get_json(a.api.rstrip('/') + '/world', timeout=6.0)
    if not w or 'track' not in w:
        print('no world from %s -- is the simulator running?' % a.api)
        return 1

    world_id = str(w.get('world_id', '?'))
    print('=== world %s  ("%s")' % (world_id, w.get('display', '?')))

    route = w['track']['route']
    centre = np.asarray(route['waypoints'], dtype=float)[:, :2]
    inner = np.asarray(route['inner'], dtype=float)[:, :2]
    outer = np.asarray(route['outer'], dtype=float)[:, :2]
    n = min(len(centre), len(inner), len(outer))
    centre, inner, outer = centre[:n], inner[:n], outer[:n]

    seg = np.linalg.norm(np.diff(centre, axis=0, append=centre[:1]), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    length = float(seg.sum())
    spacing = float(seg.mean())

    print('\n--- centre line')
    print('  length            %.3f m' % length)
    print('  points            %d  (spacing %.4f m)' % (n, spacing))
    print('  closed-loop gap   %.4f m'
          % float(np.linalg.norm(centre[0] - centre[-1])))
    print('  bounding box      x %.2f .. %.2f   y %.2f .. %.2f'
          % (centre[:, 0].min(), centre[:, 0].max(),
             centre[:, 1].min(), centre[:, 1].max()))
    print('  start (index 0)   x %.3f  y %.3f' % (centre[0][0], centre[0][1]))
    hd = centre[1] - centre[0]
    print('  start heading     %.4f rad  (%.1f deg)'
          % (math.atan2(hd[1], hd[0]), math.degrees(math.atan2(hd[1], hd[0]))))

    # --- width, the thing this tool exists for -------------------------
    wi = np.linalg.norm(inner - centre, axis=1)
    wo = np.linalg.norm(outer - centre, axis=1)
    width = wi + wo

    print('\n--- ROAD WIDTH')
    print('  full width        min %.3f   mean %.3f   max %.3f  m'
          % (width.min(), width.mean(), width.max()))
    print('  half-width inner  min %.3f   mean %.3f   max %.3f  m'
          % (wi.min(), wi.mean(), wi.max()))
    print('  half-width outer  min %.3f   mean %.3f   max %.3f  m'
          % (wo.min(), wo.mean(), wo.max()))
    for p in (1, 5, 25, 50, 75, 95):
        print('  %2dth percentile   %.3f m' % (p, float(np.percentile(width, p))))

    print('\n  clearance for a %.2f m wide car:' % CAR_WIDTH)
    print('    narrowest point   %.3f m wide -> %.3f m of slack'
          % (width.min(), width.min() - CAR_WIDTH))
    print('    centre-line margin either side at the narrowest: %.3f m'
          % (width.min() / 2.0 - CAR_HALF))
    off_limits = int((width < CAR_WIDTH).sum())
    print('    points narrower than the car: %d' % off_limits)

    print('\n  the five narrowest places:')
    print('     s [m]      width     x        y')
    for i in np.argsort(width)[:5]:
        print('    %7.2f   %6.3f   %7.3f  %7.3f'
              % (cum[i], width[i], centre[i][0], centre[i][1]))

    print('\n  width around the lap (%d bins):' % a.bins)
    edges = np.linspace(0.0, length, a.bins + 1)
    idx = np.clip(np.searchsorted(edges, cum, side='right') - 1, 0, a.bins - 1)
    for b in range(a.bins):
        m = idx == b
        if not m.any():
            continue
        lo, mean = width[m].min(), width[m].mean()
        bar = '#' * max(1, int(round(mean * 20)))
        print('    %5.1f-%5.1f m   min %.3f  mean %.3f  %s'
              % (edges[b], edges[b + 1], lo, mean, bar))

    # --- curvature -----------------------------------------------------
    print('\n--- centre-line curvature (stencil-dependent, so all scales shown)')
    print('    stencil    min radius   steering needed   drivable?')
    for st in (WHEELBASE, 0.25, 0.37, 0.50, 0.75):
        r, i = radius_profile(centre, st, spacing)
        need = math.atan(WHEELBASE / r) if r > 0 else math.pi / 2
        ok = 'yes' if need <= MAX_STEER else 'NO'
        print('    %5.2f m    %7.3f m     %6.2f deg          %-4s (at s=%.1f m)'
              % (st, r, math.degrees(need), ok, cum[i]))
    print('    car limit: %.1f deg of steer -> minimum radius %.3f m'
          % (math.degrees(MAX_STEER), WHEELBASE / math.tan(MAX_STEER)))

    # --- objects -------------------------------------------------------
    objs = w.get('objects') or w['track'].get('objects') or []
    if objs:
        print('\n--- objects (%d)' % len(objs))
        for o in objs:
            if isinstance(o, dict):
                nm = o.get('name', o.get('id', '?'))
                print('    %-10s x %.3f  y %.3f  yaw %s'
                      % (nm, float(o.get('x', 0.0)), float(o.get('y', 0.0)),
                         o.get('yaw', '-')))
            else:
                print('    %s' % o)
    else:
        print('\n--- objects: none reported by /world')
        print('    (obstacle positions change for the final; re-run then)')

    if a.save:
        np.savez(a.save, world_id=world_id, centre=centre, inner=inner,
                 outer=outer, cum_s=cum, width=width, length=length)
        print('\nsaved: %s' % a.save)
        print('  reload with: d = numpy.load("%s"); d["width"]' % a.save)
    return 0


if __name__ == '__main__':
    sys.exit(main())
