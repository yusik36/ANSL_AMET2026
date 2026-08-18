#!/usr/bin/env python3
"""Minimum-curvature racing line for a known track, computed offline.

We cannot follow this line on the car -- Physicar has no wheel encoders, and
its only source of position is laser scan matching against a boundary the
lidar can barely see, so there is no localisation to track a global path
with. The line is still worth having for two reasons:

  1. It sets the target. The lap time it implies is the best the track
     allows, so every number the local planner produces can be read against
     something rather than against nothing.
  2. It grades the local planner offline. Given only what the camera can see
     at a point on the track, does the local method choose roughly the line
     the global optimum chose? That question can be asked without spending
     simulator time, and without the car ever knowing where it is.

Method follows the standard formulation (Heilmeier et al., as used by both
the F1TENTH survey and the ForzaETH stack): offset each centre-line point
sideways by alpha_i along the local normal, and minimise the squared second
difference of the resulting path -- discrete curvature -- subject to staying
inside the track. That is a convex QP in alpha; with no scipy here it is
solved by accelerated projected gradient, which is enough for a problem this
size.

On this track the standard objective is not quite enough. Minimising the
SUM of squared curvature leaves the single tightest corner needing 21.6 deg
of steering against the 20 deg the car has, so the line is fast on paper and
undrivable in the one place that decides whether there is a lap at all.
--minmax reweights toward the worst points, which brings that corner down to
18.0 deg for about 1.4 s of lap time. Two rounds is the sweet spot; more
starts costing radius again.

    python3 tools/YS_raceline.py <route.npy>
    python3 tools/YS_raceline.py <route.npy> --minmax 2 --save line.npy
"""
import argparse
import math
import sys

import numpy as np

MAX_SPEED = 3.0                 # driver hard clamp, m/s
WHEELBASE = 0.18
MAX_STEER = math.radians(20.0)
CAR_HALF_WIDTH = 0.08           # track_width 0.16 m, from the URDF


# --------------------------------------------------------------------------

def load_route(path):
    a = np.load(path, allow_pickle=True)
    if a.ndim != 2 or a.shape[1] < 6:
        sys.exit('unexpected route shape %s' % (a.shape,))
    centre = a[:, 0:2]
    half = (np.linalg.norm(a[:, 2:4] - centre, axis=1)
            + np.linalg.norm(a[:, 4:6] - centre, axis=1)) / 2.0
    # The file closes the loop by repeating the first point; drop the copy so
    # the wrap-around stencils below do not see a zero-length segment.
    if np.allclose(centre[0], centre[-1]):
        centre, half = centre[:-1], half[:-1]
    return centre, half


def resample_closed(centre, half, n):
    """Uniform arc-length resampling. The curvature stencil below assumes
    even spacing, so this is not cosmetic."""
    closed = np.vstack([centre, centre[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    si = np.linspace(0.0, total, n, endpoint=False)
    out = np.empty((n, 2))
    out[:, 0] = np.interp(si, s, closed[:, 0])
    out[:, 1] = np.interp(si, s, closed[:, 1])
    hw = np.interp(si, s, np.concatenate([half, half[:1]]))
    return out, hw, total


def normals(pts):
    """Unit left-normals from the central-difference tangent."""
    nxt = np.roll(pts, -1, axis=0)
    prv = np.roll(pts, 1, axis=0)
    t = nxt - prv
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
    return np.stack([-t[:, 1], t[:, 0]], axis=1)


def second_difference_matrix(n):
    S = np.zeros((n, n))
    i = np.arange(n)
    S[i, (i - 1) % n] = 1.0
    S[i, i] = -2.0
    S[i, (i + 1) % n] = 1.0
    return S


def _solve_weighted(Ax, Ay, bx, by, w, limit, iters, a0=None):
    """min sum_i w_i |b_i + A_i alpha|^2 over the box, by FISTA."""
    W = w[:, None]
    H = 2.0 * (Ax.T @ (W * Ax) + Ay.T @ (W * Ay))
    c = 2.0 * (Ax.T @ (w * bx) + Ay.T @ (w * by))
    L = float(np.max(np.linalg.eigvalsh(H)))
    step = 1.0 / max(L, 1e-9)

    a = np.zeros(len(limit)) if a0 is None else a0.copy()
    y = a.copy()
    t = 1.0
    for _ in range(iters):
        a_new = np.clip(y - step * (H @ y + c), -limit, limit)
        t_new = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
        y = a_new + ((t - 1.0) / t_new) * (a_new - a)
        a, t = a_new, t_new
    return a


def solve_min_curvature(centre, hw, margin, iters=4000, minmax_rounds=0):
    """alpha minimising the discrete curvature of the offset path.

    Plain least squares minimises the SUM of squared curvature, which is the
    standard formulation and gives a smooth fast line. But what decides
    whether this track is drivable at all is a single corner -- the tightest
    one -- because below the car's minimum turn radius no amount of speed
    tuning helps. With minmax_rounds > 0 the solve is repeated with the
    weights pushed toward the worst points, which trades a little smoothness
    everywhere for radius where it actually binds.
    """
    n = len(centre)
    nrm = normals(centre)
    S = second_difference_matrix(n)

    Ax = S * nrm[:, 0]              # S @ diag(nx)
    Ay = S * nrm[:, 1]
    bx = S @ centre[:, 0]
    by = S @ centre[:, 1]

    limit = np.maximum(hw - margin, 0.0)
    w = np.ones(n)
    a = _solve_weighted(Ax, Ay, bx, by, w, limit, iters)

    for _ in range(minmax_rounds):
        # Residual at each point is its discrete curvature; weighting by it
        # walks the objective from the 2-norm toward the infinity-norm.
        d = np.hypot(bx + Ax @ a, by + Ay @ a)
        w = w * (d / max(d.max(), 1e-12)) ** 2
        w = np.maximum(w / max(w.mean(), 1e-12), 1e-6)
        a = _solve_weighted(Ax, Ay, bx, by, w, limit, iters, a0=a)

    return centre + nrm * a[:, None], a, limit


# --------------------------------------------------------------------------

def curvature(pts, span=None):
    """Menger curvature over a stencil `span` points wide.

    The answer depends on span and there is no span that is simply correct.
    Too narrow and it measures the vertex angles of the source polyline
    rather than the track; too wide and it averages a tight corner away. The
    honest thing is to report several, which report_radii below does, and to
    read the one whose arc length matches the scale you care about -- for
    "can the car steer this", that is the wheelbase.
    """
    n = len(pts)
    if span is None:
        span = max(int(n * 0.01), 3)
    a = np.roll(pts, span, axis=0)
    b = pts
    c = np.roll(pts, -span, axis=0)
    ab = np.linalg.norm(b - a, axis=1)
    bc = np.linalg.norm(c - b, axis=1)
    ca = np.linalg.norm(a - c, axis=1)
    area2 = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                   - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1]))
    denom = ab * bc * ca
    return np.where(denom > 1e-12, area2 / np.maximum(denom, 1e-12), 0.0)


def speed_profile(k, ds, a_lat, a_long, v_max=MAX_SPEED):
    """Cornering limit, then made reachable by forward/backward passes."""
    v = np.minimum(np.sqrt(a_lat / np.maximum(k, 1e-6)), v_max)
    n = len(v)
    for _ in range(3):                       # a few sweeps for the wrap-around
        for i in range(n):
            j = (i - 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * a_long * ds[j]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * a_long * ds[i]))
    return v


def arc_lengths(pts):
    return np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)


def min_radius(pts, span):
    k = curvature(pts, span)
    r = np.where(k > 1e-6, 1.0 / np.maximum(k, 1e-9), np.inf)
    finite = r[np.isfinite(r)]
    return float(finite.min()) if len(finite) else float('inf')


def report_radii(centre, line, length):
    """Minimum radius at several measurement scales, plus the steering the
    tightest corner would demand."""
    n = len(centre)
    spacing = length / n
    print('minimum radius by measurement scale')
    print('  %-8s %-9s %-12s %-12s %s'
          % ('span', 'arc [m]', 'centre', 'optimised', 'steering needed'))
    for span in (2, 3, 4, 6, 8):
        arc = span * spacing
        rc = min_radius(centre, span)
        ro = min_radius(line, span)
        # Bicycle model: the steering angle that produces radius R.
        need = math.degrees(math.atan(WHEELBASE / ro)) if ro > 0 else 90.0
        flag = '' if need <= math.degrees(MAX_STEER) else '  OVER LIMIT'
        mark = ' <- wheelbase scale' if abs(arc - WHEELBASE) < spacing else ''
        print('  %-8d %-9.3f %-12.3f %-12.3f %.1f deg%s%s'
              % (span, arc, rc, ro, need, flag, mark))
    print('  car limit: %.1f deg of steering -> %.3f m radius'
          % (math.degrees(MAX_STEER), WHEELBASE / math.tan(MAX_STEER)))


def report(name, pts, a_lats):
    ds = arc_lengths(pts)
    length = float(ds.sum())
    k = curvature(pts)
    r = np.where(k > 1e-6, 1.0 / np.maximum(k, 1e-9), np.inf)
    print('  %-18s length %.2f m   R<1m for %.1f%% of the lap'
          % (name, length, 100.0 * np.mean(r < 1.0)))
    times = {}
    for a_lat in a_lats:
        v = speed_profile(k, ds, a_lat, a_lat)
        t = float(np.sum(ds / np.maximum(v, 1e-3)))
        times[a_lat] = t
    return length, times


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('route')
    ap.add_argument('--points', type=int, default=600,
                    help='resampled points around the lap')
    ap.add_argument('--margin', type=float, default=CAR_HALF_WIDTH,
                    help='metres kept clear of each edge; defaults to the '
                         'car half-width, so the wheels ride the line')
    ap.add_argument('--iters', type=int, default=4000)
    ap.add_argument('--minmax', type=int, default=0, metavar='ROUNDS',
                    help='reweighting rounds pushing the objective toward '
                         'the worst corner rather than the average one')
    ap.add_argument('--save', help='write the optimised line as .npy')
    args = ap.parse_args()

    centre_raw, half_raw = load_route(args.route)
    centre, hw, length = resample_closed(centre_raw, half_raw, args.points)
    print('track: %.2f m, %d points, half-width %.3f..%.3f m'
          % (length, len(centre), hw.min(), hw.max()))
    print('margin: %.3f m -> usable offset +-%.3f..%.3f m\n'
          % (args.margin, max(hw.min() - args.margin, 0), max(hw.max() - args.margin, 0)))

    line, alpha, limit = solve_min_curvature(centre, hw, args.margin, args.iters,
                                            args.minmax)
    at_edge = np.mean(np.abs(np.abs(alpha) - limit) < 1e-3)

    a_lats = (2.0, 4.0, 6.0, 8.0)
    print('geometry')
    c_len, c_times = report('centre line', centre, a_lats)
    o_len, o_times = report('optimised line', line, a_lats)
    print('  offset sits on the track edge for %.0f%% of the lap' % (100 * at_edge))
    print()
    report_radii(centre, line, o_len)

    print()
    print('lap time [s] by lateral grip')
    print('  %-8s %-14s %-14s %s' % ('a_lat', 'centre line', 'optimised', 'gain'))
    for a in a_lats:
        gain = c_times[a] - o_times[a]
        print('  %-8.1f %-14.2f %-14.2f %+.2f s  (%+.1f%%)'
              % (a, c_times[a], o_times[a], -gain, -100.0 * gain / c_times[a]))

    if args.save:
        np.save(args.save, line)
        print('\nsaved optimised line -> %s  (%d points)' % (args.save, len(line)))


if __name__ == '__main__':
    main()
