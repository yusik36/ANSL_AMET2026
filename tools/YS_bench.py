#!/usr/bin/env python3
"""Automatic scorekeeper for simulator runs.

Runs alongside the driving stack and watches it -- it never publishes a
command, so it cannot change what the car does. Everything it reports comes
from simulator ground truth (`/sim/api/*`) plus the track's own route file,
so "was the car off the track" is computed, not eyeballed.

Why this exists: almost every change we make from here (lookahead distance,
target speed, HSV window, corner-cutting aggressiveness) trades one thing off
against another. "Looked better on screen" cannot see those trades. This
prints numbers we can diff between runs.

Simulator only. The real car has no ground-truth pose, so on race weekend the
equivalent is a human with a stopwatch.

Usage
-----
    python3 YS_bench.py                     # watch until Ctrl-C, print summary
    python3 YS_bench.py --laps 1            # stop automatically after 1 lap
    python3 YS_bench.py --csv run_042.csv   # also dump the raw samples
    python3 YS_bench.py --label "lookahead=0.8"

Typical loop:
    1. run it against the current code   -> baseline numbers
    2. change exactly one thing
    3. run it again                      -> compare
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time
import urllib.error
import urllib.request

DEFAULT_BASE = 'http://localhost/sim/api'
# Where physicar-sim keeps the route files on the workspace image.
ROUTE_DIRS = (
    '/opt/physicar/src/physicar-sim/share/routes',
    '/opt/physicar/install/physicar_sim/share/physicar_sim/routes',
)

# Car geometry, from the official URDF (physicar.urdf.xacro).
TRACK_WIDTH = 0.16        # wheel centre to wheel centre
WHEEL_HALF = TRACK_WIDTH / 2.0

STOPPED_SPEED = 0.05      # m/s below which we call the car stopped
SAMPLE_HZ = 20.0
# One excursion that grazes the edge can dip back inside for a sample or two.
# The rules penalise the excursion, not each flicker, so re-entries shorter
# than this rejoin the event that came before them.
OFF_TRACK_MERGE_S = 0.4


# --------------------------------------------------------------------------
# simulator access
# --------------------------------------------------------------------------

def get_json(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode('utf-8'))
    except (urllib.error.URLError, OSError, ValueError):
        return None


class Sim:
    def __init__(self, base):
        self.base = base.rstrip('/')

    def status(self):
        return get_json(self.base + '/status')

    def pose(self):
        """Ground-truth pose: {'x','y','z','yaw',...} or None."""
        return get_json(self.base + '/pose')

    def states(self):
        """Full snapshot -- may carry speed/steering, shape is not guaranteed."""
        return get_json(self.base + '/states')


# --------------------------------------------------------------------------
# track geometry
# --------------------------------------------------------------------------

class Track:
    """Centre line + boundaries, resampled and indexed by arc length.

    The route file is (N, 6): centre x,y | boundary-a x,y | boundary-b x,y.
    """

    def __init__(self, centre, half_width, length, cum_s):
        self.centre = centre              # list of (x, y)
        self.half_width = half_width      # per-point distance to each edge
        self.length = length              # total lap length, metres
        self.cum_s = cum_s                # arc length at each point

    @classmethod
    def from_api(cls, base=DEFAULT_BASE):
        """Track geometry from the running simulator.

        Preferred over a route file: the file on disk may belong to a world
        that is not loaded. Scoring a lap against the wrong track's
        boundaries reports off-track excursions that never happened, and
        misses the ones that did.
        """
        try:
            import numpy as np
        except ImportError:
            sys.exit('numpy is required')
        w = get_json(base.rstrip('/') + '/world', timeout=5.0)
        if not w or 'track' not in w:
            return None
        route = w['track']['route']
        if not all(k in route for k in ('waypoints', 'inner', 'outer')):
            return None
        centre = np.asarray(route['waypoints'], dtype=float)[:, :2]
        inner = np.asarray(route['inner'], dtype=float)[:, :2]
        outer = np.asarray(route['outer'], dtype=float)[:, :2]
        n = min(len(centre), len(inner), len(outer))
        centre, inner, outer = centre[:n], inner[:n], outer[:n]
        half = (np.linalg.norm(inner - centre, axis=1)
                + np.linalg.norm(outer - centre, axis=1)) / 2.0
        seg = np.linalg.norm(np.diff(centre, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        print('track from the simulator: world %s ("%s")'
              % (str(w.get('world_id', '?'))[:12], w.get('display', '?')))
        return cls([tuple(p) for p in centre], list(half),
                   float(cum[-1]), list(cum))

    @classmethod
    def load(cls, path):
        try:
            import numpy as np
        except ImportError:
            sys.exit('numpy is required to read the route file')
        a = np.load(path, allow_pickle=True)
        if a.ndim != 2 or a.shape[1] < 6:
            sys.exit('unexpected route shape %s in %s' % (a.shape, path))
        centre = a[:, 0:2]
        edge_a = a[:, 2:4]
        edge_b = a[:, 4:6]
        half = (np.linalg.norm(edge_a - centre, axis=1)
                + np.linalg.norm(edge_b - centre, axis=1)) / 2.0

        seg = np.linalg.norm(np.diff(centre, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        return cls([tuple(p) for p in centre], list(half),
                   float(cum[-1]), list(cum))

    def _project_segment(self, i, x, y):
        """Project onto the segment starting at vertex i.
        Returns (perp_distance, arc_length, signed_lateral) or None."""
        n = len(self.centre)
        ax, ay = self.centre[i]
        bx, by = self.centre[(i + 1) % n]
        tx, ty = bx - ax, by - ay
        tlen = math.hypot(tx, ty)
        if tlen < 1e-9:
            return None
        tx, ty = tx / tlen, ty / tlen
        vx, vy = x - ax, y - ay
        along = vx * tx + vy * ty
        lateral = -vx * ty + vy * tx              # left-positive
        clamped = max(0.0, min(tlen, along))
        # Distance to the segment, not to its infinite line: outside the span
        # the nearest point is an endpoint, so fold the overshoot back in.
        overshoot = along - clamped
        perp = math.hypot(lateral, overshoot)
        return perp, self.cum_s[i] + clamped, lateral

    def project(self, x, y):
        """Nearest point on the centre line.

        Returns (arc_length, signed_lateral_offset, half_width_here).
        Positive lateral means left of the direction of travel.

        The nearest vertex is only a starting hint -- the true closest point
        can lie on the segment *before* it, so both adjacent segments are
        tested. Projecting onto only the following segment leaves an error of
        up to half the point spacing (~0.036 m here), which is enough to make
        a single off-track excursion flicker in and out.
        """
        n = len(self.centre)
        best_i, best_d2 = 0, float('inf')
        for i, (cx, cy) in enumerate(self.centre):
            d2 = (cx - x) ** 2 + (cy - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i

        best = None
        for i in ((best_i - 1) % n, best_i):
            r = self._project_segment(i, x, y)
            if r is not None and (best is None or r[0] < best[0]):
                best = r
        if best is None:
            return self.cum_s[best_i], math.sqrt(best_d2), self.half_width[best_i]
        _, s, lateral = best
        return s, lateral, self.half_width[best_i]


def find_route(world_id, explicit):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for d in ROUTE_DIRS:
        if not os.path.isdir(d):
            continue
        if world_id:
            cand = os.path.join(d, '%s.npy' % world_id)
            if os.path.exists(cand):
                return cand
        files = sorted(f for f in os.listdir(d) if f.endswith('.npy'))
        if files:
            return os.path.join(d, files[0])
    return None


# --------------------------------------------------------------------------
# scoring
# --------------------------------------------------------------------------

class OffTrackEvent:
    def __init__(self, t, x, y, s):
        self.t0 = self.t1 = t
        self.x, self.y, self.s = x, y, s
        self.worst = 0.0

    def update(self, t, excess):
        self.t1 = t
        self.worst = max(self.worst, excess)

    @property
    def duration(self):
        return self.t1 - self.t0


class Run:
    """Accumulates samples and turns them into a score sheet."""

    def __init__(self, track, label):
        self.track = track
        self.label = label
        self.samples = []          # (t, x, y, yaw, speed, s, lateral)
        self.off_events = []
        self._off = None
        self.t_start = None
        self.laps = []             # completed lap times
        self._lap_t0 = None
        self._last_s = None
        self._lap_dist = 0.0       # ground covered since the lap started
        self._last_xy = None
        self._wrap_base = 0.0

    def add(self, t, x, y, yaw, speed):
        s, lateral, half = self.track.project(x, y)
        if self.t_start is None:
            self.t_start = t
            self._lap_t0 = t
            self._last_s = s
        self.samples.append((t, x, y, yaw, speed, s, lateral))

        # Off track: regulation counts a lap-out only when all four wheels are
        # outside, so the car centre has to clear the edge by half a track
        # width before we call it.
        excess = abs(lateral) - (half + WHEEL_HALF)
        if excess > 0:
            if self._off is None:
                last = self.off_events[-1] if self.off_events else None
                if last is not None and t - last.t1 <= OFF_TRACK_MERGE_S:
                    self._off = last          # same excursion, brief re-entry
                else:
                    self._off = OffTrackEvent(t, x, y, s)
                    self.off_events.append(self._off)
            self._off.update(t, excess)
        else:
            self._off = None

        # Lap detection: arc length wraps from near the end back to near zero,
        # AND the car has actually driven most of a lap to get there.
        #
        # The wrap alone is not enough, because the projection is ambiguous
        # exactly at the start line -- which is exactly where the car starts.
        # A pose a millimetre before the line projects onto the closing
        # segment and reports s = L, so the next sample looks like a wrap and
        # a lap of one sample interval gets recorded. With --laps 1 that ends
        # the run 50 ms after it began and writes a 0.05 s lap time into the
        # log, which reads like a scoring glitch rather than the ruined
        # measurement it is.
        L = self.track.length
        if self._last_xy is not None:
            self._lap_dist += math.hypot(x - self._last_xy[0],
                                         y - self._last_xy[1])
        self._last_xy = (x, y)
        if self._last_s is not None:
            if (self._last_s > L * 0.75 and s < L * 0.25
                    and self._lap_dist > L * 0.5):
                self.laps.append(t - self._lap_t0)
                self._lap_t0 = t
                self._lap_dist = 0.0
        self._last_s = s

    # -- derived ---------------------------------------------------------

    @property
    def elapsed(self):
        if not self.samples:
            return 0.0
        return self.samples[-1][0] - self.samples[0][0]

    def summary(self):
        if len(self.samples) < 2:
            return 'no samples collected -- is the simulator running?'

        speeds = [s[4] for s in self.samples]
        lats = [abs(s[6]) for s in self.samples]
        dt = self.elapsed / max(len(self.samples) - 1, 1)
        stopped = sum(1 for v in speeds if v < STOPPED_SPEED) * dt
        progress = self.samples[-1][5] - self.samples[0][5]
        if progress < 0:
            progress += self.track.length

        off_total = sum(e.duration for e in self.off_events)
        worst_off = max((e.worst for e in self.off_events), default=0.0)

        lines = []
        lines.append('=== BENCH RESULT %s' % (('[%s]' % self.label) if self.label else ''))
        lines.append('track length        : %.2f m' % self.track.length)
        lines.append('wall time           : %.1f s   (%d samples)'
                     % (self.elapsed, len(self.samples)))
        if self.laps:
            for i, lt in enumerate(self.laps, 1):
                lines.append('LAP %-2d              : %.2f s' % (i, lt))
        else:
            lines.append('LAP                 : not completed'
                         '  (progress %.1f m of %.1f m, %.0f%%)'
                         % (progress, self.track.length,
                            100.0 * progress / self.track.length))
        lines.append('speed avg / max     : %.2f / %.2f m/s'
                     % (sum(speeds) / len(speeds), max(speeds)))
        lines.append('stopped time        : %.1f s' % stopped)
        lines.append('centre deviation    : mean %.3f m / max %.3f m'
                     % (sum(lats) / len(lats), max(lats)))
        lines.append('off-track events    : %d  (total %.1f s, worst %.3f m past the edge)'
                     % (len(self.off_events), off_total, worst_off))
        for e in self.off_events[:8]:
            lines.append('    t=%6.1fs  at x=%5.2f y=%5.2f  (track %.1f m)  %.1fs  %.3f m out'
                         % (e.t0 - self.t_start, e.x, e.y, e.s, e.duration, e.worst))
        if len(self.off_events) > 8:
            lines.append('    ... and %d more' % (len(self.off_events) - 8))

        # Rough competition score: lap plus 5 s for each off-track excursion.
        # Cone strikes are not detected yet, so this is a floor, not the truth.
        if self.laps:
            penalty = 5.0 * len(self.off_events)
            lines.append('-- estimated score --')
            lines.append('lap %.2f + off-track %.0f = %.2f s'
                         % (self.laps[0], penalty, self.laps[0] + penalty))
            lines.append('(cone strikes not counted -- add 5 s each)')
        return '\n'.join(lines)

    def write_csv(self, path):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'speed', 'arc_s', 'lateral'])
            t0 = self.samples[0][0]
            for s in self.samples:
                w.writerow(['%.3f' % (s[0] - t0)] + ['%.4f' % v for v in s[1:]])


# --------------------------------------------------------------------------

def extract_speed(prev, now):
    """Differentiate ground-truth pose. Preferred over the reported speed:
    it is what the car actually did, not what it was told to do."""
    if prev is None:
        return 0.0
    (pt, px, py), (nt, nx, ny) = prev, now
    dt = nt - pt
    return math.hypot(nx - px, ny - py) / dt if dt > 1e-6 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default=DEFAULT_BASE, help='simulator API base URL')
    ap.add_argument('--route', help='explicit path to the track .npy')
    ap.add_argument('--laps', type=int, default=0, help='stop after N laps (0 = until Ctrl-C)')
    ap.add_argument('--csv', help='write raw samples here')
    ap.add_argument('--label', default='', help='tag for this run, shown in the summary')
    ap.add_argument('--timeout', type=float, default=200.0, help='give up after this many seconds')
    args = ap.parse_args()

    sim = Sim(args.base)
    st = sim.status()
    if st is None:
        sys.exit('cannot reach %s -- is the simulator running?' % args.base)
    world = st.get('current') if isinstance(st, dict) else None
    print('simulator up. world = %s' % world)

    # Ask the simulator what is loaded before trusting anything on disk:
    # the bundled route file belongs to a sample world, and scoring against
    # the wrong boundaries invents excursions and hides real ones.
    track = None if args.route else Track.from_api(args.base)
    if track is None:
        route = find_route(world, args.route)
        if not route:
            sys.exit('no track from the API and no route .npy found')
        track = Track.load(route)
        print('route file: %s' % os.path.basename(route))
    print('track: %.2f m, %d points, width ~%.2f m'
          % (track.length, len(track.centre),
             2 * sum(track.half_width) / len(track.half_width)))
    print('watching... Ctrl-C to stop\n')

    run = Run(track, args.label)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__('now', True))

    period = 1.0 / SAMPLE_HZ
    prev_pose = None
    t_begin = time.time()
    while not stop['now']:
        loop_t = time.time()
        p = sim.pose()
        if isinstance(p, dict) and 'x' in p and 'y' in p:
            x, y = float(p['x']), float(p['y'])
            yaw = float(p.get('yaw', 0.0))
            now = (loop_t, x, y)
            run.add(loop_t, x, y, yaw, extract_speed(prev_pose, now))
            prev_pose = now

            if args.laps and len(run.laps) >= args.laps:
                print('reached %d lap(s)' % args.laps)
                break
        if loop_t - t_begin > args.timeout:
            print('timeout after %.0f s' % args.timeout)
            break
        slack = period - (time.time() - loop_t)
        if slack > 0:
            time.sleep(slack)

    print()
    print(run.summary())
    if args.csv and run.samples:
        run.write_csv(args.csv)
        print('\nsamples -> %s' % args.csv)


if __name__ == '__main__':
    main()
