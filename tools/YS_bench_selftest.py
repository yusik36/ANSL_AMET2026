"""Offline check of YS_bench's geometry and scoring, using the real AMET 2026
route file and synthetic poses. Verifies the maths without needing the
simulator, so the harness is known-good before we spend sim credits on it.

Cases:
  1. perfect centre-line lap  -> ~0 deviation, no off-track, lap time = distance/speed
  2. constant lateral offset  -> deviation equals the offset, still on track
  3. deliberate excursion     -> exactly one off-track event, at the right time
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import YS_bench as B

FAILURES = []


def resolve_route():
    """Route file path: argv[1], else wherever physicar-sim keeps it."""
    if len(sys.argv) > 1:
        return sys.argv[1]
    for d in B.ROUTE_DIRS:
        if os.path.isdir(d):
            files = sorted(f for f in os.listdir(d) if f.endswith('.npy'))
            if files:
                return os.path.join(d, files[0])
    return None


def check(name, ok, detail=''):
    print('  %-42s %s %s' % (name, 'PASS' if ok else 'FAIL', detail))
    if not ok:
        FAILURES.append(name)


def drive(track, speed, lateral_of_s, dt=0.05, laps=1.05):
    """Generate poses that walk along the track at `speed`, displaced sideways
    by lateral_of_s(arc_length). Returns a completed Run."""
    run = B.Run(track, 'selftest')
    n = len(track.centre)
    t = 1000.0
    s = 0.0
    total = track.length * laps
    while s < total:
        # locate the centre-line segment containing s
        i = 0
        sm = s % track.length
        while i < n - 1 and track.cum_s[i + 1] < sm:
            i += 1
        ax, ay = track.centre[i]
        bx, by = track.centre[(i + 1) % n]
        seg = math.hypot(bx - ax, by - ay)
        f = 0.0 if seg < 1e-9 else (sm - track.cum_s[i]) / seg
        cx, cy = ax + (bx - ax) * f, ay + (by - ay) * f
        if seg < 1e-9:
            tx, ty = 1.0, 0.0
        else:
            tx, ty = (bx - ax) / seg, (by - ay) / seg
        # left-positive normal, matching Track.project's sign convention
        nx, ny = -ty, tx
        lat = lateral_of_s(sm)
        run.add(t, cx + nx * lat, cy + ny * lat, math.atan2(ty, tx), speed)
        t += dt
        s += speed * dt
    return run


def main():
    route = resolve_route()
    if not route or not os.path.exists(route):
        sys.exit('route .npy not found -- pass one as the first argument')
    print('route: %s' % os.path.basename(route))
    track = B.Track.load(route)
    print('track: %.2f m, %d points, mean half-width %.3f m\n'
          % (track.length, len(track.centre),
             sum(track.half_width) / len(track.half_width)))

    print('case 1: perfect centre-line lap at 2.0 m/s')
    run = drive(track, 2.0, lambda s: 0.0)
    dev = [abs(x[6]) for x in run.samples]
    expect = track.length / 2.0
    check('deviation ~ 0', max(dev) < 0.02, 'max %.4f m' % max(dev))
    check('no off-track', len(run.off_events) == 0, '%d events' % len(run.off_events))
    check('one lap detected', len(run.laps) == 1, '%d laps' % len(run.laps))
    if run.laps:
        err = abs(run.laps[0] - expect)
        check('lap time ~ length/speed', err < 0.3,
              'got %.2f s, expected %.2f s' % (run.laps[0], expect))

    print('\ncase 2: constant 0.30 m offset (inside the track)')
    run = drive(track, 2.0, lambda s: 0.30)
    dev = [abs(x[6]) for x in run.samples]
    mean_dev = sum(dev) / len(dev)
    check('deviation ~ 0.30', abs(mean_dev - 0.30) < 0.03, 'mean %.3f m' % mean_dev)
    check('still on track', len(run.off_events) == 0, '%d events' % len(run.off_events))

    print('\ncase 3: excursion between 10 m and 13 m of the lap')
    def excursion(s):
        return 0.90 if 10.0 <= s <= 13.0 else 0.0
    run = drive(track, 2.0, excursion)
    check('exactly one off-track event', len(run.off_events) == 1,
          '%d events' % len(run.off_events))
    if run.off_events:
        e = run.off_events[0]
        check('event starts near 10 m', 9.0 < e.s < 11.5, 'at %.1f m' % e.s)
        check('event lasts ~1.5 s', 1.0 < e.duration < 2.2,
              '%.2f s' % e.duration)
        # 0.90 lateral vs half-width ~0.478 + wheel half 0.08 -> ~0.34 m past
        check('excess is plausible', 0.2 < e.worst < 0.5, '%.3f m' % e.worst)

    print('\ncase 4: summary renders')
    txt = run.summary()
    ok = 'BENCH RESULT' in txt and 'off-track events' in txt
    check('summary text', ok)
    print()
    print('\n'.join('    ' + l for l in txt.splitlines()))

    print()
    if FAILURES:
        print('FAILED: %s' % ', '.join(FAILURES))
        sys.exit(1)
    print('all checks passed')


if __name__ == '__main__':
    main()
