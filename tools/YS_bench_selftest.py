"""Offline check of YS_bench's geometry and scoring, against synthetic poses
on a synthetic track. No simulator, no route file, no arguments -- so the
harness is known-good before any sim credit is spent on it.

The track is built here rather than loaded. It used to read whatever .npy
was lying in physicar-sim's directory, which stopped working once YS_bench
learned to take its track from the running simulator instead: the file was
no longer part of any workflow, so the suite could not run at all. A test
that depends on a file nobody maintains is a test that quietly stops being
run. Generating the geometry also lets the cases below state their expected
answers in closed form.

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

# Roughly the competition track's proportions -- 30.5 m round, 0.72 m wide --
# so the thresholds the cases assert on are the ones that matter in practice.
TRACK_LENGTH = 30.5
TRACK_HALF_WIDTH = 0.36


def make_track(length=TRACK_LENGTH, half_width=TRACK_HALF_WIDTH, points=400):
    """A closed circular track of the given length and width.

    A circle rather than a replica of the real course: these cases test the
    scorer's arithmetic -- arc length, lateral projection, lap detection --
    and a shape whose answers are known exactly is what makes a failure
    readable. Whether the scorer handles the real course's geometry is
    settled by running it on the real course, not here.
    """
    radius = length / (2.0 * math.pi)
    centre, cum = [], []
    for i in range(points):
        a = 2.0 * math.pi * i / points
        centre.append((radius * math.cos(a), radius * math.sin(a)))
        cum.append(length * i / points)
    return B.Track(centre, [half_width] * points, length, cum)


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
    track = make_track()
    print('synthetic track: %.2f m, %d points, half-width %.3f m\n'
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

    print('\ncase 4: starting ON the line does not score a lap')
    # The car starts at the start line, where the projection is ambiguous: a
    # pose a hair before it lands on the closing segment and reports s = L,
    # so the next sample looks like a wrap. With --laps 1 that ended the run
    # 50 ms in and logged a 0.05 s lap.
    run = B.Run(track, 'seam')
    r = track.length / (2.0 * math.pi)
    t = 1000.0
    for i in range(20):                      # creep forward 2 cm at a time
        a = (0.02 * i) / r
        run.add(t, r * math.cos(a), r * math.sin(a), a + math.pi / 2, 0.4)
        t += 0.05
    check('no phantom lap at the line', len(run.laps) == 0,
          '%d laps' % len(run.laps))

    print('\ncase 5: summary renders')
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
