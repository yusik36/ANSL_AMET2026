#!/usr/bin/env python3
"""Turn a camera frame into a drivable corridor, a steering target, and a speed.

No ROS here on purpose: every number below was measured, and keeping the
maths free of rclpy is what let it be checked against synthetic corridors and
real frames without a simulator running.

WHAT IS MEASURED, AND WHERE IT CAME FROM (simulator, 2026-08-18)

Camera geometry, chained out of physicar.urdf.xacro and then verified by
parking the car at known distances behind a cone and comparing the predicted
image row against the observed one:

    distance   0.6    0.9    1.2    1.6    2.0    2.5    3.0    4.0
    error [m]  0.05   0.09   0.07   0.05   0.12   0.22   0.50   2.12

So the corridor is trustworthy to about 2.5 m and falls apart past 3 m --
0.50 m of range error is larger than the track's own half-width. DEFAULT_MAX
below is set from that measurement, not from what would be convenient.

Surface colours, read off the frame rather than assumed:

    road            H 106      S  61-93    V  74-104
    white edge line H  24      S   6       V 220
    grass           H  31-40   S 120-154   V 148
    cone            H  60      S 240-252   V 149-180
    orange dashes   H  18      S 247       V 227

WHY ONE RULE HANDLES BOTH GRASS AND CONES

Grass and cones are both green-ish and saturated; road, white paint and
orange paint are not. So "green-ish and saturated means not drivable" marks
the track edges and the obstacles in a single pass, and a cone becomes
simply a place where the corridor is narrower. There is no obstacle mode to
enter and no control to hand back -- the thing that made the previous
avoid/lane split fragile stops existing.

The thresholds sit between the measured clusters with room either side:
orange paint at H 18 stays below the band, road at H 106 stays above it, and
white paint fails the saturation test outright.
"""
import math

import numpy as np

# --- camera, from the URDF chain ---
CAM_HEIGHT = 0.1645         # base 0.0375 + pan 0.1 + tilt 0.013 + link 0.014
CAM_FORWARD = 0.105         # ahead of the rear axle
CAL_W, CAL_H = 640.0, 480.0
FX, FY, CX, CY = 387.89, 387.19, 312.63, 229.36

# --- vehicle, from driver_params.yaml and the URDF ---
WHEELBASE = 0.18
CAR_HALF_WIDTH = 0.08
MAX_STEER = math.radians(20.0)
MAX_SPEED = 3.0

# --- the "not drivable" rule, from the measured colours above ---
BLOCK_H_MIN, BLOCK_H_MAX, BLOCK_S_MIN = 25, 90, 100

# How fast the car's lateral position can change per metre travelled. A
# stand-in for the vehicle's turning ability in the sweep below: at the
# minimum turn radius the path leaves a straight line by roughly this slope
# over the distances the camera can see.
MAX_LATERAL_SLOPE = 0.4

# --- corridor sampling ---
DEFAULT_MIN_RANGE = 0.35    # nearer than this the camera sees mostly bumper
DEFAULT_MAX_RANGE = 2.5     # where the measured range error passes 0.25 m
DEFAULT_SAMPLES = 14


def intrinsics(w, h):
    return FX * w / CAL_W, FY * h / CAL_H, CX * w / CAL_W, CY * h / CAL_H


def distance_to_row(d, w, h):
    _fx, fy, _cx, cy = intrinsics(w, h)
    z = max(d - CAM_FORWARD, 1e-6)
    return cy + fy * CAM_HEIGHT / z


def column_to_lateral(col, d, w, h):
    """Image column at ground distance d -> metres left of the car's centre."""
    fx, _fy, cx, _cy = intrinsics(w, h)
    z = max(d - CAM_FORWARD, 1e-6)
    return -(col - cx) * z / fx


def lateral_to_column(lat, d, w, h):
    fx, _fy, cx, _cy = intrinsics(w, h)
    z = max(d - CAM_FORWARD, 1e-6)
    return cx - lat * fx / z


def blocked_mask(hsv, h_min=BLOCK_H_MIN, h_max=BLOCK_H_MAX, s_min=BLOCK_S_MIN):
    """True where the surface is not drivable: grass verges and cones alike."""
    H = hsv[:, :, 0].astype(np.int16)
    S = hsv[:, :, 1].astype(np.int16)
    return (H >= h_min) & (H <= h_max) & (S >= s_min)


def scan_corridor(blocked, w, h, ranges):
    """Free width either side of the car's centre line, at each range.

    Scans outward from the centre column rather than inward from the frame
    edges: the question is how far the car can move before hitting
    something, and starting at the car answers it directly. If the centre
    itself is blocked -- the car is off the track, or a cone is dead ahead --
    that range reports no free span rather than inventing one.

    Returns a list of (distance, right_limit, left_limit) in metres, where
    right is negative and left positive.
    """
    out = []
    for d in ranges:
        row = int(round(distance_to_row(d, w, h)))
        if row < 0 or row >= h:
            continue
        line = blocked[row]
        centre = int(round(lateral_to_column(0.0, d, w, h)))
        if centre < 0 or centre >= w or line[centre]:
            out.append((d, 0.0, 0.0))
            continue

        left_col = centre
        while left_col - 1 >= 0 and not line[left_col - 1]:
            left_col -= 1
        right_col = centre
        while right_col + 1 < w and not line[right_col + 1]:
            right_col += 1

        # A span that reaches the frame edge is not a measurement of the
        # track, it is the camera running out of view, so it is reported as
        # open and the caller decides what to do with that.
        left = column_to_lateral(left_col, d, w, h)
        right = column_to_lateral(right_col, d, w, h)
        out.append((d, right, left))
    return out


def reachable_band(corridor, max_slope=MAX_LATERAL_SLOPE,
                   half_width=CAR_HALF_WIDTH):
    """Where the car's centre could actually be at each distance.

    Reading the free span at the lookahead distance alone is not enough, and
    the mistake is not subtle: a cone sitting between the car and that
    distance is simply not seen, because nothing ever asks about the ground
    the car has to cross to get there. The first version did exactly that,
    and a cone 1.2 m ahead moved the target by 1 mm.

    So the spans are swept front to back. The band starts as a point -- the
    car is where it is -- and widens by max_slope per metre travelled, which
    is the crude stand-in for "the car can move sideways while going
    forward, but not instantly". At each step the widened band is clipped to
    the free span there, shrunk by the car's half width so its body fits and
    not just its centre line.

    A plain intersection of all the spans would be simpler and wrong: on
    this track's tightest corner the track shifts about 0.86 m over a metre
    of travel, so the intersection would be empty and the car would stop in
    every corner.

    Returns [(distance, lo, hi)], with lo > hi meaning nothing is reachable
    from there on.
    """
    out = []
    lo = hi = 0.0
    prev_d = 0.0
    dead = False
    for d, r, l in corridor:
        step = max(d - prev_d, 0.0)
        prev_d = d
        if dead:
            out.append((d, 1.0, -1.0))
            continue
        lo -= max_slope * step
        hi += max_slope * step
        lo = max(lo, r + half_width)
        hi = min(hi, l - half_width)
        if lo > hi:
            dead = True
            out.append((d, 1.0, -1.0))
            continue
        out.append((d, lo, hi))
    return out


def survivable_band(corridor, max_slope=MAX_LATERAL_SLOPE,
                    half_width=CAR_HALF_WIDTH):
    """Where the car could be at each distance and still get through what
    comes after it.

    The forward sweep alone answers the wrong question. It says where the
    car can get to, so a cone beyond the lookahead does not constrain the
    target at all -- and a cone 1.2 m out left a 0.975 m target unmoved,
    which is the whole of obstacle avoidance failing quietly. Sweeping back
    from the far end instead asks which positions still have a way past what
    is coming, and that is what decides which side of a cone to aim for.

    Returns the same shape as reachable_band. If the sweep finds no way
    through at some point it stops constraining rather than condemning
    everything nearer -- the corridor usually just ran out of camera, and a
    cone is far too narrow to close this track.
    """
    out = [None] * len(corridor)
    lo = hi = None
    next_d = None
    for i in range(len(corridor) - 1, -1, -1):
        d, r, l = corridor[i]
        span_lo, span_hi = r + half_width, l - half_width
        if lo is None:
            lo, hi = span_lo, span_hi
        else:
            step = max((next_d or d) - d, 0.0)
            lo = max(span_lo, lo - max_slope * step)
            hi = min(span_hi, hi + max_slope * step)
        if lo > hi:                      # nothing gets through from here on
            lo, hi = None, None
            for j in range(i, -1, -1):
                out[j] = (corridor[j][0], corridor[j][1] + half_width,
                          corridor[j][2] - half_width)
            break
        out[i] = (d, lo, hi)
        next_d = d
    return [b for b in out if b is not None]


def pick_target(corridor, lookahead, max_slope=MAX_LATERAL_SLOPE,
                half_width=CAR_HALF_WIDTH):
    """Centre of the band that is both reachable and survivable.

    Returns (distance, lateral), or None when there is nowhere to go --
    which is the honest answer when the view is blocked, and is what the
    caller turns into a stop.
    """
    if not corridor:
        return None
    fwd = {d: (lo, hi) for d, lo, hi in
           reachable_band(corridor, max_slope, half_width)}
    bwd = {d: (lo, hi) for d, lo, hi in
           survivable_band(corridor, max_slope, half_width)}

    bands = []
    for d, _r, _l in corridor:
        if d not in fwd or d not in bwd:
            continue
        lo = max(fwd[d][0], bwd[d][0])
        hi = min(fwd[d][1], bwd[d][1])
        if lo <= hi:
            bands.append((d, lo, hi))
    if not bands:
        return None
    d, lo, hi = min(bands, key=lambda b: abs(b[0] - lookahead))
    return d, 0.5 * (lo + hi)


def pure_pursuit(target, wheelbase=WHEELBASE, max_steer=MAX_STEER):
    """Steering angle to reach a point, and the radius that implies.

    Positive steering turns left, matching Physicar: the simulator's adapter
    publishes angular.z = v*tan(steering)/wheelbase and the real driver
    leaves its steering channel non-inverted.
    """
    d, lat = target
    ld = math.hypot(d, lat)
    if ld < 1e-6:
        return 0.0, float('inf')
    alpha = math.atan2(lat, d)
    steer = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    steer = max(-max_steer, min(max_steer, steer))
    sin_a = math.sin(alpha)
    radius = float('inf') if abs(sin_a) < 1e-6 else abs(ld / (2.0 * sin_a))
    return steer, radius


def corridor_curvature(corridor):
    """How sharply the visible corridor bends, from the corridor itself.

    Fitted to the RAW free spans, not to the reachability band. The band is
    shaped by max_slope, so measuring it measures the planner's own
    assumption as much as the track: a tight slope made a 1.25 m corner
    read as 0.40 m, a loose one as 1.92 m. The free spans come straight from
    the image and do not move when a tuning constant does.

    Speed used to come from the radius the steering target implied, which
    quietly coupled it to how the target was chosen. Tightening the band
    then made the car FASTER into a corner: with less room to aim sideways
    the target stayed near straight ahead, the implied radius grew, and the
    planner concluded the corner was gentle. The corner had not changed --
    only the planner's ability to look at it.

    Fitting the corridor's own centres separates the two. Where the track
    goes is a fact about the track; where to aim is a decision about the
    car.
    """
    pts = [(d, 0.5 * (r + l)) for d, r, l in corridor if l > r]
    if len(pts) < 3:
        return 0.0
    d = np.array([p[0] for p in pts])
    lat = np.array([p[1] for p in pts])
    try:
        a, _b, _c = np.polyfit(d, lat, 2)
    except (np.linalg.LinAlgError, ValueError):
        return 0.0
    return abs(2.0 * float(a))          # lat = a d^2 -> curvature ~ 2a


def speed_for(radius, a_lat, max_speed=MAX_SPEED):
    """Cornering speed from the radius the steering target implies.

    a_lat is the aggression knob, not a measured constant. The simulator
    runs ODE's default mu=1 on every wheel, which is worth about 9.8 m/s^2
    and flatters the real car, so tuning against the simulator's true grip
    would produce speeds the real tyres cannot hold. Develop at a
    pessimistic value and raise it only when the real car shows it can take
    more.
    """
    if not math.isfinite(radius):
        return max_speed
    return min(max_speed, math.sqrt(max(a_lat, 0.0) * radius))


def lookahead_for(speed, gain, base):
    """Ld = gain*v + base.

    ForzaETH's rule of thumb, and the reason it is a rule: too short and the
    car oscillates at speed, too long and it cuts corners. Cutting is what
    this track needs -- its centre line is tighter than the car's minimum
    turn radius -- so the base is not apologetic about being generous.
    """
    return max(0.2, gain * speed + base)


def plan(hsv, speed_now, a_lat, gain, base,
         min_range=DEFAULT_MIN_RANGE, max_range=DEFAULT_MAX_RANGE,
         samples=DEFAULT_SAMPLES, block=(BLOCK_H_MIN, BLOCK_H_MAX, BLOCK_S_MIN),
         max_slope=None):
    """One frame in, (speed, steering, debug) out. None target means stop."""
    h, w = hsv.shape[:2]
    if max_slope is None:
        max_slope = MAX_LATERAL_SLOPE
    blocked = blocked_mask(hsv, *block)
    ld = lookahead_for(speed_now, gain, base)
    ranges = np.linspace(min_range, max_range, samples)
    corridor = scan_corridor(blocked, w, h, ranges)
    target = pick_target(corridor, ld, max_slope)
    if target is None:
        return 0.0, 0.0, {'corridor': corridor, 'lookahead': ld, 'target': None}

    steer, radius = pure_pursuit(target)
    # Speed from the corner, steering from the target: see corridor_curvature.
    kappa = corridor_curvature(corridor)
    corner_radius = float('inf') if kappa < 1e-6 else 1.0 / kappa
    speed = speed_for(min(radius, corner_radius), a_lat)
    return speed, steer, {'corridor': corridor, 'lookahead': ld,
                          'target': target, 'radius': radius,
                          'corner_radius': corner_radius}
