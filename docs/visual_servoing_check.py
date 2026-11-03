"""Unicycle check of range/bearing P control with camera delay and measurement noise."""
import math
import random
import statistics
from collections import deque

DT = 0.01
STANDOFF = 0.30
V_MAX = 0.05
W_MAX = 0.30
GATE = 0.20
TOL_R = 0.02
TOL_B = 0.05
HOLD = 0.5
T_MAX = 30.0
FOCAL = 277.0   # px, 640x480 with 98 deg-ish horizontal FOV placeholder


def run_verify(kv, kw, delay, period, sig_px, sig_z, rng, frames=3, move_time=None):
    """Stop-and-verify: completion needs N in-tolerance frames captured after the stop."""
    r0 = STANDOFF + rng.uniform(-0.15, 0.15)
    b0 = rng.uniform(-0.35, 0.35)
    th = rng.uniform(-math.pi, math.pi)
    x = -r0 * math.cos(th + b0)
    y = -r0 * math.sin(th + b0)
    buf = deque()
    meas = None
    next_cam = 0.0
    stop_t = None
    seen = -1.0
    good = 0
    move_until = 0.0
    v = w = 0.0
    t = 0.0
    while t < T_MAX:
        dx, dy = -x, -y
        xb = math.cos(th) * dx + math.sin(th) * dy
        yb = -math.sin(th) * dx + math.cos(th) * dy
        if t >= next_cam:
            next_cam += period
            z = math.hypot(xb, yb) + rng.gauss(0, sig_z)
            b = math.atan2(yb, xb) + rng.gauss(0, sig_px) / FOCAL
            buf.append((t + delay, t, z - STANDOFF, b))
        while buf and buf[0][0] <= t:
            meas = buf.popleft()
        if meas and meas[1] > seen:
            seen = meas[1]
            _, cap, er, eb = meas
            fresh = stop_t is not None and cap > stop_t
            inside = abs(er) < TOL_R and abs(eb) < TOL_B
            if move_time is None:          # continuous until inside, then verify
                if stop_t is None:
                    if inside:
                        stop_t = t
                        good = 0
                    else:
                        w = max(-W_MAX, min(W_MAX, kw * eb))
                        v = max(-V_MAX, min(V_MAX, kv * er))
                        v = 0.0 if abs(eb) > GATE else v
                elif fresh:
                    if inside:
                        good += 1
                    else:
                        stop_t = None
                        good = 0
            elif stop_t is None or fresh:  # move in bursts, re-measure only when stopped
                if inside:
                    good += 1
                else:
                    good = 0
                    w = max(-W_MAX, min(W_MAX, kw * eb))
                    v = max(-V_MAX, min(V_MAX, kv * er))
                    v = 0.0 if abs(eb) > GATE else v
                    move_until = t + move_time
                    stop_t = None
            if good >= frames:
                return True, t, abs(math.hypot(xb, yb) - STANDOFF), abs(math.atan2(yb, xb))
        if stop_t is not None and move_time is None:
            v = w = 0.0
        if move_time is not None and t >= move_until and stop_t is None:
            v = w = 0.0
            stop_t = t
        th += w * DT
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        t += DT
    return False, T_MAX, abs(math.hypot(xb, yb) - STANDOFF), abs(math.atan2(yb, xb))


def run(kv, kw, delay, period, sig_px, sig_z, gated, rng):
    r0 = STANDOFF + rng.uniform(-0.15, 0.15)
    b0 = rng.uniform(-0.35, 0.35)
    # object fixed at origin; robot placed so it sees the object at (r0, b0)
    th = rng.uniform(-math.pi, math.pi)
    x = -r0 * math.cos(th + b0)
    y = -r0 * math.sin(th + b0)
    buf = deque()
    meas = None
    next_cam = 0.0
    held = 0.0
    v = w = 0.0
    t = 0.0
    while t < T_MAX:
        dx, dy = -x, -y
        xb = math.cos(th) * dx + math.sin(th) * dy
        yb = -math.sin(th) * dx + math.cos(th) * dy
        if t >= next_cam:
            next_cam += period
            z = math.hypot(xb, yb) + rng.gauss(0, sig_z)
            b = math.atan2(yb, xb) + rng.gauss(0, sig_px) / FOCAL
            buf.append((t + delay, z - STANDOFF, b))
        while buf and buf[0][0] <= t:
            meas = buf.popleft()
        if meas:
            _, er, eb = meas
            w = max(-W_MAX, min(W_MAX, kw * eb))
            v = max(-V_MAX, min(V_MAX, kv * er))
            if gated and abs(eb) > GATE:
                v = 0.0
            held = held + DT if abs(er) < TOL_R and abs(eb) < TOL_B else 0.0
            if held >= HOLD:
                true_r = math.hypot(xb, yb) - STANDOFF
                true_b = math.atan2(yb, xb)
                return True, t, abs(true_r), abs(true_b)
        th += w * DT
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        t += DT
    return False, T_MAX, abs(math.hypot(xb, yb) - STANDOFF), abs(math.atan2(yb, xb))


def table(label, fn=None, **kw):
    rng = random.Random(7)
    rs = [(fn or run)(rng=rng, **kw) for _ in range(200)]
    ok = [r for r in rs if r[0]]
    if not ok:
        print(f"{label:<44} success 0%")
        return
    q = lambda xs: sorted(xs)[int(len(xs) * 0.95) - 1]
    print(f"{label:<44} success {100*len(ok)/len(rs):3.0f}%  "
          f"t_med {statistics.median(r[1] for r in ok):5.1f}s  "
          f"range_p95 {q([r[2] for r in ok])*1000:4.1f}mm  "
          f"bearing_p95 {q([r[3] for r in ok]):.3f}rad")


base = dict(delay=0.2, period=0.1, sig_px=2.0, sig_z=0.005, gated=True)
print("--- gains (delay 0.2s, 10Hz, noise 2px/5mm)")
for kv in (0.5, 1.0, 2.0):
    for kw in (0.5, 1.0, 2.0, 4.0):
        table(f"kv={kv} kw={kw}", kv=kv, kw=kw, **base)
print("--- rotate gate")
table("gated kv=1 kw=1", kv=1.0, kw=1.0, **base)
table("ungated kv=1 kw=1", kv=1.0, kw=1.0, **{**base, "gated": False})
print("--- slow sim (kv=1 kw=1)")
for delay, period in ((0.2, 0.1), (0.5, 0.2), (1.0, 0.5), (2.0, 1.0), (4.0, 2.0)):
    table(f"delay={delay} period={period}", kv=1.0, kw=1.0,
          **{**base, "delay": delay, "period": period})
print("--- noise (kv=1 kw=1, delay 0.2)")
for sp, sz in ((0, 0), (2, 0.005), (5, 0.01), (10, 0.02)):
    table(f"px={sp} z={sz}", kv=1.0, kw=1.0, **{**base, "sig_px": sp, "sig_z": sz})
print("--- completion rule vs latency (kv=1 kw=1, noise 2px/5mm)")
v = dict(kv=1.0, kw=1.0, sig_px=2.0, sig_z=0.005)
for delay, period in ((0.2, 0.1), (0.5, 0.2), (1.0, 0.5), (2.0, 1.0), (4.0, 2.0)):
    table(f"plain   delay={delay}", delay=delay, period=period, gated=True, **v)
    table(f"verify  delay={delay}", fn=run_verify, delay=delay, period=period, **v)
    table(f"burst   delay={delay}", fn=run_verify, delay=delay, period=period, move_time=0.5, **v)
