"""Stationary-only residual gyro calibration; all values are SI units."""
from collections import deque
import math
import statistics


class StationaryBias:
    def __init__(self):
        self.samples = deque()
        self.bias = [0.0, 0.0, 0.0]
        self.ready = False
        self.last_time = None

    def update(self, now, gyro, acceleration_norm, stationary):
        dt = 0 if self.last_time is None else now - self.last_time
        self.last_time = now
        # The MPU6050 on this robot can boot with a temperature-dependent
        # zero-rate offset close to 0.05 rad/s.  Wheel/command stationarity,
        # gravity magnitude, the five-second window, and the noise check below
        # distinguish that constant offset from actual vehicle motion.  Keep a
        # generous but bounded absolute guard so a real rotation is not learnt.
        good = (stationary and 0 < dt < 0.3 and
                8.8 < acceleration_norm < 10.8 and
                all(math.isfinite(x) and abs(x) < 0.10 for x in gyro))
        if not good:
            self.samples.clear()
            return list(self.bias)
        self.samples.append((now, tuple(gyro)))
        while self.samples and now - self.samples[0][0] > 5.0:
            self.samples.popleft()
        if len(self.samples) < 60 or now-self.samples[0][0] < 4.8:
            return list(self.bias)
        axes = list(zip(*(s[1] for s in self.samples)))
        # Live 15 s probe on this unit measured stationary X/Y standard
        # deviations of 0.0160/0.0136 rad/s; Z was 0.00445 rad/s.  The wheel,
        # command, acceleration, and bounded-rate gates above still reject
        # actual motion, while 0.020 avoids permanently rejecting this unit's
        # normal cross-axis noise.
        if any(statistics.pstdev(a) > 0.020 for a in axes):
            return list(self.bias)
        means = [statistics.mean(a) for a in axes]
        alpha = 1.0 if not self.ready else 1.0-math.exp(-dt/30.0)
        self.bias = [b+alpha*(m-b) for b,m in zip(self.bias, means)]
        self.ready = True
        return list(self.bias)
