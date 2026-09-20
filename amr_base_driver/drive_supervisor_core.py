"""Pure state machine for safe direction changes with a passive caster."""

import math


def clamp(value, low, high):
    return min(max(value, low), high)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def slew(current, target, limit_per_second, dt):
    maximum_change = max(0.0, limit_per_second * dt)
    return current + clamp(target - current, -maximum_change, maximum_change)


class DriveSupervisorCore:
    """Shape commands and handle forward/reverse caster realignment.

    A passive caster cannot turn while the robot is stationary.  On a linear
    direction reversal this controller stops, creeps in the new direction,
    stops again, accepts the heading reached after the caster settles, then
    blends a gentle heading hold back in.  It deliberately avoids forcing a
    return arc while the caster is still swivelling.
    """

    IDLE = 'IDLE'
    NORMAL = 'NORMAL'
    INITIAL_ALIGN = 'INITIAL_CASTER_ALIGN'
    BRAKE = 'BRAKE_FOR_REVERSAL'
    ALIGN = 'CASTER_ALIGN'
    ALIGN_STOP = 'STOP_AFTER_ALIGN'

    def __init__(
            self, linear_accel=0.12, linear_decel=0.35,
            angular_accel=0.8, align_speed=0.06, align_distance=0.12,
            align_timeout=2.8, stop_timeout=0.8, heading_kp=1.8,
            heading_max=0.28, heading_min=0.10,
            align_heading_max=0.05,
            heading_tolerance=math.radians(2.0),
            initial_alignment=True,
            straight_heading_kp=1.5, straight_heading_kd=0.45,
            straight_heading_max=0.22,
            straight_heading_tolerance=math.radians(0.8),
            straight_angular_deadband=0.01,
            heading_hold_ramp_time=1.0,
            yaw_rate_filter_tau=0.25):
        self.linear_accel = linear_accel
        self.linear_decel = linear_decel
        self.angular_accel = angular_accel
        self.align_speed = align_speed
        self.align_distance = align_distance
        self.align_timeout = align_timeout
        self.stop_timeout = stop_timeout
        self.heading_kp = heading_kp
        self.heading_max = heading_max
        self.heading_min = heading_min
        self.align_heading_max = align_heading_max
        self.heading_tolerance = heading_tolerance
        self.initial_alignment = initial_alignment
        self.straight_heading_kp = straight_heading_kp
        self.straight_heading_kd = straight_heading_kd
        self.straight_heading_max = straight_heading_max
        self.straight_heading_tolerance = straight_heading_tolerance
        self.straight_angular_deadband = straight_angular_deadband
        self.heading_hold_ramp_time = heading_hold_ramp_time
        self.yaw_rate_filter_tau = yaw_rate_filter_tau

        self.state = self.IDLE
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.last_drive_sign = 0
        self.reversal_sign = 0
        self.reference_yaw = None
        self.align_start_xy = None
        self.state_since = 0.0
        self.heading_error = 0.0
        self.straight_reference_yaw = None
        self.heading_hold_since = None
        self.filtered_yaw_rate = 0.0

    @staticmethod
    def sign(value, deadband=0.005):
        if value > deadband:
            return 1
        if value < -deadband:
            return -1
        return 0

    def enter(self, state, now):
        self.state = state
        self.state_since = now

    def stop_now(self, now):
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.reversal_sign = 0
        self.reference_yaw = None
        self.align_start_xy = None
        self.heading_error = 0.0
        self.straight_reference_yaw = None
        self.heading_hold_since = None
        self.filtered_yaw_rate = 0.0
        self.enter(self.IDLE, now)

    def _stopped(self, measured_linear, now):
        if measured_linear is not None:
            return abs(measured_linear) < 0.012
        return now - self.state_since >= self.stop_timeout

    def _heading_command(self, yaw):
        if yaw is None or self.reference_yaw is None:
            self.heading_error = 0.0
            return 0.0
        self.heading_error = wrap_angle(self.reference_yaw - yaw)
        command = clamp(
            self.heading_kp * self.heading_error,
            -self.heading_max, self.heading_max)
        if (abs(self.heading_error) > self.heading_tolerance and
                abs(command) < self.heading_min):
            command = math.copysign(self.heading_min, command)
        return command

    def _filter_yaw_rate(self, dt, yaw_rate):
        """Low-pass EKF yaw rate; its fast component comes from gyro Z."""
        if yaw_rate is None:
            return
        tau = max(0.0, self.yaw_rate_filter_tau)
        alpha = 1.0 if tau == 0.0 else clamp(dt / (tau + dt), 0.0, 1.0)
        self.filtered_yaw_rate += alpha * (
            yaw_rate - self.filtered_yaw_rate)

    def _straight_heading_command(self, now, yaw):
        if yaw is None:
            self.heading_error = 0.0
            return 0.0
        if self.straight_reference_yaw is None:
            self.straight_reference_yaw = yaw
            self.heading_hold_since = now
        self.heading_error = wrap_angle(self.straight_reference_yaw - yaw)
        if (abs(self.heading_error) <= self.straight_heading_tolerance and
                abs(self.filtered_yaw_rate) < math.radians(1.0)):
            return 0.0
        command = clamp(
            self.straight_heading_kp * self.heading_error -
            self.straight_heading_kd * self.filtered_yaw_rate,
            -self.straight_heading_max, self.straight_heading_max)
        if self.heading_hold_since is None:
            self.heading_hold_since = now
        if self.heading_hold_ramp_time <= 0.0:
            return command
        blend = clamp(
            (now - self.heading_hold_since) / self.heading_hold_ramp_time,
            0.0, 1.0)
        return blend * command

    def _align_complete(self, now, x, y):
        if now - self.state_since >= self.align_timeout:
            return True
        if self.align_start_xy is None or x is None or y is None:
            return False
        return math.hypot(
            x - self.align_start_xy[0],
            y - self.align_start_xy[1]) >= self.align_distance

    def _capture_aligned_heading(self, now, yaw):
        """Accept the post-caster heading and resume without a return arc."""
        self.last_drive_sign = self.reversal_sign
        self.reference_yaw = yaw
        self.straight_reference_yaw = yaw
        self.heading_hold_since = now
        self.heading_error = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.enter(self.NORMAL, now)

    def update(self, now, dt, target_linear, target_angular,
               input_fresh=True, x=None, y=None, yaw=None,
               measured_linear=None, measured_angular=None):
        """Return (linear, angular, state) for the current control tick."""
        dt = clamp(dt, 0.0, 0.2)
        self._filter_yaw_rate(dt, measured_angular)
        target_sign = self.sign(target_linear)

        # Deadman/watchdog and an explicit all-zero command always win.  This
        # intentionally bypasses ramps so releasing a key stops immediately.
        if (not input_fresh or
                (target_sign == 0 and abs(target_angular) < 0.005)):
            self.stop_now(now)
            return self.output_linear, self.output_angular, self.state

        # A changed/cancelled reversal request must not leave an autonomous
        # caster sequence running.
        if self.state in (
                self.INITIAL_ALIGN, self.BRAKE, self.ALIGN, self.ALIGN_STOP):
            if target_sign != self.reversal_sign:
                self.stop_now(now)
                if target_sign == 0:
                    # Pure rotation remains directly controllable.
                    self.enter(self.NORMAL, now)
                else:
                    return self.output_linear, self.output_angular, self.state

        reversal = (target_sign != 0 and self.last_drive_sign != 0 and
                    target_sign != self.last_drive_sign)
        if self.state in (self.IDLE, self.NORMAL):
            if (target_sign != 0 and self.last_drive_sign == 0 and
                    self.initial_alignment):
                self.reversal_sign = target_sign
                self.reference_yaw = yaw
                self.align_start_xy = (
                    (x, y) if x is not None and y is not None else None)
                self.enter(self.INITIAL_ALIGN, now)
            elif reversal:
                self.reversal_sign = target_sign
                self.reference_yaw = yaw
                self.straight_reference_yaw = None
                self.enter(self.BRAKE, now)

        if self.state == self.BRAKE:
            self.output_linear = slew(
                self.output_linear, 0.0, self.linear_decel, dt)
            self.output_angular = slew(
                self.output_angular, 0.0, self.angular_accel * 2.0, dt)
            if (abs(self.output_linear) < 0.003 and
                    abs(self.output_angular) < 0.01 and
                    self._stopped(measured_linear, now)):
                self.output_linear = 0.0
                self.output_angular = 0.0
                self.align_start_xy = (
                    (x, y) if x is not None and y is not None else None)
                self.enter(self.ALIGN, now)

        elif self.state in (self.INITIAL_ALIGN, self.ALIGN):
            creep = self.reversal_sign * self.align_speed
            self.output_linear = slew(
                self.output_linear, creep, self.linear_accel, dt)
            # Use only a small heading trim while the caster swivels.  Zero
            # correction allowed large initial heading excursions on the real
            # base, while the normal recovery limit fought caster bore torque.
            # The bounded trim contains the drift without commanding a sharp
            # differential-wheel correction during caster realignment.
            heading_command = clamp(
                self._heading_command(yaw),
                -self.align_heading_max, self.align_heading_max)
            self.output_angular = slew(
                self.output_angular, heading_command,
                self.angular_accel, dt)
            if self._align_complete(now, x, y):
                self.enter(self.ALIGN_STOP, now)

        elif self.state == self.ALIGN_STOP:
            self.output_linear = slew(
                self.output_linear, 0.0, self.linear_decel, dt)
            self.output_angular = slew(
                self.output_angular, 0.0, self.angular_accel * 2.0, dt)
            if (abs(self.output_linear) < 0.003 and
                    abs(self.output_angular) < 0.01 and
                    self._stopped(measured_linear, now)):
                self.output_linear = 0.0
                self.output_angular = 0.0
                # Do not force the chassis back to the pre-alignment yaw.
                # Capture the settled caster heading, then blend straight-line
                # hold in over heading_hold_ramp_time.
                self._capture_aligned_heading(now, yaw)

        else:
            self.enter(self.NORMAL, now)
            linear_limit = (self.linear_accel if
                            abs(target_linear) > abs(self.output_linear)
                            else self.linear_decel)
            self.output_linear = slew(
                self.output_linear, target_linear, linear_limit, dt)
            if (target_sign != 0 and
                    abs(target_angular) < self.straight_angular_deadband):
                angular_target = self._straight_heading_command(now, yaw)
            else:
                # Preserve intentional keyboard/Nav2 turns.  The next truly
                # straight command captures the resulting heading.
                self.straight_reference_yaw = None
                self.heading_hold_since = None
                self.heading_error = 0.0
                angular_target = target_angular
            self.output_angular = slew(
                self.output_angular, angular_target,
                self.angular_accel, dt)
            if target_sign:
                self.last_drive_sign = target_sign

        return self.output_linear, self.output_angular, self.state
