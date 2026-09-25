"""Low-speed wheel feedback watchdog with one bounded recovery opportunity."""


# The ESP32 can hard-latch an encoder stall about 1.9 s after a new command
# (startup boost + its retry grace).  Pause the command comfortably before that
# deadline so the agent can inspect the scan and request one bounded escape.
# Once a wheel was already moving, 450 ms without feedback is long enough to
# distinguish a real obstruction from the normal 20 Hz telemetry interval.
STARTUP_SETTLE_SECONDS = 0.85
MISSING_FEEDBACK_SECONDS = 0.45
RECOVERY_REARM_SECONDS = 2.0


class MotionGuard:
    def __init__(self):
        self.targets = [0.0, 0.0]
        self.settle_until = [0.0, 0.0]
        self.missing_since = [None, None]
        self.fault = ''
        self.pre_stall = ''
        self.recovery_reason = ''
        self.recovery_in_progress = False
        self.recovery_attempted = False
        self.recovery_blocked = ''
        self.recovery_failed = ''
        self.healthy_since = None

    @property
    def state(self):
        if self.fault:
            return 'FAULTED'
        if self.pre_stall:
            return 'PRE_STALL'
        if self.recovery_in_progress:
            return 'RECOVERING'
        if self.recovery_blocked:
            return 'RECOVERY_BLOCKED'
        if self.recovery_failed:
            return 'RECOVERY_FAILED'
        if self.recovery_attempted:
            return 'VERIFYING_RECOVERY'
        return 'NORMAL'

    @property
    def blocks_motion(self):
        return bool(
            self.fault or self.pre_stall
            or self.recovery_blocked or self.recovery_failed
        )

    def command(self, now, targets):
        for i, target in enumerate(targets):
            old = self.targets[i]
            if abs(target) < 8.0:
                self.missing_since[i] = None
            elif abs(old) < 8.0 or old*target < 0:
                self.settle_until[i] = now + STARTUP_SETTLE_SECONDS
                self.missing_since[i] = None
            self.targets[i] = target

    def begin_recovery(self, now):
        """Authorize exactly one recovery maneuver for a pending pre-stall."""
        if not self.pre_stall or self.fault or self.recovery_attempted:
            return False
        self.recovery_reason = self.pre_stall
        self.pre_stall = ''
        self.recovery_in_progress = True
        self.recovery_attempted = True
        self.healthy_since = None
        self.missing_since = [None, None]
        self.settle_until = [
            now + STARTUP_SETTLE_SECONDS,
            now + STARTUP_SETTLE_SECONDS,
        ]
        return True

    def finish_recovery(self, success):
        """Finish motion verification without inventing an ESP32 fault."""
        if self.pre_stall and not success and not self.recovery_attempted:
            self.recovery_reason = self.pre_stall
            self.pre_stall = ''
            self.recovery_attempted = True
        if not self.recovery_attempted:
            return False
        self.recovery_in_progress = False
        self.healthy_since = None
        # A fresh missing-feedback sample can still set ``fault`` while the
        # maneuver is running.  A generic maneuver failure is kept separate:
        # timeout, stale odometry, and service errors do not prove wheel or MCU
        # failure.
        if not success and not self.fault:
            self.recovery_failed = (
                self.recovery_reason or 'WHEEL_RECOVERY_FAILED'
            )
        return True

    def block_recovery(self):
        """Stop after every scan-safe option was refused by a safety guard."""
        if not (self.pre_stall or self.recovery_in_progress):
            return False
        if self.pre_stall:
            self.recovery_reason = self.pre_stall
            self.pre_stall = ''
        self.recovery_in_progress = False
        self.recovery_attempted = True
        self.recovery_failed = ''
        self.recovery_blocked = self.recovery_reason or 'NO_SAFE_ESCAPE'
        self.healthy_since = None
        self.missing_since = [None, None]
        return True

    def _moving_wheels_healthy(self, measured):
        moving = [
            abs(target) >= 8.0
            for target in self.targets
        ]
        return any(moving) and all(
            not expected or abs(rpm) >= 0.5
            for expected, rpm in zip(moving, measured)
        )

    def sample(self, now, measured):
        if (
            self.fault or self.pre_stall
            or self.recovery_blocked or self.recovery_failed
        ):
            return self.fault

        if self.recovery_attempted and not self.recovery_in_progress:
            if self._moving_wheels_healthy(measured):
                if self.healthy_since is None:
                    self.healthy_since = now
                elif now - self.healthy_since >= RECOVERY_REARM_SECONDS:
                    self.recovery_attempted = False
                    self.recovery_reason = ''
                    self.healthy_since = None
            else:
                self.healthy_since = None

        for i, rpm in enumerate(measured):
            if (abs(self.targets[i]) < 8.0 or now < self.settle_until[i]
                    or abs(rpm) >= 0.5):
                self.missing_since[i] = None
            elif self.missing_since[i] is None:
                self.missing_since[i] = now
            elif now - self.missing_since[i] >= MISSING_FEEDBACK_SECONDS:
                reason = (
                    ('LEFT' if i == 0 else 'RIGHT')
                    + '_NO_WHEEL_FEEDBACK'
                )
                if self.recovery_attempted:
                    self.fault = reason
                else:
                    self.pre_stall = reason
                    self.recovery_reason = reason
                self.missing_since = [None, None]
                break
        return self.fault
