"""Latched low-speed wheel feedback watchdog, independent of firmware guards."""


class MotionGuard:
    def __init__(self):
        self.targets = [0.0, 0.0]
        self.settle_until = [0.0, 0.0]
        self.missing_since = [None, None]
        self.fault = ''

    def command(self, now, targets):
        for i, target in enumerate(targets):
            old = self.targets[i]
            if abs(target) < 8.0:
                self.missing_since[i] = None
            elif abs(old) < 8.0 or old*target < 0:
                self.settle_until[i] = now + 2.0
                self.missing_since[i] = None
            self.targets[i] = target

    def sample(self, now, measured):
        for i, rpm in enumerate(measured):
            if (abs(self.targets[i]) < 8.0 or now < self.settle_until[i]
                    or abs(rpm) >= 0.5):
                self.missing_since[i] = None
            elif self.missing_since[i] is None:
                self.missing_since[i] = now
            elif now - self.missing_since[i] >= 1.5:
                self.fault = ('LEFT' if i == 0 else 'RIGHT') + '_NO_WHEEL_FEEDBACK'
        return self.fault
