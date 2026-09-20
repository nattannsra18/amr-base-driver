#pragma once
#include <stdint.h>

// Require persistent wrong-direction feedback, with a bounded reversal grace.
// A fault still latches in the main firmware; this class never clears faults.
struct DirectionMonitor {
  static constexpr uint32_t REVERSAL_GRACE_MS = 1000;
  static constexpr uint32_t WRONG_DIRECTION_CONFIRM_MS = 350;

  int sign = 0;
  uint32_t changedAt = 0;
  uint32_t wrongAt = 0;
  bool wrongActive = false;

  bool check(uint32_t now, float targetRpm, float measuredRpm) {
    const int nextSign = targetRpm >= 20.0f ? 1 :
                         (targetRpm <= -20.0f ? -1 : 0);
    if (nextSign != sign) {
      sign = nextSign;
      changedAt = now;
      wrongActive = false;
    }
    const bool wrong = sign != 0 && targetRpm * measuredRpm < -100.0f;
    // The drivetrain now uses a bounded 700 ms high-torque start/reversal
    // boost.  Do not interpret encoder coast-down during that interval as a
    // wiring fault; require persistent wrong-direction motion after the boost
    // and mechanical settling time have elapsed.
    if (!wrong || uint32_t(now-changedAt) < REVERSAL_GRACE_MS) {
      wrongActive = false;
      return false;
    }
    if (!wrongActive) {
      wrongAt = now;
      wrongActive = true;
    }
    return uint32_t(now-wrongAt) >= WRONG_DIRECTION_CONFIRM_MS;
  }
};
