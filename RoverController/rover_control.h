#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include <stdint.h>
#include "motion.h"

namespace rover {
// Duty cycles in hundredths of one percent: 946 means 9.46%.
constexpr uint16_t THROTTLE_NEUTRAL = 946;
constexpr uint16_t THROTTLE_FORWARD = 1219;
constexpr uint16_t THROTTLE_REVERSE = 826;
constexpr uint16_t STEERING_NEUTRAL = 961;
constexpr uint16_t STEERING_RIGHT = 637;
constexpr uint16_t STEERING_LEFT = 1311;

// Uno: 16 MHz / 8 prescaler / 31918 counts = 62.660568 Hz.
constexpr uint16_t PERIOD_TICKS = 31918;

inline uint16_t interpolate(int percent, uint16_t neutral,
                            uint16_t positive, uint16_t negative) {
  if (percent > 100) percent = 100;
  if (percent < -100) percent = -100;
  const int32_t magnitude = percent < 0 ? -percent : percent;
  const int32_t delta = int32_t(percent < 0 ? negative : positive) - neutral;
  const int32_t scaled = delta * magnitude;
  return uint16_t(neutral + (scaled >= 0 ? scaled + 50 : scaled - 50) / 100);
}

inline uint16_t throttleDuty(int percent) {
  return interpolate(percent, THROTTLE_NEUTRAL, THROTTLE_FORWARD, THROTTLE_REVERSE);
}

// Positive = right, negative = left.
inline uint16_t steeringDuty(int percent) {
  return interpolate(percent, STEERING_NEUTRAL, STEERING_RIGHT, STEERING_LEFT);
}

inline uint16_t dutyTicks(uint16_t hundredths) {
  return uint16_t((uint32_t(PERIOD_TICKS) * hundredths + 5000UL) / 10000UL);
}

// Nonblocking forward -> brake -> neutral -> reverse controller. Neutral alone
// does not arm reverse on this ESC, so remember forward even across stop().
class Throttle {
 public:
  enum Phase { DIRECT, BRAKING, NEUTRAL_HOLD };
  Throttle(uint16_t brakeDuty, uint32_t brakeMs, uint32_t neutralMs)
      : brakeDuty_(brakeDuty), brakeMs_(brakeMs), neutralMs_(neutralMs) {}

  void request(uint16_t duty, uint32_t now) {
    if (duty >= THROTTLE_NEUTRAL) {
      if (duty == THROTTLE_NEUTRAL) { stop(now); return; }
      // A forward command supersedes any queued reverse immediately.
      phase_ = DIRECT;
      needsBrake_ = true;
      output_ = target_ = duty;
    } else if (phase_ != DIRECT) {
      target_ = duty;  // Adjust reverse strength without restarting the timer.
    } else if (needsBrake_) {
      target_ = duty;
      output_ = brakeDuty_;
      phase_ = BRAKING;
      started_ = now;
    } else {
      output_ = target_ = duty;
    }
  }

  void stop(uint32_t now) {
    (void)now;
    output_ = target_ = THROTTLE_NEUTRAL;
    // Never resume a canceled reverse. An interrupted brake may not have
    // reached the ESC, so retain needsBrake_. A completed brake's neutral
    // interval can finish in the background with a neutral target.
    if (phase_ == BRAKING) {
      phase_ = DIRECT;
    }
  }

  // Raw duty testing deliberately bypasses the automatic handshake.
  void raw(uint16_t duty) {
    phase_ = DIRECT;
    output_ = target_ = duty;
    if (duty > THROTTLE_NEUTRAL) needsBrake_ = true;
  }

  void update(uint32_t now) {
    if (phase_ == BRAKING && uint32_t(now - started_) >= brakeMs_) {
      output_ = THROTTLE_NEUTRAL;
      phase_ = NEUTRAL_HOLD;
      started_ = now;  // Hold a full neutral interval, even after a late loop.
    } else if (phase_ == NEUTRAL_HOLD &&
               uint32_t(now - started_) >= neutralMs_) {
      needsBrake_ = false;
      phase_ = DIRECT;
      output_ = target_;
    }
  }

  uint16_t duty() const { return output_; }
  Phase phase() const { return phase_; }
  bool transitioning() const { return phase_ != DIRECT; }

 private:
  uint16_t brakeDuty_;
  uint32_t brakeMs_;
  uint32_t neutralMs_;
  uint32_t started_ = 0;
  uint16_t output_ = THROTTLE_NEUTRAL;
  uint16_t target_ = THROTTLE_NEUTRAL;
  bool needsBrake_ = false;
  Phase phase_ = DIRECT;
};

struct Step {
  int8_t throttle;  // -100 reverse .. 0 neutral .. +100 forward
  int8_t steering;  // -100 left .. 0 neutral .. +100 right
  uint32_t durationMs;
  Motion motion;
  Step(int gas, int turn, uint32_t ms)
      : throttle(gas), steering(turn), durationMs(ms), motion{TIMED,0,0,0,0} {}
  Step(Motion action)
      : throttle(action.gas), steering(action.steering), durationMs(0), motion(action) {}
};

// Each step sets BOTH channels and holds them for durationMs.
class Sequence {
 public:
  Sequence(const Step* steps, uint16_t count) : steps_(steps), count_(count) {}

  void start(uint32_t now) {
    index_ = 0;
    started_ = now;
    running_ = count_ != 0;
  }

  void stop() { running_ = false; }
  void restartHold(uint32_t now) { started_ = now; }
  bool running() const { return running_; }
  uint16_t index() const { return index_; }
  const Step& current() const { return steps_[index_]; }

  // Advance at most one step per update, giving every movement its full
  // requested duration even if the loop was briefly delayed. Unsigned
  // subtraction handles the millis() wrap at about 49.7 days.
  bool update(uint32_t now) {
    if (!running_ || uint32_t(now - started_) < current().durationMs) return false;
    if (++index_ == count_) {
      running_ = false;
    } else {
      started_ = now;
    }
    return true;
  }

 private:
  const Step* steps_;
  uint16_t count_;
  uint16_t index_ = 0;
  uint32_t started_ = 0;
  bool running_ = false;
};
}  // namespace rover

#endif
