#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include <stdint.h>

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

struct Step {
  int8_t throttle;  // -100 reverse .. 0 neutral .. +100 forward
  int8_t steering;  // -100 left .. 0 neutral .. +100 right
  uint32_t durationMs;
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
