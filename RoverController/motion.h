#ifndef ROVER_MOTION_H
#define ROVER_MOTION_H
#include <math.h>
#include <stdint.h>
namespace rover {
// All heading values are unwrapped degrees, positive to the rover's right.
enum MotionKind { TIMED, ANGLE, ACCELERATE, CRUISE, Y_TURN };
struct Motion {
  MotionKind kind;
  int16_t gas, steering;
  float target;
  uint32_t limitMs;
  int16_t reverseGas;
  Motion(MotionKind k, int g, int s, float t, uint32_t ms, int reverse=-60)
      : kind(k), gas(g), steering(s), target(t), limitMs(ms), reverseGas(reverse) {}
};
inline Motion Turn(int gas, int steering, float degrees, uint32_t timeout=15000) {
  return {ANGLE, gas, steering, degrees, timeout};
}
inline Motion Accelerate(int gas, float metresPerSecond, uint32_t timeout=10000) {
  return {ACCELERATE, gas, 0, metresPerSecond, timeout};
}
inline Motion Cruise(float metresPerSecond, int maxGas, uint32_t milliseconds) {
  return {CRUISE, maxGas, 0, metresPerSecond, milliseconds};
}
inline Motion YTurn(int forwardGas, int steering, int reverseGas=-60, uint32_t timeout=30000) {
  return {Y_TURN, forwardGas, steering, 180, timeout, reverseGas};
}
class MotionControl {
 public:
  bool active=false, failed=false;
  int gas=0, steering=0;
  bool hasTurn=false, displayFault=false;
  float turnedDegrees=0;
  void stop() { active=false; gas=steering=0; }
  bool start(Motion m, uint32_t now, float yaw, bool ready, bool speedEnabled) {
    stop(); failed=false;
    if (!ready || m.gas < -100 || m.gas > 100 || m.steering < -100 ||
        m.steering > 100 || !isfinite(m.target) || m.target<=0 || !m.limitMs ||
        ((m.kind==ANGLE || m.kind==Y_TURN) && (!m.gas || !m.steering)) ||
        (m.kind==Y_TURN && (m.gas<=0 || m.reverseGas>=0 || m.reverseGas< -100)) ||
        ((m.kind==ACCELERATE || m.kind==CRUISE) && (!speedEnabled || m.gas<=0 || m.target>3))) {
      failed=true; return false;
    }
    motion_=m; started_=last_=now; origin_=yaw; phase_=0; integral_=0;
    gas=m.gas; steering=m.steering;
    sign_=(m.steering>0 ? 1 : -1)*(m.gas>0 ? 1 : -1);
    if (m.kind==ANGLE || m.kind==Y_TURN) {
      hasTurn=true; displayFault=false; turnedDegrees=0;
    }
    active=true; return true;
  }
  // Return true when outputs change or the motion finishes/fails.
  bool update(uint32_t now, float yaw, float speed, bool fresh, bool shifting) {
    if (!active) return false;
    if (!fresh || !isfinite(yaw) || !isfinite(speed)) return fault();
    if (motion_.kind==ANGLE || motion_.kind==Y_TURN)
      turnedDegrees=(yaw-origin_)*sign_;
    if (motion_.kind==CRUISE) {
      if (uint32_t(now-started_)>=motion_.limitMs) { stop(); return true; }
      if (uint32_t(now-last_)<50 || shifting) return false;
      float dt=uint32_t(now-last_)*.001f; last_=now;
      float error=motion_.target-speed;
      // Bounded, experimental PI throttle. No automatic reverse to slow down.
      integral_+=error*dt*8;
      if (integral_<0) integral_=0;
      if (integral_>motion_.gas) integral_=motion_.gas;
      int next=int(error*25+integral_);
      if (next<0) next=0;
      if (next>motion_.gas) next=motion_.gas;
      bool changed=gas!=next; gas=next; return changed;
    }
    if (uint32_t(now-started_)>=motion_.limitMs) return fault();
    if (motion_.kind==ACCELERATE) {
      if (speed>=motion_.target) { stop(); return true; }
      return false;
    }
    float progress=(yaw-origin_)*sign_;
    float goal=motion_.kind==Y_TURN ? (phase_+1)*60.0f : motion_.target;
    if (progress<goal) return false;
    if (motion_.kind==Y_TURN && phase_<2) {
      ++phase_;
      gas=phase_==1 ? motion_.reverseGas : motion_.gas;
      steering=phase_==1 ? -motion_.steering : motion_.steering;
    } else stop();
    return true;
  }
 private:
  Motion motion_={TIMED,0,0,0,0};
  uint32_t started_=0,last_=0;
  float origin_=0,integral_=0;
  int sign_=1;
  uint8_t phase_=0;
  bool fault() {
    if (motion_.kind==ANGLE || motion_.kind==Y_TURN) displayFault=true;
    stop(); failed=true; return true;
  }
};
}
#endif
