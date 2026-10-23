#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../RoverController/rover_control.h"

int main() {
  using namespace rover;
  assert(throttleDuty(0) == 946);
  assert(throttleDuty(100) == 1219);
  assert(throttleDuty(-100) == 826);
  assert(throttleDuty(15) == 987);
  assert(throttleDuty(-15) == 928);
  assert(steeringDuty(0) == 961);
  assert(steeringDuty(100) == 637);
  assert(steeringDuty(-100) == 1311);
  assert(steeringDuty(50) == 799);
  assert(steeringDuty(-50) == 1136);
  assert(throttleDuty(150) == 1219);
  assert(throttleDuty(-150) == 826);
  for (int p = -100; p < 100; ++p) {
    assert(throttleDuty(p) <= throttleDuty(p + 1));
    assert(steeringDuty(p) >= steeringDuty(p + 1));
  }
  assert(dutyTicks(0) == 0);
  assert(dutyTicks(10000) == PERIOD_TICKS);
  assert(dutyTicks(946) == 3019);
  const Step steps[] = {{15, 0, 5000}, {15, 50, 500}, {0, 0, 2000}};
  Sequence s(steps, 3);
  s.start(100);
  assert(s.running() && s.index() == 0);
  assert(!s.update(5099));
  assert(s.update(5100) && s.index() == 1 && s.current().steering == 50);
  assert(!s.update(5599));
  assert(s.update(5600) && s.index() == 2 && s.current().throttle == 0);
  assert(s.update(7600) && !s.running());
  assert(!s.update(9000));
  s.start(UINT32_MAX - 1000);
  assert(!s.update(3998));
  assert(s.update(3999) && s.index() == 1);
  s.stop();
  assert(!s.running() && !s.update(10000));
  s.start(0);
  assert(s.index() == 0 && s.running());
  assert(s.update(9000) && s.index() == 1);
  assert(!s.update(9499));  // Delays never skip a movement.
  assert(s.update(9500) && s.index() == 2);
  Sequence empty(steps, 0);
  empty.start(0);
  assert(!empty.running() && !empty.update(1000));
  puts("PASS: calibration, monotonic mapping, PWM conversion, sequence timing, stop/restart, clock rollover");
}
