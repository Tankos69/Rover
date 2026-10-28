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
  Throttle t(throttleDuty(-60), 500, 1000);
  t.request(throttleDuty(-60), 0);  // Cold start reverse has no forward history.
  assert(!t.transitioning() && t.duty() == throttleDuty(-60));
  t.request(throttleDuty(14), 10);
  t.request(throttleDuty(-60), 20);
  assert(t.phase() == Throttle::BRAKING);
  t.update(519);
  assert(t.phase() == Throttle::BRAKING);
  t.update(520);
  assert(t.phase() == Throttle::NEUTRAL_HOLD && t.duty() == THROTTLE_NEUTRAL);
  t.request(throttleDuty(-30), 600);  // Change target without restarting hold.
  t.update(1519);
  assert(t.duty() == THROTTLE_NEUTRAL);
  t.update(1520);
  assert(!t.transitioning() && t.duty() == throttleDuty(-30));
  t.request(throttleDuty(-70), 1600);
  assert(!t.transitioning() && t.duty() == throttleDuty(-70));

  t.request(throttleDuty(14), 2000);
  t.stop(2100);
  t.request(throttleDuty(-60), 2200);  // Neutral alone does not arm reverse.
  assert(t.phase() == Throttle::BRAKING);
  t.stop(2300);  // Cancel queued reverse during braking.
  assert(t.duty() == THROTTLE_NEUTRAL);
  t.update(3300);
  assert(!t.transitioning() && t.duty() == THROTTLE_NEUTRAL);
  t.request(throttleDuty(-60), 3400);
  assert(t.phase() == Throttle::BRAKING);  // Interrupted brake must be retried.

  t.request(throttleDuty(14), 4000);
  t.request(throttleDuty(-60), 4100);
  t.update(4600);
  t.stop(4700);  // Cancel queued reverse during neutral hold too.
  t.update(5600);
  assert(!t.transitioning() && t.duty() == THROTTLE_NEUTRAL);

  t.request(throttleDuty(14), 6000);
  t.request(throttleDuty(-60), 6100);
  t.request(throttleDuty(30), 6200);  // New forward overrides queued reverse.
  t.update(9000);
  assert(!t.transitioning() && t.duty() == throttleDuty(30));
  t.request(throttleDuty(-60), UINT32_MAX - 100);
  t.update(399);  // Wrap-safe brake duration.
  assert(t.phase() == Throttle::NEUTRAL_HOLD);
  t.update(1399);
  assert(!t.transitioning() && t.duty() == throttleDuty(-60));
  t.request(throttleDuty(14), 1500);
  t.request(throttleDuty(-60), 1600);
  t.update(10000);  // A delayed loop must still output a full neutral hold.
  assert(t.phase() == Throttle::NEUTRAL_HOLD);
  t.update(10999);
  assert(t.duty() == THROTTLE_NEUTRAL);
  t.update(11000);
  assert(t.duty() == throttleDuty(-60));
  t.raw(850);
  assert(!t.transitioning() && t.duty() == 850);
  puts("PASS: reverse handshake, cancellation, target changes, raw bypass, clock rollover");
  puts("PASS: calibration, monotonic mapping, PWM conversion, sequence timing, stop/restart, clock rollover");
}
