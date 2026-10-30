#include <assert.h>
#include <stdio.h>
#include "../RoverController/motion.h"
int main() {
  using namespace rover;
  MotionControl m;
  assert(!m.start(Turn(15,-100,30),0,0,false,false));
  assert(m.start(Turn(15,-100,30),0,350,true,false));
  assert(!m.update(10,360,0,true,false)); // Wrong direction cannot satisfy target.
  assert(!m.update(20,321,0,true,false));
  assert(m.hasTurn && m.turnedDegrees==29);
  assert(m.update(30,320,0,true,false) && !m.active && !m.failed);
  assert(m.turnedDegrees==30); m.stop(); assert(m.turnedDegrees==30);
  assert(m.start(Turn(15,100,30),0,350,true,false));
  assert(m.update(30,380,0,true,false) && !m.active); // Unwrapped crossing north.
  assert(m.start(Turn(-60,100,30),0,0,true,false));
  assert(m.update(30,-30,0,true,false) && !m.active); // Reverse steering sign.
  assert(m.start(Turn(15,100,30,100),0,0,true,false));
  assert(m.update(100,0,0,true,false) && m.failed); // No rotation: timeout.
  assert(m.start(Turn(15,100,30),0,0,true,false));
  assert(m.update(1,0,0,false,false) && m.failed && m.displayFault); // Sensor loss.
  assert(m.start(Turn(15,100,30,100),UINT32_MAX-50,0,true,false));
  assert(m.update(49,0,0,true,false) && m.failed);
  assert(m.start(YTurn(15,-100),0,0,true,false));
  assert(m.turnedDegrees==0 && !m.displayFault);
  assert(m.update(100,-60,0,true,false) && m.gas==-60 && m.steering==100);
  assert(m.update(200,-120,0,true,false) && m.gas==15 && m.steering==-100);
  assert(m.update(300,-180,0,true,false) && !m.active);
  assert(!m.start(Accelerate(30,.5f),0,0,true,false)); // Explicit opt-in.
  assert(m.start(Accelerate(30,.5f),0,0,true,true));
  assert(!m.update(100,0,.49f,true,false));
  assert(m.update(200,0,.5f,true,false) && !m.active);
  assert(m.start(Cruise(.5f,40,1000),0,0,true,true));
  m.update(100,0,0,true,false);
  assert(m.gas>=0 && m.gas<=40);
  m.update(200,0,2,true,false);
  assert(m.gas==0);
  assert(m.update(1000,0,.5f,true,false) && !m.active);
  assert(!m.start(Turn(0,100,30),0,0,true,false));
  assert(!m.start(Turn(15,0,30),0,0,true,false));
  assert(!m.start(Turn(15,100,-30),0,0,true,false));
  puts("PASS: directed angle targets, wraparound, sensor loss, timeout, Y turn, speed opt-in and bounds");
}
