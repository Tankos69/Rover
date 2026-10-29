"""Exercise the sketch's real output functions with simulated Uno registers.

This checks dispatch and edge scheduling, not AVR ISR latency or real waveforms.
Run with python3 tests/output_test.py (requires g++).
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
ARDUINO = r"""
#pragma once
#include <stdint.h>
#define A0 14
#define A4 18
#define A5 19
#define OUTPUT 1
#define LOW 0
#define F(text) text
#define _BV(bit) (1U << (bit))
static volatile uint8_t ports[3] = {};
static uint8_t digitalPinToPort(uint8_t p) { return p < 8 ? 0 : p < 14 ? 1 : 2; }
static uint8_t digitalPinToBitMask(uint8_t p) { return _BV(p < 8 ? p : p < 14 ? p - 8 : p - 14); }
static volatile uint8_t* portOutputRegister(uint8_t p) { return &ports[p]; }
static void delay(unsigned long) {}
static void pinMode(uint8_t, uint8_t) {}
static void digitalWrite(uint8_t p, uint8_t v) {
  if (v) ports[digitalPinToPort(p)] |= digitalPinToBitMask(p);
  else ports[digitalPinToPort(p)] &= ~digitalPinToBitMask(p);
}
static uint32_t fakeMillis = 0;
static uint32_t millis() { return fakeMillis; }
static volatile uint16_t OCR1A, OCR1B, ICR1, TCNT1;
static volatile uint8_t TCCR1A, TCCR1B, TIMSK1, TIFR1;
#define PORTB ports[1]
enum {PB1=1, PB2=2, COM1A1=7, COM1B1=5, WGM11=1, WGM12=3,
      WGM13=4, CS11=1, OCIE1A=1, OCIE1B=2, OCF1A=1, OCF1B=2, TOV1=0};
struct FakeSerial {
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  template<class T> void print(T) {}
  template<class T> void print(T, int) {}
  void println() {}
  template<class T> void println(T) {}
};
static FakeSerial Serial;
"""
HARNESS = r"""
#include <assert.h>
#include "RoverController.ino"
static bool high(uint8_t pin) {
  return ports[digitalPinToPort(pin)] & digitalPinToBitMask(pin);
}
static void send(const char* text) {
  char line[80];
  strcpy(line, text);
  command(line);
}
int main() {
  beginSignals();
  gasDuty = 946;
  turnDuty = 961;
  writeOutputs();
  if (HARDWARE_PWM) {
    assert(TCCR1A & _BV(COM1A1));
    assert(TCCR1A & _BV(COM1B1));
    assert(ICR1 == rover::PERIOD_TICKS - 1);
    assert(OCR1A == rover::dutyTicks(THROTTLE_PIN == 9 ? gasDuty : turnDuty));
    assert(OCR1B == rover::dutyTicks(THROTTLE_PIN == 9 ? turnDuty : gasDuty));
    gasDuty = 0;
    turnDuty = 10000;
    writeOutputs();
    assert(!high(THROTTLE_PIN) && high(STEERING_PIN));
    assert(!(TCCR1A & (_BV(COM1A1) | _BV(COM1B1))));
  } else {
    assert(OCR1A == rover::PERIOD_TICKS - 1);
    TCNT1 = 0;
    TIMER1_COMPA_vect();
    assert(high(THROTTLE_PIN) && high(STEERING_PIN));
    assert(OCR1B == rover::dutyTicks(946));
    TCNT1 = OCR1B;
    TIMER1_COMPB_vect();
    assert(!high(THROTTLE_PIN) && high(STEERING_PIN));
    assert(OCR1B == rover::dutyTicks(961));
    TCNT1 = OCR1B;
    TIMER1_COMPB_vect();
    assert(!high(THROTTLE_PIN) && !high(STEERING_PIN));
    assert(!(TIMSK1 & _BV(OCIE1B)));

    // Queue a new setting during a pulse: it must wait until the next frame.
    TCNT1 = 0;
    TIMER1_COMPA_vect();
    gasDuty = 0;
    turnDuty = 10000;
    writeOutputs();
    assert(high(THROTTLE_PIN) && high(STEERING_PIN));
    assert(activeGasTicks == rover::dutyTicks(946));
    TCNT1 = 0;
    TIMER1_COMPA_vect();
    assert(!high(THROTTLE_PIN) && high(STEERING_PIN));
    assert(!(TIMSK1 & _BV(OCIE1B)));

    // Identical edges, and an ISR delayed beyond BOTH close edges, must clear
    // both outputs, not leave one high until the next frame.
    for (uint16_t delta = 0; delta < 2; ++delta) {
      gasDuty = 946;
      turnDuty = 946 + delta;
      writeOutputs();
      TCNT1 = 0;
      TIMER1_COMPA_vect();
      TCNT1 = rover::dutyTicks(turnDuty) + 20;
      TIMER1_COMPB_vect();
      assert(!high(THROTTLE_PIN) && !high(STEERING_PIN));
      assert(!(TIMSK1 & _BV(OCIE1B)));
    }
  }
  // Exercise actual serial command dispatch and nonblocking loop, including
  // output synchronization, steering during braking, and immediate STOP.
  throttle.raw(rover::THROTTLE_NEUTRAL);
  fakeMillis = 0;
  send("gas 14");
  send("gas -60");
  assert(throttle.phase() == rover::Throttle::BRAKING);
  fakeMillis = 500;
  loop();
  assert(gasDuty == rover::THROTTLE_NEUTRAL);
  send("turn 50");
  fakeMillis = 1500;
  loop();
  assert(gasDuty == rover::throttleDuty(-60));
  assert(turnDuty == rover::steeringDuty(50));
  send("gas 14");
  send("gas -60");
  send("stop");
  fakeMillis += 2000;
  loop();
  assert(gasDuty == rover::THROTTLE_NEUTRAL);
  assert(turnDuty == rover::STEERING_NEUTRAL);
  send("duty gas 8.26");
  assert(!throttle.transitioning() && gasDuty == 826);

  // Run the included route through its reverse step. The handshake adds
  // 1500 ms; all 4000 ms of the reverse movement must then be preserved.
  fakeMillis = 10000;
  send("run");
  fakeMillis = 15000; loop();
  fakeMillis = 15500; loop();
  fakeMillis = 16500; loop();
  fakeMillis = 18500; loop();
  assert(sequence.index() == 4 && throttle.transitioning());
  fakeMillis = 19000; loop();
  assert(gasDuty == rover::THROTTLE_NEUTRAL);
  fakeMillis = 20000; loop();
  assert(gasDuty == rover::throttleDuty(-15));
  fakeMillis = 23999; loop();
  assert(sequence.index() == 4);
  fakeMillis = 24000; loop();
  assert(sequence.index() == 5 && gasDuty == rover::THROTTLE_NEUTRAL);
  // Sensor driver: calibrate known stationary bias, integrate fresh gyro data,
  // reject a sample gap, and recover via a new calibration.
  fakeMillis=0; Wire.present=true;
  Wire.bytes[12]=0; Wire.bytes[13]=131; // +1 deg/s stationary gyro bias.
  imu.begin();
  for (int i=1;i<=200;++i) { fakeMillis=i*10; imu.update(fakeMillis); }
  assert(imu.ready && !imu.calibrating && !imu.fault);
  assert(fabs(imu.yaw)<.001f && fabs(imu.speed)<.001f);
  Wire.bytes[12]=0x05; Wire.bytes[13]=0x1E; // 1310 raw = +10 deg/s.
  for (int i=201;i<=300;++i) { fakeMillis=i*10; imu.update(fakeMillis); }
  assert(fabs(imu.yaw+9)<.01f); // Bias removed; left is negative.
  fakeMillis+=60; imu.update(fakeMillis);
  assert(imu.fault && !imu.ready && imu.error==Mpu6500::SAMPLE_GAP && imu.sampleGapMs==60);
  Wire.bytes[12]=Wire.bytes[13]=0;
  imu.begin();
  for (int i=1;i<=200;++i) { fakeMillis+=10; imu.update(fakeMillis); }
  assert(imu.ready && imu.error==Mpu6500::NONE);
  if (ROVER_ENABLE_IMU) {
    // Exercise serial dispatch and completion with a calibrated sensor.
    send("gas 20");
    send("angle 0 -100 30");
    assert(gasDuty==rover::throttleDuty(20));
    send("stop");
    send("angle 15 -100 30");
    assert(motion.active);
    imu.yaw=-30;
    loop();
    assert(!motion.active && gasDuty==rover::THROTTLE_NEUTRAL);
    send("yturn 15 -100");
    assert(motion.active);
    send("stop");
    assert(!motion.active && gasDuty==rover::THROTTLE_NEUTRAL);
    send("speed 30 0.5"); // Experimental mode is off by default.
    assert(!motion.active && gasDuty==rover::THROTTLE_NEUTRAL);
    const rover::Step sensorRoute[]={rover::Turn(15,-100,30),{20,0,1000}};
    sequence=rover::Sequence(sensorRoute,2);
    imu.yaw=0;
    send("run");
    assert(sequence.running() && motion.active && sequence.index()==0);
    imu.yaw=-29; loop();
    assert(sequence.index()==0);
    imu.yaw=-30; loop();
    assert(sequence.index()==1 && !motion.active && gasDuty==rover::throttleDuty(20));
    send("stop");
    send("angle 15 -100 30");
    assert(motion.active);
  }
  Wire.present=false;
  fakeMillis+=10; imu.update(fakeMillis);
  assert(imu.fault && !imu.ready && imu.error==Mpu6500::STATUS_READ);
  if (ROVER_ENABLE_IMU) {
    loop();
    assert(!motion.active && gasDuty==rover::THROTTLE_NEUTRAL);
  }

  imu.begin();
  assert(imu.error==Mpu6500::NO_DEVICE);
  Wire.present=true;
  imu.begin();
  Wire.bytes[12]=0x10; Wire.bytes[13]=0; // Movement during calibration.
  fakeMillis+=10; imu.update(fakeMillis);
  assert(imu.error==Mpu6500::CAL_MOVING);
  assert(imu.gyro[2]>5 && imu.accel[2]==1);
  Wire.bytes[12]=0; // Live data changes must not overwrite the failed sample.
  fakeMillis+=10; imu.update(fakeMillis);
  assert(imu.gyro[2]>5);
  send("status");
  imu.begin();
  Wire.bytes[4]=Wire.bytes[5]=Wire.bytes[13]=0; // No gravity: accel check fails.
  fakeMillis+=10; imu.update(fakeMillis);
  assert(imu.error==Mpu6500::CAL_MOVING);
  assert(imu.accel[2]==0 && imu.gyro[2]==0);
  send("status");

}
"""

with tempfile.TemporaryDirectory(prefix="rover-output-test-") as directory:
    tmp = Path(directory)
    (tmp / "avr").mkdir()
    (tmp / "util").mkdir()
    (tmp / "Arduino.h").write_text(ARDUINO)
    (tmp / "Wire.h").write_text(r"""
#pragma once
#define WIRE_HAS_TIMEOUT 1
struct FakeWire {
 bool present=false;
 uint8_t bytes[14]={0,0,0,0,0x40,0,0,0,0,0,0,0,0,0};
 uint8_t reg=0, written=0, offset=0, address=0;
 void begin() {} void setClock(long) {} void setWireTimeout(long,bool) {}
 void beginTransmission(uint8_t a) {address=a; written=0;}
 void write(uint8_t b) {if (!written++) reg=b;}
 uint8_t endTransmission(bool=true) {return present && address==0x68 ? 0 : 4;}
 uint8_t requestFrom(uint8_t,uint8_t n,uint8_t) {offset=0; return present ? n : 0;}
 int read() {
   if (reg==0x75) return 0x70;
   if (reg==0x3A) return 1;
   return bytes[offset++ % 14];
 }
};
static FakeWire Wire;
""")
    (tmp / "avr/io.h").write_text('#include "Arduino.h"\n')
    (tmp / "avr/interrupt.h").write_text('#define ISR(vector) void vector()\n')
    (tmp / "util/atomic.h").write_text(
        '#define ATOMIC_RESTORESTATE 0\n'
        '#define ATOMIC_BLOCK(mode) for (bool once = true; once; once = false)\n'
    )
    (tmp / "test.cpp").write_text(HARNESS)
    for gas, turn in [(9, 10), (10, 9), (4, 7), (2, 14), (14, 19), (9, 4), (7, 8)]:
        command = [
            'g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
            f'-DROVER_ENABLE_IMU={1 if gas==7 else 0}', '-D__AVR_ATmega328P__', '-DF_CPU=16000000UL',
            f'-DTHROTTLE_PIN={gas}', f'-DSTEERING_PIN={turn}',
            '-I' + str(tmp), '-I' + str(ROOT / 'RoverController'),
            str(tmp / 'test.cpp'), '-o', str(tmp / 'test'),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(tmp / 'test')], check=True)
        print(f'PASS: throttle={gas}, steering={turn}')
    for gas, turn in [(0, 10), (9, 1), (20, 10), (9, 9), (18, 10)]:
        command[command.index(next(x for x in command if x.startswith('-DTHROTTLE_PIN=')))] = f'-DTHROTTLE_PIN={gas}'
        command[command.index(next(x for x in command if x.startswith('-DSTEERING_PIN=')))] = f'-DSTEERING_PIN={turn}'
        result = subprocess.run(command, capture_output=True, text=True)
        assert result.returncode != 0 and 'static assertion failed' in result.stderr
        print(f'PASS: rejected invalid pins {gas}, {turn}')
