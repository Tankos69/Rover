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
#define A5 19
#define OUTPUT 1
#define LOW 0
#define F(text) text
#define _BV(bit) (1U << (bit))
static volatile uint8_t ports[3] = {};
static uint8_t digitalPinToPort(uint8_t p) { return p < 8 ? 0 : p < 14 ? 1 : 2; }
static uint8_t digitalPinToBitMask(uint8_t p) { return _BV(p < 8 ? p : p < 14 ? p - 8 : p - 14); }
static volatile uint8_t* portOutputRegister(uint8_t p) { return &ports[p]; }
static void pinMode(uint8_t, uint8_t) {}
static void digitalWrite(uint8_t p, uint8_t v) {
  if (v) ports[digitalPinToPort(p)] |= digitalPinToBitMask(p);
  else ports[digitalPinToPort(p)] &= ~digitalPinToBitMask(p);
}
static uint32_t millis() { return 0; }
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
}
"""

with tempfile.TemporaryDirectory(prefix="rover-output-test-") as directory:
    tmp = Path(directory)
    (tmp / "avr").mkdir()
    (tmp / "util").mkdir()
    (tmp / "Arduino.h").write_text(ARDUINO)
    (tmp / "avr/io.h").write_text('#include "Arduino.h"\n')
    (tmp / "avr/interrupt.h").write_text('#define ISR(vector) void vector()\n')
    (tmp / "util/atomic.h").write_text(
        '#define ATOMIC_RESTORESTATE 0\n'
        '#define ATOMIC_BLOCK(mode) for (bool once = true; once; once = false)\n'
    )
    (tmp / "test.cpp").write_text(HARNESS)
    for gas, turn in [(9, 10), (10, 9), (4, 7), (2, 14), (14, 19), (9, 4)]:
        command = [
            'g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
            '-D__AVR_ATmega328P__', '-DF_CPU=16000000UL',
            f'-DTHROTTLE_PIN={gas}', f'-DSTEERING_PIN={turn}',
            '-I' + str(tmp), '-I' + str(ROOT / 'RoverController'),
            str(tmp / 'test.cpp'), '-o', str(tmp / 'test'),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(tmp / 'test')], check=True)
        print(f'PASS: throttle={gas}, steering={turn}')
    for gas, turn in [(0, 10), (9, 1), (20, 10), (9, 9)]:
        command[command.index(next(x for x in command if x.startswith('-DTHROTTLE_PIN=')))] = f'-DTHROTTLE_PIN={gas}'
        command[command.index(next(x for x in command if x.startswith('-DSTEERING_PIN=')))] = f'-DSTEERING_PIN={turn}'
        result = subprocess.run(command, capture_output=True, text=True)
        assert result.returncode != 0 and 'static assertion failed' in result.stderr
        print(f'PASS: rejected invalid pins {gas}, {turn}')
