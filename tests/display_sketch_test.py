"""Host test of the standalone sketch's serial parsing and GPIO scan routing."""
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
# Reuse the Uno register/Serial stubs without running the rover test suite.
tree = ast.parse((root / 'tests/output_test.py').read_text())
arduino = next(ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign) and any(
                   isinstance(t, ast.Name) and t.id == 'ARDUINO' for t in n.targets))
harness = r'''
#include <assert.h>
#include "DisplayTest.ino"
void send(const char* line) {while (*line) consume(*line++);}
bool high(uint8_t pin) {return ports[digitalPinToPort(pin)] & digitalPinToBitMask(pin);}
int main() {
  setup();
  for (int i=0;i<4;++i) assert(shown[i]==0x7f);
  send("0042\r\n");
  assert(shown[0]==NUMERALS[0] && shown[1]==NUMERALS[0]);
  assert(shown[2]==NUMERALS[4] && shown[3]==NUMERALS[2]);
  send("123\n"); send("12x4\n");
  assert(shown[2]==NUMERALS[4] && shown[3]==NUMERALS[2]);
  send("123456789012345678901234567890\n");
  assert(shown[0]==NUMERALS[0] && shown[3]==NUMERALS[2]);
  send("9876\n");
  for (int i=0;i<4;++i) assert(shown[i]==NUMERALS[9-i]);
  send("clear\n");
  for (int i=0;i<4;++i) assert(shown[i]==0);
  send("help\n");
  for (int i=0;i<4;++i) assert(shown[i]==0);
  send("1234\n");
  assert(OCR2A==249 && OCR2B==16 && (TIMSK2 & _BV(OCIE2A)) && (TIMSK2 & _BV(OCIE2B)));
  for (int digit=0;digit<4;++digit) {
    TIMER2_COMPA_vect();
    for (int i=0;i<4;++i) assert(!high(DIGIT_PINS[i]));
    TIMER2_COMPB_vect();
    for (int i=0;i<4;++i) assert(high(DIGIT_PINS[i])==(i==digit));
    for (int i=0;i<7;++i) assert(high(SEGMENT_PINS[i])==bool(NUMERALS[digit+1] & (1<<i)));
  }
  send("digit 1 6\n");
  assert(shown[0]==NUMERALS[6] && shown[1]==0 && shown[2]==0 && shown[3]==0);
  send("digit 4 2\n");
  assert(shown[0]==0 && shown[1]==0 && shown[2]==0 && shown[3]==NUMERALS[2]);
  send("digit 5 2\n"); // Invalid: retains last pattern.
  assert(shown[3]==NUMERALS[2]);
  send("segment a\n"); assert(shown[0]==1 && shown[3]==0);
  send("segment g\n"); assert(shown[0]==0x40 && shown[3]==0);
  send("0000\n");
  for (int i=0;i<4;++i) assert(shown[i]==NUMERALS[0]);
}
'''
with tempfile.TemporaryDirectory(prefix='display-sketch-test-') as directory:
    tmp = Path(directory)
    (tmp / 'avr').mkdir()
    (tmp / 'util').mkdir()
    (tmp / 'Arduino.h').write_text(arduino.replace('static void delay', 'static inline void delay')
                                            .replace('static uint32_t millis', 'static inline uint32_t millis'))
    (tmp / 'avr/interrupt.h').write_text('#define ISR(vector) void vector()\n')
    (tmp / 'util/atomic.h').write_text(
        '#define ATOMIC_RESTORESTATE 0\n'
        '#define ATOMIC_BLOCK(mode) for(bool once=true;once;once=false)\n')
    (tmp / 'test.cpp').write_text(harness)
    subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-D__AVR_ATmega328P__', '-DF_CPU=16000000UL',
                    '-I' + str(tmp), '-I' + str(root / 'DisplayTest'),
                    str(tmp / 'test.cpp'), '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
print('PASS: standalone display parsing, leading zeros, invalid input, overflow recovery, clear, isolated diagnostics and two-phase digit routing')
