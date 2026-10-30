"""Test direct-cathode polarity and the limited-current pattern guard."""
import ast
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/output_test.py').read_text())
arduino=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id=='ARDUINO' for t in n.targets))
harness=r'''
#include <assert.h>
#include "DirectCathodeDisplayTest.ino"
void send(const char* text) {while (*text) consume(*text++);}
bool high(uint8_t p) {return ports[digitalPinToPort(p)] & digitalPinToBitMask(p);}
int main() {
 setup();
 for(int i=0;i<4;++i) {assert(shown[i]==0);assert(high(DIGIT_PINS[i]));}
 send("1471\r\n");
 const int values[4]={1,4,7,1};
 for(int i=0;i<4;++i) assert(shown[i]==NUMERALS[values[i]]);
 for(int digit=0;digit<4;++digit) {
   TIMER2_COMPA_vect();
   for(int i=0;i<4;++i) assert(high(DIGIT_PINS[i]));
   TIMER2_COMPB_vect();
   for(int i=0;i<4;++i) assert(high(DIGIT_PINS[i])==(i!=digit));
   for(int i=0;i<7;++i) assert(high(SEGMENT_PINS[i])==bool(NUMERALS[values[digit]] & (1<<i)));
 }
 send("1418\n");
 if(DISPLAY_MAX_SEGMENTS==4) {
   for(int i=0;i<4;++i) assert(shown[i]==NUMERALS[values[i]]);
   send("digit 1 8\n");
   for(int i=0;i<4;++i) assert(shown[i]==NUMERALS[values[i]]);
 } else {
   assert(shown[3]==NUMERALS[8]);
   send("0042\n");
   assert(shown[0]==NUMERALS[0] && shown[3]==NUMERALS[2]);
 }
 send("digit 4 7\n");
 assert(shown[0]==0 && shown[1]==0 && shown[2]==0 && shown[3]==NUMERALS[7]);
 send("segment g\n"); assert(shown[0]==0x40 && shown[3]==0);
 send("clear\n");
 for(int i=0;i<4;++i) assert(shown[i]==0);
 TIMER2_COMPA_vect(); TIMER2_COMPB_vect();
 for(int i=0;i<4;++i) assert(high(DIGIT_PINS[i]));
 for(int i=0;i<7;++i) assert(!high(SEGMENT_PINS[i]));
 send("123456789012345678901234\n1471\n");
 for(int i=0;i<4;++i) assert(shown[i]==NUMERALS[values[i]]);
}
'''
with tempfile.TemporaryDirectory(prefix='direct-cathode-test-') as directory:
    tmp=Path(directory)
    (tmp/'avr').mkdir(); (tmp/'util').mkdir()
    (tmp/'Arduino.h').write_text(arduino.replace('static void delay','static inline void delay')
                               .replace('static uint32_t millis','static inline uint32_t millis'))
    (tmp/'avr/interrupt.h').write_text('#define ISR(vector) void vector()\n')
    (tmp/'util/atomic.h').write_text('#define ATOMIC_RESTORESTATE 0\n#define ATOMIC_BLOCK(mode) for(bool once=true;once;once=false)\n')
    (tmp/'test.cpp').write_text(harness)
    for limit in [4,7]:
        subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror',
                        '-D__AVR_ATmega328P__','-DF_CPU=16000000UL',f'-DDISPLAY_MAX_SEGMENTS={limit}',
                        '-I'+str(tmp),'-I'+str(root/'DirectCathodeDisplayTest'),
                        str(tmp/'test.cpp'),'-o',str(tmp/'test')],check=True)
        subprocess.run([str(tmp/'test')],check=True)
        print(f'PASS: direct-cathode scan, blanking, commands, atomic pattern rejection; limit={limit}')
