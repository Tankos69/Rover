// Standalone DIRECT common-anode display test for Arduino Uno.
// One 2.2 kOhm resistor per a..g segment; NO NPN digit drivers.
// Same signal pin assignment; confirm actual display polarity/pinout.
// Disconnect motor power. See README.md in this folder.
// Serial Monitor: 115200 baud, newline or carriage return. Send e.g. 0042.
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "Select Arduino Uno (ATmega328P, 16 MHz)."
#endif

// Same mapping as the rover controller: a,b,c,d,e,f,g; digits left to right.
const uint8_t SEGMENT_PINS[7]={2,3,4,5,6,7,12};
const uint8_t DIGIT_PINS[4]={A0,A1,A2,A3};
const uint8_t NUMERALS[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
volatile uint8_t shown[4]={0x7f,0x7f,0x7f,0x7f}; // Startup: 8888.
volatile uint8_t* segmentPorts[7];
volatile uint8_t* digitPorts[4];
uint8_t segmentMasks[7],digitMasks[4];
uint8_t scanDigit=3;
char inputLine[16];
uint8_t inputLength=0;
bool lineOverflow=false;

ISR(TIMER2_COMPA_vect) {
  for (uint8_t i=0;i<4;++i) *digitPorts[i] &= ~digitMasks[i];
  scanDigit=(scanDigit+1)&3;
  // Keep all digit anodes LOW until the next slot starts.
}

ISR(TIMER2_COMPB_vect) {
  uint8_t segments=shown[scanDigit];
  for (uint8_t i=0;i<7;++i) {
    if (segments & (1<<i)) *segmentPorts[i] &= ~segmentMasks[i]; // LOW lights segment
    else *segmentPorts[i] |= segmentMasks[i];
  }
  if (segments) *digitPorts[scanDigit] |= digitMasks[scanDigit];
}

void setDigits(const uint8_t next[4]) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i=0;i<4;++i) shown[i]=next[i];
  }
}

void handleLine(const char* text) {
  if (!strcmp(text,"clear")) {
    const uint8_t blank[4]={};
    setDigits(blank);
    Serial.println(F("Display cleared."));
    return;
  }
  if (!strcmp(text,"help")) {
    Serial.println(F("Send 0042, clear, digit 1 6, or segment a (a..g)."));
    return;
  }
  if (strlen(text)==9 && !strncmp(text,"digit ",6) &&
      text[6]>='1' && text[6]<='4' && text[7]==' ' && text[8]>='0' && text[8]<='9') {
    uint8_t next[4]={};
    next[text[6]-'1']=NUMERALS[text[8]-'0'];
    setDigits(next);
    Serial.println(F("Only the requested digit is lit.")); return;
  }
  if (strlen(text)==9 && !strncmp(text,"segment ",8) && text[8]>='a' && text[8]<='g') {
    const uint8_t next[4]={uint8_t(1<<(text[8]-'a')),0,0,0};
    setDigits(next);
    Serial.println(F("Only the requested segment on digit 1 is lit.")); return;
  }
  if (strlen(text)!=4) {
    Serial.println(F("Enter exactly four digits, e.g. 0042. Display unchanged."));
    return;
  }
  uint8_t next[4];
  for (uint8_t i=0;i<4;++i) {
    if (text[i]<'0' || text[i]>'9') {
      Serial.println(F("Digits 0..9 only. Display unchanged."));
      return;
    }
    next[i]=NUMERALS[text[i]-'0'];
  }
  setDigits(next); // ASCII conversion preserves leading zeros.
  Serial.print(F("Showing: ")); Serial.println(text);
}

void consume(char c) {
  if (c=='\r' || c=='\n') {
    if (lineOverflow) Serial.println(F("Line too long; display unchanged."));
    else if (inputLength) {
      inputLine[inputLength]='\0';
      handleLine(inputLine);
    }
    inputLength=0; lineOverflow=false;
  } else if (!lineOverflow) {
    if (inputLength<sizeof(inputLine)-1) inputLine[inputLength++]=c;
    else lineOverflow=true;
  }
}

void setup() {
  for (uint8_t i=0;i<4;++i) {
    digitalWrite(DIGIT_PINS[i],LOW); pinMode(DIGIT_PINS[i],OUTPUT);
    digitPorts[i]=portOutputRegister(digitalPinToPort(DIGIT_PINS[i]));
    digitMasks[i]=digitalPinToBitMask(DIGIT_PINS[i]);
  }
  for (uint8_t i=0;i<7;++i) {
    digitalWrite(SEGMENT_PINS[i],HIGH); pinMode(SEGMENT_PINS[i],OUTPUT);
    segmentPorts[i]=portOutputRegister(digitalPinToPort(SEGMENT_PINS[i]));
    segmentMasks[i]=digitalPinToBitMask(SEGMENT_PINS[i]);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR2A=0; TCCR2B=0; TIMSK2=0; TCNT2=0;
    OCR2A=249; OCR2B=16; TIFR2=_BV(OCF2A)|_BV(OCF2B);
    TCCR2A=_BV(WGM21); TIMSK2=_BV(OCIE2A)|_BV(OCIE2B); TCCR2B=_BV(CS22);
  }
  Serial.begin(115200);
  Serial.println(F("Direct common-anode test: startup 8888. Send four digits, clear, or help."));
}

void loop() {
  // The 1 kHz timer keeps refreshing while Serial Monitor is idle.
  for (uint8_t n=0;n<32 && Serial.available();++n) consume(Serial.read());
}
