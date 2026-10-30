#ifndef ROVER_TURN_DISPLAY_H
#define ROVER_TURN_DISPLAY_H
#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "display_format.h"
#ifndef ROVER_DISPLAY_NPN
#define ROVER_DISPLAY_NPN 0
#endif
static_assert(ROVER_DISPLAY_NPN==0 || ROVER_DISPLAY_NPN==1,"Display driver mode must be 0 or 1");
// a,b,c,d,e,f,g, then digit cathodes LEFT to RIGHT.
// Default direct wiring: LOW selects a digit. Legacy NPN: HIGH selects it.
// Change these if required; display pin use is checked against motor pins.
constexpr uint8_t DISPLAY_PINS[11]={2,3,4,5,6,7,12,A0,A1,A2,A3};
constexpr bool displayUses(uint8_t pin, uint8_t i=0) {
  return i==11 ? false : DISPLAY_PINS[i]==pin || displayUses(pin,i+1);
}
constexpr bool displayPinsValid(uint8_t i=0) {
  return i==11 ? true : DISPLAY_PINS[i]>=2 && DISPLAY_PINS[i]<=A3 && displayPinsValid(i+1);
}
constexpr bool displayPinsUnique(uint8_t i=0,uint8_t j=1) {
  return i>=10 ? true : j>=11 ? displayPinsUnique(i+1,i+2) :
         DISPLAY_PINS[i]!=DISPLAY_PINS[j] && displayPinsUnique(i,j+1);
}
static_assert(displayPinsValid() && displayPinsUnique(),"Display needs 11 distinct pins D2..D13 or A0..A3");
class TurnDisplay {
 public:
  void begin() {
    for (uint8_t n=0;n<11;++n) {
      // Disable all digit commons before making segment pins outputs.
      uint8_t i=n<4 ? n+7 : n-4;
      digitalWrite(DISPLAY_PINS[i],i>=7 && !ROVER_DISPLAY_NPN ? HIGH : LOW);
      pinMode(DISPLAY_PINS[i],OUTPUT);
      ports_[i]=portOutputRegister(digitalPinToPort(DISPLAY_PINS[i]));
      masks_[i]=digitalPinToBitMask(DISPLAY_PINS[i]);
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      TCCR2A=0; TCCR2B=0; TIMSK2=0; TCNT2=0;
      OCR2A=249; OCR2B=16; TIFR2=_BV(OCF2A)|_BV(OCF2B);
      TCCR2A=_BV(WGM21);
      TIMSK2=_BV(OCIE2A)|_BV(OCIE2B);
      TCCR2B=_BV(CS22); // 16 MHz / 64 / 250 = 1000 digit scans/sec
    }
  }
  void show(float degrees,bool hasTurn,bool valid) {
    uint8_t next[4]={};
    if (hasTurn) rover::degreeDigits(degrees,valid,next);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (uint8_t i=0;i<4;++i) digits_[i]=next[i];
    }
  }
  // Called at 1 kHz from Timer2. No delays, Wire, Serial, or motor timer use.
  void tick() {
    for (uint8_t i=0;i<4;++i) {
      if (ROVER_DISPLAY_NPN) *ports_[7+i] &= ~masks_[7+i];
      else *ports_[7+i] |= masks_[7+i];
    }
    digit_=(digit_+1)&3;
  }
  // Compare B occurs about 64 us into the next slot. Change a..g only after
  // the all-off interval, then enable one digit. Also permits NPN turn-off.
  void light() {
    uint8_t segments=digits_[digit_];
    for (uint8_t i=0;i<7;++i) {
      if (segments & (1<<i)) *ports_[i] |= masks_[i];
      else *ports_[i] &= ~masks_[i];
    }
    if (segments) {
      if (ROVER_DISPLAY_NPN) *ports_[7+digit_] |= masks_[7+digit_];
      else *ports_[7+digit_] &= ~masks_[7+digit_];
    }
  }
 private:
  volatile uint8_t* ports_[11]={};
  uint8_t masks_[11]={};
  volatile uint8_t digits_[4]={};
  uint8_t digit_=3;
};
#endif
