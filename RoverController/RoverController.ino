#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rover_control.h"

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "Select Arduino Uno (ATmega328P, 16 MHz)."
#endif

// Choose two distinct pins: D2..D13 or A0..A5. Edit these and upload again.
// D0/D1 are reserved for serial. D9/D10 (in either order) use hardware PWM;
// other pairs use Timer1 interrupts and have some interrupt-related edge jitter.
#ifndef THROTTLE_PIN
#define THROTTLE_PIN 9
#endif
#ifndef STEERING_PIN
#define STEERING_PIN 10
#endif
static_assert(THROTTLE_PIN >= 2 && THROTTLE_PIN <= A5,
              "Throttle pin must be D2..D13 or A0..A5");
static_assert(STEERING_PIN >= 2 && STEERING_PIN <= A5,
              "Steering pin must be D2..D13 or A0..A5");
static_assert(THROTTLE_PIN != STEERING_PIN, "Output pins must be different");
constexpr bool HARDWARE_PWM = (THROTTLE_PIN == 9 && STEERING_PIN == 10) ||
                              (THROTTLE_PIN == 10 && STEERING_PIN == 9);

// Set true and upload again to run PROGRAM once on each power-up/reset.
constexpr bool AUTO_RUN_ON_BOOT = false;
constexpr uint32_t STARTUP_NEUTRAL_MS = 5000;

// Edit this table and upload to change the standalone route.
// {throttle %, steering %, hold time in milliseconds}
// Each row replaces both settings. Repeat a setting to keep it active.
const rover::Step PROGRAM[] = {
  { 15,  0, 5000},  // Forward 15% for 5 seconds
  { 15, 50,  500},  // Keep driving, turn right 50% for 0.5 seconds
  { 15,  0, 1000},  // Keep driving, straighten for 1 second
  {  0,  0, 2000},  // Stop for 2 seconds
  {-15,  0, 4000},  // Reverse 15% for 4 seconds
  {  0,  0, 1000},  // Stop; completion also holds neutral indefinitely
};

rover::Sequence sequence(PROGRAM, sizeof(PROGRAM) / sizeof(PROGRAM[0]));
uint16_t gasDuty = rover::THROTTLE_NEUTRAL;
uint16_t turnDuty = rover::STEERING_NEUTRAL;
bool autoPending = AUTO_RUN_ON_BOOT;
uint32_t bootTime;
char inputLine[80];
uint8_t inputLength = 0;
bool inputOverflow = false;

// For arbitrary pins, the main loop queues pulse widths and the frame ISR
// snapshots them together. Register pointers avoid slow digitalWrite() in ISRs.
volatile uint16_t requestedGasTicks;
volatile uint16_t requestedTurnTicks;
volatile uint8_t* gasPort;
volatile uint8_t* turnPort;
uint8_t gasMask;
uint8_t turnMask;
uint16_t activeGasTicks;
uint16_t activeTurnTicks;
bool gasEdgePending;
bool turnEdgePending;

// Called only with interrupts disabled. Service edges that are already due
// immediately, including coincident edges and those closer than ISR latency.
// This avoids scheduling a compare in the past and missing a falling edge.
void scheduleFallingEdges() {
  while (gasEdgePending || turnEdgePending) {
    const uint16_t gasEdge = gasEdgePending ? activeGasTicks : rover::PERIOD_TICKS;
    const uint16_t turnEdge = turnEdgePending ? activeTurnTicks : rover::PERIOD_TICKS;
    const uint16_t nextEdge = gasEdge < turnEdge ? gasEdge : turnEdge;
    OCR1B = nextEdge;
    TIFR1 = _BV(OCF1B);
    const uint16_t now = TCNT1;
    if (now < nextEdge) {
      TIMSK1 |= _BV(OCIE1B);
      return;
    }
    if (gasEdgePending && activeGasTicks <= now) {
      *gasPort &= ~gasMask;
      gasEdgePending = false;
    }
    if (turnEdgePending && activeTurnTicks <= now) {
      *turnPort &= ~turnMask;
      turnEdgePending = false;
    }
  }
  TIMSK1 &= ~_BV(OCIE1B);
}

ISR(TIMER1_COMPA_vect) {
  // In the arbitrary-pin backend, compare A resets the counter each frame.
  activeGasTicks = requestedGasTicks;
  activeTurnTicks = requestedTurnTicks;
  if (activeGasTicks) *gasPort |= gasMask;
  else *gasPort &= ~gasMask;
  if (activeTurnTicks) *turnPort |= turnMask;
  else *turnPort &= ~turnMask;
  gasEdgePending = activeGasTicks > 0 && activeGasTicks < rover::PERIOD_TICKS;
  turnEdgePending = activeTurnTicks > 0 && activeTurnTicks < rover::PERIOD_TICKS;
  scheduleFallingEdges();
}

ISR(TIMER1_COMPB_vect) {
  scheduleFallingEdges();
}

void writeOutputs() {
  const uint16_t gasTicks = rover::dutyTicks(gasDuty);
  const uint16_t turnTicks = rover::dutyTicks(turnDuty);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!HARDWARE_PWM) {
      requestedGasTicks = gasTicks;
      requestedTurnTicks = turnTicks;
    } else {
      const uint16_t dutyA = THROTTLE_PIN == 9 ? gasDuty : turnDuty;
      const uint16_t dutyB = THROTTLE_PIN == 9 ? turnDuty : gasDuty;
      // Compare registers are double-buffered by Timer1. Disconnect PWM for
      // exact 0% and 100% so there are no narrow endpoint pulses.
      OCR1A = THROTTLE_PIN == 9 ? gasTicks : turnTicks;
      OCR1B = THROTTLE_PIN == 9 ? turnTicks : gasTicks;
      if (dutyA == 0 || dutyA == 10000) {
        if (dutyA == 10000) PORTB |= _BV(PB1);
        else PORTB &= ~_BV(PB1);
        TCCR1A &= ~_BV(COM1A1);
      } else TCCR1A |= _BV(COM1A1);
      if (dutyB == 0 || dutyB == 10000) {
        if (dutyB == 10000) PORTB |= _BV(PB2);
        else PORTB &= ~_BV(PB2);
        TCCR1A &= ~_BV(COM1B1);
      } else TCCR1A |= _BV(COM1B1);
    }
  }
}

void beginSignals() {
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(STEERING_PIN, OUTPUT);
  digitalWrite(THROTTLE_PIN, LOW);
  digitalWrite(STEERING_PIN, LOW);
  gasPort = portOutputRegister(digitalPinToPort(THROTTLE_PIN));
  turnPort = portOutputRegister(digitalPinToPort(STEERING_PIN));
  gasMask = digitalPinToBitMask(THROTTLE_PIN);
  turnMask = digitalPinToBitMask(STEERING_PIN);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
    TCNT1 = 0;
    if (HARDWARE_PWM) {
      ICR1 = rover::PERIOD_TICKS - 1;
      OCR1A = rover::dutyTicks(THROTTLE_PIN == 9 ? gasDuty : turnDuty);
      OCR1B = rover::dutyTicks(THROTTLE_PIN == 9 ? turnDuty : gasDuty);
      // Fast PWM mode 14, non-inverting outputs, /8 clock.
      TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
      TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    } else {
      requestedGasTicks = rover::dutyTicks(gasDuty);
      requestedTurnTicks = rover::dutyTicks(turnDuty);
      OCR1A = rover::PERIOD_TICKS - 1;
      TCNT1 = OCR1A;  // First frame begins on the next timer tick.
      TIMSK1 = _BV(OCIE1A);
      // CTC mode 4, /8 clock. Compare A starts frames; B ends pulses.
      TCCR1B = _BV(WGM12) | _BV(CS11);
    }
  }
}

void cancelProgram() {
  autoPending = false;
  sequence.stop();
}

void neutral() {
  gasDuty = rover::THROTTLE_NEUTRAL;
  turnDuty = rover::STEERING_NEUTRAL;
  writeOutputs();
}

void applyStep() {
  gasDuty = rover::throttleDuty(sequence.current().throttle);
  turnDuty = rover::steeringDuty(sequence.current().steering);
  writeOutputs();
}

void startProgram() {
  cancelProgram();
  sequence.start(millis());
  if (sequence.running()) applyStep();
  else neutral();
}

void printDuty(uint16_t duty) {
  Serial.print(duty / 100);
  Serial.print('.');
  if (duty % 100 < 10) Serial.print('0');
  Serial.print(duty % 100);
  Serial.print('%');
}

void status() {
  Serial.print(F("Gas=")); printDuty(gasDuty);
  Serial.print(F(" Turn=")); printDuty(turnDuty);
  Serial.print(F(" Pins(gas/turn)="));
  Serial.print(THROTTLE_PIN); Serial.print('/'); Serial.print(STEERING_PIN);
  Serial.print(F(" Mode="));
  if (autoPending) Serial.println(F("startup neutral"));
  else if (sequence.running()) {
    Serial.print(F("program step ")); Serial.println(sequence.index() + 1);
  } else Serial.println(F("manual/idle"));
}

void help() {
  Serial.println(F("Commands (115200 baud, newline):"));
  Serial.println(F("gas -100..100 | turn -100..100 (positive=forward/right)"));
  Serial.println(F("duty gas 0..100 | duty turn 0..100 (raw duty percent)"));
  Serial.println(F("run | stop | status | help"));
  Serial.println(F("Manual commands cancel the program, retaining the other channel."));
  Serial.println(F("STOP sets BOTH channels neutral. Manual settings persist until changed/reset."));
}

bool number(const char* token, float& value) {
  if (!token) return false;
  char* end;
  value = strtod(token, &end);
  return end != token && *end == '\0' && isfinite(value);
}

void command(char* line) {
  for (char* p = line; *p; ++p) *p = tolower((unsigned char)*p);
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  char* a = strtok(NULL, " \t");
  char* b = strtok(NULL, " \t");
  char* extra = strtok(NULL, " \t");
  float value;
  if (!a && !strcmp(cmd, "stop")) {
    cancelProgram(); neutral();
  } else if (!a && !strcmp(cmd, "run")) {
    startProgram();
  } else if (!a && !strcmp(cmd, "status")) {
    status(); return;
  } else if (!a && !strcmp(cmd, "help")) {
    help(); return;
  } else if ((!strcmp(cmd, "gas") || !strcmp(cmd, "turn")) && !b &&
             number(a, value) && value >= -100 && value <= 100 && value == int(value)) {
    cancelProgram();
    if (!strcmp(cmd, "gas")) gasDuty = rover::throttleDuty(int(value));
    else turnDuty = rover::steeringDuty(int(value));
    writeOutputs();
  } else if (!strcmp(cmd, "duty") && a && !extra && number(b, value) &&
             value >= 0 && value <= 100 &&
             (!strcmp(a, "gas") || !strcmp(a, "turn"))) {
    cancelProgram();
    const uint16_t duty = uint16_t(value * 100.0f + 0.5f);
    if (!strcmp(a, "gas")) gasDuty = duty;
    else turnDuty = duty;
    writeOutputs();
  } else {
    Serial.println(F("Invalid command; outputs unchanged. Type help."));
    return;
  }
  status();
}

void readSerial() {
  // Bound work per loop so continuous serial traffic cannot starve the route.
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (inputOverflow) Serial.println(F("Line too long; discarded."));
      else if (inputLength) {
        inputLine[inputLength] = '\0';
        command(inputLine);
      }
      inputLength = 0;
      inputOverflow = false;
    } else if (!inputOverflow) {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
      else inputOverflow = true;
    }
  }
}

void setup() {
  beginSignals();  // Neutral outputs before serial initialization.
  bootTime = millis();
  Serial.begin(115200);
  help();
  status();
}

void loop() {
  readSerial();
  const uint32_t now = millis();
  if (autoPending && uint32_t(now - bootTime) >= STARTUP_NEUTRAL_MS) startProgram();
  if (sequence.update(now)) {
    if (sequence.running()) applyStep();
    else { neutral(); Serial.println(F("Program complete; neutral.")); }
  }
}
