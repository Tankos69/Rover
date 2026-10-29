#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rover_control.h"
#include "mpu6500.h"

#ifndef ROVER_ENABLE_IMU
#define ROVER_ENABLE_IMU 1
#endif
// Experimental IMU-only speed control: drift-prone; use wheel encoders for accuracy.
constexpr bool ENABLE_EXPERIMENTAL_SPEED = false;

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
static_assert(!ROVER_ENABLE_IMU || (THROTTLE_PIN < A4 && STEERING_PIN < A4),
              "IMU uses A4/A5 for I2C; choose other output pins");
constexpr bool HARDWARE_PWM = (THROTTLE_PIN == 9 && STEERING_PIN == 10) ||
                              (THROTTLE_PIN == 10 && STEERING_PIN == 9);

// Set true and upload again to run PROGRAM once on each power-up/reset.
constexpr bool AUTO_RUN_ON_BOOT = false;
constexpr uint32_t STARTUP_NEUTRAL_MS = 5000;

// Automatic forward-to-reverse handshake (based on the working gas -60 test).
// These holds are added BEFORE a reverse route step's requested duration.
constexpr int BRAKE_PERCENT = -60;
constexpr uint32_t BRAKE_HOLD_MS = 500;
constexpr uint32_t REVERSE_NEUTRAL_MS = 1000;
static_assert(BRAKE_PERCENT >= -100 && BRAKE_PERCENT < 0,
              "Brake command must be between -100 and -1");
static_assert(BRAKE_HOLD_MS >= 32 && REVERSE_NEUTRAL_MS >= 32,
              "Brake and neutral holds must span at least two PWM frames");

// Edit this table and upload to change the standalone route.
// {throttle %, steering %, hold time in milliseconds}
// Each row replaces both settings. Repeat a setting to keep it active.
// You may also insert sensor steps, e.g. rover::Turn(15, -100, 30),
// rover::YTurn(15, -100, -60), or experimental rover::Cruise(0.5, 40, 3000).
const rover::Step PROGRAM[] = {
  { 15,  0, 5000},  // Forward 15% for 5 seconds
  { 15, 50,  500},  // Keep driving, turn right 50% for 0.5 seconds
  { 15,  0, 1000},  // Keep driving, straighten for 1 second
  {  0,  0, 2000},  // Stop for 2 seconds
  {-15,  0, 4000},  // Reverse 15% for 4 seconds
  {  0,  0, 1000},  // Stop; completion also holds neutral indefinitely
};

Mpu6500 imu;
rover::MotionControl motion;
rover::Sequence sequence(PROGRAM, sizeof(PROGRAM) / sizeof(PROGRAM[0]));
rover::Throttle throttle(rover::throttleDuty(BRAKE_PERCENT),
                         BRAKE_HOLD_MS, REVERSE_NEUTRAL_MS);
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
  motion.stop();
}

void neutral() {
  throttle.stop(millis());
  gasDuty = throttle.duty();
  turnDuty = rover::STEERING_NEUTRAL;
  writeOutputs();
}

void movement(int gas, int steering) {
  throttle.request(rover::throttleDuty(gas), millis());
  gasDuty = throttle.duty();
  turnDuty = rover::steeringDuty(steering);
  writeOutputs();
}

void applyStep() {
  const rover::Step& step = sequence.current();
  if (step.motion.kind != rover::TIMED) {
    if (!motion.start(step.motion, millis(), imu.yaw,
                      ROVER_ENABLE_IMU && imu.fresh(millis()), ENABLE_EXPERIMENTAL_SPEED)) {
      cancelProgram(); neutral();
      Serial.println(F("Motion refused: IMU not ready, speed disabled, or invalid parameters."));
      return;
    }
    movement(motion.gas, motion.steering);
  } else movement(step.throttle, step.steering);
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
  Serial.print(F("IMU ready=")); Serial.print(imu.ready);
  Serial.print(F(" calibrating=")); Serial.print(imu.calibrating);
  Serial.print(F(" fault=")); Serial.print(imu.fault);
  Serial.print(F(" error="));
  switch (imu.error) {
    case Mpu6500::NONE: Serial.print(F("none")); break;
    case Mpu6500::NO_DEVICE: Serial.print(F("not-detected")); break;
    case Mpu6500::RESET_WRITE: Serial.print(F("reset-write")); break;
    case Mpu6500::CONFIG_WRITE: Serial.print(F("config-write")); break;
    case Mpu6500::STATUS_READ: Serial.print(F("status-read")); break;
    case Mpu6500::DATA_READ: Serial.print(F("data-read")); break;
    case Mpu6500::NO_SAMPLES: Serial.print(F("no-samples")); break;
    case Mpu6500::SAMPLE_GAP: Serial.print(F("sample-gap")); break;
    case Mpu6500::SATURATED: Serial.print(F("saturated")); break;
    case Mpu6500::CAL_MOVING: Serial.print(F("cal-moving")); break;
    case Mpu6500::CAL_GYRO_NOISE: Serial.print(F("cal-gyro-noise")); break;
    case Mpu6500::CAL_ACCEL_NOISE: Serial.print(F("cal-accel-noise")); break;
  }
  Serial.print(F(" gapMs=")); Serial.print(imu.sampleGapMs);
  Serial.print(F(" yaw(right+)=")); Serial.print(imu.yaw);
  Serial.print(F(" estimated m/s=")); Serial.print(imu.speed);
  Serial.print(F(" Gas=")); printDuty(gasDuty);
  Serial.print(F(" Turn=")); printDuty(turnDuty);
  Serial.print(F(" Pins(gas/turn)="));
  Serial.print(THROTTLE_PIN); Serial.print('/'); Serial.print(STEERING_PIN);
  Serial.print(F(" Throttle="));
  if (throttle.phase() == rover::Throttle::BRAKING) Serial.print(F("braking"));
  else if (throttle.phase() == rover::Throttle::NEUTRAL_HOLD) Serial.print(F("neutral hold"));
  else Serial.print(F("direct"));
  Serial.print(F(" Mode="));
  if (autoPending) Serial.println(F("startup neutral"));
  else if (sequence.running()) {
    Serial.print(F("program step ")); Serial.println(sequence.index() + 1);
  } else if (motion.active) Serial.println(F("sensor motion"));
  else Serial.println(F("manual/idle"));
}

void help() {
  Serial.println(F("115200 baud, newline. gas N | turn N (-100..100); duty gas/turn P (0..100)"));
  Serial.println(F("run | stop | status | help | calibrate"));
  Serial.println(F("angle GAS STEER DEG | yturn GAS STEER"));
  Serial.println(F("speed GAS M/S | cruise M/S MAXGAS SECONDS (opt-in)"));
  Serial.println(F("STOP cancels. gas adds brake/neutral; duty bypasses it."));
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
  char* fourth = strtok(NULL, " \t");
  float value, v2, v3;
  if (imu.calibrating && strcmp(cmd,"stop") && strcmp(cmd,"status") &&
      strcmp(cmd,"help") && strcmp(cmd,"calibrate")) {
    Serial.println(F("Wait for calibration with rover stationary.")); return;
  }
  if ((!strcmp(cmd,"angle") || !strcmp(cmd,"speed") ||
       !strcmp(cmd,"cruise") || !strcmp(cmd,"yturn")) && !fourth &&
      number(a,value) && number(b,v2) &&
      ((!strcmp(cmd,"angle") || !strcmp(cmd,"cruise")) ? number(extra,v3) : !extra)) {
    rover::Motion action={rover::TIMED,0,0,0,0};
    bool valid=false;
    if (!strcmp(cmd,"angle") && value!=0 && v2!=0 && value>=-100 && value<=100 && value==int(value) &&
        v2>=-100 && v2<=100 && v2==int(v2) && v3>0 && v3<=360) {
      action=rover::Turn(int(value),int(v2),v3); valid=true;
    } else if (!strcmp(cmd,"yturn") && v2!=0 && value>0 && value<=100 && value==int(value) &&
               v2>=-100 && v2<=100 && v2==int(v2)) {
      action=rover::YTurn(int(value),int(v2)); valid=true;
    } else if (!strcmp(cmd,"speed") && v2>0 && v2<=3 && value>0 && value<=100 && value==int(value)) {
      action=rover::Accelerate(int(value),v2); valid=true;
    } else if (!strcmp(cmd,"cruise") && value>0 && value<=3 && v2>0 && v2<=100 && v2==int(v2) && v3>0 && v3<=60) {
      action=rover::Cruise(value,int(v2),uint32_t(v3*1000)); valid=true;
    }
    if (!valid || !ROVER_ENABLE_IMU || !imu.fresh(millis()) ||
        ((action.kind==rover::ACCELERATE || action.kind==rover::CRUISE) && !ENABLE_EXPERIMENTAL_SPEED)) {
      Serial.println(F("Invalid motion, IMU unavailable, or experimental speed disabled.")); return;
    }
    cancelProgram();
    if (motion.start(action,millis(),imu.yaw,true,ENABLE_EXPERIMENTAL_SPEED))
      movement(motion.gas,motion.steering);
    else { neutral(); Serial.println(F("Motion parameters refused.")); }
  } else if (!a && !strcmp(cmd,"calibrate")) {
    cancelProgram(); neutral();
    if (ROVER_ENABLE_IMU) imu.begin();
    Serial.println(F("Keep rover stationary for IMU calibration (about 2 seconds)."));
  } else if (!a && !strcmp(cmd, "stop")) {
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
    if (!strcmp(cmd, "gas")) {
      throttle.request(rover::throttleDuty(int(value)), millis());
      gasDuty = throttle.duty();
    }
    else turnDuty = rover::steeringDuty(int(value));
    writeOutputs();
  } else if (!strcmp(cmd, "duty") && a && !extra && number(b, value) &&
             value >= 0 && value <= 100 &&
             (!strcmp(a, "gas") || !strcmp(a, "turn"))) {
    cancelProgram();
    const uint16_t duty = uint16_t(value * 100.0f + 0.5f);
    if (!strcmp(a, "gas")) {
      throttle.raw(duty);
      gasDuty = throttle.duty();
    }
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
  if (ROVER_ENABLE_IMU) imu.begin();
  Serial.println(F("Keep rover stationary during IMU calibration; status reports readiness."));
  help();
  status();
}

void loop() {
  readSerial();
  const uint32_t now = millis();
  if (ROVER_ENABLE_IMU) imu.update(now);
  if (autoPending && !imu.calibrating && uint32_t(now - bootTime) >= STARTUP_NEUTRAL_MS) startProgram();
  const bool wasTransitioning = throttle.transitioning();
  throttle.update(now);
  if (gasDuty != throttle.duty()) {
    gasDuty = throttle.duty();
    writeOutputs();
  }
  // Start the reverse step's movement duration only once the handshake ends.
  if (sequence.running() && (wasTransitioning || throttle.transitioning())) {
    sequence.restartHold(now);
  }
  if (motion.update(now, imu.yaw, imu.speed, imu.fresh(now), throttle.transitioning())) {
    if (motion.active) movement(motion.gas,motion.steering);
    else if (motion.failed) {
      cancelProgram(); neutral();
      Serial.println(F("Motion stopped: sensor stale/fault or timeout."));
    } else if (!sequence.running()) neutral();
  }
  if (!motion.active && sequence.update(now)) {
    if (sequence.running()) applyStep();
    else { neutral(); Serial.println(F("Program complete; neutral.")); }
  }
}
