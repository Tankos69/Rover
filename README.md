# Rover controller for Arduino Uno

Two independent **active-high logic signals at 62.66 Hz**, replacing the
receiver's throttle and steering outputs. This does **not** transmit RF.
It uses your measured frequency exactly as specified, rather than the more
usual 50 Hz servo signal. Confirm the frequency and polarity on an oscilloscope
before connecting the rover. The period is about **15.959 ms**.

## Four-digit turn-progress display

See [SH5461AS wiring and setup](DISPLAY_WIRING.md) for the bare common-cathode
LED display, resistors, four transistor digit drivers and the enable setting.
For a standalone manual digit test, see [DisplayTest](DisplayTest/README.md).
It displays measured whole degrees during angle/Y turns and retains the last
value afterward. Display support defaults off; its default pins leave D8/D11
and D9/D10 available for the motor signals.

## MPU-6500 integration

See [sensor-controlled turns, experimental speed control and Y turns](SENSOR_CONTROL.md)
for wiring, mounting, calibration, route examples and accuracy limits. Sensor
support is enabled by default and reserves A4/A5; old timed rows still work.
Speed estimation/control is experimental and disabled by default. A gyro-based
180-degree Y turn does not guarantee returning to the same position.

## Wiring and first test

1. Disconnect the receiver's two signal outputs from the motor/steering
   controller. Do not connect two signal outputs together.
2. With the default pin settings, connect **Uno D9 → throttle input**,
   **Uno D10 → steering input**, and **Uno GND → controller signal ground**.
   If you change the pins below, wire to those selected pins instead.
3. Confirm the receiving inputs accept the Uno's **5 V logic**; use appropriate
   level shifting if they require a lower voltage. These pins connect to logic
   inputs, never directly to motors. Retain the rover's motor driver and power
   circuitry. Power the Uno from an appropriate regulated supply, not a motor
   output; avoid backfeeding power rails when also connected to USB.
4. Initially test with the wheels raised and an accessible motor-power switch.
   Upload with `AUTO_RUN_ON_BOOT = false` (the default). Verify both waveforms
   before enabling motor power. Opening the Serial Monitor can reset the Uno.

This assumes two active-high PWM signals and a controller that interprets them
as measured. Electrical compatibility, braking/reverse arming behavior, and
actual movement must be tested on your rover. Software `stop` means the measured
neutral signals, not a guaranteed physical emergency stop. Manual outputs stay
at their last setting if USB is unplugged; remove motor power to stop if control
is lost. A reset or unpowered Uno cannot guarantee valid neutral signals.

## Choose the output pins

Edit these definitions near the top of `RoverController.ino`, then upload again:

```cpp
#define THROTTLE_PIN 9
#define STEERING_PIN 10
```

For example, use `THROTTLE_PIN 4` and `STEERING_PIN 7` to output on D4 and D7.
You can select any **two different pins from D2–D13 or A0–A5**; write an analog
pin as `A0`, for example. With sensor support enabled, **A4/A5 are reserved**
for I2C and cannot be selected as outputs. D0/D1 are reserved for serial control. Invalid or
duplicate pins cause a compile error. The pins do not need to be marked PWM.
Pin assignments are compiled into the program, not changed over serial.

Using D9/D10 in either order provides hardware PWM. Other pairs use Timer1
interrupts and can have edge jitter from other interrupts, including serial and
`millis()`. Avoid blocking interrupts or adding long interrupt handlers, and
verify the waveforms with your chosen pins. The nominal frequency stays 62.66 Hz.

## Upload and manual testing

Open `RoverController/RoverController.ino` in Arduino IDE, select **Arduino Uno**
and the connected port, then upload. No external Arduino libraries are needed.
Open Serial Monitor at **115200 baud**, with a newline or carriage-return line
ending. Commands are case-insensitive.

| Command | Effect |
| --- | --- |
| `gas 15` | 15% forward command |
| `gas -15` | 15% reverse command |
| `gas 0` | Neutral throttle |
| `turn 50` | 50% right command |
| `turn -50` | 50% left command |
| `turn 0` | Neutral steering |
| `duty gas 9.46` | Set throttle duty directly to 9.46% |
| `duty turn 6.37` | Set steering duty directly to 6.37% |
| `run` | Start/restart the compiled sequence immediately |
| `stop` | Cancel sequence/autostart and neutralize both channels |
| `status` | Show duty cycles and operating mode |
| `help` | Show commands |

### Automatic braking before reverse

After `gas 14`, sending `gas -60` now automatically sends:

1. A brake request at **-60%** for **500 ms**.
2. Neutral throttle for **1000 ms**.
3. Your requested reverse percentage, held until changed.

This also works for other reverse percentages and for programmed routes.
The code remembers forward driving across a neutral/`stop` command, because
neutral alone did not arm reverse in the observed rover controller.
After the handshake, changing reverse speed does not repeat it; commanding
forward makes the next reverse request require braking again. Reverse from
power-up, without prior forward driving, is sent directly. Reverse-to-forward
commands are also applied directly.

Tune `BRAKE_PERCENT`, `BRAKE_HOLD_MS`, and `REVERSE_NEUTRAL_MS` near the top of
the sketch if your controller needs different timing. The defaults use the
working -60% brake command from the manual test. Check on the actual rover;
these are timed commands without wheel-speed or controller-state feedback.
Small reverse requests can still fall within the motor controller's deadband.

The transition does not block serial input or signal generation. Steering
remains controllable. `stop` or `gas 0` immediately requests neutral and cancels
the queued reverse. A neutral arming interval already in progress finishes
without movement; an interrupted brake is retried on the next reverse request.
A forward request cancels the transition. Another negative `gas`
request updates the queued speed without restarting the hold times. `status`
reports `braking`, `neutral hold`, or `direct` along with the current duty.

**Raw `duty gas ...` commands bypass automatic braking**, cancel any queued
reverse, and apply the exact duty requested, for signal testing. Use `gas`
commands for automatic brake/neutral handling.

`gas` and `turn` accept integers from -100 to 100. `duty` accepts 0–100%
(rounded to 0.01%). Use raw duty testing carefully: values beyond the measured
endpoints may cause unexpected behavior. A valid manual movement/duty command
cancels the sequence and pending autostart, changes the selected channel, and
retains the other channel's command (including any pending throttle transition).
Invalid commands leave outputs
unchanged. There is no USB-disconnect timeout.

Percent movement commands interpolate linearly between neutral and the measured
endpoints; they are **not measured motor speed or steering angle**. In particular,
`gas 15` means 15% of the neutral-to-full-forward signal range, not 15% raw duty.
The controller's deadband may mean a low command produces no movement.

| Channel | -100 command | Neutral | +100 command |
| --- | --- | --- | --- |
| Throttle | Reverse: 8.26% | 9.46% | Forward: 12.19% |
| Steering | Left: 13.11% | 9.61% | Right: 6.37% |

For example, `gas 15` produces approximately **9.87% duty**, `gas -15`
approximately **9.28%**, and `turn 50` approximately **7.99%**.

## Standalone route

Edit `PROGRAM` near the top of the sketch. Every row contains:

```cpp
{ throttlePercent, steeringPercent, durationMilliseconds }
```

Both percentages must be integers from -100 to 100. Durations must be positive
32-bit millisecond values (use reasonable route durations under 49 days).
Each row sets **both channels** and holds them for its duration; repeat a value
to maintain it. When a reverse row needs automatic braking, the brake and
neutral holds are added **before** that row's duration starts. For example, a
4000 ms reverse row still gets 4000 ms of reverse command after the 1500 ms
handshake. You do not need to add a separate brake row.

The included route implements the requested example:

```cpp
const rover::Step PROGRAM[] = {
  { 15,  0, 5000},
  { 15, 50,  500},
  { 15,  0, 1000},
  {  0,  0, 2000},
  {-15,  0, 4000},
  {  0,  0, 1000},
};
```

Set **`AUTO_RUN_ON_BOOT = true`** and upload again for laptop-free operation.
On every power-up/reset it holds neutral for `STARTUP_NEUTRAL_MS` (5000 ms),
runs the route **once**, then holds both outputs neutral indefinitely.
Power the Uno independently for standalone use. The program is stored in flash;
serial settings are temporary and do not alter the saved route.

Add, remove, or reorder rows to define other routes, then upload again.
For a stationary turn use throttle `0`; for a pause use `{0, 0, duration}`.
The route continues without a laptop. `stop` remains available over serial.
Routes are open-loop timed commands: this provides no position or speed feedback.

## Signal timing

Timer1 provides a shared period and independent duty cycles for the selected
pins. D9/D10 pairs use hardware PWM, avoiding serial-induced PWM edge jitter.
Other pin pairs use interrupt-driven pulses; their physical edges have some
interrupt latency and jitter. Both channels' requested duty cycles are latched
together at the next frame in this mode. Coincident or closely spaced falling
edges are handled together when due, avoiding missed compare events.
With a nominal 16 MHz clock, prescaler 8 and 31918 ticks per period, the actual
frequency is **62.660568 Hz**. Duty resolution is about **0.003133 percentage
points**; interrupt latency limits practical edge accuracy on other pin pairs,
especially for very short pulses. The Uno clock's own tolerance also affects timing.
Neutral pulse widths are approximately 1.510 ms (throttle) and 1.534 ms (steering).

Timer1 is reserved for this sketch: do not combine it with the Servo library,
TimerOne, or other Timer1 users, and do not use `analogWrite()` on the selected
output pins or D9/D10. Those pins also cannot be shared with other peripherals.
Timer0 remains available for `millis()` and the nonblocking sequence scheduler.
Changes take effect on PWM frame boundaries, so allow up to one frame
(about 16 ms) for a signal change to be represented. Exact physical reaction
time depends on the rover controller as well.

## Development checks

Compile for the real board with Arduino CLI and the AVR core installed:

```sh
arduino-cli core install arduino:avr@1.8.6
arduino-cli compile --fqbn arduino:avr:uno RoverController
```

Run the host-side calibration and sequencing checks with a C++11 compiler:

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/control_test.cpp -o /tmp/rover-control-test
/tmp/rover-control-test
python3 tests/output_test.py
```

The output tests use simulated registers to exercise pin routing, frame updates,
coincident/late falling edges, 0%/100% duty, rejection of invalid pins, serial
reverse transitions, cancellation, and full route duration after braking.
They require Python 3 and g++.

These checks cannot validate electrical levels, physical PWM waveforms, motor
response, or stopping behavior; those require the actual Uno and rover.
