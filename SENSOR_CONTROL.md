# MPU-6500 motion steps

The main `RoverController` sketch now uses the confirmed MPU-6500 over I2C.
`MPUSensorTest` remains a separate unchanged diagnostic sketch. The motor signal
frequency, configurable output pins, manual duty controls and automatic
forward/brake/neutral/reverse handling remain available.

## Wire and mount

- SDA to Uno **A4**, SCL to **A5**, common GND.
- ADD/SDO to GND (0x68); 0x69 is also detected automatically.
- NCS to 3.3 V; FSYNC to GND. EDA, ECL and INT remain disconnected.
- Supply the module appropriately for its actual breakout; the chip uses 3.3 V.
  Breakout 5 V compatibility is unconfirmed: use a bidirectional I2C level
  shifter for the Uno's 5 V bus unless the board explicitly supports it.
- Mount rigidly, **X forward, Y left, Z upward**. Default integrated heading is
  positive for a right turn. Turning the rover right by hand must make `status`
  yaw increase. If mounting axes differ, remount or adapt the axis transform in
  `mpu6500.h` before driving. The sensor provides no compass/north reference.

`ROVER_ENABLE_IMU` defaults to 1 at the top of the main sketch. A4/A5 are then
reserved and selecting them as motor outputs causes a compile error. Set it to
0 to use the original timed/manual controller without sensor polling; sensor
commands will be unavailable. No external library is needed beyond built-in
`Wire`. Use Arduino AVR Boards 1.8.6 or newer for bounded Wire timeouts.

## Calibrate before moving

Power up with the rover **stationary**, on its wheels on the driving surface.
Outputs remain neutral during sensor initialization. The sketch collects 200
fresh samples (about two seconds), estimates all gyro biases and initializes
pitch. Large motion, excessive variance, saturation, I2C failure, or stale data
rejects calibration. The code blocks movement commands while calibration runs.
`status` reports ready/calibrating/fault, unwrapped yaw and estimated speed.

Send `calibrate` to stop the program and retry initialization/calibration. Keep
stationary before and throughout this command. Calibration resets relative yaw
and estimated speed to zero; it does not determine physical position or north.
Do not calibrate a moving rover even if the wheels are not driven.

Manual commands and timed routes remain usable if the sensor is absent/faulted.
Sensor motions require successful calibration and fresh data. During a sensor motion, a read failure,
gap over 50 ms during integration, stale data or a motion timeout cancels the
route and requests neutral on both channels. Neutral is not a physical brake or
emergency stop; the vehicle can coast. Use an accessible motor-power switch.

## Routes: angle instead of duration

Existing `{gas, steering, milliseconds}` rows retain their original meaning.
Use the explicit `rover::Turn` form to make the third value **degrees**:

```cpp
const rover::Step PROGRAM[] = {
  {15, 0, 1000},                  // Timed: drive straight for 1 second
  rover::Turn(15, -100, 30),      // 15% gas, full left, until 30 degrees LEFT
  {15, 0, 1000},                  // Continue with the next command
  {0, 0, 1000},
};
```

Steering is a percentage command, not a measured wheel angle. Degrees are a
positive **relative heading change**, not pitch/roll or an absolute compass
bearing. Direction comes from the steering and gas signs: reverse with right
steering produces a left heading change. Motion in the wrong direction cannot
satisfy the target. Unwrapped integration allows turns across 360 degrees.

Optional fourth argument is the entire motion's timeout in milliseconds:
`rover::Turn(15, -100, 30, 10000)`. Default is 15 seconds. Timeout includes any
reverse handshake. Ordinary timed reverse rows still retain their full movement
duration after the brake/neutral waits. An angle step measures heading throughout
the motion, including any rotation while braking; it does not pause integration.
The next route row begins as soon as the measured target is crossed. At the end
of a route, both outputs return to neutral.

Gyro drift, steering response, frame latency, mounting and coasting can cause
physical angle error/overshoot. This is a target-crossing controller, not a
settled precision-angle servo. Start with low gas in open space and compare
against an external angle reference. Software tests do not establish real accuracy.

## Three-point (Y) turn

```cpp
rover::YTurn(15, -100, -60) // forward gas, initial steering, reverse gas
```

This targets a left 180-degree heading change using three cumulative thresholds:
forward/full left to 60 degrees, reverse/opposite steering to 120 degrees, then
forward/original steering to 180 degrees. Forward-to-reverse uses the existing
brake/neutral handshake. Default reverse gas is -60 if the third argument is
omitted. An optional fourth argument sets the total timeout (default 30000 ms).
Use positive initial steering for a right Y turn. Tune gas/steering to the rover
and available space; the nominal 60/60/60 split is not a collision-aware planner.

**It cannot guarantee an exact 180-degree physical stop or a return to the same
position.** This implementation controls relative heading only. An MPU-6500 alone
cannot observe position or reliably correct yaw drift; integrating acceleration
twice will not solve that. Returning to a known point needs an external position
reference (for example camera markers, UWB or suitable GNSS), usually combined
with wheel encoders and heading feedback. Encoders alone also accumulate error.

## Experimental acceleration and speed hold

**Disabled by default.** Set `ENABLE_EXPERIMENTAL_SPEED = true` in the main sketch
only to experiment with short runs after calibration from rest. This enables:

```cpp
rover::Accelerate(30, 0.5)       // 30% gas until ESTIMATED speed reaches 0.5 m/s
rover::Cruise(0.5, 40, 3000)     // Hold estimated 0.5 m/s for 3 s, cap gas at 40%
```

Accelerate has an optional third argument for timeout (default 10000 ms). Cruise
uses a bounded PI throttle controller; its third argument is a duration in
milliseconds. Gains in `motion.h` need physical tuning. Speed commands are
forward-only, capped to targets of 3 m/s; throttle is clamped to 0..maxGas.
Overspeed commands neutral, not reverse/braking. A completed standalone command
requests neutral; in a route it immediately proceeds to the next row, so follow
Accelerate with Cruise if you want to maintain that speed.

The estimate integrates body-X acceleration after approximate pitch/gravity
compensation. It is **not a measured ground speed**. Bias, vibration, roll,
slopes, pitching and the inability to separate gravity from sustained acceleration
cause drift. The filter can interpret sustained acceleration as tilt. Turning
also invalidates the simple forward-speed model. Neutral does not reset speed:
neutral is not proof of being stationary. Recalibrate only at actual rest.

These commands cannot guarantee reaching/holding true speed. Use wheel encoders
or an independent ground-speed sensor before treating this as reliable speed
control. No position estimate or exact-return claim is made.

## Manual sensor commands

At 115200 baud, with newline:

```text
angle 15 -100 30
speed 30 0.5
cruise 0.5 40 3
yturn 15 -100
status
stop
calibrate
```

`angle` uses degrees; `cruise` uses **seconds** in serial (milliseconds in the
route API). Serial Y turn uses the default -60 reverse gas. `speed`/`cruise`
are refused unless experimental mode is enabled. Commands require fresh,
calibrated sensor data. `stop` cancels any pending sensor motion and reverse
request. A normal manual movement command also cancels the sensor motion; as
before, it changes only its selected channel, retaining the other channel's last
command. Raw `duty` commands still bypass brake handling.

## Validation

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/motion_test.cpp -o /tmp/rover-motion-test
/tmp/rover-motion-test
python3 tests/output_test.py
```

Tests simulate directed angle crossings, timeouts, stale sensors, Y-turn phases,
speed opt-in and throttle bounds, plus calibration/bias integration/read failures
and command dispatch. Compile the sketch for Arduino Uno before uploading.
Actual waveform timing, mounted-axis signs, calibration robustness, speed drift,
turn overshoot and physical Y-turn geometry require testing on the rover.

## Diagnosing `ready=0 fault=1`

`status` now includes a latched `error=...` and the latest fresh-sample interval
`gapMs`. The error is cleared by a new initialization/calibration attempt, not by
`stop` or `run`. The turn parameters and output pins do not explain this flag.

- `not-detected`: neither address returned MPU-6500 identity 0x70; check power,
  I2C wiring/address selection, NCS high, and level shifting.
- `reset-write`, `config-write`, `status-read`, `data-read`: I2C transaction failed.
- `no-samples`: no fresh data-ready indication for at least 100 ms.
- `sample-gap`: over 50 ms between samples after calibration. This can be caused
  by a software polling delay as well as sensor/bus problems; check `gapMs`.
- `saturated`: an accelerometer or gyro axis approached the configured range.
- `cal-moving`: acceleration magnitude failed the stationary gravity check.
- `cal-gyro-noise`, `cal-accel-noise`: calibration sample variance was excessive.

With motors inactive, keep the rover still, send `calibrate`, wait three seconds,
then send `status`. If ready, send `run` and capture `status` again after a stop.
An error already present before driving points to startup/calibration or idle
sampling; one appearing during driving needs investigation of motor interference,
vibration and sampling timing. Do not bypass readiness checks to force a turn.

For `cal-moving`, `status` also prints a second line with the **failed sample's**
acceleration XYZ, acceleration magnitude (`norm[g]`) and gyro XYZ. These values
stay frozen until a new initialization/calibration, so a later `status` reports
what triggered the error, not a reading taken after the movement ended.
The check rejects acceleration magnitude outside 0.85..1.15 g. Gyro stability
is checked over all 200 samples (per-axis sample variance limit 0.1 (deg/s)^2).
Stable gyro offsets are averaged and subtracted; there is no longer a 5 deg/s
absolute-rate cutoff before bias estimation. The previous cutoff incorrectly
rejected a stationary sensor reporting about 13.37 deg/s on X.
These checks are heuristics, not proof of physical stillness. A stationary, tilted sensor should still measure
approximately 1 g total; no individual accelerometer axis has to equal 1 g.
Do not widen the thresholds without first examining the failing values.


## Stable gyro offsets and mounting direction

`status` prints the estimated gyro bias as `bias[dps]=X,Y,Z`. A stationary gyro
can have a nonzero raw reading; calibration must estimate that bias rather than
require its uncorrected value to be close to zero. The running mean and variance
use Welford's algorithm, which avoids subtracting large, nearly equal values.
Variable readings are still rejected. A large offset warrants checking drift
and repeatability; successful calibration alone does not prove sensor accuracy.

**Keep the rover physically still for the entire calibration.** Constant rotation
cannot be distinguished from bias using these samples and would be incorrectly
subtracted. Keep the motors inactive. Do not calibrate while holding, carrying,
or rotating the rover, and verify yaw stays approximately steady afterward.

A sample near `Accel=-0.148,0.008,-0.964 g` has magnitude about 0.975 g, which
passes the gravity check, but shows the sensor Z axis pointing down. This
controller assumes Z **up**, X forward and Y left. Remount accordingly before
using angle/Y-turn commands; bias calibration does not correct mounting axes.
Confirm a rightward hand rotation increases `yaw(right+)` before powered turns.
