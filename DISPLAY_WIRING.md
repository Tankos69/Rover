# Common-cathode turn-progress display — current controller

The main `RoverController` now defaults to your **directly connected, common-
cathode four-digit display**, without NPN drivers, with **all numerals enabled**.
The temporary 1/4/7 restriction exists only in `DirectCathodeDisplayTest`, not
in the controller. Keep your corrected bottom-segment wire in the mapping below.

## Configure and upload

Download the **whole updated RoverController folder**, including every header.
The defaults near the top of the sketch are:

```cpp
#define ROVER_ENABLE_DISPLAY 1
#define ROVER_DISPLAY_NPN 0
```

Preserve your own motor pins, for example:

```cpp
#define THROTTLE_PIN 8
#define STEERING_PIN 11
```

The repository's motor defaults remain D9/D10. Both D8/D11 and D9/D10 are free
with this display map. A0..A3 are occupied by the display, A4/A5 by the MPU-6500.
Compile-time checks reject overlapping display and motor pins.

## Current limiting

Full-digit direct drive assumes each segment has adequate current limiting:
**2.2 kΩ per segment**, or **two 1 kΩ resistors in series per segment (2 kΩ)**,
seven segment paths in total. There is no per-pattern segment-count restriction
in this controller.
At about 5 V and 2 V red LED drop, 2.2 kΩ gives about 1.4 mA per lit segment,
about 10 mA through a selected common-cathode GPIO with all seven lit. Multiplex
operation reduces average current but not the instantaneous digit-pin current.

Power off before rewiring. Do not mix direct connections with the old NPN base
resistors/pull-downs, and do not tie all digit commons permanently to GND.
The display has no separate VCC connection; the Uno outputs supply/sink LED
current through the seven resistors. Existing circuit grounds remain common.

## Pin numbering

Verify your specific display's datasheet/pinout. For the usual 5461AS layout,
look at the front with digits upright and decimal points at the bottom:

```text
Top:       12  11  10   9   8   7
          +---------------------+
          |  digit1 ... digit4  |
          +---------------------+
Bottom:     1   2   3   4   5   6
```

A diode test with negative on digit common pin 12 and positive on segment pin
11 should light the first digit's top segment if this map and common-cathode
polarity match and the meter supplies sufficient voltage. Pin count alone does
not establish polarity. Do not test with an unresisted 5 V supply.

## Complete wiring chart — all 11 connected display pins

| Display pin (verify) | Segment or digit | Uno pin | Connection |
|---:|---|---|---|
| 1 | Segment e — lower left | D6 | Through its own segment resistor |
| 2 | Segment d — bottom horizontal | D5 | Through its own segment resistor |
| 4 | Segment c — lower right | D4 | Through its own segment resistor |
| 5 | Segment g — middle | D12 | Through its own segment resistor |
| 6 | Digit 4 common cathode — rightmost | A3 | Direct |
| 7 | Segment b — upper right | D3 | Through its own segment resistor |
| 8 | Digit 3 common cathode | A2 | Direct |
| 9 | Digit 2 common cathode | A1 | Direct |
| 10 | Segment f — upper left | D7 | Through its own segment resistor |
| 11 | Segment a — top | D2 | Through its own segment resistor |
| 12 | Digit 1 common cathode — leftmost | A0 | Direct |

**Display pin 3, the decimal point, stays disconnected.** Each segment resistor
is 2.2 kΩ or 2 kΩ as described above. Verify function-to-pin assignments on your
part rather than assuming all suppliers use the same package map.

## What is displayed

Your existing angle step works without a new display command:

```cpp
const rover::Step PROGRAM[] = {
  {14, 0, 500},
  rover::Turn(14, -100, 90),
  {0, 0, 1000}
};
```

- Initially blank; each accepted angle/Y turn starts at **0**.
- Displays the heading change measured by the gyro since the turn began,
  rounded to whole degrees. A normal left or right 90-degree turn shows 0..90.
- All numeral glyphs, 0..9, are enabled. Negative progress indicates rotation
  opposite the requested direction, for example `-12`.
- Completion or manual `stop` freezes the last measured value. The next angle/Y
  turn resets the origin and displayed progress to zero. A Y turn shows cumulative
  change across its three legs, up to the 180-degree target.
- Sensor failure or timeout during the turn shows `----`. Nonfinite/out-of-range
  values also show dashes; the numeric display range is -999..9999.
- Timed turns and ordinary manual `turn` commands do not start angle tracking.

This shows the controller's estimate, not the requested angle or an independent
measurement. Gyro drift, rounding and physical overshoot still apply. Display
rounding does not change when the motion controller advances to the next step.

Keep the rover stationary for gyro calibration; `status` must show ready before
a sensor turn can start. With motor power disconnected, you can send
`angle 14 -100 90` at 115200 baud and rotate the chassis left by hand to check the
display within its 15-second timeout. A stationary chassis with spinning wheels
does not change heading. For individual segment checks use
[the direct-cathode test](DirectCathodeDisplayTest/README.md); its full-number
option is separate from the main controller.

## Other circuits and timing

The [legacy NPN circuit](NPN_DISPLAY_WIRING.md) remains supported with
`ROVER_DISPLAY_NPN=1`. Use only the driver mode matching your physical wiring.
The [common-anode test](CommonAnodeDisplayTest/README.md) is separate hardware
and is not the main controller's display driver.

Timer2 scans one digit per millisecond, giving about 250 Hz per complete frame.
All digit cathodes go HIGH (off); about 64 microseconds later segments update
and one cathode goes LOW (on). This all-off interval reduces mixed-digit artifacts.
In NPN mode the digit polarity is reversed. No blocking refresh delays are used.

Timer1 remains responsible for 62.66 Hz motor signals. The Timer2 ISR adds some
edge jitter for arbitrary motor pins such as D8/D11; D9/D10 use hardware PWM.
Do not add `tone()`, Timer2 libraries, or `analogWrite()` on D3/D11 while the
display runs. D11 remains available as a digital rover motor-signal output.

Software tests cover motor-pin preservation, both digit-driver polarities,
full glyphs (including 8888), angle command -> gyro heading -> visible digits,
completion retention, new-turn reset, wrong-way progress and fault dashes.
Uno compilation and simulated tests cannot establish physical current, waveform
timing, mounting signs, brightness, heading accuracy or electrical compatibility.
