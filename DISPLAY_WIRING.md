# SH5461AS turn-progress display

This code drives a **bare 12-pin, common-cathode four-digit seven-segment display**
using seven segment resistors and four NPN digit drivers. It is not a TM1637 or
MAX7219 module. SH5461AS is commonly sold with this configuration, but verify
your particular part's pinout and polarity before wiring: markings from different
suppliers are not proof of an identical pinout. A common-anode part needs a
different driver and this circuit/code will not operate it correctly.

## Parts

- SH5461AS common-cathode display
- **7 × 1 kΩ** resistors (one per segment; 1/4 W is adequate)
- **4 × 2.2 kΩ** resistors (NPN base series resistors)
- **4 × 10 kΩ** resistors (base-to-ground pull-downs)
- **4 × NPN transistors**, e.g. 2N3904, BC547 or 2N2222
- Breadboard and jumper wires

Check the emitter/base/collector arrangement for your exact transistors. These
part families and package variants do not all use the same physical lead order.

At 5 V with a red segment dropping about 2 V, 1 kΩ gives about **3 mA peak per
segment**. All seven lit is about 21 mA through the active digit's transistor.
Each digit is on at most 1/4 of the time, so this is a conservative, fairly dim
starting point. Do not omit the resistors or connect digit commons directly to
GPIO. Increasing brightness needs a current-budget review or a dedicated driver.

## Identify the display pins

For the common 5461AS layout below, look at the **front, digits upright, decimal
points at the bottom**. The lower row is pins **1..6 from left to right**. The
upper row is **12..7 from left to right**:

```text
        12   11   10    9    8    7
       +--------------------------+
       |  digit1 digit2 digit3 digit4 |
       +--------------------------+
         1    2    3    4    5    6
```

**Verify this against your part's datasheet or use a diode test first.** In
common-cathode operation, the meter's negative lead goes to a digit common and
its positive lead to a segment; the segment should light if the meter supplies
sufficient voltage. For example, with this layout, negative on pin 12 and
positive on pin 11 lights digit 1's top segment. Do not use an unresisted 5 V
wire as a test probe. If the map differs, identify the a..g segment anodes and
four cathodes and connect by their functions instead of these package numbers.

## Complete wiring chart — all 11 connected display pins

This chart uses the front-view numbering above and the default `DISPLAY_PINS`
mapping in the code. Verify your particular display's pinout before wiring.
Power off the Uno, rover and sensor first.

| Display pin | Segment or digit | Uno pin | How to connect |
|---:|---|---|---|
| 1 | Segment e — lower left | D6 | D6 → **1 kΩ resistor** → display pin 1 |
| 2 | Segment d — bottom | D5 | D5 → **1 kΩ resistor** → display pin 2 |
| 4 | Segment c — lower right | D4 | D4 → **1 kΩ resistor** → display pin 4 |
| 5 | Segment g — middle | D12 | D12 → **1 kΩ resistor** → display pin 5 |
| 6 | Digit 4 common cathode — rightmost | A3 | Display pin 6 → NPN collector; A3 → **2.2 kΩ resistor** → base |
| 7 | Segment b — upper right | D3 | D3 → **1 kΩ resistor** → display pin 7 |
| 8 | Digit 3 common cathode | A2 | Display pin 8 → NPN collector; A2 → **2.2 kΩ resistor** → base |
| 9 | Digit 2 common cathode | A1 | Display pin 9 → NPN collector; A1 → **2.2 kΩ resistor** → base |
| 10 | Segment f — upper left | D7 | D7 → **1 kΩ resistor** → display pin 10 |
| 11 | Segment a — top | D2 | D2 → **1 kΩ resistor** → display pin 11 |
| 12 | Digit 1 common cathode — leftmost | A0 | Display pin 12 → NPN collector; A0 → **2.2 kΩ resistor** → base |

Each digit uses its **own NPN transistor**, with **emitter → Uno GND** and an
additional **10 kΩ resistor from base to GND**. A0–A3 control the transistor
bases; **do not connect the digit cathodes directly to these Uno pins**. Check
the transistor's actual emitter/base/collector lead order.

**Display pin 3 is the decimal point and stays disconnected.** Thus, exactly
11 of the display's 12 pins are connected. The sections below show the same
connections grouped by segments and digit drivers.

## Segment connections

Power off the Uno, rover and sensor before wiring. Each row gets its **own
1 kΩ resistor**, between the Uno pin and display segment pin:

| Segment | Position | Display pin (verify) | Uno pin |
|---|---|---:|---|
| a | top | 11 | D2 |
| b | upper right | 7 | D3 |
| c | lower right | 4 | D4 |
| d | bottom | 2 | D5 |
| e | lower left | 1 | D6 |
| f | upper left | 10 | D7 |
| g | middle | 5 | D12 |

Leave display **pin 3 (decimal point)** disconnected. Values are whole degrees,
with no decimal point or degree symbol. This bare display has no separate VCC
pin: segment current comes through the Uno output pins and resistors.

## Digit connections — four identical NPN driver circuits

For **each** transistor:

- Emitter **E → Uno GND**.
- Collector **C → its display digit common** from the table.
- Base **B → a 2.2 kΩ resistor → its Uno control pin**.
- An additional **10 kΩ resistor from B to GND** holds the digit off at reset.

| Digit, left to right | Common cathode display pin (verify) | NPN base control via 2.2 kΩ |
|---|---:|---|
| 1 | 12 | A0 |
| 2 | 9 | A1 |
| 3 | 8 | A2 |
| 4 | 6 | A3 |

```text
Uno Ax --- 2.2 kΩ ---+--- B   NPN
                    |    C -------- digit common cathode
                   10 kΩ E -------- GND
                    |
                   GND
```

All grounds (Uno, sensor, display transistor emitters and rover signal ground)
remain common. Leave the MPU-6500 on **A4/A5**, with its existing 3.3 V/level-shifter
arrangement. Display wiring does not change the sensor's voltage requirements.

This map leaves **D8/D11** available for your current throttle/steering pins and
also leaves the original D9/D10 pair available. A0..A3 are now occupied by the
display. Do not also use them for motor outputs. You may edit `DISPLAY_PINS` in
`turn_display.h` to reroute pins; they must be distinct and must not overlap the
motor pins. The sketch gives a compile error on a conflict when display support
is enabled. D0/D1 stay reserved for Serial, A4/A5 for I2C.

## Common-anode alternative

If your display is confirmed **common anode**, use the separate
[direct common-anode test and 11-pin chart](CommonAnodeDisplayTest/README.md).
It keeps the same Uno signal assignments but uses seven **2.2 kΩ segment
resistors** and direct digit-common connections, with no NPN drivers. Do not mix
that circuit with the common-cathode/NPN circuit described here. The main rover
display driver has not been converted to common anode.

## Standalone digit test

Before using the rover firmware, you can upload
[DisplayTest/DisplayTest.ino](DisplayTest/DisplayTest.ino) and send any four digits
through Serial Monitor, e.g. `0042`. See [test instructions](DisplayTest/README.md).
It uses the same wiring. Disconnect motor power while running the standalone test.

## Enable and use

Download the **whole updated RoverController folder**, including headers. In
`RoverController.ino` change:

```cpp
#define ROVER_ENABLE_DISPLAY 1
```

Display support defaults to 0 so old wiring/pin selections remain usable until
you connect the new hardware. Preserve your motor pin definitions, e.g.:

```cpp
#define THROTTLE_PIN 8
#define STEERING_PIN 11
```

Upload for Arduino Uno, calibrate stationary, then use your existing route:

```cpp
const rover::Step PROGRAM[] = {
  {14, 0, 500},
  rover::Turn(14, -100, 90),
  {0, 0, 1000}
};
```

- Before the first angle/Y turn the display is blank.
- Each accepted angle/Y turn resets the reading to **0** at that turn's origin.
- It shows integrated heading change toward the requested direction, rounded to
  the nearest whole degree: a left or right 90-degree turn normally shows 0..90.
- Rotation opposite the requested direction shows negative progress, e.g. `-12`.
- On completion or manual cancellation the last measured progress stays visible
  until another angle/Y turn begins (or power/reset). A Y turn displays cumulative
  progress across all three legs, up to its 180-degree target.
- Sensor faults/timeouts during the turn display **`----`**. Out-of-range or
  nonfinite values also show dashes. Range is -999..9999 whole degrees.
- Timed turns and ordinary `turn` commands do not start a new angle measurement.

This is the same gyro-derived estimate used to decide when the turn completes,
not the target number, a compass bearing, or an independent angle measurement.
Drift and physical overshoot still apply. A displayed 90 does not prove an exact
90-degree physical stop. Display rounding does not affect the motion controller.

For a first functional check, keep **motor power disconnected**, calibrate the
stationary rover, then send `angle 14 -100 90`. Turn the chassis left by hand:
the value should increase. Complete the test before its 15-second timeout, or
send `stop`. A stationary chassis with wheels spinning will not show a heading
change. If segments/digits are wrong, recheck pin numbering and polarity.

## Timing and verification

Timer2 scans one digit every 1 ms (250 Hz per complete four-digit refresh).
The scan blanks all digits, then uses a second compare about 64 microseconds
later to change segments and select one digit, allowing transistor turn-off.
It keeps refreshing during normal Wire/Serial waits without delays in the main
loop. Timer1 continues to generate the motor signals; Timer0 still runs millis().

Do not add `tone()`, Timer2 libraries, or `analogWrite()` on D3/D11 while the
display is enabled. **D11 remains valid as a timer-driven rover digital output**;
this driver does not connect Timer2's hardware PWM outputs to D3/D11. The short
scan ISR adds interrupt latency/jitter to motor signals on arbitrary pins,
including D8/D11. D9/D10 hardware PWM avoids that edge jitter. Verify waveforms
on the actual rover. Code tests do not establish physical brightness, flicker,
transistor lead order, pinout, or motor-signal timing.

Tests cover number formatting, signed progress/retention, fault state, multiplex
routing, digit exclusivity, pin conflict rejection and preserving motor GPIO:

```sh
g++ -std=c++11 -Wall -Wextra -Werror tests/display_test.cpp -o /tmp/rover-display-test
/tmp/rover-display-test
python3 tests/output_test.py
```

## Why equal digits may work while mixed digits do not

The digit outputs are **active HIGH because they drive NPN bases**. If you connect
the four display cathodes directly to A0..A3 instead of using transistors, LOW
turns a digit on: three digits become selected instead of one. Their different
segment patterns overlap. `4444` may look fine while `1234` looks nonsensical.
This is not proof of a broken display or incorrect numeral encoding.

Power off and replace each direct common-cathode connection with its specified
NPN driver: common cathode → collector, emitter → GND, Uno output → 2.2 kΩ →
base, and base → 10 kΩ → GND. Check transistor lead order. The segment resistors
remain in place. Do not just remove resistors or invert wires with power on.

The driver also now blanks all digits before each switch and uses a second
Timer2 compare about 64 microseconds later to change segments and enable one
digit. This leaves time for transistor turn-off and reduces switching ghosting.
No blocking delay is used. Physical wiring remains necessary; blanking does not
make direct-cathode wiring compatible. See the standalone test's `digit` and
`segment` commands to verify one position/segment at a time after wiring.
