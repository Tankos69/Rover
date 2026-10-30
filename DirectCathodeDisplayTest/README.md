# Direct common-cathode test — limited mode for your 1 kΩ resistors

This separate sketch matches **common-cathode digit pins wired directly to the
Uno**, without NPN drivers. It keeps the previous signal mapping. It is different
from the NPN-driver `DisplayTest` and the `CommonAnodeDisplayTest`.

**Disconnect motor power**, power down before wiring, and keep one segment
resistor on each a..g wire. This firmware does not generate neutral motor signals.
The MPU-6500 is not used. Verify your specific display's polarity and package
pinout; the usual front-view numbering is bottom 1..6 left-to-right and top
12..7 left-to-right, with digits upright and decimal points at the bottom.

| Display pin (verify) | Function | Uno pin | Connection |
|---:|---|---|---|
| 1 | e, lower left | D6 | Through its own **1 kΩ** resistor |
| 2 | d, bottom | D5 | Through its own **1 kΩ** resistor |
| 4 | c, lower right | D4 | Through its own **1 kΩ** resistor |
| 5 | g, middle | D12 | Through its own **1 kΩ** resistor |
| 6 | Digit 4 cathode, rightmost | A3 | Direct, no transistor |
| 7 | b, upper right | D3 | Through its own **1 kΩ** resistor |
| 8 | Digit 3 cathode | A2 | Direct, no transistor |
| 9 | Digit 2 cathode | A1 | Direct, no transistor |
| 10 | f, upper left | D7 | Through its own **1 kΩ** resistor |
| 11 | a, top | D2 | Through its own **1 kΩ** resistor |
| 12 | Digit 1 cathode, leftmost | A0 | Direct, no transistor |

Display pin 3 (decimal point) stays disconnected. Do not connect all commons to
GND: the Uno must select them individually. Do not leave transistor drivers or
their base pull-downs attached to these direct connections.

## Why the default mode limits the numbers

At approximately 5 V with 2 V red LED forward drop and 1 kΩ resistance, each
segment draws about 3 mA. The digit-common pin must sink their combined current.
A numeral 8 needs seven segments, about 21 mA, above the Uno's 20 mA per-pin
operating recommendation. A lower multiplex duty cycle does not lower that peak.

The default `DISPLAY_MAX_SEGMENTS` is **4**. It allows at most four lit segments
on a selected digit (about 12 mA nominal). Consequently it accepts numerals
**1, 4 and 7**, mixed or repeated, plus isolated single-segment tests. Numerals
0, 2, 3, 5, 6, 8 and 9 are refused because they require five or more segments.
A whole input is checked before any digit changes; a refused input retains the
previous pattern. Current estimates assume ordinary red LEDs and correct wiring.

This limited mode is for checking mixed-digit selection with your current seven
1 kΩ resistors. It is not full-number display support with those resistors.

## Upload and test

Open `DirectCathodeDisplayTest.ino`, select Arduino Uno, and upload. No extra
libraries/headers are required. It starts **blank**, not 8888. Serial Monitor:
**115200 baud**, with **Newline** (or CR/CRLF) selected. Try:

```text
1471
4714
1111
4444
7777
digit 1 4
digit 4 7
segment a
segment g
clear
```

`digit N V` selects only position N (1=leftmost); V must pass the segment limit.
`segment a` through `segment g` lights a single segment on the leftmost digit.
`clear` blanks the display. `8888`, `1234`, and `0000` are intentionally refused
in the default mode. Selecting a digit uses **LOW**; lighting a segment uses
**HIGH**. Timer2 provides multiplexing with an all-digits-off interval between
slots. No other tone/Timer2 users may run alongside this test.

## Full-number mode later

First change **each of the seven segment paths** to at least **2 kΩ**:

- One 2.2 kΩ resistor per segment, or
- **Two 1 kΩ resistors in series per segment** (14 resistors total).

Then change the definition near the top of the sketch to:

```cpp
#define DISPLAY_MAX_SEGMENTS 7
```

Re-upload. All four-digit values, including 0042 and 8888, are then accepted.
Do not enable full mode while retaining a single 1 kΩ resistor per segment.
An alternative for more brightness is the original NPN circuit with its matching
firmware. The main rover display driver still expects that NPN arrangement;
this standalone test does not convert its driver to direct-cathode operation.
