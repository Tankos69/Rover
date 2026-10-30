# Direct-drive common-anode display test — Arduino Uno

Use this sketch **only for a confirmed common-anode bare four-digit display**.
It uses the same Uno pin assignments as our previous guide, but **no NPN digit
drivers**. The existing `DisplayTest` and `RoverController` display driver remain
for the common-cathode/NPN arrangement; this test does not change their polarity.

## Confirm polarity and package pinout

A 12-pin package does not establish common-anode versus common-cathode. Look at
the front with digits upright and decimal points at the bottom. The usual layout
is lower pins 1..6 left-to-right and upper pins 12..7 left-to-right. Verify your
specific part's datasheet, or identify the pins with a multimeter diode test.

With this package layout, **red meter lead on pin 12 (digit 1 common), black on
pin 11 (segment a)** should light the leftmost digit's top segment for common
anode. The opposite polarity indicates common cathode. No light is inconclusive
if the meter lacks sufficient test voltage. Do not test with unresisted 5 V.
If the package map differs, wire by segment/common function rather than these
physical pin numbers.

## Wiring — same signal assignment, seven 2.2 kΩ resistors

Power everything off before rewiring and **disconnect motor power** for this
standalone test. It does not generate neutral throttle signals. The MPU can
remain connected on A4/A5, but is not used.

Use **one 2.2 kΩ resistor per segment**, seven in total. Do not connect segment
pins directly to the Uno or substitute the previous 1 kΩ values for this direct
digit-drive test. At 5 V with a roughly 2 V red LED drop, current is about 1.4 mA
per segment, or about 10 mA through a digit-common GPIO when all seven are lit.
This conservative setting is intentionally dim: each digit has about 1/4 duty.
Check the actual LED and Uno current ratings if using different hardware.

| Display pin (verify) | Function | Uno pin | Connection |
|---:|---|---|---|
| 1 | e — lower left | D6 | Through its own **2.2 kΩ** resistor |
| 2 | d — bottom | D5 | Through its own **2.2 kΩ** resistor |
| 4 | c — lower right | D4 | Through its own **2.2 kΩ** resistor |
| 5 | g — middle | D12 | Through its own **2.2 kΩ** resistor |
| 6 | Digit 4 common anode — rightmost | A3 | Direct connection, no transistor |
| 7 | b — upper right | D3 | Through its own **2.2 kΩ** resistor |
| 8 | Digit 3 common anode | A2 | Direct connection, no transistor |
| 9 | Digit 2 common anode | A1 | Direct connection, no transistor |
| 10 | f — upper left | D7 | Through its own **2.2 kΩ** resistor |
| 11 | a — top | D2 | Through its own **2.2 kΩ** resistor |
| 12 | Digit 1 common anode — leftmost | A0 | Direct connection, no transistor |

**Pin 3 (decimal point) stays disconnected.** Do not connect the digit commons
permanently to +5 V: they must be individually switched by A0..A3. Do not leave
NPN drivers, their base resistors or pull-downs connected in this direct wiring.
The bare display has no separate VCC/GND supply connection; LED current flows
from the selected Uno common pin through the LEDs and segment resistors into
segment GPIOs. Existing circuit grounds remain common.

## Upload and commands

Open `CommonAnodeDisplayTest.ino` in Arduino IDE, select **Arduino Uno**, choose
the port and upload. This single sketch needs no additional libraries/headers.
It replaces the rover program until you upload that program again.

Startup shows **8888**. Open Serial Monitor at **115200 baud** with **Newline**,
**Carriage return** or **Both NL & CR**. Send:

```text
1234
0042
9876
clear
digit 1 6
digit 4 2
segment a
segment g
help
```

Four-digit inputs preserve leading zeros. `digit N V` lights only position N
(1=leftmost, 4=rightmost) with numeral V (0..9). `segment a` through `segment g`
lights one segment on the leftmost position. `clear` blanks all positions.
Invalid/overlong lines retain the previous pattern.

This code selects a common anode with **HIGH**, leaves the other commons **LOW**,
and lights individual segments with **LOW**. The scan uses Timer2, 1 ms slots
and about 64 µs blanking before changing segments and enabling the next digit.
Don't combine it with tone/Timer2 libraries. Physical pinout, polarity and current
still need verification on your hardware; software tests cannot establish them.

If this test works but the common-cathode test does not, your wiring/polarity may
match common anode. Do not immediately upload the rover's common-cathode display
driver with this wiring: its display polarity would need adaptation first.
