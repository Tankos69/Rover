# Standalone SH5461AS display test

Open `DisplayTest.ino` in Arduino IDE, select Arduino Uno and the board's port,
then upload. This sketch is self-contained: no extra headers or libraries need
to be installed. It replaces the rover firmware while you test the display.
**Disconnect rover motor power**: the test does not generate neutral motor
signals. The MPU-6500 is not used and can remain connected to A4/A5.

Use exactly the resistor/transistor wiring in the [11-pin chart](../DISPLAY_WIRING.md#complete-wiring-chart--all-11-connected-display-pins).
This is for a bare common-cathode display, with seven 1 kΩ segment resistors,
four NPN digit drivers, four 2.2 kΩ base resistors and four 10 kΩ base pull-downs.
Do not connect digit commons directly to the Uno pins.

The default mapping matches the rover:

- Segments **a,b,c,d,e,f,g → D2,D3,D4,D5,D6,D7,D12**, via segment resistors.
- Digit drivers, **left to right → A0,A1,A2,A3**, via base resistors.
- Decimal point disconnected; NPN emitters and all circuit grounds to GND.

At startup all four digits show **8888**. Open Serial Monitor at **115200 baud**
and select **Newline**, **Carriage return**, or **Both NL & CR**. Type any four
digits and click Send:

```text
1234
0042
9876
0000
```

Leading zeros are retained, so `0042` displays all four characters. To put 7 on
all digits, send `7777`. Send `clear` to blank the display and `help` for usage.
Inputs must be exactly four digits, without spaces or a sign. Invalid or overly
long lines leave the previous display unchanged. The value stays until changed
or the Uno resets. Opening Serial Monitor may reset it to 8888.

For wiring checks, try `1111` (right-hand segments), `8888` (all seven segments),
and `1234` (digit order). If these are incorrect, check display pin numbering,
common-cathode polarity, transistor lead order and resistor connections.

Timer2 refreshes the display independently of serial input, at 1 kHz digit scans
(250 Hz per complete frame). Do not combine this test with `tone()` or other
Timer2 users. Physical pinout, brightness and transistor wiring need verification
on your display. After testing, upload the RoverController sketch again to
restore rover control.

## Mixed or malformed digits

**If repeated digits work but `1234` produces overlapping or malformed symbols,
check the digit-driver wiring first.** This firmware drives an NPN base HIGH to
select one digit. With common cathodes connected straight to Uno pins, LOW
selects a digit instead: three digits become active when the code intends one.
Repeated patterns can look correct even though selection is wrong. Do not use
this firmware with digit commons wired directly to GPIO; power off and install
the four NPN circuits from the wiring guide.

After correcting the hardware, these commands isolate wiring faults:

```text
digit 1 6
digit 2 6
digit 3 6
digit 4 6
segment a
segment b
segment c
segment d
segment e
segment f
segment g
1234
```

`digit N V` lights only position N (1=leftmost) with numeral V; the other three
are blank. `segment a` through `segment g` lights one segment on the leftmost
digit only. A numeral malformed even in isolation points toward segment wiring,
pinout or polarity; other positions lighting points toward digit wiring/drivers.

The scan now has two phases: all digit drivers off, then about 64 microseconds
later the segment pattern changes and the selected digit is enabled. This
provides transistor turn-off time without busy-waiting. It reduces switching
ghosting with the specified NPN circuit, but cannot correct wiring or polarity.
