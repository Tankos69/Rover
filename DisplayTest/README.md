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
