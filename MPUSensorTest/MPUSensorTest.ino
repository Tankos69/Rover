// Standalone MPU-6500 / MPU-9250 / MPU-9255 test for Arduino Uno.
// Open Serial Monitor at 115200 baud. Send h to hide readings, s to show.
// SDA -> A4, SCL -> A5; use an I2C level shifter for 3.3 V-only logic.
// VCC -> suitable supply (normally 3.3 V); GND -> GND.
// ADD/SDO -> GND, NCS -> 3.3 V, FSYNC -> GND.
// Leave EDA, ECL and INT disconnected. Share 3.3 V via a breadboard rail.
// Factory magnetometer sensitivity adjustment is applied, but compass
// calibration and mounting alignment are still needed to calculate heading.

#include <Arduino.h>
#include <Wire.h>

uint8_t mpuAddress = 0;
const uint8_t MAG_ADDRESS = 0x0C;
bool magPresent = false;
bool showData = true;
float magAdjustment[3] = {1.0f, 1.0f, 1.0f};

bool readRegisters(uint8_t address, uint8_t reg,
                   uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(address, count, (uint8_t)true);
  if (received != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t readBigEndian(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

int16_t readLittleEndian(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

void stopWithError(const __FlashStringHelper *message) {
  Serial.println(message);
  while (true) delay(1000);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'h':
      case 'H':
        showData = false;
        Serial.println(F("Readings hidden. Send 's' to show them."));
        break;
      case 's':
      case 'S':
        showData = true;
        Serial.println(F("Readings enabled. Send 'h' to hide them."));
        break;
      case '\r':
      case '\n':
        break;
    }
  }
}

bool setupMagnetometer() {
  // Disable the internal I2C master and enable bypass to address 0x0C.
  if (!writeRegister(mpuAddress, 0x6A, 0x00)) return false;
  if (!writeRegister(mpuAddress, 0x37, 0x02)) return false;
  delay(10);

  uint8_t id;
  if (!readRegisters(MAG_ADDRESS, 0x00, &id, 1)) {
    Serial.println(F("No magnetometer response at 0x0C."));
    return false;
  }
  Serial.print(F("Magnetometer ID: 0x"));
  Serial.println(id, HEX);
  if (id != 0x48) {
    Serial.println(F("Unexpected magnetometer ID; expected 0x48."));
    return false;
  }

  // Reset, power down, then enter fuse-ROM mode.
  if (!writeRegister(MAG_ADDRESS, 0x0B, 0x01)) return false;
  delay(10);
  if (!writeRegister(MAG_ADDRESS, 0x0A, 0x00)) return false;
  delay(10);
  if (!writeRegister(MAG_ADDRESS, 0x0A, 0x0F)) return false;
  delay(10);

  uint8_t adjustment[3];
  if (!readRegisters(MAG_ADDRESS, 0x10, adjustment, 3)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    magAdjustment[i] = ((float)adjustment[i] - 128.0f) / 256.0f + 1.0f;
  }

  // Power down before selecting 16-bit continuous mode at 100 Hz.
  if (!writeRegister(MAG_ADDRESS, 0x0A, 0x00)) return false;
  delay(10);
  if (!writeRegister(MAG_ADDRESS, 0x0A, 0x16)) return false;
  delay(20);
  return true;
}

void printMagnetometer() {
  uint8_t status;
  if (!readRegisters(MAG_ADDRESS, 0x02, &status, 1)) {
    Serial.print(F(" | Mag: read error"));
    return;
  }
  if (!(status & 0x01)) {
    Serial.print(F(" | Mag: waiting for sample"));
    return;
  }

  // Read XYZ and ST2 together. Reading ST2 completes the measurement read.
  uint8_t data[7];
  if (!readRegisters(MAG_ADDRESS, 0x03, data, 7)) {
    Serial.print(F(" | Mag: read error"));
    return;
  }
  if (data[6] & 0x08) {
    Serial.print(F(" | Mag: overflow (magnetic field too strong)"));
    return;
  }

  float mx = readLittleEndian(&data[0]) * 0.15f * magAdjustment[0];
  float my = readLittleEndian(&data[2]) * 0.15f * magAdjustment[1];
  float mz = readLittleEndian(&data[4]) * 0.15f * magAdjustment[2];
  Serial.print(F(" | Mag [uT]: "));
  Serial.print(mx, 2);
  Serial.print(F(", "));
  Serial.print(my, 2);
  Serial.print(F(", "));
  Serial.print(mz, 2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  // Bound I2C waits on AVR core versions that support bus timeouts.
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  delay(200);
  Serial.println(F("\nMPU accelerometer / gyro / magnetometer test"));

  for (uint8_t address = 0x68; address <= 0x69; address++) {
    uint8_t id;
    if (readRegisters(address, 0x75, &id, 1)) {
      Serial.print(F("Address 0x"));
      Serial.print(address, HEX);
      Serial.print(F(": WHO_AM_I = 0x"));
      Serial.println(id, HEX);
      if (id == 0x70 || id == 0x71 || id == 0x73) {
        mpuAddress = address;
        if (id == 0x70) {
          Serial.println(F("Main sensor identifies as MPU-6500."));
        } else if (id == 0x71) {
          Serial.println(F("Main sensor identifies as MPU-9250."));
        } else {
          Serial.println(F("Main sensor identifies as MPU-9255."));
        }
        break;
      }
    }
  }
  if (mpuAddress == 0) {
    stopWithError(F("No supported MPU found. Check wiring and power."));
  }

  if (!writeRegister(mpuAddress, 0x6B, 0x80)) {
    stopWithError(F("MPU reset failed."));
  }
  delay(100);
  if (!writeRegister(mpuAddress, 0x6B, 0x01)) {
    stopWithError(F("MPU wake-up failed."));
  }
  delay(100);

  bool ok = true;
  ok &= writeRegister(mpuAddress, 0x6C, 0x00); // All axes enabled
  ok &= writeRegister(mpuAddress, 0x1A, 0x03); // Gyro filter ~41 Hz
  ok &= writeRegister(mpuAddress, 0x19, 0x09); // 100 Hz sample rate
  ok &= writeRegister(mpuAddress, 0x1B, 0x00); // Gyro +/-250 deg/s
  ok &= writeRegister(mpuAddress, 0x1C, 0x00); // Accel +/-2 g
  ok &= writeRegister(mpuAddress, 0x1D, 0x03); // Accel filter ~41 Hz
  if (!ok) stopWithError(F("MPU configuration failed."));

  magPresent = setupMagnetometer();
  if (magPresent) {
    Serial.println(F("AK8963 magnetometer detected and configured."));
  } else {
    Serial.println(F("Magnetometer unavailable; continuing accel/gyro test."));
    Serial.println(F("An MPU-6500 alone has no magnetometer."));
  }
  Serial.println(F("Send 'h' to hide readings; 's' to show them again."));
  delay(100);
}

void loop() {
  handleSerialCommands();
  if (!showData) {
    delay(10);
    return;
  }

  uint8_t data[14];
  if (!readRegisters(mpuAddress, 0x3B, data, 14)) {
    Serial.println(F("MPU read failed. Check wiring."));
    delay(500);
    return;
  }
  float ax = readBigEndian(&data[0]) / 16384.0f;
  float ay = readBigEndian(&data[2]) / 16384.0f;
  float az = readBigEndian(&data[4]) / 16384.0f;
  float temperature = readBigEndian(&data[6]) / 333.87f + 21.0f;
  float gx = readBigEndian(&data[8]) / 131.0f;
  float gy = readBigEndian(&data[10]) / 131.0f;
  float gz = readBigEndian(&data[12]) / 131.0f;

  Serial.print(F("Accel [g]: "));
  Serial.print(ax, 3);
  Serial.print(F(", "));
  Serial.print(ay, 3);
  Serial.print(F(", "));
  Serial.print(az, 3);
  Serial.print(F(" | Gyro [deg/s]: "));
  Serial.print(gx, 2);
  Serial.print(F(", "));
  Serial.print(gy, 2);
  Serial.print(F(", "));
  Serial.print(gz, 2);
  if (magPresent) printMagnetometer();
  Serial.print(F(" | Temp [C]: "));
  Serial.println(temperature, 1);
  delay(200);
}
