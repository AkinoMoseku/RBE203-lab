// Diagnostic sketch for the pin mapping in two_wheel_robot.ino.
// Select Arduino Uno. Serial Monitor: 9600 baud.
// Support the chassis with BOTH wheels off the ground before motor tests.
// This sketch does not balance the robot. Motors remain off until L or R.
#include <Wire.h>
#include <math.h>

const uint8_t IN1M = 7, IN2M = 6, IN3M = 13, IN4M = 12;
const uint8_t PWMA = 9, PWMB = 10, STBY = 8;
const uint8_t TEST_PWM = 100;
const unsigned long TEST_MS = 350;

uint8_t imuAddress = 0;
bool motorRunning = false;
unsigned long motorStarted = 0;
unsigned long lastReport = 0;

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(STBY, LOW);
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, LOW);
  motorRunning = false;
}

bool respondsAt(uint8_t address) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  uint8_t status = Wire.endTransmission();
  return status == 0 && !Wire.getWireTimeoutFlag();
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  uint8_t status = Wire.endTransmission();
  return status == 0 && !Wire.getWireTimeoutFlag();
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  uint8_t received = Wire.requestFrom(address, count);
  if (received != count || Wire.getWireTimeoutFlag()) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

int16_t signedWord(const uint8_t *bytes) {
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

void findImu() {
  imuAddress = 0;
  bool at68 = respondsAt(0x68);
  bool at69 = respondsAt(0x69);
  Serial.print(F("I2C 0x68="));
  Serial.print(at68 ? F("ACK") : F("NO_ACK"));
  Serial.print(F(" 0x69="));
  Serial.println(at69 ? F("ACK") : F("NO_ACK"));
  if (!at68 && !at69) {
    Serial.println(F("IMU_NOT_FOUND: check sensor power, GND, SDA and SCL."));
    return;
  }
  uint8_t candidate = at68 ? 0x68 : 0x69;
  uint8_t id = 0;
  if (!readRegisters(candidate, 0x75, &id, 1)) {
    Serial.println(F("IMU_ID_READ_FAILED"));
    return;
  }
  Serial.print(F("WHO_AM_I=0x"));
  Serial.println(id, HEX);
  if (id != 0x68) {
    Serial.println(F("UNEXPECTED_DEVICE_ID: verify the actual sensor model."));
    return;
  }
  // Match the original MPU6050 initialization: awake, +/-250 dps, +/-2g.
  if (!writeRegister(candidate, 0x6B, 0x01) ||
      !writeRegister(candidate, 0x1B, 0x00) ||
      !writeRegister(candidate, 0x1C, 0x00)) {
    Serial.println(F("IMU_INIT_FAILED"));
    return;
  }
  imuAddress = candidate;
  Serial.print(F("IMU_OK address=0x"));
  Serial.println(imuAddress, HEX);
  if (imuAddress == 0x69) {
    Serial.println(F("ADDRESS_MISMATCH: the original sketch expects 0x68."));
  }
}

void reportImu() {
  if (imuAddress == 0) {
    Serial.println(F("IMU_UNAVAILABLE; send I to retry."));
    return;
  }
  uint8_t data[14];
  if (!readRegisters(imuAddress, 0x3B, data, sizeof(data))) {
    Serial.println(F("IMU_READ_FAILED"));
    return;
  }
  int16_t ax = signedWord(data), ay = signedWord(data + 2), az = signedWord(data + 4);
  int16_t gx = signedWord(data + 8);
  float rawAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  Serial.print(F("ax=")); Serial.print(ax);
  Serial.print(F(" ay=")); Serial.print(ay);
  Serial.print(F(" az=")); Serial.print(az);
  Serial.print(F(" gx=")); Serial.print(gx);
  Serial.print(F(" rawAngle=")); Serial.print(rawAngle, 1);
  // Raw accelerometer angle, not the original sketch's filtered angle.
  Serial.print(F(" rawAngleOutside30="));
  Serial.println(rawAngle > 30 || rawAngle < -30 ? F("YES") : F("NO"));
}

void startMotorTest(bool left) {
  if (motorRunning) return;
  stopMotors();
  digitalWrite(left ? IN1M : IN3M, HIGH);
  digitalWrite(left ? IN2M : IN4M, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(left ? PWMA : PWMB, TEST_PWM);
  motorStarted = millis();
  motorRunning = true;
  Serial.println(left ? F("LEFT_COMMAND PWM=100 for 350ms")
                      : F("RIGHT_COMMAND PWM=100 for 350ms"));
}

void setup() {
  const uint8_t pins[] = {STBY, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB};
  for (uint8_t i = 0; i < sizeof(pins); ++i) {
    digitalWrite(pins[i], LOW);
    pinMode(pins[i], OUTPUT);
  }
  stopMotors();
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(10000, true);
  delay(1500);
  Serial.println(F("ROBOT_DIAGNOSTIC READY; motors OFF; no balancing."));
  Serial.println(F("Support chassis, BOTH wheels clear before L/R."));
  Serial.println(F("L=left pulse R=right pulse S=stop I=IMU retry"));
  findImu();
}

void loop() {
  if (motorRunning && millis() - motorStarted >= TEST_MS) {
    stopMotors();
    Serial.println(F("MOTOR_TEST_DONE; motors OFF"));
  }
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'S' || command == 's') {
      stopMotors();
      Serial.println(F("STOP; motors OFF"));
    } else if (!motorRunning) {
      if (command == 'L' || command == 'l') startMotorTest(true);
      else if (command == 'R' || command == 'r') startMotorTest(false);
      else if (command == 'I' || command == 'i') findImu();
    }
  }
  // Do not perform I2C operations during a motor pulse.
  if (!motorRunning && millis() - lastReport >= 500) {
    lastReport = millis();
    reportImu();
  }
}
