#include <Wire.h>
#include <math.h>

const int MPU = 0x68;

// =========================
// MPU raw registers/scales
// =========================
const float ACCEL_SCALE = 16384.0;   // LSB/g for +/-2g
const float GYRO_SCALE  = 131.0;     // LSB/(deg/s) for +/-250 deg/s

// =========================
// Timing
// =========================
unsigned long lastMicros = 0;
float dt = 0.02f; // default ~20 ms

// =========================
// Raw sensor values
// =========================
int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;

// =========================
// Calibration offsets
// =========================
float axOffset = 0, ayOffset = 0, azOffset = 0;
float gxOffset = 0, gyOffset = 0, gzOffset = 0;

// =========================
// Converted physical units
// =========================
float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;

// =========================
// Filtered values
// =========================
float axLP = 0, ayLP = 0, azLP = 0;
float gxLP = 0, gyLP = 0, gzLP = 0;

// =========================
// Moving average buffers
// =========================
const int MA_WINDOW = 5;
float axBuf[MA_WINDOW] = {0}, ayBuf[MA_WINDOW] = {0}, azBuf[MA_WINDOW] = {0};
float gxBuf[MA_WINDOW] = {0}, gyBuf[MA_WINDOW] = {0}, gzBuf[MA_WINDOW] = {0};
int maIndex = 0;

// =========================
// Angles
// =========================
float roll = 0.0f;
float pitch = 0.0f;

// =========================
// Filter parameters
// =========================
const float LOW_PASS_ALPHA_SLOW = 0.15f;
const float LOW_PASS_ALPHA_FAST = 0.45f;
const float COMPLEMENTARY_ALPHA = 0.98f;

// Deadband thresholds
const float ACCEL_DEADBAND_G = 0.015f;
const float GYRO_DEADBAND_DPS = 0.35f;

// Outlier rejection thresholds
const float ACCEL_SPIKE_LIMIT_G = 1.5f;
const float GYRO_SPIKE_LIMIT_DPS = 400.0f;

// =========================
// Helpers
// =========================
float applyDeadband(float value, float threshold) {
  if (fabs(value) < threshold) return 0.0f;
  return value;
}

float adaptiveLowPass(float current, float previous) {
  float diff = fabs(current - previous);
  float alpha = (diff > 0.12f) ? LOW_PASS_ALPHA_FAST : LOW_PASS_ALPHA_SLOW;
  return alpha * current + (1.0f - alpha) * previous;
}

float movingAverage(float *buffer) {
  float sum = 0.0f;
  for (int i = 0; i < MA_WINDOW; i++) sum += buffer[i];
  return sum / MA_WINDOW;
}

bool readMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  int bytesReceived = Wire.requestFrom(MPU, 14, true);
  if (bytesReceived != 14) return false;

  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // skip temperature

  rawGx = Wire.read() << 8 | Wire.read();
  rawGy = Wire.read() << 8 | Wire.read();
  rawGz = Wire.read() << 8 | Wire.read();

  return true;
}

void calibrateMPU(int samples = 500) {
  Serial.println("Calibrating... Keep sensor still.");

  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    if (readMPU()) {
      axSum += rawAx;
      aySum += rawAy;
      azSum += rawAz;
      gxSum += rawGx;
      gySum += rawGy;
      gzSum += rawGz;
      validSamples++;
    }
    delay(5);
  }

  if (validSamples == 0) {
    Serial.println("Calibration failed: no valid samples.");
    return;
  }

  axOffset = axSum / (float)validSamples;
  ayOffset = aySum / (float)validSamples;
  azOffset = (azSum / (float)validSamples) - ACCEL_SCALE; // subtract 1g from Z if flat
  gxOffset = gxSum / (float)validSamples;
  gyOffset = gySum / (float)validSamples;
  gzOffset = gzSum / (float)validSamples;

  Serial.println("Calibration complete.");
}

void setupMPU() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accelerometer +/-2g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Gyroscope +/-250 deg/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Digital low-pass filter
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // ESP32: SDA=21, SCL=22

  Serial.println("Starting MPU6050...");

  setupMPU();
  delay(100);

  calibrateMPU();

  lastMicros = micros();

  Serial.println("MPU6050 Ready");
  Serial.println("time_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll,pitch");
}

void loop() {
  unsigned long nowMicros = micros();
  dt = (nowMicros - lastMicros) / 1000000.0f;
  lastMicros = nowMicros;

  if (dt <= 0 || dt > 0.2f) dt = 0.02f;

  if (!readMPU()) {
    Serial.println("Read error");
    delay(20);
    return;
  }

  // =========================
  // Convert to physical units
  // =========================
  float ax = (rawAx - axOffset) / ACCEL_SCALE;
  float ay = (rawAy - ayOffset) / ACCEL_SCALE;
  float az = (rawAz - azOffset) / ACCEL_SCALE;

  float gx = (rawGx - gxOffset) / GYRO_SCALE;
  float gy = (rawGy - gyOffset) / GYRO_SCALE;
  float gz = (rawGz - gzOffset) / GYRO_SCALE;

  // =========================
  // Outlier rejection
  // =========================
  if (fabs(ax) > ACCEL_SPIKE_LIMIT_G) ax = axLP;
  if (fabs(ay) > ACCEL_SPIKE_LIMIT_G) ay = ayLP;
  if (fabs(az) > ACCEL_SPIKE_LIMIT_G + 1.0f) az = azLP;

  if (fabs(gx) > GYRO_SPIKE_LIMIT_DPS) gx = gxLP;
  if (fabs(gy) > GYRO_SPIKE_LIMIT_DPS) gy = gyLP;
  if (fabs(gz) > GYRO_SPIKE_LIMIT_DPS) gz = gzLP;

  // =========================
  // Moving average buffers
  // =========================
  axBuf[maIndex] = ax;
  ayBuf[maIndex] = ay;
  azBuf[maIndex] = az;

  gxBuf[maIndex] = gx;
  gyBuf[maIndex] = gy;
  gzBuf[maIndex] = gz;

  maIndex = (maIndex + 1) % MA_WINDOW;

  float axMA = movingAverage(axBuf);
  float ayMA = movingAverage(ayBuf);
  float azMA = movingAverage(azBuf);

  float gxMA = movingAverage(gxBuf);
  float gyMA = movingAverage(gyBuf);
  float gzMA = movingAverage(gzBuf);

  // =========================
  // Adaptive low-pass filter
  // =========================
  axLP = adaptiveLowPass(axMA, axLP);
  ayLP = adaptiveLowPass(ayMA, ayLP);
  azLP = adaptiveLowPass(azMA, azLP);

  gxLP = adaptiveLowPass(gxMA, gxLP);
  gyLP = adaptiveLowPass(gyMA, gyLP);
  gzLP = adaptiveLowPass(gzMA, gzLP);

  // =========================
  // Deadband thresholding
  // =========================
  ax_g = applyDeadband(axLP, ACCEL_DEADBAND_G);
  ay_g = applyDeadband(ayLP, ACCEL_DEADBAND_G);
  az_g = applyDeadband(azLP, ACCEL_DEADBAND_G);

  gx_dps = applyDeadband(gxLP, GYRO_DEADBAND_DPS);
  gy_dps = applyDeadband(gyLP, GYRO_DEADBAND_DPS);
  gz_dps = applyDeadband(gzLP, GYRO_DEADBAND_DPS);

  // =========================
  // Accelerometer angle estimate
  // =========================
  float accelRoll  = atan2(ay_g, az_g) * 180.0f / PI;
  float accelPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;

  // =========================
  // Complementary filter fusion
  // =========================
  roll  = COMPLEMENTARY_ALPHA * (roll  + gx_dps * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  pitch = COMPLEMENTARY_ALPHA * (pitch + gy_dps * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;

  // =========================
  // Serial CSV output
  // =========================
  Serial.print(millis()); Serial.print(",");
  Serial.print(ax_g, 4); Serial.print(",");
  Serial.print(ay_g, 4); Serial.print(",");
  Serial.print(az_g, 4); Serial.print(",");
  Serial.print(gx_dps, 4); Serial.print(",");
  Serial.print(gy_dps, 4); Serial.print(",");
  Serial.print(gz_dps, 4); Serial.print(",");
  Serial.print(roll, 3); Serial.print(",");
  Serial.println(pitch, 3);

  delay(20); // ~50 Hz
}