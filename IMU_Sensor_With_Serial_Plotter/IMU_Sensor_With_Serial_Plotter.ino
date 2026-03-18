#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// =========================
// I2C pins
// =========================
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// =========================
// Filter settings
// =========================
const int MA_WINDOW = 5;
const float ACCEL_DEADBAND = 0.03f;
const float GYRO_DEADBAND  = 0.01f;
const float MAG_DEADBAND   = 0.05f;

// =========================
// Adaptive filter settings
// =========================
const float ADAPTIVE_ALPHA_MIN = 0.08f;
const float ADAPTIVE_ALPHA_MAX = 0.45f;
const float GYRO_MOTION_LOW    = 0.05f;
const float GYRO_MOTION_HIGH   = 1.50f;

// =========================
// Calibration settings
// =========================
const int CAL_SAMPLES = 300;

// =========================
// Plot mode choice
// 1 = accel only
// 2 = gyro only
// 3 = mag only
// 4 = roll pitch yaw linAx linAy linAz linA_mag
// =========================
const int PLOT_MODE = 4;

// Offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float mx_offset = 0, my_offset = 0, mz_offset = 0;

// Low-pass filtered values
float ax_lpf = 0, ay_lpf = 0, az_lpf = 0;
float gx_lpf = 0, gy_lpf = 0, gz_lpf = 0;
float mx_lpf = 0, my_lpf = 0, mz_lpf = 0;

// Linear acceleration filtered values
float linAx_lpf = 0, linAy_lpf = 0, linAz_lpf = 0;
float linA_mag_lpf = 0;

// Moving average buffers
float ax_buf[MA_WINDOW] = {0}, ay_buf[MA_WINDOW] = {0}, az_buf[MA_WINDOW] = {0};
float gx_buf[MA_WINDOW] = {0}, gy_buf[MA_WINDOW] = {0}, gz_buf[MA_WINDOW] = {0};
float mx_buf[MA_WINDOW] = {0}, my_buf[MA_WINDOW] = {0}, mz_buf[MA_WINDOW] = {0};
float linAx_buf[MA_WINDOW] = {0}, linAy_buf[MA_WINDOW] = {0}, linAz_buf[MA_WINDOW] = {0};
float linA_mag_buf[MA_WINDOW] = {0};

int bufIndex = 0;

// Timing
unsigned long lastMicros = 0;
float dt = 0.02f;

// =========================
// Helper functions
// =========================
float lowPass(float input, float prev, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

float applyDeadband(float value, float threshold) {
  if (fabs(value) < threshold) return 0.0f;
  return value;
}

float meanArray(float *arr, int len) {
  float sum = 0.0f;
  for (int i = 0; i < len; i++) sum += arr[i];
  return sum / len;
}

void addToBuffer(float *arr, float value) {
  arr[bufIndex] = value;
}

float constrainFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

float getAdaptiveAlpha(float gx, float gy, float gz) {
  float motion = sqrt(gx * gx + gy * gy + gz * gz);

  if (motion <= GYRO_MOTION_LOW) return ADAPTIVE_ALPHA_MIN;
  if (motion >= GYRO_MOTION_HIGH) return ADAPTIVE_ALPHA_MAX;

  float t = (motion - GYRO_MOTION_LOW) / (GYRO_MOTION_HIGH - GYRO_MOTION_LOW);
  t = constrainFloat(t, 0.0f, 1.0f);

  return ADAPTIVE_ALPHA_MIN + t * (ADAPTIVE_ALPHA_MAX - ADAPTIVE_ALPHA_MIN);
}

void calibrateSensor() {
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float mx_sum = 0, my_sum = 0, mz_sum = 0;

  sensors_event_t accel, mag, gyro, temp;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    ax_sum += accel.acceleration.x;
    ay_sum += accel.acceleration.y;
    az_sum += accel.acceleration.z;

    gx_sum += gyro.gyro.x;
    gy_sum += gyro.gyro.y;
    gz_sum += gyro.gyro.z;

    mx_sum += mag.magnetic.x;
    my_sum += mag.magnetic.y;
    mz_sum += mag.magnetic.z;

    delay(10);
  }

  ax_offset = ax_sum / CAL_SAMPLES;
  ay_offset = ay_sum / CAL_SAMPLES;
  az_offset = (az_sum / CAL_SAMPLES) - 9.81f;

  gx_offset = gx_sum / CAL_SAMPLES;
  gy_offset = gy_sum / CAL_SAMPLES;
  gz_offset = gz_sum / CAL_SAMPLES;

  mx_offset = mx_sum / CAL_SAMPLES;
  my_offset = my_sum / CAL_SAMPLES;
  mz_offset = mz_sum / CAL_SAMPLES;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lsm.begin()) {
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  calibrateSensor();
  lastMicros = micros();
}

// =========================
// Loop
// =========================
void loop() {
  unsigned long now = micros();
  dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  // Raw calibrated values
  float ax = accel.acceleration.x - ax_offset;
  float ay = accel.acceleration.y - ay_offset;
  float az = accel.acceleration.z - az_offset;

  float gx = gyro.gyro.x - gx_offset;
  float gy = gyro.gyro.y - gy_offset;
  float gz = gyro.gyro.z - gz_offset;

  float mx = mag.magnetic.x - mx_offset;
  float my = mag.magnetic.y - my_offset;
  float mz = mag.magnetic.z - mz_offset;

  // Adaptive filtering
  float adaptiveAlpha = getAdaptiveAlpha(gx, gy, gz);

  ax_lpf = lowPass(ax, ax_lpf, adaptiveAlpha);
  ay_lpf = lowPass(ay, ay_lpf, adaptiveAlpha);
  az_lpf = lowPass(az, az_lpf, adaptiveAlpha);

  gx_lpf = lowPass(gx, gx_lpf, adaptiveAlpha);
  gy_lpf = lowPass(gy, gy_lpf, adaptiveAlpha);
  gz_lpf = lowPass(gz, gz_lpf, adaptiveAlpha);

  mx_lpf = lowPass(mx, mx_lpf, adaptiveAlpha);
  my_lpf = lowPass(my, my_lpf, adaptiveAlpha);
  mz_lpf = lowPass(mz, mz_lpf, adaptiveAlpha);

  addToBuffer(ax_buf, ax_lpf);
  addToBuffer(ay_buf, ay_lpf);
  addToBuffer(az_buf, az_lpf);

  addToBuffer(gx_buf, gx_lpf);
  addToBuffer(gy_buf, gy_lpf);
  addToBuffer(gz_buf, gz_lpf);

  addToBuffer(mx_buf, mx_lpf);
  addToBuffer(my_buf, my_lpf);
  addToBuffer(mz_buf, mz_lpf);

  bufIndex = (bufIndex + 1) % MA_WINDOW;

  float ax_f = applyDeadband(meanArray(ax_buf, MA_WINDOW), ACCEL_DEADBAND);
  float ay_f = applyDeadband(meanArray(ay_buf, MA_WINDOW), ACCEL_DEADBAND);
  float az_f = applyDeadband(meanArray(az_buf, MA_WINDOW), ACCEL_DEADBAND);

  float gx_f = applyDeadband(meanArray(gx_buf, MA_WINDOW), GYRO_DEADBAND);
  float gy_f = applyDeadband(meanArray(gy_buf, MA_WINDOW), GYRO_DEADBAND);
  float gz_f = applyDeadband(meanArray(gz_buf, MA_WINDOW), GYRO_DEADBAND);

  float mx_f = applyDeadband(meanArray(mx_buf, MA_WINDOW), MAG_DEADBAND);
  float my_f = applyDeadband(meanArray(my_buf, MA_WINDOW), MAG_DEADBAND);
  float mz_f = applyDeadband(meanArray(mz_buf, MA_WINDOW), MAG_DEADBAND);

  // Orientation
  float roll = atan2(ay_f, az_f) * 180.0f / PI;
  float pitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;

  float rollRad = roll * PI / 180.0f;
  float pitchRad = pitch * PI / 180.0f;

  float mx_comp = mx_f * cos(pitchRad) + mz_f * sin(pitchRad);
  float my_comp = mx_f * sin(rollRad) * sin(pitchRad) + my_f * cos(rollRad) - mz_f * sin(rollRad) * cos(pitchRad);

  float yaw = atan2(-my_comp, mx_comp) * 180.0f / PI;
  if (yaw < 0) yaw += 360.0f;

  // Linear acceleration
  float gX = -9.81f * sin(pitchRad);
  float gY =  9.81f * sin(rollRad) * cos(pitchRad);
  float gZ =  9.81f * cos(rollRad) * cos(pitchRad);

  float linAx = accel.acceleration.x - gX;
  float linAy = accel.acceleration.y - gY;
  float linAz = accel.acceleration.z - gZ;

  linAx_lpf = lowPass(linAx, linAx_lpf, adaptiveAlpha);
  linAy_lpf = lowPass(linAy, linAy_lpf, adaptiveAlpha);
  linAz_lpf = lowPass(linAz, linAz_lpf, adaptiveAlpha);

  addToBuffer(linAx_buf, linAx_lpf);
  addToBuffer(linAy_buf, linAy_lpf);
  addToBuffer(linAz_buf, linAz_lpf);

  float linAx_f = applyDeadband(meanArray(linAx_buf, MA_WINDOW), ACCEL_DEADBAND);
  float linAy_f = applyDeadband(meanArray(linAy_buf, MA_WINDOW), ACCEL_DEADBAND);
  float linAz_f = applyDeadband(meanArray(linAz_buf, MA_WINDOW), ACCEL_DEADBAND);

  float linA_mag = sqrt(linAx_f * linAx_f + linAy_f * linAy_f + linAz_f * linAz_f);
  linA_mag_lpf = lowPass(linA_mag, linA_mag_lpf, adaptiveAlpha);
  addToBuffer(linA_mag_buf, linA_mag_lpf);
  float linA_mag_f = applyDeadband(meanArray(linA_mag_buf, MA_WINDOW), ACCEL_DEADBAND);

  // =========================
  // SERIAL PLOTTER OUTPUT
  // numbers only, tab-separated
  // =========================
  if (PLOT_MODE == 1) {
    Serial.print(ax_f, 4); Serial.print('\t');
    Serial.print(ay_f, 4); Serial.print('\t');
    Serial.println(az_f, 4);
  }
  else if (PLOT_MODE == 2) {
    Serial.print(gx_f, 4); Serial.print('\t');
    Serial.print(gy_f, 4); Serial.print('\t');
    Serial.println(gz_f, 4);
  }
  else if (PLOT_MODE == 3) {
    Serial.print(mx_f, 4); Serial.print('\t');
    Serial.print(my_f, 4); Serial.print('\t');
    Serial.println(mz_f, 4);
  }
  else if (PLOT_MODE == 4) {
    Serial.print(roll, 2);     Serial.print('\t');
    Serial.print(pitch, 2);    Serial.print('\t');
    Serial.print(yaw, 2);      Serial.print('\t');
    Serial.print(linAx_f, 4);  Serial.print('\t');
    Serial.print(linAy_f, 4);  Serial.print('\t');
    Serial.print(linAz_f, 4);  Serial.print('\t');
    Serial.println(linA_mag_f, 4);
  }

  delay(20); // ~50 Hz
}