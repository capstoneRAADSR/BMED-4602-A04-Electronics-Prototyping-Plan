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
const float LPF_ALPHA = 0.2f;   // base low-pass filter strength
const int MA_WINDOW = 5;        // moving average window
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
// Output mode
// 0 = CSV for Serial Monitor / logging
// 1 = accel plot
// 2 = gyro plot
// 3 = mag plot
// 4 = roll/pitch/yaw + linear accel plot
// 5 = all plot labels
// 6 = human-readable wearable orientation meanings
// =========================
const int OUTPUT_MODE = 6;

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

// Linear acceleration moving average buffers
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

  if (motion <= GYRO_MOTION_LOW) {
    return ADAPTIVE_ALPHA_MIN;
  }
  if (motion >= GYRO_MOTION_HIGH) {
    return ADAPTIVE_ALPHA_MAX;
  }

  float t = (motion - GYRO_MOTION_LOW) / (GYRO_MOTION_HIGH - GYRO_MOTION_LOW);
  t = constrainFloat(t, 0.0f, 1.0f);

  return ADAPTIVE_ALPHA_MIN + t * (ADAPTIVE_ALPHA_MAX - ADAPTIVE_ALPHA_MIN);
}

// =========================
// Human-readable meaning helpers
// =========================
const char* getRollMeaning(float roll) {
  if (roll >= -10 && roll <= 10) return "device flat / level";
  if (roll > 10 && roll <= 45) return "right side tilted down";
  if (roll < -10 && roll >= -45) return "left side tilted down";
  if (roll > 45 && roll <= 80) return "strong right tilt";
  if (roll < -45 && roll >= -80) return "strong left tilt";
  if (roll > 80 || roll < -80) return "device on its side";
  return "roll unknown";
}

const char* getPitchMeaning(float pitch) {
  if (pitch >= -10 && pitch <= 10) return "device level front-to-back";
  if (pitch > 10 && pitch <= 45) return "front tilted up";
  if (pitch < -10 && pitch >= -45) return "front tilted down";
  if (pitch > 45 && pitch <= 80) return "front strongly tilted up";
  if (pitch < -45 && pitch >= -80) return "front strongly tilted down";
  if (pitch > 80) return "device nearly upright, top up";
  if (pitch < -80) return "device nearly upright, top down";
  return "pitch unknown";
}

const char* getYawMeaning(float yaw) {
  if ((yaw >= 337.5 && yaw <= 360.0) || (yaw >= 0.0 && yaw < 22.5)) return "facing north";
  if (yaw >= 22.5 && yaw < 67.5) return "facing northeast";
  if (yaw >= 67.5 && yaw < 112.5) return "facing east";
  if (yaw >= 112.5 && yaw < 157.5) return "facing southeast";
  if (yaw >= 157.5 && yaw < 202.5) return "facing south";
  if (yaw >= 202.5 && yaw < 247.5) return "facing southwest";
  if (yaw >= 247.5 && yaw < 292.5) return "facing west";
  if (yaw >= 292.5 && yaw < 337.5) return "facing northwest";
  return "heading unknown";
}

void calibrateSensor() {
  Serial.println("Keep sensor still. Calibrating...");

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
  az_offset = (az_sum / CAL_SAMPLES) - 9.81f; // assumes mostly upright/still

  gx_offset = gx_sum / CAL_SAMPLES;
  gy_offset = gy_sum / CAL_SAMPLES;
  gz_offset = gz_sum / CAL_SAMPLES;

  mx_offset = mx_sum / CAL_SAMPLES;
  my_offset = my_sum / CAL_SAMPLES;
  mz_offset = mz_sum / CAL_SAMPLES;

  Serial.println("Calibration complete.");
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lsm.begin()) {
    Serial.println("Failed to detect LSM9DS1");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  calibrateSensor();

  lastMicros = micros();

  Serial.println("LSM9DS1 ready.");

  if (OUTPUT_MODE == 0) {
    Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,linAx,linAy,linAz,linA_mag");
  } else if (OUTPUT_MODE == 6) {
    Serial.println("Human-readable wearable orientation mode");
  }
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

  // =========================
  // Adaptive low-pass filter
  // =========================
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

  // Moving average buffers
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

  // Moving average output
  float ax_f = meanArray(ax_buf, MA_WINDOW);
  float ay_f = meanArray(ay_buf, MA_WINDOW);
  float az_f = meanArray(az_buf, MA_WINDOW);

  float gx_f = meanArray(gx_buf, MA_WINDOW);
  float gy_f = meanArray(gy_buf, MA_WINDOW);
  float gz_f = meanArray(gz_buf, MA_WINDOW);

  float mx_f = meanArray(mx_buf, MA_WINDOW);
  float my_f = meanArray(my_buf, MA_WINDOW);
  float mz_f = meanArray(mz_buf, MA_WINDOW);

  // Deadband thresholding
  ax_f = applyDeadband(ax_f, ACCEL_DEADBAND);
  ay_f = applyDeadband(ay_f, ACCEL_DEADBAND);
  az_f = applyDeadband(az_f, ACCEL_DEADBAND);

  gx_f = applyDeadband(gx_f, GYRO_DEADBAND);
  gy_f = applyDeadband(gy_f, GYRO_DEADBAND);
  gz_f = applyDeadband(gz_f, GYRO_DEADBAND);

  mx_f = applyDeadband(mx_f, MAG_DEADBAND);
  my_f = applyDeadband(my_f, MAG_DEADBAND);
  mz_f = applyDeadband(mz_f, MAG_DEADBAND);

  // Orientation estimates
  float roll = atan2(ay_f, az_f) * 180.0f / PI;
  float pitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0f / PI;

  float rollRad = roll * PI / 180.0f;
  float pitchRad = pitch * PI / 180.0f;

  float mx_comp = mx_f * cos(pitchRad) + mz_f * sin(pitchRad);
  float my_comp = mx_f * sin(rollRad) * sin(pitchRad) + my_f * cos(rollRad) - mz_f * sin(rollRad) * cos(pitchRad);

  float yaw = atan2(-my_comp, mx_comp) * 180.0f / PI;
  if (yaw < 0) yaw += 360.0f;

  // =========================
  // Approximate linear acceleration (gravity removed)
  // =========================
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

  float linAx_f = meanArray(linAx_buf, MA_WINDOW);
  float linAy_f = meanArray(linAy_buf, MA_WINDOW);
  float linAz_f = meanArray(linAz_buf, MA_WINDOW);

  linAx_f = applyDeadband(linAx_f, ACCEL_DEADBAND);
  linAy_f = applyDeadband(linAy_f, ACCEL_DEADBAND);
  linAz_f = applyDeadband(linAz_f, ACCEL_DEADBAND);

  float linA_mag = sqrt(linAx_f * linAx_f + linAy_f * linAy_f + linAz_f * linAz_f);

  linA_mag_lpf = lowPass(linA_mag, linA_mag_lpf, adaptiveAlpha);
  addToBuffer(linA_mag_buf, linA_mag_lpf);

  float linA_mag_f = meanArray(linA_mag_buf, MA_WINDOW);
  linA_mag_f = applyDeadband(linA_mag_f, ACCEL_DEADBAND);

  // =========================
  // Output modes
  // =========================

  // CSV mode
  if (OUTPUT_MODE == 0) {
    Serial.print(ax_f, 4); Serial.print(",");
    Serial.print(ay_f, 4); Serial.print(",");
    Serial.print(az_f, 4); Serial.print(",");
    Serial.print(gx_f, 4); Serial.print(",");
    Serial.print(gy_f, 4); Serial.print(",");
    Serial.print(gz_f, 4); Serial.print(",");
    Serial.print(mx_f, 4); Serial.print(",");
    Serial.print(my_f, 4); Serial.print(",");
    Serial.print(mz_f, 4); Serial.print(",");
    Serial.print(roll, 2); Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(yaw, 2); Serial.print(",");
    Serial.print(linAx_f, 4); Serial.print(",");
    Serial.print(linAy_f, 4); Serial.print(",");
    Serial.print(linAz_f, 4); Serial.print(",");
    Serial.println(linA_mag_f, 4);
  }

  // Accel plot
  else if (OUTPUT_MODE == 1) {
    Serial.print(ax_f, 4); Serial.print("\t");
    Serial.print(ay_f, 4); Serial.print("\t");
    Serial.println(az_f, 4);
  }

  // Gyro plot
  else if (OUTPUT_MODE == 2) {
    Serial.print(gx_f, 4); Serial.print("\t");
    Serial.print(gy_f, 4); Serial.print("\t");
    Serial.println(gz_f, 4);
  }

  // Mag plot
  else if (OUTPUT_MODE == 3) {
    Serial.print(mx_f, 4); Serial.print("\t");
    Serial.print(my_f, 4); Serial.print("\t");
    Serial.println(mz_f, 4);
  }

  // Orientation plot + linear acceleration plot
  else if (OUTPUT_MODE == 4) {
    Serial.print(roll, 2); Serial.print("\t");
    Serial.print(pitch, 2); Serial.print("\t");
    Serial.print(yaw, 2); Serial.print("\t");
    Serial.print(linAx_f, 4); Serial.print("\t");
    Serial.print(linAy_f, 4); Serial.print("\t");
    Serial.print(linAz_f, 4); Serial.print("\t");
    Serial.println(linA_mag_f, 4);
  }

  // All labeled values
  else if (OUTPUT_MODE == 5) {
    Serial.print("ax:"); Serial.print(ax_f, 4);
    Serial.print(" ay:"); Serial.print(ay_f, 4);
    Serial.print(" az:"); Serial.print(az_f, 4);
    Serial.print(" gx:"); Serial.print(gx_f, 4);
    Serial.print(" gy:"); Serial.print(gy_f, 4);
    Serial.print(" gz:"); Serial.print(gz_f, 4);
    Serial.print(" mx:"); Serial.print(mx_f, 4);
    Serial.print(" my:"); Serial.print(my_f, 4);
    Serial.print(" mz:"); Serial.print(mz_f, 4);
    Serial.print(" roll:"); Serial.print(roll, 2);
    Serial.print(" pitch:"); Serial.print(pitch, 2);
    Serial.print(" yaw:"); Serial.print(yaw, 2);
    Serial.print(" linAx:"); Serial.print(linAx_f, 4);
    Serial.print(" linAy:"); Serial.print(linAy_f, 4);
    Serial.print(" linAz:"); Serial.print(linAz_f, 4);
    Serial.print(" linA_mag:"); Serial.println(linA_mag_f, 4);
  }

  // Human-readable wearable orientation meanings
  else if (OUTPUT_MODE == 6) {
    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print(" deg -> ");
    Serial.print(getRollMeaning(roll));

    Serial.print(" | Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" deg -> ");
    Serial.print(getPitchMeaning(pitch));

    Serial.print(" | Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" deg -> ");
    Serial.print(getYawMeaning(yaw));

    Serial.print(" | Linear Accel (m/s^2): ");
    Serial.print("X=");
    Serial.print(linAx_f, 3);
    Serial.print(" Y=");
    Serial.print(linAy_f, 3);
    Serial.print(" Z=");
    Serial.print(linAz_f, 3);
    Serial.print(" Mag=");
    Serial.println(linA_mag_f, 3);
  }

  delay(20); // ~50 Hz
}