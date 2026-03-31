#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
 
Adafruit_MPU6050 mpu;
 
void setup(void) {
  Serial.begin(115200);
  delay(1000);  // give time for Serial to initialize
 
  Serial.println("Adafruit MPU6050 test!");
 
  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10); // stuck here if sensor not found
    }
  }
  Serial.println("MPU6050 Found!");
 
  // Set accelerometer and gyro ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println("+-2G"); break;
    case MPU6050_RANGE_4_G:  Serial.println("+-4G"); break;
    case MPU6050_RANGE_8_G:  Serial.println("+-8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
  }
 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println("+-250 deg/s"); break;
    case MPU6050_RANGE_500_DEG:  Serial.println("+-500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+-1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+-2000 deg/s"); break;
  }
 
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ:  Serial.println("94 Hz"); break;
    case MPU6050_BAND_44_HZ:  Serial.println("44 Hz"); break;
    case MPU6050_BAND_21_HZ:  Serial.println("21 Hz"); break;
    case MPU6050_BAND_10_HZ:  Serial.println("10 Hz"); break;
    case MPU6050_BAND_5_HZ:   Serial.println("5 Hz"); break;
  }
 
  Serial.println("\nSetup complete!\n");
}
 
void loop() {
  // Get sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 
  // Print acceleration
  Serial.print("Acceleration X: "); Serial.print(a.acceleration.x);
  Serial.print(", Y: "); Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.println(a.acceleration.z);
 
  // Print rotation
  Serial.print("Rotation X: "); Serial.print(g.gyro.x);
  Serial.print(", Y: "); Serial.print(g.gyro.y);
  Serial.print(", Z: "); Serial.println(g.gyro.z);
 
  Serial.println(); // blank line
  delay(500);       // read twice per second
}