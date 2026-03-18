void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// create sensor object
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 + LSM9DS1 IMU Test");

  // start I2C
  Wire.begin(21, 22);

  // initialize sensor
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM9DS1 Found!");

  // configure ranges
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void loop() {

  // read sensor
  lsm.read();

  // accelerometer
  Serial.print("Accel X: ");
  Serial.print(lsm.accelData.x);
  Serial.print("  Y: ");
  Serial.print(lsm.accelData.y);
  Serial.print("  Z: ");
  Serial.println(lsm.accelData.z);

  // gyroscope
  Serial.print("Gyro X: ");
  Serial.print(lsm.gyroData.x);
  Serial.print("  Y: ");
  Serial.print(lsm.gyroData.y);
  Serial.print("  Z: ");
  Serial.println(lsm.gyroData.z);

  // magnetometer
  Serial.print("Mag X: ");
  Serial.print(lsm.magData.x);
  Serial.print("  Y: ");
  Serial.print(lsm.magData.y);
  Serial.print("  Z: ");
  Serial.println(lsm.magData.z);

  Serial.println("----------------------");

  delay(500);
}