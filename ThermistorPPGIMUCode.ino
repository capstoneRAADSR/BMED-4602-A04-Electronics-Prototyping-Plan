#include <PulseSensorPlayground.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int OUTPUT_TYPE = SERIAL_PLOTTER;
const int PULSE_INPUT = 36;
const int PULSE_BLINK = 2;
const int PULSE_FADE = 5;
const int THRESHOLD = 1900;
PulseSensorPlayground pulseSensor;

const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;
float lastTempC = 0.0;
float lastTempF = 0.0;

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);  // give time for Serial to initialize

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  if (!pulseSensor.begin()) {
    for (;;) {
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      Serial.println('!');
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }

  sensors.begin();

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

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
  // --- PulseSensor internal processing ---
  if (pulseSensor.UsingHardwareTimer) {
    delay(20);
  } else {
    if (pulseSensor.sawNewSample()) {
      if ((pulseSensor.samplesUntilReport--) == 0) {
        pulseSensor.samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
      }
    }
  }

  // FAST: raw signal every loop
  int signal = pulseSensor.getLatestSample();

  // --- Get latest BPM/IBI (always available) ---
  int bpm = pulseSensor.getBeatsPerMinute();
  int ibi = pulseSensor.getInterBeatIntervalMs();

  // --- SLOW: update temperature every 1 sec ---
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;

    sensors.requestTemperatures();
    lastTempC = sensors.getTempCByIndex(0);
    lastTempF = sensors.getTempFByIndex(0);
  }

  // Print EVERYTHING every loop (for smooth plotting)
  Serial.print(signal);      // raw waveform
  Serial.print(" ");
  Serial.print(bpm);
  Serial.print(" ");
  Serial.print(ibi);    // scaled
  Serial.print(" ");
  Serial.print(lastTempC);
  Serial.print(" ");
  Serial.println(lastTempF); // ONLY ONE println

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