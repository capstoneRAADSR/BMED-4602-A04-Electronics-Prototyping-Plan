#include <math.h>

// ==========================================
// Thermistor pin
// ==========================================
const int THERM_PIN = A0;   // Arduino Uno analog input

// ==========================================
// Thermistor / divider parameters
// ==========================================
// Typical 10k NTC thermistor setup
const float SERIES_RESISTOR = 10000.0;     // fixed resistor value in ohms
const float NOMINAL_RESISTANCE = 10000.0;  // thermistor resistance at 25C
const float NOMINAL_TEMPERATURE = 25.0;    // 25C reference
const float BETA_COEFFICIENT = 3950.0;     // common beta value (check your thermistor datasheet)

// ADC / supply
const float VCC = 5.0;                     // Uno typically 5V
const int ADC_MAX = 1023;                  // 10-bit ADC

// ==========================================
// Timing
// ==========================================
unsigned long lastMillis = 0;
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz output

// ==========================================
// Filtering settings
// ==========================================
const int OVERSAMPLE_COUNT = 15;    // average many ADC reads
const int MEDIAN_WINDOW = 5;        // median of small batch
const int MA_WINDOW = 8;            // moving average window

const float LOW_PASS_ALPHA_SLOW = 0.12;
const float LOW_PASS_ALPHA_FAST = 0.35;

// Outlier rejection threshold in Celsius
const float TEMP_SPIKE_LIMIT_C = 3.0;

// Deadband threshold for tiny fluctuations
const float TEMP_DEADBAND_C = 0.03;

// ==========================================
// Calibration
// ==========================================
// If you compare against a trusted thermometer and want to shift final result:
float tempCalibrationOffsetC = 0.0;

// ==========================================
// Buffers / states
// ==========================================
float tempBuffer[MA_WINDOW] = {0};
int tempBufIndex = 0;
bool bufferFilled = false;

float tempFiltered = 25.0;
float lastGoodTemp = 25.0;

// ==========================================
// Helper: insertion sort for median
// ==========================================
void sortArray(float *arr, int size) {
  for (int i = 1; i < size; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

float getMedian(float *arr, int size) {
  float temp[size];
  for (int i = 0; i < size; i++) temp[i] = arr[i];

  sortArray(temp, size);

  if (size % 2 == 1) {
    return temp[size / 2];
  } else {
    return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
  }
}

float movingAverage(float *buffer, int size, bool filled, int currentIndex) {
  int count = filled ? size : currentIndex;
  if (count <= 0) return buffer[0];

  float sum = 0.0;
  for (int i = 0; i < count; i++) sum += buffer[i];
  return sum / count;
}

float adaptiveLowPass(float current, float previous) {
  float diff = fabs(current - previous);
  float alpha = (diff > 0.4) ? LOW_PASS_ALPHA_FAST : LOW_PASS_ALPHA_SLOW;
  return alpha * current + (1.0 - alpha) * previous;
}

float applyDeadband(float value, float previous, float threshold) {
  if (fabs(value - previous) < threshold) return previous;
  return value;
}

// ==========================================
// Read ADC with oversampling + median
// ==========================================
float readStableADC() {
  float samples[MEDIAN_WINDOW];

  for (int i = 0; i < MEDIAN_WINDOW; i++) {
    long sum = 0;
    for (int j = 0; j < OVERSAMPLE_COUNT; j++) {
      sum += analogRead(THERM_PIN);
      delayMicroseconds(300);
    }
    samples[i] = sum / (float)OVERSAMPLE_COUNT;
    delay(2);
  }

  return getMedian(samples, MEDIAN_WINDOW);
}

// ==========================================
// Convert ADC reading to thermistor resistance
// Divider assumed:
// VCC -> fixed resistor -> analog node -> thermistor -> GND
// ==========================================
float adcToResistance(float adcValue) {
  if (adcValue <= 0.5) return 9999999.0;              // protect divide-by-zero
  if (adcValue >= ADC_MAX - 0.5) return 0.1;          // protect edge case

  float voltage = (adcValue / ADC_MAX) * VCC;

  // Using divider equation:
  // Vout = VCC * (R_therm / (R_fixed + R_therm))
  // => R_therm = R_fixed * Vout / (VCC - Vout)
  float resistance = SERIES_RESISTOR * voltage / (VCC - voltage);

  return resistance;
}

// ==========================================
// Convert resistance to Celsius using Beta formula
// ==========================================
float resistanceToCelsius(float resistance) {
  float steinhart;

  steinhart = resistance / NOMINAL_RESISTANCE;              // (R/Ro)
  steinhart = log(steinhart);                               // ln(R/Ro)
  steinhart /= BETA_COEFFICIENT;                            // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);        // + (1/To)
  steinhart = 1.0 / steinhart;                              // invert
  steinhart -= 273.15;                                      // K -> C

  return steinhart;
}

// ==========================================
// Optional startup baseline stabilization
// ==========================================
void initializeThermistorBaseline(int samples = 20) {
  Serial.println("Stabilizing thermistor baseline...");

  float sum = 0.0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    float adc = readStableADC();
    float resistance = adcToResistance(adc);
    float tempC = resistanceToCelsius(resistance);

    if (!isnan(tempC) && tempC > -40 && tempC < 150) {
      sum += tempC;
      valid++;
    }
    delay(50);
  }

  if (valid > 0) {
    tempFiltered = sum / valid;
    lastGoodTemp = tempFiltered;
  }

  for (int i = 0; i < MA_WINDOW; i++) {
    tempBuffer[i] = tempFiltered;
  }
  bufferFilled = true;

  Serial.println("Baseline ready.");
}

void setup() {
  Serial.begin(115200);

  // On Uno, analogRead defaults are fine.
  // Throw away a few first reads for stability.
  for (int i = 0; i < 10; i++) {
    analogRead(THERM_PIN);
    delay(10);
  }

  initializeThermistorBaseline();

  Serial.println("Thermistor Ready");
  Serial.println("time_ms,adc_raw,resistance_ohm,tempC_raw,tempC_ma,tempC_filtered,tempF,kelvin");
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis < SAMPLE_INTERVAL_MS) return;
  lastMillis = now;

  // ==========================================
  // Step 1: stable ADC reading
  // ==========================================
  float adcRaw = readStableADC();

  // ==========================================
  // Step 2: convert to resistance
  // ==========================================
  float resistance = adcToResistance(adcRaw);

  // ==========================================
  // Step 3: convert to raw temperature
  // ==========================================
  float tempC_raw = resistanceToCelsius(resistance);

  // Sanity check
  if (isnan(tempC_raw) || tempC_raw < -40.0 || tempC_raw > 150.0) {
    Serial.println("Sensor error");
    return;
  }

  // Apply calibration offset
  tempC_raw += tempCalibrationOffsetC;

  // ==========================================
  // Step 4: outlier rejection
  // Reject sudden impossible jumps
  // ==========================================
  if (fabs(tempC_raw - lastGoodTemp) > TEMP_SPIKE_LIMIT_C) {
    tempC_raw = lastGoodTemp;
  } else {
    lastGoodTemp = tempC_raw;
  }

  // ==========================================
  // Step 5: moving average
  // ==========================================
  tempBuffer[tempBufIndex] = tempC_raw;
  tempBufIndex++;

  if (tempBufIndex >= MA_WINDOW) {
    tempBufIndex = 0;
    bufferFilled = true;
  }

  int count = bufferFilled ? MA_WINDOW : tempBufIndex;
  float tempC_ma = movingAverage(tempBuffer, MA_WINDOW, bufferFilled, count);

  // ==========================================
  // Step 6: adaptive low-pass filter
  // ==========================================
  tempFiltered = adaptiveLowPass(tempC_ma, tempFiltered);

  // ==========================================
  // Step 7: deadband
  // Tiny changes get flattened out
  // ==========================================
  static float previousOutput = tempFiltered;
  tempFiltered = applyDeadband(tempFiltered, previousOutput, TEMP_DEADBAND_C);
  previousOutput = tempFiltered;

  // ==========================================
  // Final converted units
  // ==========================================
  float tempF = tempFiltered * 9.0 / 5.0 + 32.0;
  float tempK = tempFiltered + 273.15;

  // ==========================================
  // CSV serial output
  // ==========================================
  Serial.print(now); Serial.print(",");
  Serial.print(adcRaw, 2); Serial.print(",");
  Serial.print(resistance, 2); Serial.print(",");
  Serial.print(tempC_raw, 3); Serial.print(",");
  Serial.print(tempC_ma, 3); Serial.print(",");
  Serial.print(tempFiltered, 3); Serial.print(",");
  Serial.print(tempF, 3); Serial.print(",");
  Serial.println(tempK, 3);
}