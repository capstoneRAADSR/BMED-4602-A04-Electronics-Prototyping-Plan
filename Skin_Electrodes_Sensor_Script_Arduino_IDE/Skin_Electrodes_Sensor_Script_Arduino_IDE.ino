#include <math.h>

const int EDA_PIN = A0;

// Divider resistor (very important)
const float SERIES_RESISTOR = 100000.0; // 100k resistor typical for GSR

const float VCC = 5.0;
const int ADC_MAX = 1023;

// ==============================
// Sampling parameters
// ==============================

const int OVERSAMPLE = 15;
const int MEDIAN_WINDOW = 5;
const int MA_WINDOW = 10;

unsigned long lastSample = 0;
const int SAMPLE_INTERVAL = 50; // 20 Hz

// ==============================
// Filtering
// ==============================

float edaBuffer[MA_WINDOW];
int bufferIndex = 0;

float filteredEDA = 0;
float baselineEDA = 0;

const float LOWPASS_SLOW = 0.10;
const float LOWPASS_FAST = 0.35;

const float SPIKE_LIMIT = 50.0;

// ==============================
// Utility Functions
// ==============================

void sortArray(float *arr, int n)
{
  for (int i = 1; i < n; i++)
  {
    float key = arr[i];
    int j = i - 1;

    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j--;
    }

    arr[j + 1] = key;
  }
}

float medianFilter(float *arr, int size)
{
  float temp[size];

  for (int i = 0; i < size; i++)
    temp[i] = arr[i];

  sortArray(temp, size);

  return temp[size / 2];
}

float movingAverage(float *buf)
{
  float sum = 0;

  for (int i = 0; i < MA_WINDOW; i++)
    sum += buf[i];

  return sum / MA_WINDOW;
}

float adaptiveLowPass(float current, float previous)
{
  float diff = fabs(current - previous);

  float alpha = diff > 5 ? LOWPASS_FAST : LOWPASS_SLOW;

  return alpha * current + (1 - alpha) * previous;
}

// ==============================
// ADC stable read
// ==============================

float stableADC()
{
  float samples[MEDIAN_WINDOW];

  for (int i = 0; i < MEDIAN_WINDOW; i++)
  {
    long sum = 0;

    for (int j = 0; j < OVERSAMPLE; j++)
    {
      sum += analogRead(EDA_PIN);
      delayMicroseconds(200);
    }

    samples[i] = sum / (float)OVERSAMPLE;
  }

  return medianFilter(samples, MEDIAN_WINDOW);
}

// ==============================
// Convert ADC to skin resistance
// ==============================

float adcToResistance(float adc)
{
  float voltage = adc * VCC / ADC_MAX;

  float resistance = SERIES_RESISTOR * voltage / (VCC - voltage);

  return resistance;
}

// ==============================
// Convert resistance to conductance
// (standard EDA measure)
// ==============================

float resistanceToConductance(float resistance)
{
  float conductance = (1.0 / resistance) * 1000000.0;

  return conductance;
}

// ==============================
// Baseline calibration
// ==============================

void calibrateEDA()
{
  Serial.println("Calibrating baseline...");

  float sum = 0;

  for (int i = 0; i < 40; i++)
  {
    float adc = stableADC();

    float R = adcToResistance(adc);

    float eda = resistanceToConductance(R);

    sum += eda;

    delay(100);
  }

  baselineEDA = sum / 40.0;

  filteredEDA = baselineEDA;

  Serial.print("Baseline EDA: ");
  Serial.println(baselineEDA);
}

// ==============================
// Setup
// ==============================

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < MA_WINDOW; i++)
    edaBuffer[i] = 0;

  calibrateEDA();

  Serial.println("EDA Ready");
  Serial.println("time_ms,adc,resistance,conductance_uS,filteredEDA,EDA_change");
}

// ==============================
// Loop
// ==============================

void loop()
{
  if (millis() - lastSample < SAMPLE_INTERVAL)
    return;

  lastSample = millis();

  float adc = stableADC();

  float resistance = adcToResistance(adc);

  float eda = resistanceToConductance(resistance);

  // spike rejection
  if (fabs(eda - filteredEDA) > SPIKE_LIMIT)
    eda = filteredEDA;

  // moving average
  edaBuffer[bufferIndex] = eda;

  bufferIndex++;

  if (bufferIndex >= MA_WINDOW)
    bufferIndex = 0;

  float edaMA = movingAverage(edaBuffer);

  // adaptive lowpass
  filteredEDA = adaptiveLowPass(edaMA, filteredEDA);

  float edaChange = filteredEDA - baselineEDA;

  // =====================
  // Serial Output
  // =====================

  Serial.print(millis());
  Serial.print(",");

  Serial.print(adc);
  Serial.print(",");

  Serial.print(resistance);
  Serial.print(",");

  Serial.print(eda);
  Serial.print(",");

  Serial.print(filteredEDA);
  Serial.print(",");

  Serial.println(edaChange);
}