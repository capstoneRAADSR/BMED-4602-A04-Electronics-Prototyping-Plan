#include <math.h>

// ==========================================
// Pin
// ==========================================
const int PPG_PIN = 36;   // GPIO36 / VP on ESP32

// ==========================================
// ADC settings
// ==========================================
const int ADC_MAX = 4095;       // ESP32 is 12-bit
const float VREF = 3.3;

// ==========================================
// Timing
// ==========================================
unsigned long lastSampleMicros = 0;
const unsigned long SAMPLE_INTERVAL_US = 10000; // 100 Hz

// ==========================================
// Filtering settings
// ==========================================
const int OVERSAMPLE_COUNT = 8;
const int MEDIAN_WINDOW = 5;
const int MA_WINDOW = 8;

const float LP_ALPHA_SLOW = 0.08f;
const float LP_ALPHA_FAST = 0.22f;

// Baseline remover alpha
const float BASELINE_ALPHA = 0.01f;

// Peak detection
const float MIN_PEAK_THRESHOLD = 18.0f;      // tune this
const unsigned long REFRACTORY_MS = 300;     // ignore extra peaks for 300 ms
const int BPM_AVG_COUNT = 5;

// Signal sanity
const float SPIKE_LIMIT = 800.0f;

// ==========================================
// Buffers / states
// ==========================================
float ppgBuffer[MA_WINDOW] = {0};
int ppgBufIndex = 0;
bool ppgBufFilled = false;

unsigned long beatTimes[BPM_AVG_COUNT] = {0};
int beatIndex = 0;
int validBeatCount = 0;

float filteredSignal = 0.0f;
float baseline = 0.0f;
float acSignal = 0.0f;
float previousAC = 0.0f;
float previousOutput = 0.0f;

float bpm = 0.0f;
float ibiMs = 0.0f;  // inter-beat interval

unsigned long lastBeatTime = 0;
bool peakArmed = true;

// ==========================================
// Helpers
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

  if (size % 2 == 1) return temp[size / 2];
  return 0.5f * (temp[size / 2 - 1] + temp[size / 2]);
}

float movingAverage(float *buffer, int size, bool filled, int currentCount) {
  int count = filled ? size : currentCount;
  if (count <= 0) return 0.0f;

  float sum = 0.0f;
  for (int i = 0; i < count; i++) sum += buffer[i];
  return sum / count;
}

float adaptiveLowPass(float current, float previous) {
  float diff = fabs(current - previous);
  float alpha = (diff > 35.0f) ? LP_ALPHA_FAST : LP_ALPHA_SLOW;
  return alpha * current + (1.0f - alpha) * previous;
}

float stableADCRead() {
  float samples[MEDIAN_WINDOW];

  for (int i = 0; i < MEDIAN_WINDOW; i++) {
    long sum = 0;
    for (int j = 0; j < OVERSAMPLE_COUNT; j++) {
      sum += analogRead(PPG_PIN);
      delayMicroseconds(120);
    }
    samples[i] = sum / (float)OVERSAMPLE_COUNT;
    delayMicroseconds(150);
  }

  return getMedian(samples, MEDIAN_WINDOW);
}

void updateBPM(unsigned long beatTime) {
  beatTimes[beatIndex] = beatTime;
  beatIndex = (beatIndex + 1) % BPM_AVG_COUNT;

  if (validBeatCount < BPM_AVG_COUNT) validBeatCount++;

  if (validBeatCount < 2) return;

  float sumIBI = 0.0f;
  int intervals = 0;

  for (int i = 1; i < validBeatCount; i++) {
    int idx1 = (beatIndex - i - 1 + BPM_AVG_COUNT) % BPM_AVG_COUNT;
    int idx2 = (beatIndex - i + BPM_AVG_COUNT) % BPM_AVG_COUNT;

    unsigned long t1 = beatTimes[idx1];
    unsigned long t2 = beatTimes[idx2];

    if (t1 > 0 && t2 > 0 && t2 > t1) {
      float interval = (float)(t2 - t1);
      if (interval > 300 && interval < 2000) { // 30-200 BPM valid range
        sumIBI += interval;
        intervals++;
      }
    }
  }

  if (intervals > 0) {
    ibiMs = sumIBI / intervals;
    bpm = 60000.0f / ibiMs;
  }
}

// ==========================================
// Signal startup stabilization
// ==========================================
void initializePPG(int samples = 100) {
  Serial.println("Initializing PPG baseline...");

  float sum = 0.0f;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    float raw = stableADCRead();
    if (!isnan(raw)) {
      sum += raw;
      valid++;
    }
    delay(5);
  }

  if (valid > 0) {
    baseline = sum / valid;
    filteredSignal = baseline;
  }

  for (int i = 0; i < MA_WINDOW; i++) {
    ppgBuffer[i] = baseline;
  }
  ppgBufFilled = true;

  Serial.println("PPG baseline ready.");
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  // Optional, depends on board/core:
  // analogSetAttenuation(ADC_11db);

  for (int i = 0; i < 10; i++) {
    analogRead(PPG_PIN);
    delay(10);
  }

  initializePPG();

  lastSampleMicros = micros();

  Serial.println("PPG Ready");
  Serial.println("time_ms,raw_adc,filtered_dc,ac_ppg,peak_flag,bpm,ibi_ms");
}

void loop() {
  unsigned long nowMicros = micros();
  if (nowMicros - lastSampleMicros < SAMPLE_INTERVAL_US) return;
  lastSampleMicros = nowMicros;

  unsigned long nowMs = millis();

  // ==========================================
  // Step 1: read stable ADC
  // ==========================================
  float rawADC = stableADCRead();

  if (isnan(rawADC) || rawADC < 0 || rawADC > ADC_MAX) return;

  // ==========================================
  // Step 2: reject wild spikes
  // ==========================================
  if (fabs(rawADC - filteredSignal) > SPIKE_LIMIT) {
    rawADC = filteredSignal;
  }

  // ==========================================
  // Step 3: moving average
  // ==========================================
  ppgBuffer[ppgBufIndex] = rawADC;
  ppgBufIndex++;

  if (ppgBufIndex >= MA_WINDOW) {
    ppgBufIndex = 0;
    ppgBufFilled = true;
  }

  int count = ppgBufFilled ? MA_WINDOW : ppgBufIndex;
  float rawMA = movingAverage(ppgBuffer, MA_WINDOW, ppgBufFilled, count);

  // ==========================================
  // Step 4: adaptive low-pass for DC-ish signal
  // ==========================================
  filteredSignal = adaptiveLowPass(rawMA, filteredSignal);

  // ==========================================
  // Step 5: slow baseline tracking
  // Remove DC component to isolate pulsatile AC signal
  // ==========================================
  baseline = BASELINE_ALPHA * filteredSignal + (1.0f - BASELINE_ALPHA) * baseline;
  acSignal = filteredSignal - baseline;

  // Small deadband to flatten tiny jitter
  if (fabs(acSignal) < 1.5f) acSignal = 0.0f;

  // ==========================================
  // Step 6: peak detection
  // Look for upward crossing + local max behavior
  // ==========================================
  bool peakFlag = false;

  unsigned long timeSinceLastBeat = nowMs - lastBeatTime;

  if (timeSinceLastBeat > REFRACTORY_MS) {
    if (peakArmed &&
        acSignal > MIN_PEAK_THRESHOLD &&
        previousAC > 0 &&
        acSignal < previousAC) {
      // local max detected slightly after crest
      peakFlag = true;
      peakArmed = false;
      lastBeatTime = nowMs;
      updateBPM(nowMs);
    }

    // re-arm after signal falls back near/below zero
    if (acSignal < 2.0f) {
      peakArmed = true;
    }
  }

  previousAC = acSignal;
  previousOutput = filteredSignal;

  // ==========================================
  // Step 7: serial CSV output
  // ==========================================
  Serial.print(nowMs); Serial.print(",");
  Serial.print(rawADC, 2); Serial.print(",");
  Serial.print(filteredSignal, 2); Serial.print(",");
  Serial.print(acSignal, 2); Serial.print(",");
  Serial.print(peakFlag ? 1 : 0); Serial.print(",");
  Serial.print(bpm, 2); Serial.print(",");
  Serial.println(ibiMs, 2);
}