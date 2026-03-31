const int GSR = 34;
int sensorValue = 0;
int gsr_average = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("GSR Sensor Ready!");
}

void loop() {
  long sum = 0;
  for(int i = 0; i < 10; i++) {
    sensorValue = analogRead(GSR);
    sum += sensorValue;
    delay(5);
  }
  gsr_average = sum / 10;
  Serial.print("GSR: ");
  Serial.println(gsr_average);
  delay(100);
}