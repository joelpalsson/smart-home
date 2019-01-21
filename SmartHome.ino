const double tempSensorPin = 0;
const double maxSensorInput = 1024;
const double maxVoltage = 5;
const double scaleFactor = 100;
const double offset = 50;
int sensorInput;
double temp;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorInput = analogRead(tempSensorPin);
  temp = sensorInput / maxSensorInput * maxVoltage * scaleFactor - offset;
  Serial.println(temp);
  delay(3000);
}
