const int button1Pin = 3;
const int button2Pin = 2;
const int tempSensorPin = 0;
const double maxSensorInput = 1024;
const double maxVoltage = 5;
const double scaleFactor = 100;
const double offset = 50;

int button1State;
int button2State;
int sensorInput;
double temp;

void setup() {
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  sensorInput = analogRead(tempSensorPin);
  temp = sensorInput / maxSensorInput * maxVoltage * scaleFactor - offset;
  Serial.println(temp);
  button1State = digitalRead(button1Pin);
  Serial.println(button1State);
  button2State = digitalRead(button2Pin);
  Serial.println(button2State);  
  delay(2000);
}
