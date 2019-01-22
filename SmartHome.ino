#include <SoftwareSerial.h>

SoftwareSerial btSerial(6, 5); // RX, TX

const int button1Pin = 3;
const int button2Pin = 2;
const int temperatureSensorPin = 0;
const double maxSensorInput = 1024;
const double maxVoltage = 5;
const double scaleFactor = 100;
const double offset = 50;

int sensorInput;
byte temperature;
byte button1State;
byte button2State;
byte sendBuffer[3];
byte nbrBytesSent;

void setup() {
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  Serial.begin(9600);
  btSerial.begin(9600);
  delay(200);
}

void loop() {
  sensorInput = analogRead(temperatureSensorPin);
  temperature = (byte) (sensorInput / maxSensorInput * maxVoltage * scaleFactor - offset);
  //Serial.println(temperature);
  button1State = digitalRead(button1Pin);
  //Serial.println(button1State);
  button2State = digitalRead(button2Pin);
  //Serial.println(button2State);  
  sendBuffer[0] = temperature;
  sendBuffer[1] = button1State;
  sendBuffer[2] = button2State;
  nbrBytesSent = btSerial.write(sendBuffer, 3);
  delay(1000);
}
