#include <SoftwareSerial.h>

SoftwareSerial btSerial(6, 5); // RX, TX

const int led1Pin = 13;
const int led2Pin = 12;
const int led3Pin = 11;
const int button1Pin = 3;
const int button2Pin = 2;
const int temperatureSensorPin = 0;
const double maxSensorInput = 1024;
const double maxVoltage = 5;
const double scaleFactor = 100;
const double offset = 50;

char command;
int sensorInput;
byte temperature;
byte button1State;
byte button2State;
byte sendBuffer[3];
byte nbrBytesSent;

void setup() {
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
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
  
  // Send sensor input
  sendBuffer[0] = temperature;
  sendBuffer[1] = button1State;
  sendBuffer[2] = button2State;
  nbrBytesSent = btSerial.write(sendBuffer, 3);

  // Check for incoming data
  if (btSerial.available()) {
    command = btSerial.read();
    Serial.println(command);
    switch (command) {
      case '1':
        digitalWrite(led1Pin, !digitalRead(led1Pin));
        break;
      case '2':
        digitalWrite(led2Pin, !digitalRead(led2Pin));
        break;
      case '3':
        digitalWrite(led3Pin, !digitalRead(led3Pin));
    }
  }
  
  delay(1000);
}
