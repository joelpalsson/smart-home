#include <SoftwareSerial.h>

SoftwareSerial btSerial(6, 5); // RX, TX

const int led1Pin = 13;
const int led2Pin = 12;
const int led3Pin = 11;
const int button1Pin = 3;
const int button2Pin = 2;
const int temperatureSensorPin = 0;

const byte temperatureKey = 1;
const byte windowsKey = 2;
const byte alarmKey = 3;

char command;
int sensorInput;
byte prevSensorInput;
byte temperature;
byte prevTemperature;
byte button1State;
byte prevButton1State;
byte button1PushCounter;
bool windowsOpen;
byte button2State;
byte button2PushCounter;
byte prevButton2State;
bool alarmOn;
byte sendBuffer[2];
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

  // Check for incoming data
  if (btSerial.available()) {
    command = btSerial.read();
    Serial.println(command);
    switch (command) {
      case '0':
        sendData(temperatureKey, getTemperature(analogRead(temperatureSensorPin)));
        sendData(windowsKey, (byte) windowsOpen);
        sendData(alarmKey, (byte) alarmOn);
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

  sensorInput = analogRead(temperatureSensorPin);
  //Serial.println(sensorInput);
  if (sensorInput != prevSensorInput) {
    temperature = getTemperature(sensorInput);
    //Serial.println(temperature);
    if (temperature != prevTemperature) {
      // Send temperature
      nbrBytesSent = sendData(temperatureKey, temperature);
      //Serial.println(nbrBytesSent);
    }
    temperature = prevTemperature;
  }
  prevSensorInput = sensorInput;

  button1State = digitalRead(button1Pin);
  if (button1State != prevButton1State) {
    button1PushCounter++;
    if (button1PushCounter % 2 == 0) {
      button1PushCounter = 0;
      windowsOpen = !windowsOpen;
      Serial.println("windows state changed!");
      // Send windows status
      nbrBytesSent = sendData(windowsKey, (byte) windowsOpen);
    }
  }
  prevButton1State = button1State;

  button2State = digitalRead(button2Pin);
  //Serial.println(button2State);
  if (button2State != prevButton2State) {
    button2PushCounter++;
    if (button2PushCounter % 2 == 0) {
      button2PushCounter = 0;
      alarmOn = !alarmOn;
      Serial.println("alarm state changed!");
      // Send alarm status
      nbrBytesSent = sendData(alarmKey, (byte) alarmOn);
    }
  }
  prevButton2State = button2State;

  delay(1000);
}

byte getTemperature(int sensorInput) {
  const double maxSensorInput = 1024;
  const double maxVoltage = 5;
  const double scaleFactor = 100;
  const double offset = 50;
  return (byte) (sensorInput / maxSensorInput * maxVoltage * scaleFactor - offset);
}

int sendData(byte key, byte value) {
  sendBuffer[0] = key;
  sendBuffer[1] = value;
  return btSerial.write(sendBuffer, 2);
}
