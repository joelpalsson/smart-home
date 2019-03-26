#include <SoftwareSerial.h>

SoftwareSerial btSerial(6, 5); // RX, TX

const byte led1Pin = 13;
const byte led2Pin = 12;
const byte led3Pin = 11;
const byte button1Pin = 3;
const byte button2Pin = 2;
const byte sensorPin = 0;

const byte temperatureKey = 1;
const byte windowsKey = 2;
const byte alarmKey = 3;

byte temperature;
bool windowsOpen;
bool alarmOn;

byte button1State = 0;
byte button1PushCounter = 0;
byte button2State = 0;
byte button2PushCounter = 0;


void setup() {
  // Setup the pins
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);

  // Setup the serial connections
  Serial.begin(9600);
  btSerial.begin(9600);
  delay(200);

  // Read the sensors
  temperature = analogRead(sensorPin);
  windowsOpen = false;
  alarmOn = false;
}

void loop() {
  // Check for incoming data
  if (btSerial.available()) {
    // Read the data
    char command = btSerial.read();
    Serial.println("command received: " + String(command));
    // Handle the command
    handleCommand(command);
  }

  // Measure the temperature
  byte newTemperature = getTemperature(analogRead(sensorPin));
  // Check if the temperature has changed
  if (newTemperature != temperature) {
    temperature = newTemperature;
    Serial.println("temperature changed!");
    // Send the temperature
    sendData(temperatureKey, temperature);
  }

  // Check if the button simulating the window status has been pressed
  if (buttonPressed(button1Pin, &button1State, &button1PushCounter)) {
    // Update the window status
    windowsOpen = !windowsOpen;
    Serial.println("windows state changed!");
    // Send the window status
    sendData(windowsKey, (byte) windowsOpen);
  }

  // Check if the button simulating the alarm status has been pressed
  if (buttonPressed(button2Pin, &button2State, &button2PushCounter)) {
    // Update the alarm status
    alarmOn = !alarmOn;
    Serial.println("alarm state changed!");
    // Send the alarm status
    sendData(alarmKey, (byte) alarmOn);
  }

  // Wait a second
  delay(1000);
}

byte getTemperature(int sensorInput) {
  const double maxSensorInput = 1024;
  const double maxVoltage = 5;
  const double scaleFactor = 100;
  const double offset = 50;
  return (byte) (sensorInput / maxSensorInput * maxVoltage * scaleFactor - offset);
}

void handleCommand(char command) {
  switch (command) {
    case '0':
      // Connection established, send sensor data
      sendData(temperatureKey, temperature);
      sendData(windowsKey, (byte) windowsOpen);
      sendData(alarmKey, (byte) alarmOn);
    case '1':
      // Toggle the kitchen light
      digitalWrite(led1Pin, !digitalRead(led1Pin));
      break;
    case '2':
      // Toggle the bedroom lightt
      digitalWrite(led2Pin, !digitalRead(led2Pin));
      break;
    case '3':
      // Toggle the living room light
      digitalWrite(led3Pin, !digitalRead(led3Pin));
  }
}

int sendData(byte key, byte value) {
  byte sendBuffer[2];
  sendBuffer[0] = key;
  sendBuffer[1] = value;
  return btSerial.write(sendBuffer, 2);
}

bool buttonPressed(int buttonPin, byte *buttonState, byte *buttonPushCounter) {
  bool buttonPressed = false;
  byte newButtonState = digitalRead(buttonPin);
  if (newButtonState != *buttonState) {
    *buttonState = newButtonState;
    *buttonPushCounter = *buttonPushCounter + 1;
    if (*buttonPushCounter == 2) {
      *buttonPushCounter = 0;
      buttonPressed = true;
    }
  }
  return buttonPressed;
}
