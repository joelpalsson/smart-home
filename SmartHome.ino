#include <SoftwareSerial.h>

SoftwareSerial btSerial(6, 5); // RX, TX

const int led1Pin = 13;
const int led2Pin = 12;
const int led3Pin = 11;
const int button1Pin = 3;
const int button2Pin = 2;
const int sensorPin = 0;

const byte temperatureKey = 1;
const byte windowsKey = 2;
const byte alarmKey = 3;

byte prevTemperature;
byte prevButton1State;
byte button1PushCounter;
bool windowsOpen;
byte prevButton2State;
byte button2PushCounter;
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
    char command = btSerial.read();
    Serial.println(command);
    // Handle the command
    handleCommand(command);
  }

  // Measure the temperature
  byte temperature = getTemperature(analogRead(sensorPin));
  // Check if the temperature has changed
  if (temperature != prevTemperature) {
    Serial.println("temperature changed!");
    // Send the temperature to the Android unit 
    nbrBytesSent = sendData(temperatureKey, temperature);
    prevTemperature = temperature;
  }
  
  // Check if the button simulating the window status has been pressed
  if (buttonPressed(button1Pin, &prevButton1State, &button1PushCounter)) {
    // Update the window status
    windowsOpen = !windowsOpen;
    Serial.println("windows state changed!");
    // Send the window status to the Android unit
    nbrBytesSent = sendData(windowsKey, (byte) windowsOpen);
  }

  // Check if the button simulating the alarm status has been pressed
  if (buttonPressed(button2Pin, &prevButton2State, &button2PushCounter)) {
    // Update the alarm status
    alarmOn = !alarmOn;
    Serial.println("alarm state changed!");
    // Send the alarm status to the Android unit
    nbrBytesSent = sendData(alarmKey, (byte) alarmOn); 
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
      sendData(temperatureKey, getTemperature(analogRead(sensorPin)));
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

int sendData(byte key, byte value) {
  sendBuffer[0] = key;
  sendBuffer[1] = value;
  return btSerial.write(sendBuffer, 2);
}

bool buttonPressed(int buttonPin, byte *prevButtonState, byte *buttonPushCounter) {
  bool buttonPressed = false;
  byte buttonState = digitalRead(buttonPin);
  if (buttonState != *prevButtonState) {
    *buttonPushCounter = *buttonPushCounter + 1;
    if (*buttonPushCounter == 2) {
      *buttonPushCounter = 0;
      buttonPressed = true;
    }
    *prevButtonState = buttonState;
  }
  return buttonPressed;
}
