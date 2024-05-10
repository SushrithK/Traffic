#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

// Define pin numbers
#define IR_RECEIVER1_PIN 18
#define IR_RECEIVER2_PIN 19
#define IR_RECEIVER3_PIN 20
#define IR_RECEIVER4_PIN 21

const int redLEDs[] = {2, 8, 13, 22};      // GPIO pins connected to the red LEDs
const int yellowLEDs[] = {1, 7, 12, 26};   // GPIO pins connected to the yellow LEDs
const int greenLEDs[] = {0, 6, 11, 27};    // GPIO pins connected to the green LEDs

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval for non-blocking delay

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket server running on port 81

// Function declarations
void turnYellow(int num);
void normalOperation();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void handleEmergencySignal(int num);
int delayCheck(int num);
void turnGreen(int num);

void setup() {
  lcd.init();
  lcd.backlight();
  for (int i = 0; i < 4; i++) {
    pinMode(redLEDs[i], OUTPUT);
    pinMode(yellowLEDs[i], OUTPUT);
    pinMode(greenLEDs[i], OUTPUT);
  }

  pinMode(IR_RECEIVER1_PIN, INPUT);
  pinMode(IR_RECEIVER2_PIN, INPUT);
  pinMode(IR_RECEIVER3_PIN, INPUT);
  pinMode(IR_RECEIVER4_PIN, INPUT);

  Serial.begin(9600); // Initialize serial communication

  // Connect to WiFi
  WiFi.begin("Rusho", "minktan75");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  mqttClient.setServer("test.mosquitto.org", 1883);
  mqttClient.setCallback(callback);
  reconnect();

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  webSocket.loop();

  if (mqttClient.connected()) { // Only activate IR receiver functionality if MQTT client is connected
    if (digitalRead(IR_RECEIVER1_PIN) == LOW) {
      handleEmergencySignal(0);
    } else if (digitalRead(IR_RECEIVER2_PIN) == LOW) {
      handleEmergencySignal(1);
    } else if (digitalRead(IR_RECEIVER3_PIN) == LOW) {
      handleEmergencySignal(2);
    } else if (digitalRead(IR_RECEIVER4_PIN) == LOW) {
      handleEmergencySignal(3);
    } else {
      lcd.clear();
      normalOperation();
    }
  }
}

void handleEmergencySignal(int num) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ambulance");
  lcd.setCursor(0, 1);
  lcd.print("Incoming");
  turnGreen(num);
  Serial.print("IR Receiver ");
  Serial.print(num + 1);
  Serial.println(": Signal Detected");
  delayCheck(5000); // Maintain green light for 5 seconds
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT messages here
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("PicoReceiver")) {
      Serial.println("Connected to MQTT");
      mqttClient.subscribe("ambulance_topic"); // Subscribe to the MQTT topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // Handle WebSocket events here if needed
}

// Function to turn the green light on for the specified traffic light index
void turnGreen(int num) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(redLEDs[i], i == num ? LOW : HIGH);
    digitalWrite(yellowLEDs[i], LOW);
    digitalWrite(greenLEDs[i], i == num ? HIGH : LOW);
  }
}

// Function to turn the yellow light on for the specified traffic light index
void turnYellow(int num) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(redLEDs[i], i == num ? LOW : HIGH);
    digitalWrite(yellowLEDs[i], i == num ? HIGH : LOW);
    digitalWrite(greenLEDs[i], LOW);
  }
}

// Function to perform normal operation of traffic lights
void normalOperation() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    for (int i = 0; i < 4; i++) {
      turnYellow(i);
      delay(500); // Yellow light for 0.5 second
      turnGreen(i);
      delay(1000); // Green light for 1 second

      if (delayCheck(1000)) // Check if a signal is detected during the green or yellow phase
        return;
    }
    previousMillis = currentMillis;
  }
}

// Function to check for signal from IR receivers
int delayCheck(int num) {
  for (int i = 0; i < num; i++) {
    if (digitalRead(IR_RECEIVER1_PIN) == LOW ||
        digitalRead(IR_RECEIVER2_PIN) == LOW ||
        digitalRead(IR_RECEIVER3_PIN) == LOW ||
        digitalRead(IR_RECEIVER4_PIN) == LOW) {
      return 1;
    }
    delay(1);
  }
  return 0;
}
