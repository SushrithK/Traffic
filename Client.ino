#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Rusho";
const char* password = "minktan75";
const char* mqttServer = "test.mosquitto.org"; // MQTT broker address
const int mqttPort = 1883;
const char* mqttTopic = "ambulance_topic"; // MQTT topic to publish to

const char* driverName = "Ramesh"; // Change this to the actual driver name
const char* vehicleNumber = "KA51MK3024"; // Change this to the actual vehicle number

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600);
  delay(1000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  mqttClient.setServer(mqttServer, mqttPort);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Create a JSON message with driver name and vehicle number
  StaticJsonDocument<200> doc;
  doc["driverName"] = driverName;
  doc["vehicleNumber"] = vehicleNumber;
  
  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Publish the JSON message to the MQTT topic
  mqttClient.publish(mqttTopic, jsonString.c_str());

  delay(5000); // Publish every 5 seconds
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("PicoTransmitter")) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}