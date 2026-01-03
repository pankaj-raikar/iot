#include <WiFi.h>
#include <PubSubClient.h>

// ---------------- PINS ----------------
#define SOIL_PIN   34   // D34 - Analog
#define RAIN_PIN   26   // D26 - Digital
#define RELAY_PIN  27   // D27 - Relay IN

// ---------------- WIFI ----------------
const char* WIFI_SSID = "Unknown";
const char* WIFI_PASS = "12345678";

// ---------------- MQTT ----------------
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;

// ---------------- TOPICS ----------------
const char* TOPIC_SOIL      = "irrigation/soil";
const char* TOPIC_RAIN      = "irrigation/rain";
const char* TOPIC_PUMPCMD   = "irrigation/pumpCmd";
const char* TOPIC_PUMPSTATE = "irrigation/pumpState";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------------- FUNCTIONS ----------------
void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT Message [");
  Serial.print(topic);
  Serial.print("] : ");
  Serial.println(msg);

  if (String(topic) == TOPIC_PUMPCMD) {
    if (msg == "1") {
      digitalWrite(RELAY_PIN, LOW);   // Relay ON (active LOW)
      mqtt.publish(TOPIC_PUMPSTATE, "1");
      Serial.println("Pump ON");
    } else {
      digitalWrite(RELAY_PIN, HIGH);  // Relay OFF
      mqtt.publish(TOPIC_PUMPSTATE, "0");
      Serial.println("Pump OFF");
    }
  }
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  while (!mqtt.connected()) {
    Serial.print("Connecting MQTT...");
    String clientId = "ESP32-Irrigation-" + String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");
      mqtt.subscribe(TOPIC_PUMPCMD);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying...");
      delay(2000);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 STARTED");

  pinMode(RAIN_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH); // Pump OFF at start

  connectWiFi();
  connectMQTT();
}

// ---------------- LOOP ----------------
unsigned long lastSend = 0;

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    int soil = analogRead(SOIL_PIN);   // 0–4095
    int rain = digitalRead(RAIN_PIN);  // 1=dry, 0=wet

    Serial.print("Soil: ");
    Serial.print(soil);
    Serial.print(" | Rain: ");
    Serial.println(rain);

    mqtt.publish(TOPIC_SOIL, String(soil).c_str());
    mqtt.publish(TOPIC_RAIN, String(rain).c_str());
  }
}
