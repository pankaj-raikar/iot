#include <WiFi.h>
#include <PubSubClient.h>

// ---------- PINS ----------
#define SOIL_PIN 34   // D34 (Analog)
#define RAIN_PIN 26   // D26 (Digital)

// ---------- WIFI ----------
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------- MQTT BROKER ----------
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;

// ---------- MQTT TOPICS ----------
const char* TOPIC_SOIL = "irrigation/soil";
const char* TOPIC_RAIN = "irrigation/rain";

WiFiClient espClient;
PubSubClient mqtt(espClient);

void connectWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  while (!mqtt.connected()) {
    Serial.print("Connecting MQTT...");
    String clientId = "esp32-irrig-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying...");
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RAIN_PIN, INPUT);

  connectWiFi();
  connectMQTT();
}

unsigned long lastSend = 0;

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    int soil = analogRead(SOIL_PIN);     // 0..4095
    int rain = digitalRead(RAIN_PIN);    // usually 1=dry, 0=wet (depends on module)

    // Serial monitor check
    Serial.print("Soil Value: ");
    Serial.print(soil);
    Serial.print(" | Rain Value: ");
    Serial.println(rain);

    // Publish to MQTT
    mqtt.publish(TOPIC_SOIL, String(soil).c_str());
    mqtt.publish(TOPIC_RAIN, String(rain).c_str());
  }
}
