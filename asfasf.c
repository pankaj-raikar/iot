#define SOIL_PIN 34   // D34
#define RAIN_PIN 26   // D26

void setup() {
  Serial.begin(115200);
  pinMode(RAIN_PIN, INPUT);
}

void loop() {
  int soil = analogRead(SOIL_PIN);
  int rain = digitalRead(RAIN_PIN);

  Serial.print("Soil Value: ");
  Serial.print(soil);
  Serial.print(" | Rain Value: ");
  Serial.println(rain);

  delay(1000);
}
