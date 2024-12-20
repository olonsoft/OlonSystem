#include <Arduino.h>
#include <OlonSystem.h>

#ifdef OLON_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

Olon::SystemClass espSystem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  espSystem.init();

#ifdef OLON_JSON_SUPPORT
  JsonDocument doc;
  espSystem.toJson(doc.to<JsonObject>());
  serializeJsonPretty(doc, Serial);
#endif
}

void loop() {
  Serial.println("Restarting in 15 secs");
  delay(15000);
  ESP.restart();
}
