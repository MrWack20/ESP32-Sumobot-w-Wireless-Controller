#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  delay(2000); // Longer delay to ensure everything stabilizes
  
  Serial.println("\n\n======================");
  Serial.println("Starting MAC Address Read...");
  
  // Try to initialize WiFi
  WiFi.mode(WIFI_MODE_STA);
  delay(100);
  
  // Check if WiFi is ready
  Serial.print("WiFi Mode: ");
  Serial.println(WiFi.getMode());
  
  // Get MAC address
  String mac = WiFi.macAddress();
  
  if (mac == "" || mac == "00:00:00:00:00:00") {
    Serial.println("ERROR: MAC Address not available!");
    Serial.println("Trying to disconnect and reconnect WiFi...");
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_MODE_STA);
    delay(500);
    mac = WiFi.macAddress();
  }
  
  Serial.print("MAC Address: ");
  Serial.println(mac);
  Serial.println("======================\n");
}

void loop() {
  // Keep it empty
}