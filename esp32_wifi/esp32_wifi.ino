#include <WiFi.h>
#include <ESP32Ping.h>

#define SERIAL_BAUDRATE 115200

const char* ssid = "CASITA_ARCOS-2.4Ghz";
const char* password = "diegote10";

const char* host = "192.168.1.6";


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial)delay(1000);
  Serial.println("Serial started ok");

  initialize_wifi();
  scan_wifi();
  initialize_wifi_events(ssid, password);
}

void loop() {
  ping(host);
  delay(5000);
}
