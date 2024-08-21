#include <WiFi.h>
#include <MQTT.h>

const char* wifi_ssid = "CASITA_ARCOS-2.4Ghz";
const char* wifi_psw = "diegote10";

const char* mqtt_server = "salt-ma.cloud.shiftr.io";
const char* mqtt_publish_name = "TinchOoX10";
const char* mqtt_user_name = "salt-ma";
const char* mqtt_user_token = "1f05DXKCezK1hnck";


WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;
int i = 0;
char buffer[100];

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_publish_name, mqtt_user_name, mqtt_user_token)) {  
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("games");
}

void messageReceived(String &topic, String &payload) {
  Serial.println(topic + ": " + payload);
}

void setup() {
  Serial.begin(115200);

  // start wifi and mqtt
  WiFi.begin(wifi_ssid, wifi_psw);
  client.begin(mqtt_server, net);
  client.onMessage(messageReceived);

  connect();
}

void loop() {
  client.loop();
  delay(10);

  // check if connected
  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    sprintf(buffer, "cs- %d\n", i);
    client.publish("/games", buffer);
    client.publish("/tree_fall", "!sound");
    i++;
  }
}