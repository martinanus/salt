#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <MQTT.h>

const char* wifi_ssid = "CASITA_ARCOS-2.4Ghz";
const char* wifi_psw = "diegote10";

const char* mqtt_server = "salt-ma.cloud.shiftr.io";
const char* mqtt_publish_name = "SALT";

// const char* mqtt_user_name = "salt-ma";
// const char* mqtt_user_token = "1f05DXKCezK1hnck";

const char* mqtt_user_name = "salt-ma2";
const char* mqtt_user_token = "F6EXqH8wRkBnjmn4";

const char* mqtt_suscribe_topic = "salt_remote_command";
const char* mqtt_publish_topic = "salt_remote_log";


WiFiClient wifi_client;
MQTTClient mqtt_client;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600, 60000);  // UTC-3 offset

const int BUFFER_SIZE = 256;
char suscribeBuffer[BUFFER_SIZE];
char uartRxBuffer[BUFFER_SIZE];
int uartRxBufferIndex = 0;

unsigned long previousMillis = 0;
const long interval = 10000;


void mqtt_connect() {
  Serial.println("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("connecting to mqtt server...");
  while (!mqtt_client.connect(mqtt_publish_name, mqtt_user_name, mqtt_user_token)) {  
    Serial.print(".");
    delay(1000);
  }

  Serial.println("connected!");

  mqtt_client.subscribe(mqtt_suscribe_topic);
  sprintf(suscribeBuffer, "Suscribed to %s topic", mqtt_suscribe_topic);
  Serial.println(suscribeBuffer);
}

void messageReceived(String &topic, String &payload) {
  Serial2.println(payload);
  Serial.println(payload);
}

void sendDateTime(){
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  
  struct tm *ptm = gmtime((time_t *)&epochTime);
  
  char formattedDateTime[29];
  sprintf(formattedDateTime, "DATETIME:%02d/%02d/%04d %02d:%02d:%02d", 
          ptm->tm_mday, 
          ptm->tm_mon + 1, 
          ptm->tm_year + 1900, 
          ptm->tm_hour, 
          ptm->tm_min, 
          ptm->tm_sec);
  
  Serial2.println(formattedDateTime);
  Serial.println(formattedDateTime);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  
  WiFi.begin(wifi_ssid, wifi_psw);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  
  delay(2000);

  timeClient.begin();
  sendDateTime();
  Serial.println("DateTime sended");


  mqtt_client.begin(mqtt_server, wifi_client);
  mqtt_client.onMessage(messageReceived);
  mqtt_connect();
}

void loop() {
  mqtt_client.loop();

  // if (!mqtt_client.connected()) {
  //   mqtt_connect();
  // }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendDateTime();
  }


  while (Serial2.available() > 0) {
    char incomingByte = Serial2.read();

    if (incomingByte == '\n') {
      uartRxBuffer[uartRxBufferIndex] = '\0';      
      mqtt_client.publish(mqtt_publish_topic, uartRxBuffer);
      uartRxBufferIndex = 0;
    } else {
      if (uartRxBufferIndex < BUFFER_SIZE - 1) {
        uartRxBuffer[uartRxBufferIndex++] = incomingByte;
      } else {
        uartRxBufferIndex = 0;
      }
    }
  }
}