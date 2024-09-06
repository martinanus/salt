#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <MQTT.h>

const char* wifi_ssid1 = "CASITA_ARCOS-2.4Ghz";
const char* wifi_psw1 = "diegote10";

const char* wifi_ssid2 = "TinchOoXMobile";
const char* wifi_psw2 = "diegoarmando";

char * active_ssid;

const char* mqtt_server = "salt-ma.cloud.shiftr.io";
const char* mqtt_publish_name = "SALT";

const char* mqtt_user_name = "salt-ma";
const char* mqtt_user_token = "1f05DXKCezK1hnck";

const char* mqtt_suscribe_topic = "salt_remote_command";
const char* mqtt_publish_log_topic = "salt_remote_log";
const char* mqtt_publish_ack_topic = "salt_remote_ack";



WiFiClient wifi_client;
MQTTClient mqtt_client;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600, 60000);  // UTC-3 offset

const int BUFFER_SIZE = 256;
const long DATETIME_TX_INTERVAL = 10000;  // 10s
const long WIFI_CONNECTION_TIME = 5000;   // 5s

char dateTime[BUFFER_SIZE];
char uartRxBuffer[BUFFER_SIZE];
int uartRxBufferIndex = 0;
unsigned long lastDatetimeTxMillis = 0;



void wifi_connect() {
  while (WiFi.status() != WL_CONNECTED) {
    if (network_connect(wifi_ssid1, wifi_psw1)){
      active_ssid = (char*) wifi_ssid1;
      break;
    }
    if (network_connect(wifi_ssid2, wifi_psw2)){
      active_ssid = (char*) wifi_ssid2;
      break;
    }
  }
}

bool network_connect(const char* wifi_ssid, const char* wifi_psw) {
  unsigned long currentMillis;
  unsigned long wifiConnectionMillis = millis();


  WiFi.begin(wifi_ssid, wifi_psw);
  Serial.printf("Connecting to WiFi %s...", wifi_ssid);
  while (WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - wifiConnectionMillis > WIFI_CONNECTION_TIME){
      Serial.println("not connected");
      return false;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println("connected!");
  return true;
}

void mqtt_connect() {
  char mqttConnectBuf[BUFFER_SIZE];
  
  while (WiFi.status() != WL_CONNECTED) {
      wifi_connect();
  }

  Serial.printf("Connecting to MQTT server %s...", mqtt_server);
  while (!mqtt_client.connect(mqtt_publish_name, mqtt_user_name, mqtt_user_token)) {  
    Serial.print(".");
    delay(1000);
  }
  Serial.println("connected!");

  mqtt_client.subscribe(mqtt_suscribe_topic);
  Serial.printf("Suscribed to %s topic\r\n", mqtt_suscribe_topic);
  
  setDateTime();

  sprintf(mqttConnectBuf, "%s MQTT_CONNECTED_W_SSID: %s", dateTime, active_ssid);
  mqtt_client.publish(mqtt_publish_log_topic, mqttConnectBuf);
  Serial.println(mqttConnectBuf);
}

void messageReceived(String &topic, String &payload) {
  Serial2.println(payload);
  Serial.println(payload);
}

void setDateTime(){
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  
  struct tm *ptm = gmtime((time_t *)&epochTime);
  
  
  sprintf(dateTime, "%02d/%02d/%04d %02d:%02d:%02d", 
          ptm->tm_mday, 
          ptm->tm_mon + 1, 
          ptm->tm_year + 1900, 
          ptm->tm_hour, 
          ptm->tm_min, 
          ptm->tm_sec);
  
}

void sendDateTime(){
  char datetimeCommand[BUFFER_SIZE];

  setDateTime();

  sprintf(datetimeCommand, "0|DATETIME:%s",dateTime);
  
  Serial2.println(datetimeCommand);
  Serial.println(datetimeCommand);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  delay(500);
  
  wifi_connect();
  
  delay(200);

  timeClient.begin(); 

  delay(200);

  mqtt_client.begin(mqtt_server, wifi_client);
  mqtt_client.onMessage(messageReceived);
  mqtt_connect();
}

void loop() {
  mqtt_client.loop();

  if (!mqtt_client.connected()) {
    Serial.println("MQQT disconnected");
     mqtt_connect();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastDatetimeTxMillis > DATETIME_TX_INTERVAL) {
    lastDatetimeTxMillis = currentMillis;
    sendDateTime();
  }


  while (Serial2.available() > 0) {
    char incomingByte = Serial2.read();

    if (incomingByte == '\n') {
      uartRxBuffer[uartRxBufferIndex] = '\0';
      if (strncmp(uartRxBuffer, "ACK", 3) == 0) {
        mqtt_client.publish(mqtt_publish_ack_topic, uartRxBuffer);
      }
      else{
        mqtt_client.publish(mqtt_publish_log_topic, uartRxBuffer);
      }      
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