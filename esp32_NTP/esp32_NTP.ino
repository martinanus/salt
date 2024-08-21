#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

// Wi-Fi credentials
const char* ssid = "CASITA_ARCOS-2.4Ghz";
const char* password = "diegote10";


// NTP client settings
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600, 60000);  // UTC-3 offset

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize NTPClient
  timeClient.begin();
}

void loop() {
  timeClient.update();
  
  unsigned long epochTime = timeClient.getEpochTime();

  // Convert epoch time to tm structure
  struct tm *ptm = gmtime((time_t *)&epochTime);

  // Format date and time as dd/MM/yyyy hh:mm:ss
  char formattedDateTime[20];
  sprintf(formattedDateTime, "%02d/%02d/%04d %02d:%02d:%02d", 
          ptm->tm_mday, 
          ptm->tm_mon + 1, 
          ptm->tm_year + 1900, 
          ptm->tm_hour, 
          ptm->tm_min, 
          ptm->tm_sec);

  // Print the formatted date and time
  Serial.println(formattedDateTime);

  delay(1000);
}
