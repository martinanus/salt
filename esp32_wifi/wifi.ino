
void connected_to_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) {
  Serial.println("[+] Connected to the WiFi network");
}

void disconnected_from_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) {
  Serial.println("[-] Disconnected from the WiFi AP");
  WiFi.begin(ssid, password);
}

void got_ip_from_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) {
  Serial.print("[+] Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}


void initialize_wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); 
  delay(1000); 
}


void initialize_wifi_events(const char* ssid, const char* password){
  WiFi.mode(WIFI_STA); //Optional
  WiFi.onEvent(connected_to_ap, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(got_ip_from_ap, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(disconnected_from_ap, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
}

void scan_wifi() {
  Serial.println("Scanning for available WiFi networks...");

  // Perform the WiFi scan
  int numberOfNetworks = WiFi.scanNetworks();

  Serial.println("Scan complete.");
  if (numberOfNetworks == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.print(numberOfNetworks);
    Serial.println(" networks found:");
    for (int i = 0; i < numberOfNetworks; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      Serial.print(" ");
      Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
      Serial.println();
    }
  }
  Serial.println();
}

void connect_wifi(const char* ssid, const char* password){
    int try_n = 0;
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");


    while(WiFi.status() != WL_CONNECTED && try_n<100){
        Serial.print(".");
        delay(100);
        try_n++;
    }

  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to the WiFi network");
  }{
    Serial.println("\nConnection to wifi network failed");
  }
}

void get_network_info(){
    if(WiFi.status() == WL_CONNECTED) {
        Serial.print("[*] Network information for ");
        Serial.println(ssid);

        Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
        Serial.print("[+] Gateway IP : ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("[+] Subnet Mask : ");
        Serial.println(WiFi.subnetMask());
        Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
        Serial.print("[+] ESP32 IP : ");
        Serial.println(WiFi.localIP());
    }
}


String get_wifi_status(int status){
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
    return "";
}