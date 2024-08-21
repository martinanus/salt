#define LED_PIN 23      

char buf[100];
int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); 

  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN, HIGH); 
  sprintf(buf, "[*] Hello ESP32 from Arduino IDE %d", i);
  Serial.println(buf);
  delay(2000);
  digitalWrite(LED_PIN, LOW); 
  sprintf(buf, "[*] Bye bye %d", i);
  Serial.println(buf);
  delay(2000);
  
  i++;
}
