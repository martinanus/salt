
const int BUFFER_SIZE = 256;
char suscribeBuffer[BUFFER_SIZE];
char uartRxBuffer[BUFFER_SIZE];
int uartRxBufferIndex = 0;

unsigned long lastMillis = 0;
int i = 0;



void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Let's start...");
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);

}

void loop() {
  delay(1000);

  Serial.println("NEW MESSAGE");
  /*
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();

    if (incomingByte == '\n') {
      uartRxBuffer[uartRxBufferIndex] = '\0';      
      Serial.println(uartRxBuffer);
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      uartRxBufferIndex = 0;
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    } else {
      if (uartRxBufferIndex < BUFFER_SIZE - 1) {
        uartRxBuffer[uartRxBufferIndex++] = incomingByte;
      } else {
        uartRxBufferIndex = 0;
      }
    }
  }
  */
  
}