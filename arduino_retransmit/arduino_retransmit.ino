
const int BUFFER_SIZE = 256;
char suscribeBuffer[BUFFER_SIZE];
char uartRxBuffer[BUFFER_SIZE] = {"23.4"};
int uartRxBufferIndex = 0;

unsigned long lastMillis = 0;
int i = 0;



void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);

}

void loop() {
  delay(1000);

  Serial.println(uartRxBuffer);
  
  
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();

    if (incomingByte == '\n') {
      uartRxBuffer[uartRxBufferIndex] = '\0';      
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