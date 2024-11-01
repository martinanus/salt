
const int BUFFER_SIZE = 256;
char suscribeBuffer[BUFFER_SIZE];
char uartRxBuffer[BUFFER_SIZE] = {"0.0"};  
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

void speeds_limitado() {
  float v = 0.0; 

  while (v < 32){
    v = v + 1.4;
    Serial.println(v);    
    delay(400);
  }
  
  for (int i = 0; i< 4; i++){
    Serial.println(v);    
    delay(1000);
  }  

  while (v > 22){
    v = v - 0.7;
    Serial.println(v);    
    delay(400);    
  }

  for (int i = 0; i< 4; i++){
    Serial.println(v);    
    delay(1000);
  }  


   while (v < 40){
    v = v + 0.9;
    Serial.println(v);
    
    delay(400);
  }
  
  for (int i = 0; i< 4; i++){
    Serial.println(v);    
    delay(1000);
  }  

  while (v > 2){
    v = v - 1.7;
    Serial.println(v);    
    delay(400);    
  }

  v = 0;
  
  for (int i = 0; i< 5; i++){
    Serial.println(v);    
    delay(1000);
  }      
}

void speeds_total() {
  float v = 0.0; 

  while (v < 50){
    v = v + 3.3;
    Serial.println(v);    
    delay(400);
  }
  
  for (int i = 0; i< 2; i++){
    Serial.println(v);    
    delay(1000);
  }  

  while (v > 30){
    v = v - 2.7;
    Serial.println(v);    
    delay(400);    
  }
  for (int i = 0; i< 2; i++){
    Serial.println(v);    
    delay(1000);
  }  

  while (v < 40){
    v = v + 2.3;
    Serial.println(v);    
    delay(400);
  }
  
  for (int i = 0; i< 2; i++){
    Serial.println(v);    
    delay(1000);
  } 

  for (int i = 0; i< 2; i++){
    Serial.println(v);    
    delay(1000);
  }  

  while (v > 3){
    v = v - 2.4;
    Serial.println(v);    
    delay(400);    
  }    
     
}

void loop() {
  delay(1000);  

  Serial.println(uartRxBuffer);  
  
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == 'L') {
      speeds_limitado();
      uartRxBuffer[uartRxBufferIndex++] = '0';
    } else if (incomingByte == 'T') {
      speeds_total();
      uartRxBuffer[uartRxBufferIndex++] = '0';
    } else if (incomingByte == '\n') {
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