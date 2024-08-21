const int BUFFER_SIZE = 256; // Adjust buffer size as needed
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  // Initialize the Serial communication at a baud rate of 115200
  Serial2.begin(115200);
  Serial.begin(115200);
}

void loop() {
  // Check if data is available to read

  Serial2.println("HOLA");
  Serial.println("HOLA");
  delay(1000);

  
  while (Serial2.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial2.read();

    // Check for the end-of-line character
    if (incomingByte == '\n') {
      // Null-terminate the string
      buffer[bufferIndex] = '\0';

      // Transmit the complete sentence
      Serial.println(buffer);

      // Reset buffer index for the next sentence
      bufferIndex = 0;
    } else {
      // Store the byte in the buffer if there is space
      if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = incomingByte;
      } else {
        // Buffer overflow, reset the buffer
        bufferIndex = 0;
      }
    }
  }
}
