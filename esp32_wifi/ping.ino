

void ping(const char * host) {
  // Perform a ping operation
  Serial.print("Pinging ");
  Serial.print(host);
  Serial.println("...");

  if (Ping.ping(host)) {
    Serial.println("Ping successful.");
  } else {
    Serial.println("Ping failed.");
  }
  delay(5000);
}
