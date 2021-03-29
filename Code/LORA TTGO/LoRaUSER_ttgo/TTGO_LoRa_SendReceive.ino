#define REPETITIONS 3
#define DELAY_BETWEEN_TRIES 100

String receive_packet(){
  
  int packetSize = LoRa.parsePacket();
  while(!packetSize) {
    packetSize = LoRa.parsePacket(); 
  }
    Serial.print("Received packet '");

  digitalWrite(blueLED, ON);  // Turn blue LED on

  // read packet
  packet = "";                   // Clear packet
  while (LoRa.available()) {
    packet += (char)LoRa.read(); // Assemble new packet
  }
  rssi = LoRa.packetRssi();

  // u8g2 Info
  u8g2.clearBuffer();  
  u8g2.setCursor(0,12); u8g2.print("LoRa Receiver");
  u8g2.setCursor(0,26); u8g2.print("Received packet:");
  u8g2.setCursor(0,42); u8g2.print("    '" + packet + "'");
  u8g2.setCursor(0,58); u8g2.print("RSSI " + rssi);
  u8g2.sendBuffer();

  digitalWrite(blueLED, OFF); // Turn blue LED off
  
  Serial.println(packet + "' with RSSI " + rssi);   

  return packet;
  
}

void send_packet(String msg){

  for (int i = 0; i < REPETITIONS; i++) {
    
    digitalWrite(blueLED, ON);  // Turn blue LED on
    // send packet
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    digitalWrite(blueLED, OFF); // Turn blue LED off
    delay(DELAY_BETWEEN_TRIES);
  
  }

  // u8g2 Info
  u8g2.clearBuffer();  
  u8g2.setCursor(0,12); u8g2.print("LoRa Sender");
  u8g2.setCursor(0,30); u8g2.print("Sent Packet:");
  u8g2.setCursor(0,48); u8g2.print(" # " + msg);
  u8g2.sendBuffer();
  
}
