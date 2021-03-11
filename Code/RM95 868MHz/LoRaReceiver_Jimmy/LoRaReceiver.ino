#include <SPI.h>
#include <LoRa.h>

#define ss 10
#define reset 7
#define dio0 6

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver 1");

  LoRa.setPins(ss, reset, dio0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

String read_packet(){
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  while(!packetSize){
    packetSize = LoRa.parsePacket();
  }

  Serial.print("Received packet '");

  // read packet
  String message = "";
  while (LoRa.available()) {
    message.concat((char)LoRa.read());
  }
  Serial.print(message);
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());

  return message;
}

void send_packet(String packet){
  Serial.print("Sending packet: ");
  Serial.println(packet);
  // send packet
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
}

String send_sync(String packet, String ID){
  // Create messagge and send
  String msg = ID + " " + packet;
  send_packet(msg);

  // Wait for response
  String received_msg = read_packet();
  while(received_msg != "ID"){
    received_msg = read_packet();
  }

  Serial.print("Message sent successfully");
  
}

String read_sync(String ID){
  String message = read_packet();
  String ID_read = message.substring(0, 2);

  while(ID_read != ID){
    message = read_packet();
    ID_read = message.substring(0,2);
  }
  
  send_packet(ID);
  Serial.println("Paquete leído y confirmación enviada.");
  return message.substring(2);
}

void loop() {
  read_sync("42");
  delay(500);
}
