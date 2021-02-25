#include <SPI.h>
#include <LoRa.h>

#define ss 10
#define reset 9
#define dio0 2

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("\n\nLoRa Sender by Jaime Gimeno\n\n");

  LoRa.setPins(ss, reset, dio0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void send_packet(String packet){
  Serial.print("Sending packet: ");
  Serial.println(packet);
  // send packet
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
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

void send_sync(String packet, String ID){
  // Create messagge and send
  String msg = ID + " " + packet;
  send_packet(msg);

  // Wait for response
  String received_msg = read_packet();
  while(received_msg != ID){
    received_msg = read_packet();
  }

  Serial.println("Mensage enviado y confirmaión recibida.");
  
}

void loop() {
  Serial.println("Counter: "+String(counter));
  delay(1000);
  Serial.println("Simado");
  send_sync("coordenadas aqui " + String(counter), "420");

  counter = counter + 1;

  delay(1000);
}
