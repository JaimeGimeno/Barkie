/* Example from Sandeep Mistry 
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */

// BLUETOOTH
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

int STATE = 0;

// LORA ####################
#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino
// #include <U8x8lib.h>

#define OFF 0   // For LED
#define ON 1

String msg;

// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST211
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)


// DISPLAY ###################
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

/* Pick One. Hardware I2C does NOT work! This article helped: https://robotzero.one/heltec-wifi-kit-32/ 
* TTGo boards similar to Heltec boards, LED_BUILTIN = 2 instead of pin 25
* Some OLED displays don't handle ACK correctly so SW I2C works better. Thank you Olikraus!
* TTGo OLED has pin 16 reset unlike other  OLED displays
*/

// UNCOMMENT one of the constructor lines below
//U8X8_SSD1306_128X64_NONAME_SW_I2C Display(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Unbuffered, basic graphics, software I2C
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Page buffer, SW I2C
U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

const int blueLED = LED_BUILTIN; 
int counter = 0;

String rssi = "";
String packet = "";

void setup() {
  Serial.begin(9600);
  while (!Serial);

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("Bluetooth device started, waiting for request...");

  Serial.println("LoRa Sender");

  Display.begin();
  Display.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  Display.setFont(u8g2_font_ncenB10_tr);
  
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  pinMode(blueLED, OUTPUT); // For LED feedback

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(12); // ranges from 6-12, default 7 see API docs

  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected  to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);

  //LEDS
  //pinMode(blue_LED, OUTPUT);
  //digitalWrite(blue_LED, LOW);
  
  
}

//void make_blue_led_blink(){ // Esta funcion eliminarla al final (quita tiempo)
//  for(int i = 0; i <= 2; i++) {
//    digitalWrite(blue_LED, HIGH);
//    delay(100);
//    digitalWrite(blue_LED, LOW);
//    delay(100);
//  }
//}


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

  // Display Info
  Display.clearBuffer();  
  Display.setCursor(0,12); Display.print("LoRa Receiver");
  Display.setCursor(0,26); Display.print("Received packet:");
  Display.setCursor(0,42); Display.print("    '" + packet + "'");
  Display.setCursor(0,58); Display.print("RSSI " + rssi);
  Display.sendBuffer();

  digitalWrite(blueLED, OFF); // Turn blue LED off
  
  Serial.println(packet + "' with RSSI " + rssi);   

  return packet;
}

void send_packet(String msg){

  digitalWrite(blueLED, ON);  // Turn blue LED on
  // send packet
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  digitalWrite(blueLED, OFF); // Turn blue LED off

  // Display Info
  Display.clearBuffer();  
  Display.setCursor(0,12); Display.print("LoRa Sender");
  Display.setCursor(0,30); Display.print("Sent Packet:");
  Display.setCursor(0,48); Display.print(" # " + msg);
  Display.sendBuffer();
}

void loop() {
  if (SerialBT.available()) { // SerialBT.available()
    STATE = SerialBT.read();

    if (STATE == '1') { // STATE == '1'

      // Send request to dog
      Serial.print("Sending 'request'");
      send_packet("request");

      // Wait for response      
      msg = receive_packet();
      // Send coordinates to app
      SerialBT.print(msg);

      STATE = 0;
    }
  
  }

  delay(20);
}
