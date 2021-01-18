#include <LoRaLib.h>

//Bluetooth
#include <SoftwareSerial.h>
#define RxD 4
#define TxD 5
SoftwareSerial Bluetooth(RxD,TxD);
int STATE = 0;

//LoRa libraries and declarations
#include <LoRaLib.h>
// create instance of LoRa class using SX1278 module
// this pinout corresponds to RadioShield
// https://github.com/jgromes/RadioShield
  // NSS pin:   10 (4 on ESP32/ESP8266 boards)
  // DIO0 pin:  2
  // DIO1 pin:  3
  // MOSI pin:  11
  // MISO pin:  12
  // SCLK pin:  13
SX1278 lora = new LoRa;
//boolean for send and receive LoRa
boolean success = false;
//receiveLoRa
String receivedString;
//sendLoRa
String sentString;

//Pins and LEDs
#define redLED 6
#define greenLED 7
#define blueLED 8

void setup() {

  //Serial
  Serial.begin(9600);
  //Bluetooth
  Bluetooth.begin(9600);

  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  //Title
  Serial.println("Dog_Module.");
  Serial.println("by Alfonso Forcen");
  Serial.println();
  
  //Initialize LoRa
  success = false;
  while (!success) {
    success = initializeLoRa ();
  }
  //Pins
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);

  //Be sure every single LED is working
  checkLEDs();

}

void makeLEDBlink (int LED) {

   
    //Make green LED blink to show success
    int i = 0;
    digitalWrite(redLED, LOW);
    for(i = 0; i < 3; i++) {

      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      
    }
    digitalWrite(redLED, HIGH);
  
}

void checkLEDs() {

    makeLEDBlink(redLED);
    makeLEDBlink(greenLED);
    makeLEDBlink(blueLED);
  
}

// initialize SX1278 with default settings
boolean initializeLoRa () { 

  Serial.print(F("Initializing ... "));
  // carrier frequency:           434.0 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            9
  // coding rate:                 7
  // sync word:                   0x12
  // output power:                17 dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)
  int state = lora.begin();
  if(state == ERR_NONE) {
    Serial.println(F("success!"));
    success = true;
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    delay (200);
  }
  return success;
}


void checkLoRa() {

  Serial.print(F("Scanning channel for LoRa preamble ... "));

  // start scanning current channel
  int state = lora.scanChannel();

  if(state == PREAMBLE_DETECTED) {
    // LoRa preamble was detected
    Serial.println(F(" detected preamble!"));
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);

  } else if(state == CHANNEL_FREE) {
    // no preamble was detected, channel is free
    Serial.println(F(" channel is free!"));
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);

  }
  
}

//Returns true if success, false if error
bool receiveLoRa () {

  boolean solution = false;
  
  Serial.print(F("Waiting for incoming transmission ... "));

  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  // NOTE: for spreading factor 6, the packet length
  //       must be known in advance, and provided to
  //       receive() method!
  int state = lora.receive(receivedString);

  // you can also receive data as byte array
  /*
    size_t len = 8;
    byte byteArr[len];
    int state = lora.receive(byteArr, len);
  */

  if (state == ERR_NONE) {
    
    solution = true;

    makeLEDBlink(greenLED);
    
    // packet was successfully received
    Serial.println(F("success!"));

    // print data of the packet
    Serial.print(F("Data:\t\t\t"));
    Serial.println(receivedString);

    // print RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("RSSI:\t\t\t"));
    Serial.print(lora.getRSSI());
    Serial.println(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("SNR:\t\t\t"));
    Serial.print(lora.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    // of the last received packet
    Serial.print(F("Frequency error:\t"));
    Serial.print(lora.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == ERR_RX_TIMEOUT) {
    
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == ERR_CRC_MISMATCH) { 
    
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  }

  return solution;
  
}

bool sendLoRa (){
  
  boolean solution = false;

  Serial.print(F("Sending packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  // NOTE: transmit() is a blocking method!
  //       See example TransmitInterrupt for details
  //       on non-blocking transmission method.
  int state = lora.transmit(sentString);

  // you can also transmit byte array up to 256 bytes long
  /*
    size_t len = 8;
    byte byteArr[len] = {0x01, 0x23, 0x45, 0x56,
                         0x78, 0xAB, 0xCD, 0xEF};
    int state = lora.transmit(byteArr, len);
  */

  if (state == ERR_NONE) {
    
    solution = true;

    makeLEDBlink(greenLED);
    
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("Datarate:\t"));
    Serial.print(lora.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F(" too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F(" timeout!"));

  }

  return solution;
  delay(200);
  
}

void loop() {

  //Check if phone is requesting
  Serial.println("Waiting for Bluetooth request");
  if (Bluetooth.available() > 0) {

    //Read the request
    STATE = Bluetooth.read();
    Serial.print("STATE = ");
    Serial.println(STATE);

    //Request received from phone
    if (STATE == '1') {

       makeLEDBlink(blueLED);

      //Send the request to collar
      sentString = "request";
      success = false;
      Serial.println("Sending radio request.");
      while (!success) {

        success = sendLoRa();
         
      }
      Serial.print("Message sent: ");
      Serial.println(sentString);
      
      //Wait to receive the answer from the collar
      success = false;
      Serial.println("Waiting for receive GPS.");
      while (!success) {

        success = receiveLoRa();
        
      }

      Serial.print("Coordinates received: ");
      Serial.println(receivedString);
      Bluetooth.print(receivedString);

      makeLEDBlink(blueLED);
    
      STATE = 0;
      
    }
    
  }
  delay(200);

}
