//GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
#define TX 5
#define RX 4
SoftwareSerial SoftSerial(TX, RX); //tx, rx
float flat, flon;
unsigned long age;

//LoRa libraries and declarations
#include <LoRaLib.h>
// create instance of LoRa class using SX1278 module
// this pinout corresponds to RadioShield
// https://github.com/jgromes/RadioShield
#define NSS 10 
#define DIO0 2
#define DIO1 6 //Arduino estropeado 6 //Arduino estropeado 6 //Arduino estropeado 6 //Arduino estropeado 6 //Arduino estropeado 6 //Arduino estropeado 
#define MOSI 11
#define MISO 12
#define SCLK 13
SX1278 lora = new LoRa (NSS, DI00, DI01); //nss, di00, di01
//boolean for send and receive LoRa
boolean success = false;
//receiveLoRa
String receivedString;
//sendLoRa
String sentString;

//Pins and LEDs
#define redLED 8
#define greenLED 9

//Constants
#define DELAY 1000

void setup() {

  //Serial
  Serial.begin(9600);
  //SoftwareSerial
  SoftSerial.begin(9600);
  
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
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);

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
  
}

// initialize SX1278 with default settings
boolean initializeLoRa () { 

  Serial.print("Delaying ");
  Serial.print(DELAY);
  Serial.println("ms ...");
  delay(DELAY);
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
  
}

bool getGPSCoordinates () {

  //Declarations
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    
    while (SoftSerial.available()){
      
      char c = SoftSerial.read();
      if (gps.encode(c)) newData = true;
      
    }
    
  }

  if (newData){

    success = true;

    // Getting the data
    gps.f_get_position(&flat, &flon, &age);

    // Serial
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  
  }

  // More statistics
  gps.stats(&chars, &sentences, &failed);

  // Print those in serial
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);

  // Just in case GPS fails
  if (chars == 0) Serial.println("** No characters received from GPS: check wiring **");

  return success;

}  

void loop() {

  //Turn red LED on to sshow that the module is working
  digitalWrite(redLED, HIGH);
  
  //Check if user is requesting
  success = false;
  while (!success) {

    success = receiveLoRa();
    delay(400);
        
  }
  
  Serial.print("Received String: ");
  Serial.println(receivedString);

  //If it gets a request, then it looks for and send the coordinates
  if (receivedString.equals("request")) {

    //Get the GPS coordinates
    success = false;
    while (!success) {

      success = getGPSCoordinates();
        
    }

    //Create the String response (sentString)
    sentString.concat(flat * 1000000);
    sentString.concat(" ");
    sentString.concat(flon * 1000000);

    //And send it
    delay(5000);
    Serial.print("Sending String: ");
    Serial.println(sentString);

    success = false;
    while (!success) {

      success = sendLoRa();
      delay(400);
        
    }
      
  }
   
  delay(200); 
 
 }
