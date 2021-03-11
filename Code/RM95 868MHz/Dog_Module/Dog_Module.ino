//GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
#define TX 5
#define RX 4
SoftwareSerial SoftSerial(TX, RX); //tx, rx
float flat, flon;
unsigned long age;
boolean success = false;
    
//Strings
String requestString = "request";
int requestString_size = 9;
String receivedString = "";
String sentString = "";

//LoRa RM_RF95
  
  //Libraries
  #include <SPI.h> 
  #include <RH_RF95.h>
  
  //Pins ¡¡¡NO CAMBIAR!!! 
  #define RFM95_CS 10 
  #define RFM95_RST 9 
  #define RFM95_INT 2 
  
  // Change to 434.0 or other frequency, must match RX's freq!
  #define RF95_FREQ 868.0 
  
  // Singleton instance of the radio driver
  RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Pins and LEDs
#define redLED 9
#define greenLED 8

//Constants
#define DELAY 1000

void setup() {

  //Serial begin
  while (!Serial);
  Serial.begin(9600);
  delay(100);
  
  //SoftwareSerial
  SoftSerial.begin(9600);
  
  //Title
  Serial.println("Dog_Module.");
  Serial.println("by Alfonso Forcen");
  Serial.println();

    //LoRa initialization
    Serial.println("LoRa radio initialization...");
    //Possible failure -> wait 5secs and retry
    while (!rf95.init()) {
      Serial.println("LoRa radio init failed");
      Serial.println("\nRetrying in 5secs...");
      delay(5000);
      Serial.println("Retrying");
      //while (1);
    }
    Serial.println("LoRa radio init OK!");

    //Setting frequency
    /*Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM*/
    Serial.print("Setting Frequency to: ");
    Serial.println(RF95_FREQ);
    while (!rf95.setFrequency(RF95_FREQ)) { //868MHz this case, RFM95
      Serial.println("setFrequency failed");
      Serial.println("\nRetrying in 5secs...");
      delay(5000);
      Serial.println("Retrying");
      //while (1);
    }

    //Setting maximum power of transmission
    /* Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:*/
    rf95.setTxPower(23, false);
  
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

//Returns true if success, false if error
bool receiveLoRa () {

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  Serial.println("Waiting for reply...");
  delay(10);

  //Set timeout in case does not work
  //if (rf95.waitAvailableTimeout(TIMEOUT_TIME)) { //10secs
    
    //Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      receivedString = (char*)buf;
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    
    } else {
      
      Serial.println("Receive failed");
    
    }
  
  //} else {
  //  
  //  Serial.println("No reply, is there a listener around?");
  //
  //}
  
}

//Returns true if success, false if error
bool sendLoRa (String msg){

  //Prepare and send the message
  int msg_size = msg.length() + 1;
  char char_msg[msg_size];
  strcpy(char_msg, msg.c_str());
  msg[msg_size-1] = 0; //def
  rf95.send((uint8_t *)char_msg, msg_size);

  //Wait for message sent
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  
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

  //Turn red LED on to show that the module is working
  digitalWrite(redLED, HIGH);
  
  //Check if user is requesting
  if (rf95.available()){

    receiveLoRa();
    
  }
  
  Serial.print("Received String: ");
  Serial.println(receivedString);

  //If it gets a request, then it looks for and send the coordinates
  if (receivedString.equals(requestString)) {

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

    sendLoRa(sentString);

    sentString = "";
      
  }
   
  delay(200); 
 
 }
