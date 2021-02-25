//Bluetooth
#include <SoftwareSerial.h>
#define RxD 4
#define TxD 5
SoftwareSerial Bluetooth(RxD,TxD);
int STATE = 0;

//Pins and LEDs
#define redLED 8
#define greenLED 7
#define blueLED 6

//Constants
#define DELAY 1000
#define TIMEOUT_TIME 10000

//Strings
String requestString = "request";
int requestString_size = 9;
String receivedString = "";

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

//SETUP
void setup() {

  //Serial begin
  while (!Serial);
  Serial.begin(9600);
  delay(100);
  
  //Bluetooth
  Bluetooth.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  //Title
  Serial.println("User_Module.");
  Serial.println("by Alfonso Forcen");
  Serial.println();

  //LoRa settings
  
    //Pin setup
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    //Manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

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

//Returns true if success, false if error
bool receiveLoRa () {

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  Serial.println("Waiting for reply...");
  delay(10);

  //Set timeout in case does not work
  if (rf95.waitAvailableTimeout(TIMEOUT_TIME)) { //10secs
    
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
  
  } else {
    
    Serial.println("No reply, is there a listener around?");
  
  }
  
}

//Returns true if success, false if error
bool sendLoRa (String msg, int msg_size){

  //Prepare and send the message
  char char_msg[msg_size];
  strcpy(char_msg, msg.c_str());
  msg[msg_size-1] = 0; //def
  rf95.send((uint8_t *)char_msg, msg_size);

  //Wait for message sent
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  
}

void loop() {

  //Check if phone is requesting
  
  //Serial.println("Waiting for Bluetooth request");
  //if (Bluetooth.available() > 0) {

    //Read the request
    STATE = Bluetooth.read();
    Serial.print("STATE = ");
    Serial.println(STATE);
    
    STATE = '1';

    //Request received from phone
    if (STATE == '1') {

      makeLEDBlink(blueLED);

      //Send the request to collar
      Serial.println("Sending radio request.");
      sendLoRa(requestString, requestString_size);
      Serial.print("Sending 'request'");
      
      //Wait to receive the answer from the collar
      Serial.println("Waiting for receive GPS.");
      receiveLoRa();
      Serial.print("[1] Got it: ");
      Serial.println(receivedString);
      
      Bluetooth.print(receivedString);

      makeLEDBlink(blueLED);
    
      STATE = 0;
      
    }
    
  delay(30000);

}
