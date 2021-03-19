/* Example from Sandeep Mistry 
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */

#include <SPI.h>
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino

#define OFF 0   // For LED
#define ON 1

// I2C OLED u8g2 works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

//VARIABLES
float value;
float volt;
int VOLTPIN = A0;

void setup() {

  //Begins
  Serial.begin(9600);
  while (!Serial);
  pinMode(VOLTPIN, INPUT);

  //Display begin
  u8g2.begin();
  u8g2.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  u8g2.setFont(u8g2_font_ncenB10_tr);      
  
  
}

void loop() {

  //¡¡COMPRAR ZÉNER!!
  
  value = analogRead(VOLTPIN);
  volt = value / 4095 * 3.3; //Valor entre 0 - 3.3V 

  // u8g2 Info
  u8g2.clearBuffer();  
  u8g2.setCursor(0,12); u8g2.print("BatteryTrack");
  u8g2.setCursor(0,26); u8g2.print("Value: ");u8g2.print(value);
  u8g2.setCursor(0,42); u8g2.print("Volts: ");u8g2.print(volt);u8g2.print("volt");
  u8g2.sendBuffer();

  
}
