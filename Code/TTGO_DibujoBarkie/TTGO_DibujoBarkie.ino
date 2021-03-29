#include<Arduino.h>
#include<U8g2lib.h>
 
#ifdef U8X8_HAVE_HW_SPI
#include<SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include<Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C

// Copy the contents of your .xbm file below
#define ironman_width 82
#define ironman_height 64
static unsigned char ironman_bits[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
   0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0xc0,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 0x62, 0x0e, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x63, 0x18, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x98, 0x21, 0x30, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x80, 0x01, 0x98, 0x30, 0xe0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x00, 0x08, 0x18, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x00, 0x00, 0x08, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x01, 0x00, 0x0c, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x01, 0x00, 0x0c, 0x94, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
   0x00, 0x06, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff,
   0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x08,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x08, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x06, 0x3e, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0xfe, 0x07, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x06, 0x1b, 0x1c, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x31, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1, 0x21,
   0x30, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x20, 0x30,
   0xc3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x39, 0x30, 0x20, 0x43,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x10, 0x30, 0x63, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x18, 0x20, 0x23, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x18, 0x20, 0x31, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x1b, 0x10, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x13, 0x10, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x1e, 0x10, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
   0x80, 0x08, 0x00, 0xf0, 0x0f, 0xc0, 0x01, 0xfc, 0x01, 0xc3, 0xc1, 0xf0,
   0x3f, 0x00, 0x70, 0x1c, 0xc0, 0x03, 0x9c, 0x07, 0xc3, 0xc0, 0x70, 0x00,
   0x00, 0x30, 0x18, 0xc0, 0x03, 0x0c, 0x06, 0xe3, 0xc0, 0x70, 0x00, 0x00,
   0x30, 0x18, 0xc0, 0x07, 0x0c, 0x06, 0x63, 0xc0, 0x30, 0x00, 0x00, 0x30,
   0x18, 0xe0, 0x06, 0x0c, 0x0c, 0x73, 0xc0, 0x70, 0x00, 0x00, 0x30, 0x18,
   0x60, 0x06, 0x0c, 0x06, 0x3b, 0xc0, 0x30, 0x00, 0x00, 0xf0, 0x0f, 0x60,
   0x0e, 0x0c, 0x06, 0x1b, 0xc0, 0x70, 0x15, 0x00, 0x70, 0x1d, 0x30, 0x0c,
   0x5c, 0x03, 0x1f, 0xc0, 0xf0, 0x1f, 0x00, 0x30, 0x30, 0x30, 0x0c, 0xfc,
   0x01, 0x1b, 0xc0, 0x70, 0x00, 0x00, 0x30, 0x30, 0x30, 0x1c, 0x9c, 0x01,
   0x33, 0xc0, 0x30, 0x00, 0x00, 0x30, 0x30, 0xf8, 0x1f, 0x0c, 0x03, 0x73,
   0xc0, 0x30, 0x00, 0x00, 0x30, 0x30, 0x78, 0x1d, 0x0c, 0x03, 0xe3, 0xc0,
   0x70, 0x00, 0x00, 0x30, 0x38, 0x1c, 0x38, 0x1c, 0x06, 0xc3, 0xc0, 0x30,
   0x00, 0x00, 0xf0, 0x1f, 0x0c, 0x30, 0x0c, 0x06, 0x83, 0xc1, 0xf0, 0x3f,
   0x00, 0xf0, 0x06, 0x0c, 0x30, 0x0c, 0x0c, 0x03, 0xc3, 0x60, 0x2b, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

 
void draw(void) {
 // graphic commands to redraw the complete screen should be placed here  
 u8g2.drawXBMP( 24, 0, ironman_width, ironman_height, ironman_bits);
}
 
void setup(void) {
 u8g2.begin();
}
 
void loop(void) {
 // picture loop
 u8g2.firstPage();
 do {
     draw();
    } while( u8g2.nextPage() );
 
 // rebuild the picture after some delay
 delay(1000);
}