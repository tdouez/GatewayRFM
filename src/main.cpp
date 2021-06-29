#include <Arduino.h>
//--------------------------------------------------------------------
//                 +/          
//                 `hh-        
//        ::        /mm:       
//         hy`      -mmd       
//         omo      +mmm.  -+` 
//         hmy     .dmmm`   od-
//        smmo    .hmmmy    /mh
//      `smmd`   .dmmmd.    ymm
//     `ymmd-   -dmmmm/    omms
//     ymmd.   :mmmmm/    ommd.
//    +mmd.   -mmmmm/    ymmd- 
//    hmm:   `dmmmm/    smmd-  
//    dmh    +mmmm+    :mmd-   
//    omh    hmmms     smm+    
//     sm.   dmmm.     smm`    
//      /+   ymmd      :mm     
//           -mmm       +m:    
//            +mm:       -o    
//             :dy             
//              `+:     
//--------------------------------------------------------------------
//   __|              _/           _ )  |                       
//   _| |  |   ` \    -_)   -_)    _ \  |   -_)  |  |   -_)     
//  _| \_,_| _|_|_| \___| \___|   ___/ _| \___| \_,_| \___|  
//--------------------------------------------------------------------    
// 2021/01/01 - FB V1.00
// 2021/06/10 - FB V1.01
//--------------------------------------------------------------------
#include <Arduino.h>
#include <WiFiManager.h>
#include <WiFiClient.h>
//#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h> 
#include <PubSubClient.h>
#include "SSD1306Wire.h"
#include <mdns.h>
#include <SPIFFS.h>
#define MAX_XXTEA_DATA8  200
#include <xxtea-lib.h>


#define VERSION   "v1.0.1"

#define MY_BAUD_RATE 115200

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

#define OLED_ADDR 0x3C
#define OLED_SDA  21
#define OLED_SCL  22


#define SX1278_SCK  5
#define SX1278_MISO 19
#define SX1278_MOSI 27
#define SX1278_CS   18
#define SX1278_RST  14
#define SX1278_IRQ  26

#define TRIGGER_PIN 4


#define GATEWAY_ADDRESS       1
#define CLIENT_LINKY_ADDRESS  2

#define ENTETE  '$'

#define CRYPT_PASS "FumeeBleue"

#define RFM_TX_POWER 20   // 5..23 dBm, 13 dBm is default

#define RH_RF95_MAX_MESSAGE_LEN 200
#define MAX_BUFFER      32
#define MAX_BUFFER_URL  64
#define DEFAULT_PORT_MQTT 1883

#define ETIQU_ADSC     1
#define ETIQU_VTIC     2
#define ETIQU_NGTF     3
#define ETIQU_LTARF    4
#define ETIQU_EAST     5
#define ETIQU_IRMS1    6
#define ETIQU_IRMS2    7
#define ETIQU_IRMS3    8
#define ETIQU_URMS1    9
#define ETIQU_URMS2    10
#define ETIQU_URMS3    11
#define ETIQU_PREF     12
#define ETIQU_PCOUP    13
#define ETIQU_SINSTS   14
#define ETIQU_SINSTS1  15
#define ETIQU_SINSTS2  16
#define ETIQU_SINSTS3  17
#define ETIQU_STGE     18
#define ETIQU_MSG1     19
#define ETIQU_NTARF    20
#define ETIQU_NJOURF   21
#define ETIQU_NJOURF1  22
#define ETIQU_EAIT     23
#define ETIQU_SINSTI   24
#define ETIQU_EASF01   25
#define ETIQU_EASF02   26
#define ETIQU_EASF03   27
#define ETIQU_EASD01   28
#define ETIQU_EASD02   29
#define ETIQU_EASD03   30
#define ETIQU_ERQ1     31
#define ETIQU_ERQ2     32
#define ETIQU_ERQ3     33

#define ETIQU_SINSTSmin   50
#define ETIQU_SINSTSmax   51
#define ETIQU_STEP        90
#define ETIQU_BOOT        91


#define fb_width 128
#define fb_height 64
const unsigned char fb_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xe0, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
   0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x18, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xc0, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38,
   0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x78, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x80, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78,
   0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x78, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xc0, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c,
   0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x3c, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0xe0, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e,
   0xe0, 0xc7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x3f, 0xf0, 0xc7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x87, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f,
   0xf0, 0x87, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0x0f, 0xf8, 0x83, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0xfc, 0x83, 0x07, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07,
   0xfc, 0x83, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x07, 0xfe, 0x81, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xff, 0x81, 0x07, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03,
   0xff, 0xc1, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x83, 0xff, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x81, 0xff, 0xc0, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xc1,
   0x7f, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0xc0, 0x3f, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xc0, 0x3f, 0xf0, 0x01, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xe0,
   0x1f, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0xe0, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xf0, 0x0f, 0xfc, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xf0,
   0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x70, 0xf0, 0x0f, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xf0, 0x07, 0x7e, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8,
   0x07, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0xf8, 0x03, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf8, 0x03, 0x1f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf8,
   0x03, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0xf9, 0x01, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x81, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf8, 0x81, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x81, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0xf8,
   0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0xf0, 0x01, 0x07, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
   0xe0, 0x0f, 0x00, 0x00, 0x00, 0x03, 0x00, 0xf0, 0x01, 0x07, 0xfc, 0x31,
   0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0x01, 0x07, 0x0c, 0x33, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x98, 0xdd,
   0x83, 0x07, 0x1e, 0xe0, 0x01, 0x0e, 0x0c, 0x33, 0x3c, 0x0c, 0xc3, 0x03,
   0x60, 0x60, 0x98, 0x73, 0xc6, 0x0c, 0x33, 0xe0, 0x03, 0x0e, 0x0c, 0x33,
   0x66, 0x0c, 0x63, 0x06, 0xe0, 0x6f, 0x98, 0x31, 0x66, 0x98, 0x61, 0xc0,
   0x03, 0x0c, 0xfc, 0x31, 0xc3, 0x0c, 0x33, 0x0c, 0x60, 0x60, 0x98, 0x31,
   0xe6, 0x9f, 0x7f, 0xc0, 0x03, 0x0c, 0x0c, 0x33, 0xff, 0x0c, 0xf3, 0x0f,
   0x60, 0x60, 0x98, 0x31, 0x66, 0x80, 0x01, 0x80, 0x03, 0x08, 0x0c, 0x33,
   0x03, 0x0c, 0x33, 0x00, 0x60, 0x60, 0x98, 0x31, 0x66, 0x80, 0x01, 0x00,
   0x07, 0x00, 0x0c, 0x33, 0x03, 0x0c, 0x33, 0x00, 0x60, 0x60, 0x9c, 0x31,
   0xc6, 0x10, 0x43, 0x00, 0x06, 0x00, 0x0c, 0x33, 0x86, 0x8c, 0x63, 0x08,
   0x60, 0xc0, 0x9b, 0x31, 0x86, 0x0f, 0x3e, 0x00, 0x04, 0x00, 0xfc, 0x31,
   0x7c, 0x78, 0xc3, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00 };

#define radio0_width 17
#define radio0_height 9
const unsigned char radio0_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio1_width 17
#define radio1_height 9
const unsigned char radio1_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio2_width 17
#define radio2_height 9
const unsigned char radio2_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x04, 0x00, 0x00, 0x44, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio3_width 17
#define radio3_height 9
const unsigned char radio3_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x00, 0x00, 0x04, 0x00, 0x00,
   0x04, 0x01, 0x00, 0x44, 0x01, 0x00, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio4_width 17
#define radio4_height 9
const unsigned char radio4_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x00, 0x00, 0x04, 0x04, 0x00,
   0x04, 0x05, 0x00, 0x44, 0x05, 0x00, 0x54, 0x05, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio5_width 17
#define radio5_height 9
const unsigned char radio5_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x15, 0x10, 0x00, 0x04, 0x14, 0x00,
   0x04, 0x15, 0x00, 0x44, 0x15, 0x00, 0x54, 0x15, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio6_width 17
#define radio6_height 9
const unsigned char radio6_bits[] PROGMEM = {
   0x11, 0x00, 0x00, 0x1f, 0x40, 0x00, 0x15, 0x50, 0x00, 0x04, 0x54, 0x00,
   0x04, 0x55, 0x00, 0x44, 0x55, 0x00, 0x54, 0x55, 0x00, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };

#define radio7_width 17
#define radio7_height 9
const unsigned char radio7_bits[] PROGMEM = {
   0x11, 0x00, 0x01, 0x1f, 0x40, 0x01, 0x15, 0x50, 0x01, 0x04, 0x54, 0x01,
   0x04, 0x55, 0x01, 0x44, 0x55, 0x01, 0x54, 0x55, 0x01, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x01 };


String Etiquette;
String Version_Linky;
String Step;
String Valeur;
unsigned long Nb_rcv = 0;
unsigned long Nb_sent = 0;

uint32_t SEND_FREQUENCY_DISPLAY = 1000; // Minimum time between send (in milliseconds). 
uint32_t lastTime_display = 0;
bool shouldSaveConfig = false;
bool mqttactive = false;
bool mqttconnected = false;
bool first_start = true;
const int RSSI_MAX =-50;          // define maximum strength of signal in dBm
const int RSSI_MIN =-100;         // define minimum strength of signal in dBm

char url_mqtt[MAX_BUFFER_URL];
char memo_url_mqtt[MAX_BUFFER_URL];
unsigned int port_mqtt;
unsigned int memo_port_mqtt;
char user_mqtt[MAX_BUFFER];
char memo_user_mqtt[MAX_BUFFER];
char pwd_mqtt[MAX_BUFFER];
char memo_pwd_mqtt[MAX_BUFFER];
char token_mqtt[MAX_BUFFER];
char memo_token_mqtt[MAX_BUFFER];
int lora_rssi=0;
unsigned int nb_boot_linky=0;
unsigned int nb_decode_failed=0;

unsigned int SINSTS=0;


String info_config = "";

DynamicJsonDocument jsonDocModule(1024);


uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
SSD1306Wire display(OLED_ADDR, OLED_SDA, OLED_SCL);
AsyncWebServer server(80);
WiFiClient client;
PubSubClient client_mqtt(client);
DNSServer dns;
WiFiManager wm;


//----------------------------------------------------------------------- loadConfig
void loadConfig() {

  if (SPIFFS.exists("/config.json")) {
    Serial.println(F("Lecture config.json"));
    File configFile = SPIFFS.open("/config.json", "r");
    if (configFile) {
      DynamicJsonDocument jsonDoc(512);
      DeserializationError error = deserializeJson(jsonDoc, configFile);
      if (error) Serial.println(F("Unable to parse config.cfg"));
      JsonObject json = jsonDoc.as<JsonObject>();
      Serial.println(F("Contenu:"));
      serializeJson(json, Serial);
  
      if (json["url_mqtt"].isNull() == false) {
        strcpy(url_mqtt, json["url_mqtt"]);
        strcpy(memo_url_mqtt, json["url_mqtt"]);
      }
      else url_mqtt[0] = 0;

      if (json["user_mqtt"].isNull() == false) {
        strcpy(user_mqtt, json["user_mqtt"]);
        strcpy(memo_user_mqtt, json["user_mqtt"]);
      }
      else user_mqtt[0] = 0;

      if (json["pwd_mqtt"].isNull() == false) {
        strcpy(pwd_mqtt, json["pwd_mqtt"]);
        strcpy(memo_pwd_mqtt, json["pwd_mqtt"]);
      }
      else pwd_mqtt[0] = 0;

      if (json["token_mqtt"].isNull() == false) {
        strcpy(token_mqtt, json["token_mqtt"]);
        strcpy(memo_token_mqtt, json["token_mqtt"]);
      }
      else token_mqtt[0] = 0;

      if (json["port_mqtt"].isNull() == false) {
        port_mqtt = json["port_mqtt"];
        if (port_mqtt == 0) port_mqtt = DEFAULT_PORT_MQTT;
        memo_port_mqtt = port_mqtt;
      }
      else port_mqtt = DEFAULT_PORT_MQTT;
      
      configFile.close();
    }
    else Serial.println(F("Unable to read config.json file !!"));
  }
  
}

//----------------------------------------------------------------------- saveConfig
void saveConfig() {

  Serial.println(F("Sauvegarde config.json"));

  
  File configFile = SPIFFS.open("/config.json", "w");
  DynamicJsonDocument jsonDoc(512);
  JsonObject json = jsonDoc.to<JsonObject>();

  json["version"] = VERSION;
  json["url_mqtt"] = url_mqtt;
  json["user_mqtt"] = user_mqtt;
  json["pwd_mqtt"] = pwd_mqtt;
  json["token_mqtt"] = token_mqtt;
  json["port_mqtt"] = port_mqtt;

  serializeJson(json, configFile);
  configFile.close();

}

//---------------------------------------------------------------- configModeCallback
void configModeCallback(WiFiManager *myWiFiManager) {

  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 5, "Config mode");
  display.drawString(64, 25, WiFi.softAPIP().toString());
  display.drawString(64, 45, myWiFiManager->getConfigPortalSSID());
  display.display();
}

//---------------------------------------------------------------- saveConfigCallback
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//---------------------------------------------------------------- start_mdns_service
void start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    mdns_hostname_set("fbgateway");
    //set default instance
    mdns_instance_name_set("FB Gateway");
}


//------------------------------------------------------------------ dBmtoPercentage
int dBmtoPercentage(int dBm)
{
  int quality;
    if(dBm <= RSSI_MIN)
    {
        quality = 0;
    }
    else if(dBm >= RSSI_MAX)
    {  
        quality = 100;
    }
    else
    {
        quality = 2 * (dBm + 100);
   }

     return quality;
}

//------------------------------------------------------------------ draw_rssi
void draw_rssi() {
      
  int rssi = dBmtoPercentage(WiFi.RSSI());

  if (rssi >= 0) display.drawXbm(100, 2, 17, 7, radio0_bits);
  if (rssi >= 20 && rssi < 30) display.drawXbm(100, 2, 17, 9, radio1_bits);
  if (rssi >= 30 && rssi < 40) display.drawXbm(100, 2, 17, 9, radio2_bits);
  if (rssi >= 40 && rssi < 50) display.drawXbm(100, 2, 17, 9, radio3_bits);
  if (rssi >= 50 && rssi < 65) display.drawXbm(100, 2, 17, 9, radio4_bits);
  if (rssi >= 65 && rssi < 75) display.drawXbm(100, 2, 17, 9, radio5_bits);
  if (rssi >= 75 && rssi < 90) display.drawXbm(100, 2, 17, 9, radio6_bits);
  if (rssi >= 90) display.drawXbm(100, 2, 17, 9, radio7_bits);
}


// ---------------------------------------------------- reconnect_mqtt
void reconnect_mqtt() {
  // Loop until we're reconnected
  while (!client_mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    info_config = "Cnx MQTT..";
    // Attempt to connect
    if (user_mqtt[0] != 0) { // si user renseigné
      if (client_mqtt.connect("FBGateway", user_mqtt, pwd_mqtt)) {
        Serial.println("connected");
        info_config = "MQTT connecté";
        mqttconnected = true;
      } else {
        Serial.print("failed, rc=");
        Serial.print(client_mqtt.state());
        info_config = "Pb cnx mqtt:" + String(client_mqtt.state());
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 45, info_config);
        display.display();
        mqttconnected = false;
      }
    }
    else {
      if (client_mqtt.connect("FBGateway")) {
        Serial.println("connected");
        info_config = "MQTT connecté";
        mqttconnected = true;
      } else {
        Serial.print("failed, rc=");
        Serial.print(client_mqtt.state());
        info_config = "Pb cnx mqtt:" + String(client_mqtt.state());
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 45, info_config);
        display.display();
        mqttconnected = false;
      }
    }
  }
}

// ---------------------------------------------------- traduction_etiquette
String traduction_etiquette(int etiq)
{
String rc;

  Serial.print("traduction etiquette:");
  Serial.print(etiq);
  Serial.print("->");

  switch (etiq) {
    case ETIQU_ADSC :
      rc = "ADSC";
      break;

    case ETIQU_VTIC :
      rc = "VTIC";
      break;

    case ETIQU_NGTF :
      rc = "NGTF";
      break;

    case ETIQU_LTARF :
      rc = "LTARF";
      break;

    case ETIQU_EAST:
      rc = "EAST";
      break;

    case ETIQU_IRMS1:
      rc = "IRMS1";
      break;

    case ETIQU_IRMS2:
      rc = "IRMS2";
      break;

    case ETIQU_IRMS3:
      rc = "IRMS3";
      break;

    case ETIQU_URMS1:
      rc = "URMS1";
      break;

    case ETIQU_URMS2:
      rc = "URMS2";
      break;

    case ETIQU_URMS3:
      rc = "URMS3";
      break;

    case ETIQU_PREF:
      rc = "PREF";
      break;

    case ETIQU_PCOUP:
      rc = "PCOUP";
      break;

    case ETIQU_SINSTS:
      rc = "SINSTS";
      break;

    case ETIQU_SINSTSmin:
      rc = "SINSTSmin";
      break;

    case ETIQU_SINSTSmax:
      rc = "SINSTSmax";
      break;

    case ETIQU_SINSTS1:
      rc = "SINSTS1";
      break;

    case ETIQU_SINSTS2:
      rc = "SINSTS2";
      break;

    case ETIQU_SINSTS3:
      rc = "SINSTS3";
      break;

    case ETIQU_STGE:
      rc = "STGE";
      break;

    case ETIQU_MSG1:
      rc = "MSG1";
      break;

    case ETIQU_NTARF:
      rc = "NTARF";
      break;

    case ETIQU_NJOURF:
      rc = "NJOURF";
      break;

    case ETIQU_NJOURF1:
      rc = "NJOURF1";
      break;

    case ETIQU_EAIT:
      rc = "EAIT";
      break;

    case ETIQU_SINSTI:
      rc = "SINSTSI";
      break;

    case ETIQU_EASF01:
      rc = "EASF01";
      break;

    case ETIQU_EASF02:
      rc = "EASF02";
      break;

    case ETIQU_EASF03:
      rc = "EASF03";
      break;

    case ETIQU_EASD01:
      rc = "EASD01";
      break;

    case ETIQU_EASD02:
      rc = "EASD02";
      break;

    case ETIQU_EASD03:
      rc = "EASD03";
      break;

    case ETIQU_ERQ1:
      rc = "ERQ1";
      break;

    case ETIQU_ERQ2:
      rc = "ERQ2";
      break;

    case ETIQU_ERQ3:
      rc = "ERQ3";
      break;

    case ETIQU_STEP:
      rc = "STEP";
      break;

    case ETIQU_BOOT:
      rc = "BOOT";
      break;

    default :
      rc = String(etiq);
      break;
  }

  Serial.println(rc);

  return rc;
}

// ---------------------------------------------------- traitement_data
void traitement_data(int origine, String data)
{
String mqtt_buffer;
int index=1;
int etiqu;
  
  Serial.print(F("traitement_data:"));
  Serial.println(data);

  while (index > 0) {  

    index = data.indexOf(";");
    if (index > 0) {
      // Recherche etiquette ---------
      etiqu = data.substring(0, index).toInt();
      Serial.print(etiqu);
      Serial.print(F("->"));

      // Recherche valeur -------------
      data = data.substring(index+1);
      index = data.indexOf(";");
      if (index > 0) {
        Valeur = data.substring(0, index);
        Serial.println(Valeur);

        data = data.substring(index+1);
        index = data.indexOf(";");

        Etiquette = traduction_etiquette(etiqu);

        // Recup valeur SINSTS pour affichage
        if (Etiquette == "SINSTS") SINSTS=Valeur.toInt();

        // Verif boot linky ----
        if (Etiquette == "BOOT") {
          nb_boot_linky++;
          Version_Linky = Valeur;
        }
        // Recup Step ----
        if (Etiquette == "STEP") {
          Step = Valeur;
        }

        // MQTT send --------------       
        if (mqttactive == true && mqttconnected == true) {
          mqtt_buffer = token_mqtt + String("/") + Etiquette;
          Serial.print(F("Send mqtt:"));
          Serial.print(mqtt_buffer);
          Serial.print("/");
          Serial.println(Valeur);
          //client_mqtt.publish(mqtt_buffer.c_str(), Valeur.c_str(), true); 
          client_mqtt.publish(mqtt_buffer.c_str(), Valeur.c_str());
          Nb_sent++;
        }
      }
    }
    lastTime_display = millis();
  } 
}

// ---------------------------------------------------- onReceive
void onReceive(int packetSize) {
String read_buffer;
int origine;
int destinataire;
int index;
 
    
  if (packetSize) {
    read_buffer = LoRa.readString();
    lora_rssi = LoRa.packetRssi();

    Serial.print(F("Receive:"));
    Serial.println(read_buffer);

    // Test entête présente -----------
    if (read_buffer.charAt(0) == ENTETE) {
      index = 0;
      // Recherche origine ----------------
      read_buffer = read_buffer.substring(index+1);
      index = read_buffer.indexOf(";");
      if (index > 0) {
        origine = read_buffer.substring(0, index).toInt();
        Serial.print("Origine:");
        Serial.println(origine);

        // Recherche destinataire ----------------
        read_buffer = read_buffer.substring(index+1);
        index = read_buffer.indexOf(";");
        if (index > 0) {
          destinataire = read_buffer.substring(0, index).toInt();
          Serial.print("Dest:");
          Serial.println(destinataire);

          if (destinataire == GATEWAY_ADDRESS) { // est-ce pour moi ?
            Nb_rcv++;
            read_buffer = read_buffer.substring(index+1);
            //Serial.println(read_buffer);    
            // Decode données -----------------
            read_buffer = xxtea.decrypt(read_buffer);
            if (read_buffer != "-FAIL-") traitement_data(origine, read_buffer);
              else nb_decode_failed++;
          } // destinataire
        }
      }
    } // read_buffer
  } // packetSize
}

//----------------------------------------------------------------------- page_config_json
void page_info_json(AsyncWebServerRequest *request)
{
String strJson = "{\n";

  Serial.println(F("Page config.json"));
  
  // version ---------------------
  strJson += F("\"version\": \"");
  strJson += VERSION;
  strJson += F("\",\n");

  // boot_linky ---------------------
  strJson += F("\"boot_linky\": \"");
  strJson += nb_boot_linky;
  strJson += F("\",\n");

  // Version Linky ---------------------
  strJson += F("\"Version Linky\": \"");
  strJson += Version_Linky;
  strJson += F("\",\n");

  // lora_rssi ---------------------
  strJson += F("\"lora_rssi\": \"");
  strJson += lora_rssi;
  strJson += F("\",\n");

  // nb_rcv ---------------------
  strJson += F("\"nb_rcv\": \"");
  strJson += Nb_rcv;
  strJson += F("\",\n");

  // nb_decode_failed ---------------------
  strJson += F("\"nb_decode_failed\": \"");
  strJson += nb_decode_failed;
  strJson += F("\",\n");
    
  // nb_sent ---------------------
  strJson += F("\"nb_sent\": \"");
  strJson += Nb_sent;
  strJson += F("\",\n");

  // Etiquette ---------------------
  strJson += F("\"etiquette\": \"");
  strJson += Etiquette;
  strJson += F("\",\n");

  // Step ---------------------
  strJson += F("\"step\": \"");
  strJson += Step;
  strJson += F("\",\n");

  // info_config ---------------------
  strJson += F("\"info_config\": \"");
  strJson += info_config;
  strJson += F("\"\n");

  strJson += F("}");

  request->send(200, "text/json", strJson);
}


//----------------------------------------------------------------------- page_config_json
void page_config_json(AsyncWebServerRequest *request)
{
String strJson = "{\n";

  Serial.println(F("Page config.json"));
  
  // version ---------------------
  strJson += F("\"version\": \"");
  strJson += VERSION;
  strJson += F("\",\n");

  // url mqtt ---------------------
  strJson += F("\"url_mqtt\": \"");
  strJson += url_mqtt;
  strJson += F("\",\n");

  // port mqtt ---------------------
  strJson += F("\"port_mqtt\": \"");
  strJson += port_mqtt;
  strJson += F("\",\n");

  // user mqtt ---------------------
  strJson += F("\"user_mqtt\": \"");
  strJson += user_mqtt;
  strJson += F("\",\n");

  // password mqtt ---------------------
  strJson += F("\"pwd_mqtt\": \"");
  strJson += pwd_mqtt;
  strJson += F("\",\n");

  // token mqtt ---------------------
  strJson += F("\"token_mqtt\": \"");
  strJson += token_mqtt;
  strJson += F("\"\n");

  strJson += F("}");

  request->send(200, "text/json", strJson);
}

//----------------------------------------------------------------------- page_config_htm
void page_config_htm(AsyncWebServerRequest *request)
{
boolean flag_restart = false;


  Serial.println(F("Page config"));

  int params = request->params();
  for(int i=0;i<params;i++){
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    
    if (strstr(p->name().c_str(), "url_mqtt")) strcpy(url_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "port_mqtt")) port_mqtt = atoi(p->value().c_str());
    if (strstr(p->name().c_str(), "user_mqtt")) strcpy(user_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "pwd_mqtt")) strcpy(pwd_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "token_mqtt")) strcpy(token_mqtt, p->value().c_str());
    
    // check if restart required 
    if (strcmp(url_mqtt, memo_url_mqtt) != 0) flag_restart = true;
    if (strcmp(user_mqtt, memo_user_mqtt) != 0) flag_restart = true;
    if (strcmp(pwd_mqtt, memo_pwd_mqtt) != 0) flag_restart = true;
    if (strcmp(token_mqtt, memo_token_mqtt) != 0) flag_restart = true;
    if (port_mqtt != memo_port_mqtt) flag_restart = true;
  }
  saveConfig();
  request->send (200, "text/plain", "OK");

  if (flag_restart == true) {
    info_config = "Reboot module";
    ESP.restart();
  }  
}

//-----------------------------------------------------------------------
void loadPages()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/w3.css", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/w3.css", "text/css");
  });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/script.js", "text/javascript");
  });

  server.on("/jquery.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/jquery.js", "text/javascript");
  });

  server.on("/notify.js", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/notify.js", "text/javascript");
  });

  server.on("/fb.svg", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/fb.svg", "image/svg+xml");
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/favicon.ico", "image/x-icon");
  });

   server.on("/info.json", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    page_info_json(request);
  });

  server.on("/config.json", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    page_config_json(request);
  });

  server.on("/config.htm", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    page_config_htm(request);
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.println("Page not found");
    Serial.println(request->method());
    Serial.println(request->url());
    request->send(404, "text/plain", "The content you are looking for was not found.");
  });
}


// ---------------------------------------------------- SETUP
void setup()
{
  Serial.begin(MY_BAUD_RATE);
  //WiFi.mode(WIFI_STA);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  
  Serial.println(F("   __|              _/           _ )  |"));
  Serial.println(F("   _| |  |   ` \\    -_)   -_)    _ \\  |   -_)  |  |   -_)"));
  Serial.println(F("  _| \\_,_| _|_|_| \\___| \\___|   ___/ _| \\___| \\_,_| \\___|"));
  Serial.print(F("                                             "));
  Serial.println(VERSION);

  SPIFFS.begin();
  loadConfig();

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
    
  display.clear();
  display.drawString(3, 8, VERSION);
  display.drawXbm(0, 0, 128, 64, fb_bits);

  display.display();
  WiFiManager wm;

  wm.setAPCallback(configModeCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setMinimumSignalQuality(10);
  wm.setConfigPortalTimeout(360);
  wm.setClass("invert"); // dark theme

  if (digitalRead(TRIGGER_PIN) == LOW) {
    Serial.println("Start ConfigPortal");
    wm.startConfigPortal("FB Gateway");
  }
  else wm.autoConnect("FB Gateway");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  xxtea.setKey(CRYPT_PASS);

  LoRa.setPins(SX1278_CS, SX1278_RST, SX1278_IRQ);
  
  Serial.print(F("Init RF95: "));
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting RFM95 failed!");
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(3, 10, "RFM95 failed!!");
    display.display();
    while (1);
  }
  else {
    LoRa.enableCrc();
    LoRa.setTxPower(RFM_TX_POWER);
    Serial.println("OK.");
  }

  loadPages();
  server.begin();
  
  start_mdns_service();

}


// ---------------------------------------------------- loop
void loop()
{
  uint32_t currentTime = millis();
  

  onReceive(LoRa.parsePacket());

  if (url_mqtt[0] != 0 && token_mqtt[0] != 0) mqttactive = true;
    else mqttactive = false;
  
  if (currentTime - lastTime_display > SEND_FREQUENCY_DISPLAY || first_start == true) {
    display.clear();
    
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    if (!mqttactive) {
      info_config = "mqtt non configuré";
      display.drawString(64, 53, info_config);
    }
    else {
      if (mqttconnected) info_config = "mqtt connecté";
      display.drawString(64, 53, info_config);
    }
    display.setFont(ArialMT_Plain_16); 
    display.drawString(64, 30, String(SINSTS) + " VA");
    
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(1, 1, WiFi.localIP().toString());
    display.drawString(2, 15, String(Nb_rcv) + "/" + String(Nb_sent));
        
    draw_rssi();

    display.display();
    lastTime_display = currentTime;
  }

  // premier démarrage, config mqtt
  if (first_start) {
    first_start = false;
    Serial.print("Cnx mqtt:");
    Serial.print(url_mqtt);
    Serial.print(":");
    Serial.println(port_mqtt);
    client_mqtt.setServer(url_mqtt, port_mqtt);
    
  }

  // mqtt actif ?
  if (mqttactive) {
    if (!client_mqtt.connected()) {
      reconnect_mqtt();
    }
    client_mqtt.loop();
  }
  
}