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
// 2021/08/09 - FB V1.02 - Add POST request
// 2021/08/30 - FB V1.03 - Update POST request
// 2021/09/01 - FB V1.04 - Add module name
// 2021/12/01 - FB V1.05 - Not blocking Mqtt reconnect & change icons display
//--------------------------------------------------------------------
#include <Arduino.h>
#include <WiFiManager.h>
#include <WiFiClient.h>
//#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoHttpClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h> 
#include <PubSubClient.h>
#include "SSD1306Wire.h"
#include <mdns.h>
#include <SPIFFS.h>
#define MAX_XXTEA_DATA8  200
#include <xxtea-lib.h>


#define VERSION   "v1.0.5"

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

// 'mqtt_tiny', 20x20px
const unsigned char mqtt_tiny[] PROGMEM = {
  0xFF, 0xC1, 0x0F, 0xFF, 0x07, 0x0F, 0xF8, 0x0F, 0x0E, 0x80, 0x3F, 0x0C, 
  0x00, 0x7E, 0x08, 0x0F, 0xFC, 0x08, 0x7F, 0xF0, 0x01, 0xFF, 0xE0, 0x01, 
  0xFD, 0xC3, 0x03, 0xE0, 0xC7, 0x07, 0x80, 0x8F, 0x07, 0x07, 0x1F, 0x0F, 
  0x1F, 0x1E, 0x0F, 0x3F, 0x3C, 0x0E, 0x7F, 0x3C, 0x0E, 0xFF, 0x78, 0x0E, 
  0xFB, 0x78, 0x0C, 0xFF, 0x78, 0x0C, 0xFF, 0xF9, 0x0C, 0xFF, 0x71, 0x0C, 
  };

// 'post_tiny', 20x19px
const unsigned char post_tiny[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x40, 0x00, 0x00, 0xC0, 0x00, 
  0x00, 0xE0, 0x01, 0x00, 0xFC, 0x03, 0x04, 0xEF, 0x07, 0xC2, 0xFF, 0x07, 
  0xC1, 0xFF, 0x03, 0xE1, 0xE3, 0x01, 0x61, 0xC0, 0x00, 0x11, 0x40, 0x00, 
  0x01, 0x20, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 
  0x01, 0x80, 0x00, 0x03, 0x40, 0x00, 0xFF, 0x3F, 0x00,
  };

String Etiquette;
String Version_Linky;
String Step;
String Valeur;
unsigned long Nb_rcv = 0;
unsigned long Nb_sent_mqtt = 0;
unsigned long Nb_sent_post = 0;

uint32_t SEND_FREQUENCY_DISPLAY = 1000; // Minimum time between send (in milliseconds). 
uint32_t lastTime_display = 0;
bool shouldSaveConfig = false;
bool mqttactive = false;
bool postactive = false;
bool mqttconnected = false;
bool first_start = true;
const int RSSI_MAX =-50;          // define maximum strength of signal in dBm
const int RSSI_MIN =-100;         // define minimum strength of signal in dBm

char module_name[MAX_BUFFER];
char memo_module_name[MAX_BUFFER];
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
char url_post[MAX_BUFFER];
char memo_url_post[MAX_BUFFER];
char token_post[MAX_BUFFER];
char memo_token_post[MAX_BUFFER];
int lora_rssi=0;
unsigned int nb_boot_linky=0;
unsigned int nb_decode_failed=0;
int httpCode=0;
char buffer[64]; 
long lastReconnectAttempt = 0;

String info_config = "";

struct teleinfo_s {
  String _ADSC = "";  // Adresse Compteur
  String VTIC = "";    
  String NGTF="";
  String LTARF="";  // Libelle tarif
  unsigned long EAST=0; // Energie active soutiree totale
  unsigned long EAIT=0; // Energie active injectee
  unsigned int IRMS1=0; // Courant efficace, phase 1
  unsigned int IRMS2=0; // Courant efficace, phase 2
  unsigned int IRMS3=0; // Courant efficace, phase 3
  unsigned int URMS1=0; // Tension efficace, phase 1
  unsigned int URMS2=0; // Tension efficace, phase 2
  unsigned int URMS3=0; // Tension efficace, phase 3
  unsigned int PREF=0;
  unsigned int PCOUP=0; // Puissance coupure
  unsigned int SINSTS=0; // Puissance apparente
  unsigned int SINSTSmin=0;
  unsigned int SINSTSmax=0;
  unsigned int SINSTS1=0; // Puissance apparente phase 1
  unsigned int SINSTS2=0; // Puissance apparente phase 2
  unsigned int SINSTS3=0; // Puissance apparente phase 3
  unsigned int SINSTI=0; // Puissance apparente injectee
  String STGE=""; // Registre de Statuts
  String MSG1="";
  String NTARF=""; // Index tarifaire en cours
  String NJOURF=""; //Jour en cours
  String NJOURF1=""; // Prochain jour
  unsigned long EASF01=0;
  unsigned long EASF02=0;
  unsigned long EASF03=0;
  unsigned long EASD01=0;
  unsigned long EASD02=0;
  unsigned long EASD03=0;
  unsigned long ERQ1=0; // Energie reactive Q1 totale
  unsigned long ERQ2=0; // Energie reactive Q2 totale
  unsigned long ERQ3=0; // Energie reactive Q3 totale
  unsigned long ERQ4=0; // Energie reactive Q4 totale
} teleinfo; 

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
  
      if (json["module_name"].isNull() == false) {
        strcpy(module_name, json["module_name"]);
        if (module_name[0] == 0) sprintf(module_name, "EMT_%06X", ESP.getChipCores());
        strcpy(memo_module_name, module_name);
      } 
      else sprintf(module_name, "EMT_%06X", ESP.getChipCores());

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

      if (json["url_post"].isNull() == false) {
        strcpy(url_post, json["url_post"]);
        strcpy(memo_url_post, json["url_post"]);
      }
      else url_post[0] = 0;

      if (json["token_post"].isNull() == false) {
        strcpy(token_post, json["token_post"]);
        strcpy(memo_token_post, json["token_post"]);
      }
      else token_post[0] = 0;
      
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

  json["module_name"] = module_name;
  json["url_mqtt"] = url_mqtt;
  json["user_mqtt"] = user_mqtt;
  json["pwd_mqtt"] = pwd_mqtt;
  json["token_mqtt"] = token_mqtt;
  json["port_mqtt"] = port_mqtt;
  json["url_post"] = url_post;
  json["token_post"] = token_post;

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

//------------------------------------------------------------------ draw_display
void draw_display() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // draw ip --------------------
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(1, 1, WiFi.localIP().toString());
  // draw wifi level -------------
  draw_rssi();
  // draw first separator --------------
  display.drawLine(0, 12, 128, 12);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 20, String(teleinfo.SINSTS) + " VA");
  display.drawString(64, 37, String(Nb_rcv));

  if (mqttconnected) {
    display.drawXbm(8, 15, 20, 20, mqtt_tiny);
    //display.drawString(2, 15, String(Nb_rcv) + " / " + String(Nb_sent_mqtt));
    display.drawString(2, 37, String(Nb_sent_mqtt));
  }

  if (postactive) {
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawXbm(105, 15, 20, 19, post_tiny);
    //display.drawString(2, 15, String(Nb_rcv) + " / " + String(Nb_sent_mqtt));
    display.drawString(128, 37, String(Nb_sent_post));
  }

  // draw second separator --------------
  display.drawLine(0, 52, 128, 52);
  // draw information --------------
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 54, info_config);

  display.display();
}

// ---------------------------------------------------- reconnect_mqtt
boolean reconnect_mqtt() {
  int nb_cnx = 0;

  // Attempt to connect
  if (user_mqtt[0] != 0) { // si user renseigné
    if (client_mqtt.connect("FBGateway", user_mqtt, pwd_mqtt)) {
      Serial.println("connected");
      mqttconnected = true;
      return client.connected();
    } 
    else {
      nb_cnx++;
      Serial.print("failed, rc=");
      Serial.print(client_mqtt.state());
      info_config = "Pb cnx mqtt:" + String(client_mqtt.state()) + " " + String(nb_cnx);
      mqttconnected = false;
    }
  }
  else {
    if (client_mqtt.connect("FBGateway")) {
      Serial.println("connected");
      mqttconnected = true;
      return client.connected();
    } 
    else {
      nb_cnx++;
      Serial.print("failed, rc=");
      Serial.print(client_mqtt.state());
      info_config = "Pb cnx mqtt:" + String(client_mqtt.state()) + " " + String(nb_cnx);
      mqttconnected = false;
    }
  }
  return false;
}

// ---------------------------------------------------- traduction_etiquette
String traduction_etiquette(int etiq, String valeur)
{
String rc;

  Serial.print("traduction etiquette:");
  Serial.print(etiq);
  Serial.print("->");

  switch (etiq) {
    case ETIQU_ADSC :
      teleinfo._ADSC = Valeur;
      rc = "ADSC";
      break;

    case ETIQU_VTIC :
      teleinfo.VTIC = Valeur;
      rc = "VTIC";
      break;

    case ETIQU_NGTF :
      teleinfo.NGTF = Valeur;
      rc = "NGTF";
      break;

    case ETIQU_LTARF :
      teleinfo.LTARF = Valeur;
      rc = "LTARF";
      break;

    case ETIQU_EAST:
      teleinfo.EAST = Valeur.toDouble();
      rc = "EAST";
      break;

    case ETIQU_IRMS1:
      teleinfo.IRMS1 = Valeur.toInt();
      rc = "IRMS1";
      break;

    case ETIQU_IRMS2:
      teleinfo.IRMS2 = Valeur.toInt();
      rc = "IRMS2";
      break;

    case ETIQU_IRMS3:
      teleinfo.IRMS3 = Valeur.toInt();
      rc = "IRMS3";
      break;

    case ETIQU_URMS1:
      teleinfo.URMS1 = Valeur.toInt();
      rc = "URMS1";
      break;

    case ETIQU_URMS2:
      teleinfo.URMS2 = Valeur.toInt();
      rc = "URMS2";
      break;

    case ETIQU_URMS3:
      teleinfo.URMS3 = Valeur.toInt();
      rc = "URMS3";
      break;

    case ETIQU_PREF:
      teleinfo.PREF = Valeur.toInt();
      rc = "PREF";
      break;

    case ETIQU_PCOUP:
      teleinfo.PCOUP = Valeur.toInt();
      rc = "PCOUP";
      break;

    case ETIQU_SINSTS:
      teleinfo.SINSTS = Valeur.toInt();
      rc = "SINSTS";
      break;

    case ETIQU_SINSTSmin:
      teleinfo.SINSTSmin = Valeur.toInt();
      rc = "SINSTSmin";
      break;

    case ETIQU_SINSTSmax:
      teleinfo.SINSTSmax = Valeur.toInt();
      rc = "SINSTSmax";
      break;

    case ETIQU_SINSTS1:
      teleinfo.SINSTS1 = Valeur.toInt();
      rc = "SINSTS1";
      break;

    case ETIQU_SINSTS2:
      teleinfo.SINSTS2 = Valeur.toInt();
      rc = "SINSTS2";
      break;

    case ETIQU_SINSTS3:
      teleinfo.SINSTS3 = Valeur.toInt();
      rc = "SINSTS3";
      break;

    case ETIQU_STGE:
      teleinfo.STGE = Valeur;
      rc = "STGE";
      break;

    case ETIQU_MSG1:
      teleinfo.MSG1 = Valeur;
      rc = "MSG1";
      break;

    case ETIQU_NTARF:
      teleinfo.NTARF = Valeur;
      rc = "NTARF";
      break;

    case ETIQU_NJOURF:
      teleinfo.NJOURF = Valeur;
      rc = "NJOURF";
      break;

    case ETIQU_NJOURF1:
      teleinfo.NJOURF1 = Valeur;
      rc = "NJOURF1";
      break;

    case ETIQU_EAIT:
      teleinfo.EAIT = Valeur.toDouble();
      rc = "EAIT";
      break;

    case ETIQU_SINSTI:
      teleinfo.SINSTI = Valeur.toInt();
      rc = "SINSTSI";
      break;

    case ETIQU_EASF01:
      teleinfo.EASF01 = Valeur.toDouble();
      rc = "EASF01";
      break;

    case ETIQU_EASF02:
      teleinfo.EASF02 = Valeur.toDouble();
      rc = "EASF02";
      break;

    case ETIQU_EASF03:
      teleinfo.EASF03 = Valeur.toDouble();
      rc = "EASF03";
      break;

    case ETIQU_EASD01:
      teleinfo.EASD01 = Valeur.toDouble();
      rc = "EASD01";
      break;

    case ETIQU_EASD02:
      teleinfo.EASD02 = Valeur.toDouble();
      rc = "EASD02";
      break;

    case ETIQU_EASD03:
      teleinfo.EASD03 = Valeur.toDouble();
      rc = "EASD03";
      break;

    case ETIQU_ERQ1:
      teleinfo.ERQ1 = Valeur.toDouble();
      rc = "ERQ1";
      break;

    case ETIQU_ERQ2:
      teleinfo.ERQ2 = Valeur.toDouble();
      rc = "ERQ2";
      break;

    case ETIQU_ERQ3:
      teleinfo.ERQ3 = Valeur.toDouble();
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
String mqtt_buffer, url;
int index=1;
int etiqu;
boolean flag_first = true;
  
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

        Etiquette = traduction_etiquette(etiqu, Valeur);

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

          client_mqtt.publish(mqtt_buffer.c_str(), Valeur.c_str());
          Nb_sent_mqtt++;
        }

        // POST init ----------
        if (postactive == true) {
          if (flag_first == true) {
            flag_first = false;
            url = F("/maj_post.php?token=");
            url += token_post;
          }
          url += F("&");
          url += Etiquette;
          url += F("=");
          url += Valeur;
        }
      }
    }
	
    // POST send ----------
    if (postactive == true) {
      Serial.print(F("Send post:"));
      Serial.println(url);

      HttpClient http_client = HttpClient(client, url_post, 80);
      http_client.get(url);
      httpCode = http_client.responseStatusCode();

      if(httpCode > 0) {
        Serial.print(F("Retour http get: "));
        Nb_sent_post++;
      }
      else {
        Serial.print(F("Erreur http get: "));
        info_config = "Erreur http get:" + httpCode;
      }
      Serial.println(httpCode);
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
  
  // module name---------------------
  strJson += F("\"module_name\": \"");
  strJson += module_name;
  strJson += F("\",\n");

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
    
  // nb_sent_mqtt ---------------------
  strJson += F("\"nb_sent_mqtt\": \"");
  strJson += Nb_sent_mqtt;
  strJson += F("\",\n");

  // nb_sent_post ---------------------
  strJson += F("\"nb_sent_post\": \"");
  strJson += Nb_sent_post;
  strJson += F("\",\n");

  // Etiquette ---------------------
  strJson += F("\"etiquette\": \"");
  strJson += Etiquette;
  strJson += F("\",\n");

  // Step ---------------------
  strJson += F("\"step\": \"");
  strJson += Step;
  strJson += F("\",\n");

  // httpCode ---------------------
  strJson += F("\"httpCode\": \"");
  strJson += httpCode;
  strJson += F("\",\n");
  
  // info_config ---------------------
  strJson += F("\"info_config\": \"");
  strJson += info_config;
  strJson += F("\"\n");

  strJson += F("}");

  request->send(200, "text/json", strJson);
}

//----------------------------------------------------------------------- page_config_json
void page_teleinfo_json(AsyncWebServerRequest *request)
{
String strJson = "{\n";

  Serial.println(F("Page teleinfo.json"));
  
  // ADSC ---------------------
  strJson += F("\"ADSC\": \"");
  strJson += teleinfo._ADSC;
  strJson += F("\",\n");

  // VTIC ---------------------
  strJson += F("\"VTIC\": \"");
  strJson += teleinfo.VTIC;
  strJson += F("\",\n");

  // NGTF ---------------------
  strJson += F("\"NGTF\": \"");
  strJson += teleinfo.NGTF;
  strJson += F("\",\n");

  // LTARF ---------------------
  strJson += F("\"LTARF\": \"");
  strJson += teleinfo.LTARF;
  strJson += F("\",\n");

  // EAST ---------------------
  strJson += F("\"EAST\": \"");
  strJson += teleinfo.EAST;
  strJson += F("\",\n");

  // EAIT ---------------------
  strJson += F("\"EAIT\": \"");
  strJson += teleinfo.EAIT;
  strJson += F("\",\n");

  // IRMS1 ---------------------
  strJson += F("\"IRMS1\": \"");
  strJson += teleinfo.IRMS1;
  strJson += F("\",\n");

  // IRMS2 ---------------------
  strJson += F("\"IRMS2\": \"");
  strJson += teleinfo.IRMS2;
  strJson += F("\",\n");

  // IRMS3 ---------------------
  strJson += F("\"IRMS3\": \"");
  strJson += teleinfo.IRMS3;
  strJson += F("\",\n");

  // URMS1 ---------------------
  strJson += F("\"URMS1\": \"");
  strJson += teleinfo.URMS1;
  strJson += F("\",\n");

  // URMS2 ---------------------
  strJson += F("\"URMS2\": \"");
  strJson += teleinfo.URMS2;
  strJson += F("\",\n");

  // URMS3 ---------------------
  strJson += F("\"URMS3\": \"");
  strJson += teleinfo.URMS3;
  strJson += F("\",\n");
    
  // PREF ---------------------
  strJson += F("\"PREF\": \"");
  strJson += teleinfo.PREF;
  strJson += F("\",\n");

  // PCOUP ---------------------
  strJson += F("\"PCOUP\": \"");
  strJson += teleinfo.PCOUP;
  strJson += F("\",\n");

  // SINSTS ---------------------
  strJson += F("\"SINSTS\": \"");
  strJson += teleinfo.SINSTS;
  strJson += F("\",\n");

  // SINSTSmin ---------------------
  strJson += F("\"SINSTSmin\": \"");
  strJson += teleinfo.SINSTSmin;
  strJson += F("\",\n");

  // SINSTSmax ---------------------
  strJson += F("\"SINSTSmax\": \"");
  strJson += teleinfo.SINSTSmax;
  strJson += F("\",\n");

  // SINSTS1 ---------------------
  strJson += F("\"SINSTS1\": \"");
  strJson += teleinfo.SINSTS1;
  strJson += F("\",\n");

  // SINSTS2 ---------------------
  strJson += F("\"SINSTS2\": \"");
  strJson += teleinfo.SINSTS2;
  strJson += F("\",\n");

  // SINSTS3 ---------------------
  strJson += F("\"SINSTS3\": \"");
  strJson += teleinfo.SINSTS3;
  strJson += F("\",\n");

  // SINSTI ---------------------
  strJson += F("\"SINSTI\": \"");
  strJson += teleinfo.SINSTI;
  strJson += F("\",\n");

  // STGE ---------------------
  strJson += F("\"STGE\": \"");
  strJson += teleinfo.STGE;
  strJson += F("\",\n");

  // MSG1 ---------------------
  strJson += F("\"MSG1\": \"");
  strJson += teleinfo.MSG1;
  strJson += F("\",\n");

  // NTARF ---------------------
  strJson += F("\"NTARF\": \"");
  strJson += teleinfo.NTARF;
  strJson += F("\",\n");

  // NJOURF ---------------------
  strJson += F("\"NJOURF\": \"");
  strJson += teleinfo.NJOURF;
  strJson += F("\",\n");

  // NJOURF1 ---------------------
  strJson += F("\"NJOURF1\": \"");
  strJson += teleinfo.NJOURF1;
  strJson += F("\",\n");

  // EASF01 ---------------------
  strJson += F("\"EASF01\": \"");
  strJson += teleinfo.EASF01;
  strJson += F("\",\n");

  // EASF02 ---------------------
  strJson += F("\"EASF02\": \"");
  strJson += teleinfo.EASF02;
  strJson += F("\",\n");

  // EASF03 ---------------------
  strJson += F("\"EASF03\": \"");
  strJson += teleinfo.EASF03;
  strJson += F("\",\n");

  // EASD01 ---------------------
  strJson += F("\"EASD01\": \"");
  strJson += teleinfo.EASD01;
  strJson += F("\",\n");

  // EASD02 ---------------------
  strJson += F("\"EASD02\": \"");
  strJson += teleinfo.EASD02;
  strJson += F("\",\n");

  // EASD03 ---------------------
  strJson += F("\"EASD03\": \"");
  strJson += teleinfo.EASD03;
  strJson += F("\",\n");

  // ERQ1 ---------------------
  strJson += F("\"ERQ1\": \"");
  strJson += teleinfo.ERQ1;
  strJson += F("\",\n");

  // ERQ2 ---------------------
  strJson += F("\"ERQ2\": \"");
  strJson += teleinfo.ERQ2;
  strJson += F("\",\n");

  // ERQ3 ---------------------
  strJson += F("\"ERQ3\": \"");
  strJson += teleinfo.ERQ3;
  strJson += F("\",\n");
  
  // ERQ4 ---------------------
  strJson += F("\"ERQ4\": \"");
  strJson += teleinfo.ERQ4;
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

  // module_name ---------------------
  strJson += F("\"module_name\": \"");
  strJson += module_name;
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
  strJson += F("\",\n");

  // url post ---------------------
  strJson += F("\"url_post\": \"");
  strJson += url_post;
  strJson += F("\",\n");

  // token post ---------------------
  strJson += F("\"token_post\": \"");
  strJson += token_post;
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
    
    if (strstr(p->name().c_str(), "module_name")) strcpy(module_name, p->value().c_str());
    if (strstr(p->name().c_str(), "url_mqtt")) strcpy(url_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "port_mqtt")) port_mqtt = atoi(p->value().c_str());
    if (strstr(p->name().c_str(), "user_mqtt")) strcpy(user_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "pwd_mqtt")) strcpy(pwd_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "token_mqtt")) strcpy(token_mqtt, p->value().c_str());
    if (strstr(p->name().c_str(), "url_post")) strcpy(url_post, p->value().c_str());
    if (strstr(p->name().c_str(), "token_post")) strcpy(token_post, p->value().c_str());
    
    // check if restart required 
    if (strcmp(module_name, memo_module_name) != 0) flag_restart = true;
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

  server.on("/elec.png", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/elec.png", "image/png");
  });

  server.on("/lora.png", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/lora.png", "image/png");
  });

  server.on("/post.png", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/post.png", "image/png");
  });

  server.on("/mqtt.png", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/mqtt.png", "image/png");
  });

  server.on("/lora.png", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/lora.png", "image/png");
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

  server.on("/teleinfo.json", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    page_teleinfo_json(request);
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
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
    
  display.clear();
  display.drawString(3, 8, VERSION);
  display.drawXbm(0, 0, 128, 64, fb_bits);

  display.display();

  //----------------------------------------------------WIFI
  WiFiManager wm;

  WiFi.hostname(module_name);
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

  //----------------------------------------------------LORA
  xxtea.setKey(CRYPT_PASS);

  LoRa.setPins(SX1278_CS, SX1278_RST, SX1278_IRQ);
  
  Serial.print(F("Init RF95: "));
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting RFM95 failed!");
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
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

  //----------------------------------------------------SERVER
  loadPages();
  server.begin();
  
  //----------------------------------------------------MDSN
  start_mdns_service();
  
}


// ---------------------------------------------------- loop
void loop()
{
  uint32_t currentTime = millis();
  

  onReceive(LoRa.parsePacket());

  if (url_mqtt[0] != 0 && token_mqtt[0] != 0) mqttactive = true;
    else mqttactive = false;

  if (url_post[0] != 0 && token_post[0] != 0) postactive = true;
        else postactive = false;
  
  if (currentTime - lastTime_display > SEND_FREQUENCY_DISPLAY || first_start == true) {
    
    draw_display();

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
      long now = millis();
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;
        // Attempt to reconnect
        if (reconnect_mqtt()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      // Client connected
      client_mqtt.loop();
    }
  }

  
  
}