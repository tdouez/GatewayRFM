/*

  RAW Use example of xxtea-lib Cryptographic Library

  This example shows the calling convention for the various functions.

  For more information about this library please visit us at
  http://github.com/boseji/xxtea-lib

  Created by Abhijit Bose (boseji) on 04/03/16.
  Copyright 2016 - Under creative commons license 3.0:
        Attribution-ShareAlike CC BY-SA

  @version API 2.0.0
  @author boseji - salearj@hotmail.com

*/

#include <xxtea-lib.h>

void setup() {
  Serial.begin(115200);

  Serial.println();
  // Key for Encrypt - ! Carefull no to more than 16 bytes ! - Or See `Limitations`
  uint8_t keybuf[] = "Hello Password";
  uint8_t plaintext[] = "Hi There we can work with this";
  uint8_t buffer[200];
  size_t len = 200, i;

  // Setup the Key - Once
  if(xxtea_setup_key(keybuf, strlen((char *)keybuf)) != XXTEA_STATUS_SUCCESS) {
    Serial.println(" Assignment Failed!");
    return;
  }

  // Perform Encryption on the Data
  len = 200;  // - Initialize the Maximum buffer length
  if(xxtea_encrypt(plaintext, strlen((char*)plaintext), buffer, &len) != XXTEA_STATUS_SUCCESS) {
    Serial.println(" Encryption Failed!");
    return;
  } else {
    Serial.println(" Encrypted Data: ");
    for(i = 0;i<len;i++)
      Serial.print(buffer[i], HEX);
    Serial.println();
  }

  // Perform Decryption
  if(xxtea_decrypt(buffer, len) != XXTEA_STATUS_SUCCESS) {
    Serial.println(" Decryption Failed!");
    return;
  } else {
    Serial.print(" Decrypted Data: ");
    Serial.println((char *)buffer);
  }
}

void loop() {}
