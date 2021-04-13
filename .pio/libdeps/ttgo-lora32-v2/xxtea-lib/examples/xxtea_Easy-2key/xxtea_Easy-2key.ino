/*

  Easy to Use example of xxtea-lib Cryptographic Library
  Describing how to use 2 keys.

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

  // Text to Encrypt - ! Carefull no to more than 80 bytes ! - Or See `Limitations`
  String plaintext1 = F("Hi There we can work with this");
  String plaintext2 = F("Hi This is the second test string we Wish to Encrypt");
  // Key for Encrypt - ! Carefull no to more than 16 bytes ! - Or See `Limitations`
  String key1 = F("First Pasword");
  String key2 = F("Second Pasword");

  // Set the Password
  xxtea.setKey(key1);

  // Perform Encryption on the Data
  Serial.print(F(" Encrypted Data 1: "));
  String result = xxtea.encrypt(plaintext1);
  result.toLowerCase(); // (Optional)
  Serial.println(result);

  // Perform Decryption
  Serial.print(F(" Decrypted Data 1: "));
  Serial.println(xxtea.decrypt(result));

  // Set the second Password
  xxtea.setKey(key2);

  // Perform Encryption on the Data
  Serial.print(F(" Encrypted Data 2: "));
  result = xxtea.encrypt(plaintext2);
  result.toLowerCase(); // (Optional)
  Serial.println(result);

  // Perform Decryption
  Serial.print(F(" Decrypted Data 2: "));
  Serial.println(xxtea.decrypt(result));
}

void loop() {}
