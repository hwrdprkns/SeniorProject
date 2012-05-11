/*
  Mega multple serial test
 
 Receives from the main serial port, sends to the others. 
 Receives from serial port 1, sends to the main serial (Serial 0).
 
 This example works only on the Arduino Mega
 
 The circuit: 
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:
 
 created 30 Dec. 2008
 by Tom Igoe
 
 This example code is in the public domain.
 modified by Weiyi Zheng
 */
#include "Command.h"

Command com;


void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  com.start_wifi_connection();
  com.start_nav_recv();
}

void loop() {
  // read from port 2, send to port 0:
  if (Serial2.available()) {
    int inByte = Serial2.read();
    Serial.write(inByte); 
  }
  if (Serial.available()) {
    int outByte = Serial.read();
    Serial2.write(outByte);
  }
}
