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
 adds interrupt
 */

#define BAUD 115200
// adjust this base on how often you read your ring buffer
#define SERIAL_BUFFER_SIZE 64
// adjust this base on how often you receive message
#define SERIAL_INTERVAL_USEC 30000
#define LEDpin 13

void setup() {
  // initialize both serial ports:
  Serial.begin(57600);
  Serial3.begin(57600);
  //Timer3.initialize((long)1000000*SRLBUF_BYTE_SIZE*1/BAUD);

}


void loop() {
  //delay(500);
  //Serial.println("I am stealing cycles");
char c;
 while ( Serial.available() ) {
   c = Serial.read();
   Serial3.write(c);
 }
 while ( Serial3.available() ) {
   c = Serial3.read();
   Serial.write(c);
 }
   
  //Serial.println("ring buffer output done");

}

