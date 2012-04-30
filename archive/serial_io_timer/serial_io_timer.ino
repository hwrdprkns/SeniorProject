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

#include "TimerThree.h"
#define BAUD 115200
// adjust this base on how often you read your ring buffer
#define SERIAL_BUFFER_SIZE 64
// adjust this base on how often you receive message
#define SERIAL_INTERVAL_USEC 30000
#define LEDpin 13

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};

ring_buffer rx_buf= { {0}, 0, 0};

inline void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
  else {
    Serial.println("ring buffer is too small");
  }
}

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);
  //Timer3.initialize((long)1000000*SRLBUF_BYTE_SIZE*1/BAUD);
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
  pinMode(LEDpin,OUTPUT);
  digitalWrite(LEDpin,LOW);
}

// Volatile, since it is modified in an ISR.
volatile boolean inService = false;

void SrlRead() {
  if (inService) {
    Serial.println("timer kicked too fast");
    return;
  }
  interrupts();
  digitalWrite(LEDpin,HIGH);
  inService = true;
  while(Serial1.available()) {
    unsigned char k = Serial1.read();
    store_char( k, &rx_buf);
  } 
  inService = false;
  digitalWrite(LEDpin,LOW);
}

void loop() {
  //delay(500);
  //Serial.println("I am stealing cycles");

  while ( rx_buf.tail != rx_buf.head) {
   Serial.write(rx_buf.buffer[rx_buf.tail]);
    rx_buf.tail = (unsigned int) (rx_buf.tail+ 1) % SERIAL_BUFFER_SIZE;
  }
  
  //Serial.println("ring buffer output done");

}

