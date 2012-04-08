#include "Command.h"
#include "Streaming.h"

int debug = 1;

Command com;
int sequenceNumber = 1;
int i = 1;
String atcmd = "";

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

void setup()
{
  ARsrl.begin(115200);
  PCsrl.begin(115200);
  if (debug) {
	//never use three ! together in arduino code
	PCsrl << "yeahhh!!fuckya!\r\n";
  }
  Timer3.initialize(SERIAL_INTERVAL_USEC);
  Timer3.attachInterrupt(SrlRead);
}

// Volatile, since it is modified in an ISR.
volatile boolean inService = false;

void SrlRead() {
  if (inService) {
    PCsrl.println("timer kicked too fast");
    return;
  }
  interrupts();
  inService = true;
  while(ARsrl.available()) {
    unsigned char k = ARsrl.read();
    store_char( k, &rx_buf);
  } 
  inService = false;
}

void read_rx_buf() {
	while ( rx_buf.tail != rx_buf.head) {
		if (debug) {
			PCsrl.write(rx_buf.buffer[rx_buf.tail]);
		}
		rx_buf.tail = (unsigned int) (rx_buf.tail+ 1) % SERIAL_BUFFER_SIZE;
	}
}


void loop()
{
  if ( com.s2ip_running == 0 ) {
   com.s2ip_running = com.start_s2ip();
   //upon exit s2ip_running == 1
  }
  
  if ( com.s2ip_running == 1) {
    delay(200);
    ARsrl<< com.LEDAnim(5);
	delay(3000); 
	
	/* code that run for take off ?*/
	if (com.drone_is_init == 0) {
		com.drone_is_init = com.init_drone();
	}
	// drone take off
	if (com.drone_is_init == 1) {
		com.drone_takeoff();
	}
	delay(500);
	com.drone_landing();

	com.quit_s2ip();
    com.s2ip_running == 0;
    delay(100000);
  }
}
