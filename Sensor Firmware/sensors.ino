/* Download and install NRF library
 *  https://github.com/stanleyseow/RF24
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define DHT_PIN 2
 
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9, 10);
 
int pin = 8;

float temp = 0;
float humidity = 0;
float soil_moisture = 0;


 
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
 
void setup() {
 
  pinMode(pin, OUTPUT);
  // Print preamble
  Serial.begin(57600);
  printf_begin();
  printf("\nRemote Switch Arduino\n\r");
 
  // Setup and configure rf radio
  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(15, 15);
 
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.startListening();
  radio.printDetails();               // Print radio info for debugging
}
 
// Send callback to confirm message
void sendCallback(unsigned short callback) {
  // First, stop listening so we can talk
  radio.stopListening();
 
  // Send the final one back.
  radio.write( &callback, sizeof(unsigned short) );
  printf("Sent response %d \n\r", callback);
 
  // Now, resume listening so we catch the next packets.
  radio.startListening();
}
 
// Perform action based on value of rawMessage
void performAction(unsigned short rawMessage) {
  if (rawMessage == 0) {
    digitalWrite(8, LOW);
    sendCallback(rawMessage);
  } else if (rawMessage == 1) {
    digitalWrite(8, HIGH);
    sendCallback(rawMessage);
  }
}
void loop(void) {
 
  // if there is data ready
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    unsigned short message;
    bool done;
    unsigned short rawMessage;
    done = false;
    while ( radio.available() )
    {
      // Fetch the payload, and see if this was the last one.
      radio.read( &rawMessage, sizeof(unsigned long) );
 
      // Spew it
      printf("Got message %d...", rawMessage);
 
      performAction(rawMessage);
 
      delay(10);
    }
  }
}

