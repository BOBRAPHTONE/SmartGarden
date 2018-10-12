/* Simplified_pingpair_dyn_DHT11_photo
 *
 * Simplified 21-September-201
 * February 27, 2016
 * Sean Taylor
 */

/*
 * =======================================================================================================
 */

/*
 * nRF24L01+ Pin Layout
 *       Nano/Uno          nRF24L01          Nano/Uno
 *                     +-------+-------+
 *         GND <-------|1[GND ]|[3V3 ]2|-------> 3V3
 *                     +-------+-------+
 *   D7(or D9) <-------|3[CE  ]|[CSN ]4|-------> D8(or D10)
 *                     +-------+-------+
 *         D13 <-------|5[SCK] |[MOSI]6|-------> D11
 *                     +-------+-------+
 *         D12 <-------|7[MISO]|[IRQ ] |
 *                     +-------+-------+
 */

/*
 * =======================================================================================================
 */

/* Circuit:

  DHT11
  pin
   1 --- VCC   3.3 - 5.5V
   2 --- 10KOhm between 1 & 2 then --- D2
   3
   4 --- GND

   CdS Resistor

    +---(~)---+--------A0
    |         |
    VCC       +--[5KOhm]--GND

    DS18B20

    D4-----------------------+-------+
                             |        \   ___________
    5V--------------[5KOhm]--+----------{|___________|  use a digital pin to power sensor?
                                      /
    GND------------------------------+
*/

/*
 * =======================================================================================================
 */

#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <printf.h>  // Printf is used for debug 

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <OneWire.h>


/*
 * =======================================================================================================
 */

/*
 * #DEFINE
 */

#define SENSOR            1         //change this number for each sensor deployed
#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define led               5         // indicate a read on the DT11
#define DHTTYPE           DHT11     // DHT 11 
#define DS18B20           4         // Temperature pobe DS18B20
#define SLEEPLOOP         105        // 8 seconds * 7 approximately 1 minute * 15 =105   // approximately a 15 minute sleep loop 

/*
 * =======================================================================================================
 */


// DEFINE VARIABLES

DHT_Unified dht(DHTPIN, DHTTYPE);

int sleepON = 0;  // turn the sleep power down ON or OFF, start with OFF at the moment set at compile time
/*
 * later have the RPi control Base turn sleep ON or OFF
 */

uint32_t delayMS;

// Analog pins
int Apin0;
int Apin1;
int Apin2;
int Apin3;
int Apin4;
int Apin5;
int Apin6;
int Apin7;

// CONSTANTS DEFINING THE SENSOR INDICATORS

int Sensor = SENSOR; // 1 = CHR(49), 2 = CHR(50), 3 = CHR(51) printable characters used for debug
//int SenStat = 0;
//int Role = 0;
int DS18B20C;

int temperatureC;
int humidity;
//int voltage;

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius, fahrenheit;

unsigned long int startTime = millis();
unsigned long int thisTime = millis();

/*
 * =======================================================================================================
 */

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8

RF24 radio(7, 8);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//
// Payload
//

const int min_payload_size = 4;    // clean up later
const int max_payload_size = 32;   // clean up later

char send_payload_char[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ789012";
char receive_payload[max_payload_size + 1]; // +1 to allow room for a terminating NULL char

/*
 * =======================================================================================================
 */

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;

void setup_watchdog(uint8_t prescalar);
void do_sleep(void);

const short sleep_cycles_per_transmission = 4;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;

OneWire  ds(DS18B20);  // on pin DS18B20 (a 4.7K resistor is necessary)

/*
 * =======================================================================================================
 */

void setup(void)
{
  /*
   * indicate that the sensor is operating
   */

  pinMode(13, OUTPUT);
  pinMode(led, OUTPUT);    // clean up later

  flashLED(13);

  delay(20);

  //
  // Print preamble
  //

  Serial.begin(115200);

  /*
   * Use printf for radio debug
   */

  dht.begin();
  printf_begin(); //Printf used for debug, clean up later

  Serial.println("+-----------------------------------------------+");
  Serial.println("|          Basic Sensor ATmega328P-PU           |");
  Serial.println("+-----------------------------------------------+");

  Serial.print("Sensor Unit #");
  Serial.println(Sensor);
  Serial.println("+-----------------------------------------------+");

  Serial.println("|        DHTxx Unified Sensor Example           |");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("+-----------------------------------------------+");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("+-----------------------------------------------+");

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);

  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("+-----------------------------------------------+");

  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // Prepare sleep parameters

  setup_watchdog(wdt_8s);

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // enable dynamic payloads
  radio.enableDynamicPayloads();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);
  //radio.setDataRate(RF24_1MBPS);
  radio.setDataRate(RF24_250KBPS);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)
  // *** we will add the other pipes later when adding Transmitter Nodes <------> ST 9/21/2015

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  Serial.println("+-----------------------------------------------+");
  Serial.println("|          The nRF24L01 Configuration :         |");
  Serial.println("+-----------------------------------------------+");

  radio.printDetails();

  Serial.println("+-----------------------------------------------------------------+");
  Serial.println("|  void Setup(void) is complete ... now we enter void loop(void)  |");
  Serial.println("+-----------------------------------------------------------------+");
}

/*
 * =======================================================================================================
 */

void loop(void)
{

  getTemperatureHumidity();
  getDS18B20();

  int Apin0 = analogRead(A0); // A0 is the photo resistor (light sensor)
  int Apin1 = analogRead(A1);
  int Apin2 = analogRead(A2);
  int Apin3 = analogRead(A3);
  int Apin4 = analogRead(A4);
  int Apin5 = analogRead(A5);
  int Apin6 = analogRead(A6);
  int Apin7 = analogRead(A7);

  thisTime = millis();

  int send_payload[] = {Sensor, startTime, thisTime, Apin0, Apin1, Apin2, Apin3, Apin4, Apin5, Apin6, Apin7, temperatureC, humidity, DS18B20C};

  int payload_size = sizeof send_payload;

  radio.stopListening();

  // Take the time, and send it.  This will block until complete
  Serial.print(F("Now sending length "));
  Serial.println(payload_size);

  radio.write( send_payload , payload_size );

  Serial.print("payload_size = ");
  Serial.println(payload_size);

  Serial.print("send_payload = [..");

  Serial.print(send_payload[0]);
  Serial.print(" , ");
  Serial.print(send_payload[1]);
  Serial.print(" , ");
  Serial.print(send_payload[2]);
  Serial.print(" , ");
  Serial.print(send_payload[3]);
  Serial.print(" , ");
  Serial.print(send_payload[4]);
  Serial.print(" , ");
  Serial.print(send_payload[5]);
  Serial.print(" , ");
  Serial.print(send_payload[6]);
  Serial.print(" , ");
  Serial.print(send_payload[7]);
  Serial.print(" , ");
  Serial.print(send_payload[8]);
  Serial.print(" , ");
  Serial.print(send_payload[9]);
  Serial.print(" , ");
  Serial.print(send_payload[10]);
  Serial.print(" , ");
  Serial.print(send_payload[11]);
  Serial.print(" , ");
  Serial.print(send_payload[12]);
  Serial.print(" , ");
  Serial.print(send_payload[13]);
  Serial.println(" ..]");

  /*
   * =======================================================================================================
   */

  // Now, continue listening
  radio.startListening();

  // Wait here until we get a response, or timeout
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 1000 )
      timeout = true;

  // Describe the results
  if ( timeout )
  {
    Serial.println(F("Failed, response timed out."));
  }
  else
  {
    // Grab the response, compare, and send to debugging spew
    uint8_t len = radio.getDynamicPayloadSize();

    // If a corrupt dynamic payload is received, it will be flushed
    if (!len) {
      return;
    }

    radio.read( receive_payload, len );

    // Put a zero at the end for easy printing
    receive_payload[len] = 0;

    // Spew it
    Serial.print(F("Got response size="));
    Serial.print(len);
    Serial.print(F(" value="));
    Serial.println(receive_payload);

  }

  // Try again 1s later
  //delay(1000);
  // Shut down the system
  delay(500);                     // Experiment with some delay here to see if it has an effect
  // Power down the radio.
  //radio.powerDown();              // NOTE: The radio MUST be powered back up again manually
  if (sleepON) {
    for (int i = 0; i < SLEEPLOOP; i++) {
      // Sleep the MCU.
      do_sleep();                     // keep waking then sleeping until ready to send data
    }
  }
}

/*
 * =======================================================================================================
 */
void flashLED(int n) {
  digitalWrite(n, HIGH);
  delay(30);
  digitalWrite(n, LOW);
  delay(20);
  digitalWrite(n, HIGH);
  delay(30);
  digitalWrite(n, LOW);
  delay(10);
  digitalWrite(n, HIGH);
  delay(30);
  digitalWrite(n, LOW);
  delay(20);
  digitalWrite(n, HIGH);
  delay(30);
  digitalWrite(n, LOW);
  delay(10);
}

/*
 * =======================================================================================================
 */
void getTemperatureHumidity(void) {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
    temperatureC = event.temperature * 1000;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {

    flashLED(led);

    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
    humidity = event.relative_humidity * 1000;
  }
}

/*
 * =======================================================================================================
 */

void wakeUp() {
  sleep_disable();
}

/*
 * =======================================================================================================
 */

// Sleep helpers

//Prescaler values
// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(uint8_t prescalar) {

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}

ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  Serial.println(F("WDT"));
}

/*
 * =======================================================================================================
 */

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
  // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(0);
  WDTCSR &= ~_BV(WDIE);
}

/*
 * =======================================================================================================
 */

void getDS18B20() {
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  DS18B20C = celsius * 1000;
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
