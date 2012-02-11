/**
 * InfraRed Remote Control
 * 
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * This uses the IRremote library:
 * http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value.  This will give a unique value for each
 * different code (probably), for most code systems.
 */

#include <IRremote.h>
#include <EEPROM.h>
#include "EEPROMWriteAnything.h"

/**
 * Enable debug console
 * @var bool
 */
#define DEBUG          true

/**
 * Output relay pin
 * Digital pin 2 -> PD2 (pin 4)
 */
#define RELAY_PIN      2

/**
 * IR signal receiver pin
 * Digital pin 5 -> PD5 (pin 11)
 */
#define IR_PIN          5

/**
 * Memory time button pin
 * Digital pin 7 -> PD7 (pin 13)
 */
#define MEMORY_TIME_PIN  7

/**
 * Memory code button pin
 * Digital pin 8 -> PB0 (pin 14)
 */
#define MEMORY_CODE_PIN  8

/**
 * Status led pin
 * Digital pin 13 -> PB5 (pin 19)
 */
#define STATUS_LED     13

/**
 * EEPROM address to save IR code
 */
#define ADDR_CODE      0

/**
 * EEPROM address to save time value
 */
#define ADDR_TIME      100

/**
 * Timeout restrictions
 */
#define MAX_TIME      10000
#define DEFAULT_TIME  5000

/**
 * FNV hash algorithm
 * @see http://isthe.com/chongo/tech/comp/fnv/#FNV-param
 */
#define FNV_PRIME_32   16777619
#define FNV_BASIS_32   2166136261

/**
 * Stored IR code
 * @var int
 */
unsigned long storedCode;

/**
 * Stored time
 * @var int
 */
unsigned int storedTime;

/**
 * Current time 
 * @var ulong
 */
unsigned long timeMs  = millis();
unsigned long codeMs  = millis();
/**
 * Current relay status
 * @var int
 */
int relayStatus       = 0;

/**
 * Decoded result
 * @var decode_results
 */
decode_results results;

/**
 * Init IR Receiver
 * @var IRrecv
 */
IRrecv irrecv(IR_PIN);

/**
 * Initialize controller
 */
void setup()
{
  // Start the receiver
  irrecv.enableIRIn();
  
  // Setup pins
  pinMode(MEMORY_TIME_PIN, INPUT);
  pinMode(MEMORY_CODE_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  
  // Enable pull-up resistors
  digitalWrite(MEMORY_TIME_PIN, HIGH);
  digitalWrite(MEMORY_CODE_PIN, HIGH);
  
  // Read stored code and time
  EEPROM_readAnything(ADDR_CODE, storedCode);
  EEPROM_readAnything(ADDR_TIME, storedTime);
  
  // Check timeout
  if(storedTime == 0 || storedTime > MAX_TIME) {
    storedTime = DEFAULT_TIME;
  }
  
  // Start debug console if debug enabled
  if(DEBUG) {
    Serial.begin(9600);
  }
  
  // End setup
}

/**
 * Blink status led
 * 
 * @param void
 * @return void
 */
void blink()
{
  digitalWrite(STATUS_LED, HIGH);
  delay(200);
  digitalWrite(STATUS_LED, LOW);
  delay(200);
} 

/**
 * Compare two tick values, returning 0 if newval is shorter,
 * 1 if newval is equal, and 2 if newval is longer
 * Use a tolerance of 20%
 * 
 * @author Ken Shirriff
 * @param int oldval
 * @param int newval
 * @return int
 * @see http://www.arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */
int compare(unsigned int oldval, unsigned int newval)
{
  if (newval < oldval * .8) {
    return 0;
  } 
  else if (oldval < newval * .8) {
    return 2;
  } 
  else {
    return 1;
  }
}

/**
 * Converts the raw code values into a 32-bit hash code.
 * Hopefully this code is unique for each button.
 * 
 * @author Ken Shirriff
 * @param decode_results results
 * @return ulong
 * @link http://www.arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */
unsigned long decodeHash(decode_results *results)
{
  unsigned long hash = FNV_BASIS_32;
  for (int i = 1; i+2 < results->rawlen; i++) {
    int value =  compare(results->rawbuf[i], results->rawbuf[i+2]);
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  return hash;
}

/**
 * Check and save timeout
 * @param uint
 */
void saveTimeout(unsigned int newTime)
{
   if(newTime > MAX_TIME) {
     newTime = 1000;
     blink(); 
   }
   
   blink();
   EEPROM_writeAnything(ADDR_TIME, newTime);
   storedTime = newTime;
}
/**
 * Main loop
 */
void loop()
{
  // Increase timeout
  if(digitalRead(MEMORY_TIME_PIN) == LOW)
  {
    if((millis() - timeMs) > 1000)
    {
      saveTimeout(storedTime + 1000);
    }
    timeMs = millis();
  }
  
  if (irrecv.decode(&results))
  {
    int saveMode = 0;
    
    // Decode IR code
    unsigned long hash = decodeHash(&results);
    
    // Save code?
    if (digitalRead(MEMORY_CODE_PIN) == LOW)
    {
      EEPROM_writeAnything(ADDR_CODE, hash);
      storedCode = hash;
      saveMode = 1;
      blink();
    }

    if(storedCode == hash)
    {
      if((millis() - codeMs) > 250 && !relayStatus)
      {
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(STATUS_LED, HIGH);
        relayStatus = !relayStatus;
      }
      codeMs = millis();
    }
    
    if(DEBUG) 
    {
      Serial.print("Stored Code: ");
      Serial.println(storedCode, HEX);
      Serial.print("Received Code: ");
      Serial.println(hash, HEX);
    }
    
    // Resume decoding
    irrecv.resume();
  }
  
  if((millis() - codeMs) > storedTime && relayStatus)
  {
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(RELAY_PIN, LOW);
    codeMs = millis();
    relayStatus = !relayStatus;
  }
}

