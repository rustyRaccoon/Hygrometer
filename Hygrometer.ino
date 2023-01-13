/*
  Humidity/Temperature sensor setup with DHT-11
  Two digits for humidity
  Two digits for temperature
  Button to wake up from sleep
  NPNs activated via 8th bit in 74HC595s, always alternating
  
  !! Needs to be set up with 8 MHz internal clock AND bootloader apparently !!
  
  In the first version (1.0) one of the 3-pin headers is turned around, switching the TENS_GND and ONES_GND!!
  This needs to be corrected in updateDisplay() when setting the NPN bits

  Author:   rustyRaccoon
  Date:     2023-01-13
  Version:  6.0
*/

#include <avr/sleep.h>  //sleep Modes
#include <avr/power.h>  //power management
#include <avr/wdt.h>    //doggy stuff

//define attiny pins
#define CLK PB0
#define DATA PB1
#define LATCH PB2
#define SENSOR PB3
#define INT_PIN PB4

//define other stuff
#define LED_DELAY 5

//fixed variables
uint8_t ignoreButton;         //time to ignore button presses for
const byte numLookup[] = {    //array lookup for number display; ascending order: 0, 1, 2, ...
  0b01111110, //0
  0b00110000, //1
  0b01101101, //2
  0b01111001, //3
  0b00110011, //4
  0b01011011, //5
  0b01011111, //6
  0b01110000, //7
  0b01111111, //8
  0b01111011  //9
};

// changing variables
short ones_data;            //16-bits for display of ones
short tens_data;            //16-bits for display of tens
byte sevSeg;                //7-segment bit pattern
byte measurements;          //number of measurements taken
byte sleepCounter;          //sleep cycles since last measurement
byte humI;                  //integral part of humidity measurement
byte humD;                  //decimal part of humidity measurement (we'll throw this away but it's returned so we need to put it somewhere)
byte tempI;                 //integral part of temperature measurement
byte tempD;                 //decimal part of temperature measurement (we'll throw this away but it's returned so we need to put it somewhere)
bool firstPair;             //tracks which pair of 7-segments is on
bool btnPress;              //tracks if button is pressed
bool readTriggered;         //tracks if a reading was requested
bool awakeTime;             //tracks if sleeping is allowed
uint32_t lastAcquisition;   //tracks the last acquisition time
uint32_t sleepTimer;        //tracks the wakeup time
uint32_t buttonPressTime;   //tracks when the button was last pressed
uint32_t onTime;            //tracks the time one pair of digits should be on

//shifts 16 bits out MSB first, on the rising edge of the clock.
void shiftOut(int dataPin, int clockPin, short toBeSent) {
  int i = 0;
  int pinState = 0;

  //clear everything out just in case
  digitalWrite(dataPin, 0);
  digitalWrite(clockPin, 0);

  //loop through bits in the data bytes
  for (i = 0; i <= 15; i++) {
    digitalWrite(clockPin, 0);
    //if the value AND a bitmask result is true then set pinState to 1
    if (toBeSent & (1 << i)) {
      pinState = 1;
    }
    else {
      pinState = 0;
    }

    digitalWrite(dataPin, pinState);  //sets the pin to HIGH or LOW depending on pinState
    digitalWrite(clockPin, 1);        //shifts bits on upstroke of clock pin
    digitalWrite(dataPin, 0);         //zero the data pin after shift to prevent bleed through
  }
  digitalWrite(clockPin, 0);          //stop shifting
}

void start_signal(byte SENSOR_PIN) {
  pinMode(SENSOR_PIN, OUTPUT);        //set pin as output
  digitalWrite(SENSOR_PIN, LOW);      //set pin LOW
  delay(18);                          //wait 18 ms
  digitalWrite(SENSOR_PIN, HIGH);     //set pin HIGH
  pinMode(SENSOR_PIN, INPUT_PULLUP);  //set pin as input and pull to VCC (10k)
}

boolean read_dht11(byte SENSOR_PIN) {
  uint16_t rawHumidity = 0;
  uint16_t rawTemperature = 0;
  uint16_t data = 0;
  uint8_t checkSum = 0;
  unsigned long startTime;

  for (int8_t i = -3; i < 80; i++) {  //loop 80 iterations, representing 40 bits * 2 (HIGH + LOW)
    byte high_time;                   //stores the HIGH time of the signal
    startTime = micros();             //stores the time the data transfer started

    /*
    sensor should pull line LOW and keep for 80µs (while SENSOR_PIN == HIGH)
    then pull HIGH and keep for 80µs (while SENSOR_PIN == LOW)
    then pull LOW again, aka send data (while SENSOR_PIN == HIGH)
    */
    do {                                                  //waits for sensor to respond
      high_time = (unsigned long)(micros() - startTime);  //update HIGH time
      if (high_time > 90) {                               //times out after 90 microseconds
        //Serial.println("ERROR_TIMEOUT");
        return false;
      }
    }
    while (digitalRead(SENSOR_PIN) == (i & 1) ? HIGH : LOW);

    //actual data starts at iteration 0
    if (i >= 0 && (i & 1)) {  //if counter is odd, do this (only counts t_on time and ignores t_off)
      data <<= 1;             //left shift data stream by 1 since we are at a the next bit

      //t_on of bit 0 is maximum 30µs and of bit 1 is at least 68µs
      if (high_time > 30) {
        data |= 1; //we got a one
      }
    }

    switch ( i ) {
      case 31:                  //bit 0-16 is humidity
        rawHumidity = data;
        break;
      case 63:                  //bit 17-32 is temperature
        rawTemperature = data;
      case 79:                  //bit 33-40 is checksum
        checkSum = data;
        data = 0;
        break;
    }
  }

  //humidity
  humI = rawHumidity >> 8;
  rawHumidity = rawHumidity << 8;
  humD = rawHumidity >> 8;

  //temperature
  tempI = rawTemperature >> 8;
  rawTemperature = rawTemperature << 8;
  tempD = rawTemperature >> 8;

  if ((byte)checkSum == (byte)(tempI + tempD + humI + humD)) {
    return true;
  }
  else {
    return false;
  }
}

void updateDisplay(){
  cli(); //disable all interrupts for the duration of the read since the communication is rather timing-sensitive
  start_signal(SENSOR); //send start sequence

  if (read_dht11(SENSOR)) {    
    // update tens bit string
    tens_data = 0b0000000000000000;     //reset to all 0s
    tens_data |= numLookup[humI / 10];  //bitwise OR the result with the output short
    tens_data = tens_data << 8;         //shift by 8 so it's almost in the right place (see below)
    tens_data |= numLookup[tempI / 10]; //bitwise OR the result with the output short
    tens_data = tens_data << 1;         //shift by 1 so everything is in the right place
    tens_data |= 0b0000000100000000;    //set NPN for tens pair to active and ones NPN to inactive

    // update ones bit string
    ones_data = 0b0000000000000000;     //reset to all 0s
    ones_data |= numLookup[humI % 10];  //bitwise OR the result with the output short
    ones_data = ones_data << 8;         //shift by 8 so it's almost in the right place (see below)
    ones_data |= numLookup[tempI % 10]; //bitwise OR the result with the output short
    ones_data = ones_data << 1;         //shift by 1 so everything is in the right place
    ones_data |= 0b0000000100000001;    //set NPN for ones pair to active and tens NPN to inactive
  }
  else {
    tens_data = 0b1001111110011110;
    ones_data = 0b0000101000001011;
  }
  
  sei(); //enable all interrupts again since communication with sensor is over
}

void goToSleep() {
  //turn off 7-segments and NPNs
  digitalWrite(LATCH, 0);
  shiftOut(DATA, CLK, 0b0000000000000000);
  digitalWrite(LATCH, 1);

  set_sleep_mode (SLEEP_MODE_PWR_DOWN); //set deep sleep mode
  ADCSRA = 0;                           //turn off ADC
  power_all_disable ();                 //power off ADC, Timer 0 and 1, serial interface
  cli();                                //timed sequence coming up, so disable interrupts
  btnPress = false;                     //reset button flag
  measurements = 0;                     //reset measurement counter
  resetWatchdog ();                     //get watchdog ready
  sleep_enable ();                      //ready to sleep
  sei();                                //interrupts are required now
  sleep_cpu ();                         //sleep
  sleep_disable ();                     //precaution
  power_all_enable ();                  //power everything back on
}

ISR(PCINT0_vect) {
  if(millis()-buttonPressTime >= ignoreButton){
    buttonPressTime = millis();
    btnPress = true;
    awakeTime = true;
    sleepTimer = millis();
  }
}

//watchdog interrupt
ISR(WDT_vect) {
  wdt_disable(); //disable watchdog
}

void resetWatchdog() {
  MCUSR = 0;                                    //clear various "reset" flags
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);  //allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0); //set WDIE, and 8 seconds delay
  wdt_reset();
}

void setup() {
  resetWatchdog(); //do this first in case WDT fires
  cli(); //disable interrupts during setup

  pinMode(INT_PIN, INPUT_PULLUP); //set interrupt pin as input w/ internal pullup
  pinMode(DATA, OUTPUT);          //set serial data as output
  pinMode(CLK, OUTPUT);           //set shift register clock as output
  pinMode(LATCH, OUTPUT);         //set output register (latch) clock as output
  pinMode(SENSOR, INPUT);         //set DHT11 pin as input

  // Interrupts
  PCMSK = bit(INT_PIN); //enable interrupt handler (ISR)
  GIFR  |= bit(PCIF);   //clear any outstanding interrupts
  GIMSK |= bit(PCIE);   //enable PCINT interrupt in the general interrupt mask

  //default conditions
  /*
  bit 0-6: single digits
  bit 7: NPN for single digits
  bit 8-14: tens digits
  bit 15: NPN for tens digits
  */
  ones_data = 0b0000000000000000;
  tens_data = 0b0000000000000000;
  measurements = 0;
  sleepCounter = 0;
  firstPair = true;
  btnPress = false;
  awakeTime = false;
  readTriggered = false;
  lastAcquisition = 0;
  sleepTimer = 0;
  onTime = 0;
  humI = 0;
  humD = 0;
  tempI = 0;
  tempD = 0;
  buttonPressTime = 0;
  ignoreButton = 50;

  digitalWrite(LATCH, 0);         //set latch pin LOW so nothing gets shifted out
  shiftOut(DATA, CLK, tens_data); //shift out LED states for 7-segments of tens
  digitalWrite(LATCH, 1);         //sent everything out in parallel

  delay(2000);      //wait for DHT to be ready
  updateDisplay();  //read DHT11 once in the beginning
  
  sei(); //enable interrupts after setup
}

void loop() {
  //if 5 minutes are not over yet, increase counter and sleep again; button press overrules this
  if (sleepCounter <= 38 && !btnPress) {
    sleepCounter++;
    goToSleep();
  }
  else {
    awakeTime = true; //keep awake until action is done
    
    //need to wait two seconds until sensor is ready (I think; maybe next time I'll try without this but now I'm pressed for time)
    if (!readTriggered && (millis() - lastAcquisition) > 2000) {
      readTriggered = true;
      lastAcquisition = millis();
    }
    else if (readTriggered && (millis() - lastAcquisition) > 2000) { //slow sensor, so readings may be up to 2 seconds old
      updateDisplay();

      readTriggered = false;
      measurements++;
      lastAcquisition = millis();
    }

    if (btnPress) {
      if((millis()-onTime)>=LED_DELAY){
        onTime = millis(); //update timestamp
        
        //shift out the next batch of data to the display
        digitalWrite(LATCH, 0); //set latch pin LOW so nothing gets shifted out

        if (firstPair) {
          shiftOut(DATA, CLK, tens_data); //shift out LED states for 7-segments of tens
          firstPair = false;              //reset first digit flag
        }
        else {
          shiftOut(DATA, CLK, ones_data); //shift out LED states for 7-segments of ones
          firstPair = true;               //set first digit flag
        }

        digitalWrite(LATCH, 1); //sent everything out in parallel
        //delay(LED_DELAY); //wait for some time until switching to the other displays
        
        //sleep after 5s display time
        if ((millis() - sleepTimer) > 5000) { 
          sleepCounter = 0;
          awakeTime = false;
          goToSleep();
        }
      }
    }
    else {
      if (measurements >= 2) {
        sleepCounter = 0;
        awakeTime = false;
        goToSleep();
      }
    }
  }
}
