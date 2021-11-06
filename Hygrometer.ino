/*
Humidity/Temperature sensor setup with DHT-11
Two digits for humidity
Two digits for temperature
Button to wake up from sleep
NPNs activated via 8th bit in 74HC595s, always alternating

Author:   ElectroBadger
Date:     2021-11-02
Version:  1.0
*/

/*
Reduce power consumption:
- Run at 1 MHz internal clock
- Turn off ADC
- Use SLEEP_MODE_PWR_DOWN
*/

#include "DHT.h" //DHT-11 sensor
#include <avr/sleep.h> // Sleep Modes
#include <avr/power.h> // Power management
#include <avr/wdt.h> //Doggy stuff

//define attiny pins
#define INT_PIN PB4
#define DATA PB1
#define SENSOR PB3
#define LATCH PB2
#define CLK PB0

//define other stuff
#define SENSOR_TYPE DHT11
#define LED_DELAY 50

//changing variables
short ones_data; //16-bits for display of ones
short tens_data; //16-bits for display of tens
byte sevSeg, measurements; //7-segment bit pattern / wait time between LEDs [ms] / # of measurements taken
bool firstPair, btnPress; //tracks which pair of 7-segments is on; tracks button presses
uint32_t oldMillis, sleepTimer; //tracks the last acquisition time and wakeup time

//Initialize sensor
DHT dht(SENSOR, SENSOR_TYPE);

//Shifts 16 bits out MSB first, on the rising edge of the clock.
void shiftOut(int dataPin, int clockPin, short toBeSent){
  int i=0;
  int pinState = 0;
  
  //Clear everything out just in case
  digitalWrite(dataPin, 0);
  digitalWrite(clockPin, 0);

  //Loop through bits in the data bytes, COUNTING DOWN in the for loop so that
  //0b00000000 00000001 or "1" will go through such that it will be pin Q0 that lights.
  for(i=0; i<=15; i++){
    digitalWrite(clockPin, 0);
    //if the value passed to myDataOut AND a bitmask result
    //is true then set pinState to 1
    if(toBeSent & (1<<i)){
      pinState = 1;
    }
    else{
      pinState = 0;
    }
    
    digitalWrite(dataPin, pinState); //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(clockPin, 1); //Shifts bits on upstroke of clock pin
    digitalWrite(dataPin, 0); //Zero the data pin after shift to prevent bleed through
  }
  digitalWrite(clockPin, 0); //Stop shifting
}

//Converts an int <10 to a bit pattern for 7-segment displays
short toSegments(int value){
  byte pattern = 0b00000000; //create empty pattern

  //Using a switch...case (3878 bytes) if...else if...else uses 3946 bytes
  switch(value){
    case 0:
      pattern = 0b01111110;
      break;
    case 1:
      pattern = 0b00110000;
      break;
    case 2:
      pattern = 0b01101101;
      break;
    case 3:
      pattern = 0b01111001;
      break;
    case 4:
      pattern = 0b00110011;
      break;
    case 5:
      pattern = 0b01011011;
      break;
    case 6:
      pattern = 0b01011111;
      break;
    case 7:
      pattern = 0b01110000;
      break;
    case 8:
      pattern = 0b01111111;
      break;
    case 9:
      pattern = 0b01111011;
      break;
    default:
      pattern = 0b00000000;
      break;
  }

  return pattern;
}

void goToSleep(){
  //Turn off 7-segments and NPNs
  digitalWrite(LATCH, 0);
  shiftOut(DATA, CLK, 0b0000000000000000); 
  digitalWrite(LATCH, 1);
  //Set deep sleep mode
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0; // turn off ADC
  power_all_disable (); // power off ADC, Timer 0 and 1, serial interface
  cli(); // timed sequence coming up, so disable interrupts
  btnPress = false;
  measurements = 0;
  resetWatchdog (); // get watchdog ready
  sleep_enable (); // ready to sleep
  sei(); // interrupts are required now
  sleep_cpu (); // sleep                
  sleep_disable (); // precaution
  power_all_enable (); // power everything back on
}
        
ISR(PCINT_VECTOR){
  btnPress = true;
  sleepTimer = millis();
}

// watchdog interrupt
ISR(WDT_vect){
  wdt_disable(); //disable watchdog
}

void resetWatchdog(){
  MCUSR = 0; //clear various "reset" flags    
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF); //allow changes, disable reset, clear existing interrupt
  //set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0); //set WDIE, and 8 seconds delay
  wdt_reset(); //pat the dog
  }
 
void setup(){
  resetWatchdog(); // do this first in case WDT fires
  cli(); //Disable interrupts during setup
  
  pinMode(INT_PIN, INPUT_PULLUP); //Set interrupt pin as input w/ internal pullup
  pinMode(DATA, OUTPUT); //Set serial data as output
  pinMode(CLK, OUTPUT); //Set shift register clock as output
  pinMode(LATCH, OUTPUT); //Set output register (latch) clock as output
  
  // Interrupts
  PCMSK = bit(INT_PIN); //Enable interrupt handler (ISR)
  GIFR  |= bit(PCIF); // clear any outstanding interrupts
  GIMSK |= bit(PCIE); //Enable PCINT interrupt in the general interrupt mask

  //default conditions
  /*  bit 0-6: ones digits
  bit 7: NPN for units digits
  bit 8-14: ones digits
  bit 15: NPN for tens digits
  */
  ones_data = 0b0000000000000000;
  tens_data = 0b0000000000000000;
  measurements = 0;
  firstPair = true;
  btnPress = false;
  oldMillis = 0;
  sleepTimer = 0;
  
  //Start sensor
  dht.begin();
  delay(1000); //wait 1s for sensor to stabilize
        
  sei(); //Enable interrupts after setup
}

void loop(){ 
  if((millis()-oldMillis) > 1000){
    //Slow sensor, so readings may be up to 2 seconds old
    byte hum = dht.readHumidity(); //Read humidity
    byte temp = dht.readTemperature(); //Read temperatuer in °C
    
    //update tens bit string
    tens_data = 0b0000000000000000; //reset to all 0s
    sevSeg = toSegments(hum/10); //convert tens of humidity to 7-segment logic
    tens_data |= sevSeg; // bitwise OR the result with the output short
    tens_data = tens_data << 8; //shift by 8 so it's almost in the right place (see below)
    sevSeg = toSegments(temp/10); //convert tens of temperature to 7-segment logic
    tens_data |= sevSeg; // bitwise OR the result with the output short
    tens_data = tens_data << 1; //shift by 1 so everything is in the right place
    tens_data |= 0b0000000100000000; //set NPN for tens pair to active and ones NPN to inactive 
    
    //update ones bit string
    ones_data = 0b0000000000000000; //reset to all 0s
    sevSeg = toSegments(hum%10); //convert ones of humidity to 7-segment logic
    ones_data |= sevSeg; // bitwise OR the result with the output short
    ones_data = ones_data << 8; //shift by 8 so it's almost in the right place (see below)
    sevSeg = toSegments(temp%10); //convert ones of temperature to 7-segment logic
    ones_data |= sevSeg; // bitwise OR the result with the output short
    ones_data = ones_data << 1; //shift by 1 so everything is in the right place
    ones_data |= 0b0000000000000001; //set NPN for ones pair to active and tens NPN to inactive 
    
    oldMillis = millis(); //I don't much care about the few ms lost
  }             //during data acquisition
  
  if(btnPress){
    //shift out the next batch of data to the display
    digitalWrite(LATCH, 0); //Set latch pin LOW so nothing gets shifted out
    if(firstPair){
      shiftOut(DATA, CLK, tens_data); //Shift out LED states for 7-segments of tens
      firstPair = false;
    }
    else{
      shiftOut(DATA, CLK, ones_data); //Shift out LED states for 7-segments of ones
      firstPair = true;
    }
    digitalWrite(LATCH, 1); //sent everything out in parallel
    delay(LED_DELAY); //wait for some time until switching to the other displays
    
    if((millis()-sleepTimer) > 6000){ //Sleep after 6s display time
      goToSleep();
    }
  }
  else{
    if(measurements > 5){
      goToSleep();
    }
  }
}
