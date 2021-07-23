// 7/23/21

#include <Arduino.h>
#include <Wire.h>
#include "kxtj3-1057.h"

#define R_LED_PIN PIN_PD6
#define G_LED_PIN PIN_PB2
#define B_LED_PIN PIN_PB1

#define LED_CTRL PIN_PD5

#define BS_CLR PIN_PD7    //active low clears all registers
#define BS_SER_IN PIN_PB0 //serial input on SRCK
#define BS_RCK PIN_PB6    // Register clock, shifts bits from shift reg to storage reg
#define BS_SRCK PIN_PB7   // Serial clock
#define BS_G PIN_PD4      //active low led enable

#define LIGHT_SENSE_1 PIN_PC3
#define TEMP_2 PIN_PC2
#define TEMP_1 PIN_PC1
#define VIN_ADC PIN_PC0

#define LP1 0x1000
#define LP2 0x2000
#define LP3 0x4000
#define LP4 0x8000
#define LP5 0x0800
#define LP6 0x0400
#define LP7 0x0080
#define LP8 0x0040
#define LP9 0x0020
#define LP10 0x0010
#define LP11 0x0008
#define LP12 0x0004

boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;

int sCurve[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05,
    0x05, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B,
    0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0F, 0x0F, 0x10, 0x11, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x29, 0x2B, 0x2C,
    0x2E, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E, 0x40, 0x43, 0x45, 0x47, 0x4A, 0x4C, 0x4F,
    0x51, 0x54, 0x57, 0x59, 0x5C, 0x5F, 0x62, 0x64, 0x67, 0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C,
    0x7F, 0x82, 0x85, 0x88, 0x8B, 0x8E, 0x91, 0x94, 0x97, 0x9A, 0x9C, 0x9F, 0xA2, 0xA5, 0xA7, 0xAA,
    0xAD, 0xAF, 0xB2, 0xB4, 0xB7, 0xB9, 0xBB, 0xBE, 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xCA, 0xCC, 0xCE,
    0xD0, 0xD2, 0xD3, 0xD5, 0xD7, 0xD8, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5,
    0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xED, 0xEE, 0xEF, 0xEF, 0xF0, 0xF1, 0xF1, 0xF2,
    0xF2, 0xF3, 0xF3, 0xF4, 0xF4, 0xF5, 0xF5, 0xF6, 0xF6, 0xF6, 0xF7, 0xF7, 0xF7, 0xF8, 0xF8, 0xF8,
    0xF9, 0xF9, 0xF9, 0xF9, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFC,
    0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
    0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF};

#define LOW_POWER
//#define HIGH_RESOLUTION

#define KXTJ3_DEBUG Serial

float sampleRate = 6.25; // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2;  // Accelerometer range = 2, 4, 8, 16g

KXTJ3 myIMU(0x0F); // Address can be 0x0E or 0x0F


ISR(TIMER0_COMPA_vect)
{

  if (toggle0)
  {
    digitalWrite(R_LED_PIN, HIGH);
    toggle0 = 0;
  }
  else
  {
    digitalWrite(R_LED_PIN, LOW);
    toggle0 = 1;
  }
}

ISR(TIMER1_COMPA_vect)
{

  if (toggle1)
  {
    digitalWrite(G_LED_PIN, HIGH);
    toggle1 = 0;
  }
  else
  {
    digitalWrite(G_LED_PIN, LOW);
    toggle1 = 1;
  }
}

ISR(TIMER2_COMPA_vect)
{

  if (toggle2)
  {
    digitalWrite(B_LED_PIN, HIGH);
    toggle2 = 0;
  }
  else
  {
    digitalWrite(B_LED_PIN, LOW);
    toggle2 = 1;
  }
}

void setupTimer0() { //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 0 for interrupt frequency 500 Hz:
  cli();      // stop interrupts
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0 = 0;  // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR0A = 249; // = 8000000 / (64 * 500) - 1 (must be <256)
  // turn on CTC mode
  TCCR0B |= (1 << WGM01);
  // Set CS02, CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei(); // allow interrupts
}

void setupTimer1() { //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 1 for interrupt frequency 500 Hz:
  cli();      // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR1A = 15999; // = 8000000 / (1 * 500) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
}

void setupTimer2() {             //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 2 for interrupt frequency 500 Hz:
  cli();      // stop interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0;  // initialize counter value to 0
  // set compare match register for 500 Hz increments
  OCR2A = 1; // = 8000000 / (64 * 500) - 1 (must be <256)
  // turn on CTC mode
  TCCR2B |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 64 prescaler
  TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei(); // allow interrupts
}

void setupRGB(){
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);

}

void setupMainLight(){
  pinMode(LED_CTRL, OUTPUT);
}

void setupSideLights(){
  pinMode(BS_RCK, OUTPUT);
  pinMode(BS_SRCK, OUTPUT);
  pinMode(BS_SER_IN, OUTPUT);
  pinMode(BS_G, OUTPUT);
  pinMode(BS_CLR, OUTPUT);

  digitalWrite(BS_G, LOW);
  digitalWrite(BS_CLR, HIGH);
}

void setupIMU()
{
  if (myIMU.begin(sampleRate, accelRange) != 0)
  {
    Serial.print("Failed to initialize IMU.\n");
  }
  else
  {
    Serial.print("IMU initialized.\n");
  }

  // Detection threshold, movement duration and polarity
  myIMU.intConf(123, 1, 10, HIGH);

  uint8_t readData = 0;

  // Get the ID:
  myIMU.readRegister(&readData, KXTJ3_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);
}

void readSensors() {
  Serial.print("Temp 1 (Celcius) = ");
  Serial.println(analogRead(TEMP_1) / 3.103 -50);

  Serial.print("Temp 2 (Celcius) = ");
  Serial.println(analogRead(TEMP_2) / 3.103 - 50);

  Serial.print("Light Sensor 1 (LUX) = ");
  Serial.println(analogRead(LIGHT_SENSE_1)*2.2);

  Serial.print("Vin ADC (Volts) = ");
  Serial.println(analogRead(VIN_ADC) * .08 - 27);
}

    void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial port at 9600 bps:
  setupRGB();
  setupMainLight();
  setupSideLights();
  setupIMU();
  //setupTimer0();
  //setupTimer1();
  //setupTimer2();
  readSensors();

}

void shiftByte(int bits) //shiftOut should be rewritten to get better peformance (currently uses digitalWrite)
{

  digitalWrite(BS_RCK, LOW);
  digitalWrite(BS_SRCK, LOW);

  shiftOut(BS_SER_IN, BS_SRCK, MSBFIRST, bits);

  digitalWrite(BS_SER_IN, LOW);
  //return the latch pin high to signal chip that it
  //no longer needs to listen for information
  digitalWrite(BS_RCK, HIGH);

}

void setLP(int data){
  digitalWrite(BS_G, HIGH); //make sure nothing is displayed during data transmission
  shiftByte(data);
  shiftByte(data >> 8);
  digitalWrite(BS_G, LOW); //turn display back on again
}

void loop()
{
  Serial.print("Loop");

  readSensors();

  setLP(LP1 | LP12);
    delay(500);
    setLP(LP2 | LP11);
    delay(500);


    myIMU.standby(false);

    int16_t dataHighres = 0;

    if (myIMU.readRegisterInt16(&dataHighres, KXTJ3_OUT_X_L) != 0)

      Serial.print(" Acceleration X RAW = ");
    Serial.println(dataHighres);

    if (myIMU.readRegisterInt16(&dataHighres, KXTJ3_OUT_Z_L) != 0)

      Serial.print(" Acceleration Z RAW = ");
    Serial.println(dataHighres);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration X float = ");
    Serial.println(myIMU.axisAccel(X), 4);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration Y float = ");
    Serial.println(myIMU.axisAccel(Y), 4);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration Z float = ");
    Serial.println(myIMU.axisAccel(Z), 4);

    myIMU.standby(true);

    delay(1000);
}
