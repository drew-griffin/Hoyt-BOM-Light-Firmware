#include <Arduino.h>

#define R_LED_PIN PIN_PD6
#define G_LED_PIN PIN_PB2
#define B_LED_PIN PIN_PB1

#define LED_CTRL PIN_PD5

#define BS_CLR PIN_PD7    //active low clears all registers
#define BS_SER_IN PIN_PB0 //serial input on SRCK
#define BS_RCK PIN_PB6    // Register clock, shifts bits from shift reg to storage reg
#define BS_SRCK PIN_PB7   // Serial clock
#define BS_G PIN_PD4      //active low led enable

boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;

ISR(TIMER0_COMPA_vect)
{ //timer0 interrupt 2kHz toggles pin 8
  //generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
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

// ISR(TIMER1_COMPA_vect)
// { //timer1 interrupt 1Hz toggles pin 13 (LED)
//   //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
//   if (toggle1)
//   {
//     digitalWrite(G_LED_PIN, HIGH);
//     toggle1 = 0;
//   }
//   else
//   {
//     digitalWrite(G_LED_PIN, LOW);
//     toggle1 = 1;
//   }
// }

// ISR(TIMER2_COMPA_vect)
// { //timer1 interrupt 8kHz toggles pin 9
//   //generates pulse wave of frequency 8kHz/2 = 4kHz (takes two cycles for full wave- toggle high then toggle low)
//   if (toggle2)
//   {
//     digitalWrite(B_LED_PIN, HIGH);
//     toggle2 = 0;
//   }
//   else
//   {
//     digitalWrite(B_LED_PIN, LOW);
//     toggle2 = 1;
//   }
// }

void setupTimer0() { //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 0 for interrupt frequency 1000 Hz:
  cli();      // stop interrupts
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0 = 0;  // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR0A = 124; // = 8000000 / (64 * 1000) - 1 (must be <256)
  // turn on CTC mode
  TCCR0B |= (1 << WGM01);
  // Set CS02, CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei(); // allow interrupts
}

void setupTimer1() { //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 1 for interrupt frequency 1000 Hz:
  cli();      // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR1A = 7999; // = 8000000 / (1 * 1000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
}

void setupTimer2() {             //calculated with http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  // TIMER 2 for interrupt frequency 1000 Hz:
  cli();      // stop interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0;  // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR2A = 249; // = 8000000 / (32 * 1000) - 1 (must be <256)
  // turn on CTC mode
  TCCR2B |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 32 prescaler
  TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei(); // allow interrupts
}

void setupRGB(){
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);

  analogWrite(R_LED_PIN, 2);
  analogWrite(G_LED_PIN, 2);
  analogWrite(B_LED_PIN, 2);
  delay(2000);
}

void setupMainLight(){
  pinMode(LED_CTRL, OUTPUT);
  analogWrite(LED_CTRL, 20);
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

void setup()
{
  // put your setup code here, to run once:
  setupRGB();
  setupMainLight();
  setupSideLights();
  setupTimer0();
  setupTimer1();
  setupTimer2();
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

void loop()
{
  int data = 0x0000;
  shiftByte(data);
  shiftByte(data >> 8);
  delay(250);
  data = 0xFFFF;
  shiftByte(data);
  shiftByte(data >> 8);
  delay(250);

}
