// #include <Arduino.h>

// #define R_LED_PIN PIN_PD6
// #define G_LED_PIN PIN_PB2
// #define B_LED_PIN PIN_PB1

// #define LED_CTRL PIN_PD5

// #define BS_CLR PIN_PD7    //active low clears all registers
// #define BS_SER_IN PIN_PB0 //serial input on SRCK
// #define BS_RCK PIN_PB6    // Register clock, shifts bits from shift reg to storage reg
// #define BS_SRCK PIN_PB7   // Serial clock
// #define BS_G PIN_PD4      //active low led enable

// void setupTimer() {
//   TCCR1A = 0; // set entire TCCR1A register to 0
//   TCCR1B = 0; // same for TCCR1B
//   TCNT1 = 0;  //initialize counter value to 0
//   // set compare match register for 1hz increments
//   OCR1A = 15624; // = (8*10^6) / (1*1024) - 1 (must be <65536)
//   // turn on CTC mode
//   TCCR1B |= (1 << WGM12);
//   // Set CS10 and CS12 bits for 1024 prescaler
//   TCCR1B |= (1 << CS12) | (1 << CS10);
//   // enable timer compare interrupt
//   TIMSK1 |= (1 << OCIE1A);
// }

// void setupRGB() {
//   pinMode(R_LED_PIN, OUTPUT);
//   pinMode(G_LED_PIN, OUTPUT);
//   pinMode(B_LED_PIN, OUTPUT);

//   //analogWrite(R_LED_PIN, 2);
//   //analogWrite(G_LED_PIN, 2);
//   //analogWrite(B_LED_PIN, 2);
// }

// void setupMainLight() {
//   pinMode(LED_CTRL, OUTPUT);
//   analogWrite(LED_CTRL, 20);
// }

// void setupSideLights() {
//   pinMode(BS_RCK, OUTPUT);
//   pinMode(BS_SRCK, OUTPUT);
//   pinMode(BS_SER_IN, OUTPUT);
//   pinMode(BS_G, OUTPUT);
//   pinMode(BS_CLR, OUTPUT);

//   digitalWrite(BS_G, LOW);
//   digitalWrite(BS_CLR, HIGH);
// }
