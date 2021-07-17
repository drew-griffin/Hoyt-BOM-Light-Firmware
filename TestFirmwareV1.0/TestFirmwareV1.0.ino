#define R_LED_PIN PIN_PD6 
#define G_LED_PIN PIN_PB2 
#define B_LED_PIN PIN_PB1 

#define LED_CTRL PIN_PD5 

#define BS_CLR PIN_PD7 //active low clears all registers
#define BS_SER_IN PIN_PB0 //serial input on SRCK
#define BS_RCK PIN_PB6  // Register clock, shifts bits from shift reg to storage reg
#define BS_SRCK PIN_PB7 // Serial clock
#define BS_G PIN_PD4 //active low led enable

void setup() {
  // put your setup code here, to run once:
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
  pinMode(LED_CTRL, OUTPUT);
  analogWrite(LED_CTRL, 20);

  analogWrite(R_LED_PIN, 2);
  analogWrite(G_LED_PIN, 2);
  analogWrite(B_LED_PIN, 2);

  pinMode(BS_RCK, OUTPUT);
  pinMode(BS_SRCK, OUTPUT);
  pinMode(BS_SER_IN, OUTPUT);
  pinMode(BS_G, OUTPUT);
  pinMode(BS_CLR, OUTPUT);

  digitalWrite(BS_G, LOW);
  digitalWrite(BS_CLR, HIGH);
}

void loop() {
  int data=0x0000;

    
  //for(int i = 0; i<17;i++){
    //data = (data << 1);
    shiftByte(data);
    shiftByte(data >> 8);
    delay(1);
    shiftByte(data);
    shiftByte(data >> 8);
    delay(1);
    shiftByte(data);
    shiftByte(data >> 8);
    delay(1);
    data=0xFFFF;
    shiftByte(data);
    shiftByte(data >> 8);
    delay(1);
  //}
  
}

void shiftByte(int bits) {
  digitalWrite(BS_RCK, LOW);
  digitalWrite(BS_SRCK, LOW);
  shiftOut(BS_SER_IN, BS_SRCK, MSBFIRST, bits);
  digitalWrite(BS_SER_IN, LOW);
    //return the latch pin high to signal chip that it
    //no longer needs to listen for information
  digitalWrite(BS_RCK, HIGH);
}
