/*
  DC_Motor_Driver
  https://github.com/techn0man1ac/DC_Motor_Driver

  by Techn0man1ac Labs, 2022
*/

#define LED 13

//Left motor driver
#define EN1 10 // PWM
#define REV1 5
#define FW1 6

//Right motor driver
#define EN0 9 // PWM
#define REV0 7
#define FW0 8

#define PassReboot 32123 // Passwors for reboot CPU

int LeftMotor = 0; // -255..255
int RightMotor = 0; // In here right motor speed

long distanceLMotor = 0;
long distanceRMotor = 0;

long distanceLMTemp = 0;
long distanceRMTemp = 0;
bool MoveOn = false; // Move enable

unsigned long previousMillis = 0;
#define interval 100

unsigned int ADCBattRaw = 0;
float BattVoltage = 0.0;
unsigned int CPUVoltage = 0;

void setup() {
  pinMode(EN0, OUTPUT);
  digitalWrite(EN0, HIGH);
  pinMode(EN1, OUTPUT);
  digitalWrite(EN1, HIGH);
  pinMode(REV0, OUTPUT);
  digitalWrite(REV0, LOW);
  pinMode(REV1, OUTPUT);
  digitalWrite(REV1, LOW);
  pinMode(FW0, OUTPUT);
  digitalWrite(FW0, LOW);
  pinMode(FW1, OUTPUT);
  digitalWrite(FW1, LOW);
  pinMode(LED, OUTPUT);

  // Pins D9 and D10 - 4 kHz
  // TCCR1A = 0b00000001; // 8bit
  // TCCR1B = 0b00000010; // x8 phase correct
  // https://nerdytechy.com/how-to-change-the-pwm-frequency-of-arduino/

  MtrsSpdSet(64, -64); // Test move
  delay(500);
  MtrsSpdSet(-64, 64);
  delay(500);
  MtrsSpdSet(0, 0);

  Serial.begin(115200); // UART speed ESP8266 <-> Arduino Nano, must be same all boards
  Serial.setTimeout(5);  // sets the maximum milliseconds to wait for serial data

  attachInterrupt(1, sensLeftHall, CHANGE); // pin 3, Left
  attachInterrupt(0, sensRightHall, CHANGE); // pin 2, Right

  Serial.println("V0.2 Start "); // We redy
}

void sensLeftHall() {
  if (LeftMotor > 0) {
    distanceLMotor = distanceLMotor + 10;
  }
  if (LeftMotor < 0) {
    distanceLMotor = distanceLMotor - 10;
  }
}

void sensRightHall() {
  if (RightMotor > 0) {
    distanceRMotor = distanceRMotor + 10; // 10 mm, 1 cm
  }
  if (RightMotor < 0) {
    distanceRMotor = distanceRMotor - 10;
  }
}

void loop() {
  unsigned int ADCBattRaw = analogRead(A1);
  BattVoltage = ADCBattRaw * (15.5 / 1023.0); // 15.5 V max

  if (Serial.available() > 0) { //If in UART buffer have some datas
    char data = Serial.read(); // Read first character

    if (data == 'L') {
      LeftMotor = SerialParse();
    }

    if (data == 'R') {
      RightMotor = SerialParse();
    }

    if (data == 'T') {
      bool LEDState = SerialParse();
      digitalWrite(LED, LEDState);
    }

    if (data == 'M') { // Move
      if (MoveOn == false) {
        distanceLMTemp = SerialParse();
        distanceRMTemp = SerialParse();

        distanceLMTemp = distanceLMotor + distanceLMTemp;
        distanceRMTemp = distanceRMotor + distanceRMTemp;
        MoveOn = true;
      } else {
        Serial.println("On road=" + String(MoveOn) + " " + String(distanceLMTemp) + " " + String(distanceRMTemp));  // Send to device
      }
    }

    if (data == 'C') { // Forget distance
      distanceLMotor = 0;
      distanceRMotor = 0;
    }

    if (data == 'B') { // reBoot CPU
      int Password = SerialParse();
      if (Password == PassReboot) { // b32123 -> Reboot CPU
        reboot();
      }
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    CPUVoltage = readVcc(); // Eat 2 ms CPU time, need location here
    Serial.println(String(distanceRMotor) + "," + String(distanceLMotor) + "," + String(BattVoltage) + "," + String(CPUVoltage));  // Send to device
  }

  if (BattVoltage > 10.0 && CPUVoltage > 4700) { // If battery CPU voltages in safe level
    MtrsSpdSet(LeftMotor, RightMotor); // l-255..255, r-255..255
  } else {
    MtrsSpdSet(0, 0); // All stop, battery protect
  }
}

void MtrsSpdSet(int lMotor, int rMotor) {
  if (lMotor > 0) {
    lMotor = 255 - lMotor;
    digitalWrite(REV1, HIGH);
    digitalWrite(FW1, LOW);
  } else if (lMotor == 0) {
    digitalWrite(REV1, LOW);
    digitalWrite(FW1, LOW);
  } else if (lMotor < 0) {
    lMotor = abs(lMotor);
    lMotor = 255 - lMotor;
    digitalWrite(REV1, LOW);
    digitalWrite(FW1, HIGH);
  }

  if (rMotor > 0) {
    rMotor = 255 - rMotor; // Inverse 255-245 -> 10, 255-100 -> 155
    digitalWrite(REV0, HIGH);
    digitalWrite(FW0, LOW);
  } else if (rMotor == 0) {
    digitalWrite(REV0, LOW);
    digitalWrite(FW0, LOW);
  } else if (rMotor < 0) {
    rMotor = abs(rMotor);
    rMotor = 255 - rMotor;
    digitalWrite(REV0, LOW);
    digitalWrite(FW0, HIGH);
  }

  analogWrite(EN1, lMotor);
  analogWrite(EN0, rMotor);
}

int readVcc(void) {
  // https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  int result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

int SerialParse() {
  int dataParse = Serial.parseInt();
  return dataParse;
}

void reboot(void) // Reboot CPU
{
  Serial.end ();  // if necessary

  asm ("cli");          // interrupts off

  // reset USART to reset defaults
#ifdef __AVR_ATmega8__
  UDR = 0;
  UCSRA = _BV (UDRE);
  UCSRB = 0;
  UCSRC = _BV (URSEL) | _BV (UCSZ1) |  _BV (UCSZ0);
  UBRRL = 0;
  UBRRH = 0;
#else
  UDR0   = 0;
  UCSR0A = _BV (UDRE0);
  UCSR0B = 0;
  UCSR0C = _BV (UCSZ01) |  _BV (UCSZ00);
  UBRR0L = 0;
  UBRR0H = 0;
#endif

  asm volatile ("eor r1,r1");     // make sure zero-register is zero
  asm volatile ("ldi r16,0xFF");  // end of RAM (0xFF)
  asm volatile ("sts 0x5E,r16");  // SPH
  asm volatile ("sts 0x5D,r16");  // SPL
  asm volatile ("eor r31,r31");   // Clear Z register
  asm volatile ("eor r30,r30");
  asm volatile ("ijmp");          // jump to (Z)
}
