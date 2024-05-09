#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <Servo.h>

/**
 * * Pin Information
 * * 2 -> PD2 = IR_LED_1
 * * 3 -> PD3 = IR_LED_2
 * * 6 -> PD6 = RST_PIN for RFID
 * * 7 -> PD7 = SDA_PIN for RFID
 * * 9 -> PB1 = SERVO Motor for IR_LED_1
 * * 10 -> PB2 = Servo Motor for IR_LED_2
 */

#define IR_LED_1 2
#define IR_LED_2 3
#define RST_PIN 6
#define SS_PIN 7
#define servo1 PB2
#define servo2 PB1
#define irLed1 PD2
#define irLed2 PD3
#define MOTOR_DELAY 1000

// prototypes;
ISR(INT0_vect);
ISR(INT1_vect);
void delay_ms(int ms);
void printDec(byte *buffer, byte bufferSize);
void entryOpen();
void exitOpen();
void entryClose();
void exitClose();
// Global Values
bool prevSensor1 = true;
bool prevSensor2 = true;
bool sensor1First = true;
bool sensor2First = true;
bool senseRfid = false;
char senser = '1';
Servo servo;
MFRC522 rfid(SS_PIN, RST_PIN);       // Instance of the class
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  DDRD &= ~(1 << irLed1);                  // set irled1 as input
  DDRD &= ~(1 << irLed2);                  // set irled2 as input
  PORTD |= (1 << irLed1) | (1 << irLed2);  // enable the pull up resistor for both the leds.
  EICRA |= (1 << ISC00) | (1 << ISC10);    // any change
  EIMSK |= (1 << INT0) | (1 << INT1);      // enable interrupts for both.
  sei();                                   // enable global interrupts.
  Serial.begin(115200);                    // enable serial for the given baudrate.
  Serial.println();
  Serial.println("Arduino Started.");
  SPI.begin();                     // Init SPI bus
  rfid.PCD_Init();                 // Init MFRC522
  rfid.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details

  // setting up lcd.
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Parking");
  entryOpen();
  entryClose();
  exitOpen();
  exitClose();
}

void loop() {
  if (!senseRfid) {
    Serial.print("^");
    delay_ms(500);
    return;
  }

  lcd.clear();
  lcd.print("Enter Card");
  Serial.println();
  Serial.print("Sense RFID for the sensor: ");
  Serial.println(senser);

  entryClose();
  exitClose();

  while (1) {
    Serial.print(".");
    delay_ms(100);
    if (!rfid.PICC_IsNewCardPresent())
      continue;

    // Verify if the NUID has been readed
    if (!rfid.PICC_ReadCardSerial())
      continue;

    Serial.print(F("vehicle"));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    lcd.clear();
    lcd.print("Please Wait");

    while (!Serial.available()) {
      Serial.print("*");
      delay_ms(100);
    }

    String resp = Serial.readString();
    Serial.println(resp);

    lcd.clear();
    lcd.setCursor(0, 0);
    if (resp[0] == 'n') {
      lcd.print("No Space");
      lcd.setCursor(0, 1);
      lcd.print("Available");
      entryClose();
    } else if (resp[0] == 'e') {
      // lcd.print("Welcome To");
      // lcd.setCursor(0, 1);
      // lcd.print("Parking");
      // entryOpen();
      // delay_ms(2000);
      String slot = resp.substring(1);
      lcd.clear();
      lcd.print("Park On");
      lcd.setCursor(0, 1);
      lcd.print("Slot " + slot);
    } else if (resp[0] == 'r') {
      lcd.print("Thanks For");
      lcd.setCursor(0, 1);
      lcd.print("Coming.");
      exitOpen();
    } else {
      lcd.print("Invalid");
    }

    // Halt PICC
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    break;
  }

  senseRfid = false;
}

ISR(INT0_vect) {
  // todo make sure to disable the other interrupt while we're handling this one
  _delay_ms(50);  // Debouncing
  bool currentValue = (PIND & (1 << PD2)) != 0;
  if (sensor1First || currentValue != prevSensor1) {
    if (!currentValue) {
      Serial.println("The car has just arrived, close the entry");
      senseRfid = true;
    }
    prevSensor1 = currentValue;
    if (sensor1First)
      sensor1First = false;
  }
}

ISR(INT1_vect) {
  // todo make sure to disable the other interrupt while we're handling this one
  _delay_ms(50);  // Debouncing
  bool currentValue = (PIND & (1 << PD3)) != 0;
  if (sensor2First || currentValue != prevSensor2) {
    if (!currentValue) {
      Serial.println("The car has just arrived");
      senseRfid = true;
    }
    prevSensor2 = currentValue;
    if (sensor2First)
      sensor2First = false;
  }
}

void delay_ms(int ms) {
  while (ms--) {
    TCCR0A = 0x0;                    // Normal Mode
    TCCR0B = 1 << CS01 | 1 << CS00;  // With Pre-Scalling 64
    TCNT0 = 0x6;                     // Delay of 1 ms.
    while (!(TIFR0 & (1 << TOV0)))   // Loop till we the TOV0 doesn't become 1.
    {
    }
    TIFR0 |= 1 << TOV0;  // Clear the interrupt flag.
    TCNT0 = 0;
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : "-");
    Serial.print(buffer[i], DEC);
  }
}

void entryOpen() {
  servo.attach(10);
  servo.write(0);
  delay(MOTOR_DELAY);
  servo.detach();
}

void entryClose() {
  servo.attach(10);
  servo.write(40);
  delay(MOTOR_DELAY);
  servo.detach();
}

void exitOpen() {
  servo.attach(9);
  servo.write(80);
  delay(MOTOR_DELAY);
  servo.detach();
}

void exitClose() {
  servo.attach(9);
  servo.write(40);
  delay(MOTOR_DELAY);
  servo.detach();
}
