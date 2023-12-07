#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

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
#define servo1 PB1
#define servo2 PB2
#define irLed1 PD2
#define irLed2 PD3

// prototypes;
ISR(INT0_vect);
ISR(INT1_vect);
void senseRfid();
void setupServos();
void servo1Open();
void servo1Close();
void servo2Open();
void servo2Close();

void setup() {}
void loop() {}

ISR(INT0_vect)
{
}

ISR(INT1_vect)
{
}

void senseRfid()
{
}

void setupServos()
{
}

void servo1Open()
{
}

void servo1Close()
{
}

void servo2Open()
{
}

void servo2Close()
{
}