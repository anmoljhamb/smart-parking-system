#include <avr/io.h>
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
void delay_ms(int ms);

// Global Values
bool prevSensor1 = true;
bool prevSensor2 = true;
bool sensor1First = true;
bool sensor2First = true;

void setup()
{
    DDRD &= ~(1 << irLed1);                 // set irled1 as input
    DDRD &= ~(1 << irLed2);                 // set irled2 as input
    PORTB |= (1 << irLed1) | (1 << irLed2); // enable the pull up resistor for both the leds.
    EICRA |= (1 << ISC00) | (1 << ISC10);   // any change
    EIMSK |= (1 << INT0) | (1 << INT1);     // enable interrupts for both.
    sei();                                  // enable global interrupts.
    Serial.begin(9600);                     // enable serial for the given baudrate.
}

void loop() {}

ISR(INT0_vect)
{
    _delay_ms(50); // Debouncing
    bool currentValue = (PIND & (1 << PD2)) != 0;
    if (sensor1First || currentValue != prevSensor1)
    {
        if (!currentValue)
        {
            senseRfid();
        }
        prevSensor1 = currentValue;
        if (sensor1First)
            sensor1First = false;
    }
}

ISR(INT1_vect)
{
    _delay_ms(50); // Debouncing
    bool currentValue = (PIND & (1 << PD3)) != 0;
    if (sensor2First || currentValue != prevSensor2)
    {
        if (!currentValue)
        {
            Serial.println("The car has just arrived");
            senseRfid();
        }
        prevSensor2 = currentValue;
        if (sensor2First)
            sensor2First = false;
    }
}

void senseRfid()
{
}

void setupServos()
{
    DDRB |= (1 << servo1) | (1 << servo2);                  // Set PB1 as output
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Fast PWM, non-inverting mode // Setup for both OC1A, AND OC1B
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // Fast PWM, prescaler = 8
    ICR1 = 39999;                                           // 20ms PWM period

    // Set both initially to be in the middle
    OCR1A = 1999;
    OCR1B = 1999;
}

void servo1Open()
{
    OCR1A = 2999;
}

void servo1Close()
{
    OCR1A = 1999;
}

void servo2Open()
{
    OCR2A = 2999;
}

void servo2Close()
{
    OCR2A = 1999;
}

void delay_ms(int ms)
{
    while (ms--)
    {
        TCCR0A = 0x0;                   // Normal Mode
        TCCR0B = 1 << CS01 | 1 << CS00; // With Pre-Scalling 64
        TCNT0 = 0x6;                    // Delay of 1 ms.
        while (!(TIFR0 & (1 << TOV0)))  // Loop till we the TOV0 doesn't become 1.
        {
        }
        TIFR0 |= 1 << TOV0; // Clear the interrupt flag.
        TCNT0 = 0;
    }
}