/*
* sinewave_pcm
*
* Generates 8-bit PCM sinewave on pin 6 using pulse-width modulation (PWM).
* For Arduino with Atmega368P at 16 MHz.
*
* Uses timers 1 and 0. Timer 1 reads the sinewave table, SAMPLE_RATE times a
second.
* The sinewave table has 256 entries. Consequently, the sinewave has a
frequency of
* f = SAMPLE_RATE / 256
* Each entry in the sinewave table defines the duty-cycle of Timer 0. Timer
0
* holds pin 6 high from 0 to 255 ticks out of a 256-tick cycle, depending on
* the current duty cycle. Timer 0 repeats 62500 times per second (16000000 /
256),
* much faster than the generated sinewave generated frequency.
*
* References:
* http://www.atmel.com/dyn/resources/prod_documents/doc2542.pdf
* http://www.analog.com/library/analogdialogue/archives/38-08/dds.html
* http://www.evilmadscientist.com/article.php/avrdac
* http://www.arduino.cc/playground/Code/R2APCMAudio
* http://www.scienceprog.com/generate-sine-wave-modulated-pwm-with-avrmicrocontroller/
* http://www.scienceprog.com/avr-dds-signal-generator-v10/
* http://documentation.renesas.com/eng/products/region/rtas/mpumcu/apn/
sinewave.pdf
* http://ww1.microchip.com/downloads/en/AppNotes/00655a.pdf
*
* By Gary Hill
* Adapted from a script by Michael Smith <michael@hurts.ca>
*/
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#define SAMPLE_RATE 8000 // 8 ksps
/*
* The sinewave data needs to be unsigned, 8-bit
*
* sinewavedata.h should look like this:
* const int sinewave_length=256;
*
* const unsigned char sinewave_data[] PROGMEM = {0x80,0x83, ...
*
*/
#include "sinewavedata.h"

int outputPin = 6; // (PCINT22/OC0A/AIN0)PD6, Arduino Digital Pin 6
volatile uint16_t sample;
// This is called at SAMPLE_RATE kHz to load the next sample.
ISR(TIMER1_COMPA_vect) {
if (sample >= sinewave_length) {
sample = -1;
}
else {
OCR0A = pgm_read_byte(&sinewave_data[sample]);
}
++sample;
}
void startPlayback()
{
pinMode(outputPin, OUTPUT);
// Set Timer 0 Fast PWM Mode (Section 14.7.3)
// WGM = 0b011 = 3 (Table 14-8)
// TOP = 0xFF, update OCR0A register at BOTTOM
TCCR0A |= _BV(WGM01) | _BV(WGM00);
TCCR0B &= ~_BV(WGM02);
// Do non-inverting PWM on pin OC0A, arduino digital pin 6
// COM0A = 0b10, clear OC0A pin on compare match,
// set 0C0A pin at BOTTOM (Table 14-3)
TCCR0A = (TCCR0A | _BV(COM0A1)) & ~_BV(COM0A0);
// COM0B = 0b00, OC0B disconnected (Table 14-6)
TCCR0A &= ~(_BV(COM0B1) | _BV(COM0B0));
// No prescaler, CS = 0b001 (Table 14-9)
TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01))) | _BV(CS00);
// Set initial pulse width to the first sample.
OCR0A = pgm_read_byte(&sinewave_data[0]);
// Set up Timer 1 to send a sample every interrupt.
cli(); // disable interrupts
// Set CTC mode (Section 15.9.2 Clear Timer on Compare Match)
// WGM = 0b0100, TOP = OCR1A, Update 0CR1A Immediate (Table 15-4)
// Have to set OCR1A *after*, otherwise it gets reset to 0!
TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
// No prescaler, CS = 0b001 (Table 15-5)
TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
// Set the compare register (OCR1A).
// OCR1A is a 16-bit register, so we have to do this with
// interrupts disabled to be safe.
OCR1A = F_CPU / SAMPLE_RATE; // 16e6 / 8000 = 2000
// Enable interrupt when TCNT1 == OCR1A (p.136)
TIMSK1 |= _BV(OCIE1A);
sample = 0;
sei(); // enable interrupts
}
void setup()
{
startPlayback();
}
void loop()
{
while (true);
}
