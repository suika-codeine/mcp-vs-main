//include this .c file's header file
#include "Controller.h"
#include "adc.h"  // Include ADC library
//#define F_CPU 16000000UL // 16MHz
#define OCR1A_VALUE 249  // 1ms Timer Compare Match Value




volatile uint16_t mseconds = 0;
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t running = 1;  // Stopwatch state (1 = running, 0 = stopped)




// ------------------------ Timer Initialization ------------------------ //
void timer1_init() {
    TCCR1B = 0;  // Reset Timer1
    TCNT1 = 0;   // Reset counter
    TCCR1B |= (1 << WGM12);  // CTC Mode
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64
    OCR1A = OCR1A_VALUE;  // Compare value for 1ms interval
    TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 Compare Match A interrupt
}




// ------------------------ Button Interrupts ------------------------ //
void button_init() {
    DDRD &= ~((1 << PD2) | (1 << PD3));  // PD2 (INT0) & PD3 (INT1) as input
    PORTD |= (1 << PD2) | (1 << PD3);  // Enable pull-ups




    EICRA |= (1 << ISC01) | (1 << ISC00);  // INT0: Rising edge (Start/Stop)
    EICRA |= (1 << ISC11) | (1 << ISC10);  // INT1: Rising edge (Reset)
   
    EIMSK |= (1 << INT0) | (1 << INT1);  // Enable external interrupts INT0 & INT1
}




// ------------------------ Timer Interrupt (1ms) ------------------------ //
ISR(TIMER1_COMPA_vect) {
    if (running) {
        mseconds++;
        if (mseconds >= 1000) {
            mseconds = 0;
            seconds++;
            if (seconds >= 60) {
                seconds = 0;
                minutes++;
            }
        }
    }
}




// ------------------------ Start/Stop Interrupt (INT0) ------------------------ //
ISR(INT0_vect) {
    running = !running;  // Toggle stopwatch state
}




// ------------------------ Reset Interrupt (INT1) ------------------------ //
ISR(INT1_vect) {
    mseconds = 0;
    seconds = 0;
    minutes = 0;
    running = 0;
}




// ------------------------ Main Function ------------------------ //
int main(void) {
    cli();  // Disable global interrupts
    serial0_init();  // Initialize serial communication
    timer1_init();  // Initialize Timer1
    button_init();  // Initialize buttons
    sei();  // Enable global interrupts




    char buffer[32];




    while (1) {
        sprintf(buffer, "Time: %02d:%02d.%03d\r\n", minutes, seconds, mseconds);
       
        // Send time over serial
        for (char *ptr = buffer; *ptr; ptr++) {
            while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty buffer
            UDR0 = *ptr;  // Transmit character
        }




        _delay_ms(100);  // Update every 100ms to avoid excessive serial output
    }
}
