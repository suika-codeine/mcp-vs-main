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

void timer3_pwm_init() {
    // Set Pin 5 (PE3 / OC3A) as output
    DDRE |= (1 << PE3);

    // Fast PWM, TOP = ICR3 (Mode 14)
    TCCR3A |= (1 << COM3A1);  // Non-inverting PWM on OC3A
    TCCR3A |= (1 << WGM31);   // Fast PWM part 1
    TCCR3B |= (1 << WGM32) | (1 << WGM33);  // Fast PWM part 2

    // Set TOP value for 20ms period (50Hz) → ICR3
    ICR3 = 39999;  // 20 ms with 16MHz and prescaler 8

    // Set initial duty cycle to 1.5ms pulse (centered servo)
    OCR3A = 3000;

    // Start Timer3 with prescaler = 8
    TCCR3B |= (1 << CS31);  // Prescaler = 8
}

// ------------------------ Button Interrupts ------------------------ //
void button_init() {
    DDRD &= ~((1 << PD2) | (1 << PD3));  // PD2 (INT0) & PD3 (INT1) as input
    PORTD |= (1 << PD2) | (1 << PD3);  // Enable pull-ups


    EICRA |= (1 << ISC21) | (1 << ISC20);  // INT0: Rising edge (Start/Stop)
    EICRA |= (1 << ISC31) | (1 << ISC30);  // INT1: Rising edge (Reset)
   
    EIMSK |= (1 << INT3) | (1 << INT2);  // Enable external interrupts INT2 & INT3
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
ISR(INT3_vect) {
    running = !running;  // Toggle stopwatch state
}


// ------------------------ Reset Interrupt (INT1) ------------------------ //
ISR(INT2_vect) {
    mseconds = 0;
    seconds = 0;
    minutes = 0;
    running = 0;
}


// ------------------------ Main Function ------------------------ //
int main(void) {
    cli();  // Disable global interrupts
    serial0_init();  // Initialise serial communication
    timer1_init();  // Initialise Timer1
    timer3_pwm_init(); //Initialise Timer3
    button_init();  // Initialise buttons
    sei();  // Enable global interrupts
    char buffer[60];


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
