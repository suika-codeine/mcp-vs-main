// Example ATmega2560 Project
// Measures the number of falling edges (button presses) per second

#include "Controller.h"
#include <avr/io.h>
#include "adc.h"
#include <avr/interrupt.h>

#define NUM_LEDS 9      // Number of LEDs
#define LDR_PIN 0       // LDR connected to ADC Channel 0 (A0)
#define BUTTON_PIN PD2  // External interrupt pin for push button (INT0)

volatile uint16_t adc_value = 0;  // Store ADC value
volatile uint16_t edge_count = 0; // Counter for falling edges
volatile uint8_t flag = 0;        // Flag for printing the count

void init_timer1() {
    TCCR1B |= (1 << WGM12);  // Configure Timer1 in CTC (Clear Timer on Compare Match) mode
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 Compare Match A interrupt
    OCR1A = 15624;           // Set compare value for 1 second interval (16MHz clock, prescaler 1024)
    TCCR1B |= (1 << CS12) | (1 << CS10);  // Set prescaler to 1024 (16MHz/1024)
}

int main(void) {
    // Set PORTC as output for LEDs
    DDRC = 0xFF;  
    PORTC = 0x00; // Start with all LEDs OFF

    // Initialize ADC and serial communication
    adc_init();  
    serial0_init();  

    // Configure PD2 as input with internal pull-up
    DDRD &= ~(1 << BUTTON_PIN);  
    PORTD |= (1 << BUTTON_PIN);  

    // Disable global interrupts while configuring
    cli();

    // Configure INT0 on falling edge (button press)
    EICRA |= (1 << ISC01);  // Falling edge trigger
    EICRA &= ~(1 << ISC00);  
    EIMSK |= (1 << INT0);  

    // Initialize Timer1 for 1-second intervals
    init_timer1();

    // Enable global interrupts
    sei();

    while (1) {
        // Read LDR value
        adc_value = adc_read(LDR_PIN);  
        uint8_t num_leds = (adc_value * NUM_LEDS) / 1024;  

        // Update LED display
        if (num_leds > 0) {
            PORTC = (1 << num_leds) - 1;  
        } else {
            PORTC = 0;  
        }

        // Print edge count every second
        if (flag == 1) {
            char serialString[60];
            sprintf(serialString, "\nFalling Edges in Last Second: %d", edge_count);
            serial0_print_string(serialString);
            
            flag = 0;  // Reset flag
        }

        _delay_ms(100);  // Small delay for stability
    }
    return 0;
}

// External Interrupt Service Routine (ISR) for INT0 (Button Press)
ISR(INT0_vect) {
    edge_count++;  // Count the number of button presses (falling edges)
}

// Timer1 Compare Match Interrupt - Fires every 1 second
ISR(TIMER1_COMPA_vect) {
    flag = 1;        // Signal to print the count
    edge_count = 0;  // Reset count after printing
}
