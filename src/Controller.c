// Example ATmega2560 Project
// File: ATmega2560Project.c
// An example file for second-year mechatronics project


#include "Controller.h"
#include <avr/io.h>
#include "adc.h"  // Include ADC library
#include <avr/interrupt.h>
#include "milliseconds.h"


#define NUM_LEDS 9      // Number of LEDs
#define LDR_PIN 0       // LDR connected to ADC Channel 0 (A0)
#define BUTTON_PIN PD0  // External interrupt pin for push button (INT0)


volatile uint16_t adc_value = 0;  // Store ADC value
volatile uint8_t flag = 0;
volatile uint8_t flag2 = 0;
volatile uint16_t edge_count = 0;

void int_timer1() {
    TCCR1B = 0; // reset TCCR1B
    TCCR1B |= (1 << WGM12);      // Configure Timer1 in CTC (Clear Timer on Compare Match) mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024 (16MHz/1024)
    OCR1A = 15624;                // Set compare value for 1 second interval (16MHz clock, prescaler 1024)
    TIMSK1 |= (1 << OCIE1A);     // Enable Timer1 Compare Match A interrupt   
}

volatile uint16_t average_ADC() {
    const ADC_READS = 10;
    volatile uint16_t sum_ADC_Values = 0;
    volatile uint16_t average;

    for (int i = 0; i < ADC_READS; i++)
    {
        sum_ADC_Values += adc_read(LDR_PIN);
    }
    average = sum_ADC_Values / ADC_READS;
    return average;
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


    // Interrupt setup
    cli();  // Disable global interrupts


  // Configure INT0 on falling edge (button press)
    EICRA |= (1 << ISC01);  // Falling edge
    EICRA &= ~(1 << ISC00);  // Ensure ISC00 is cleared
    EIMSK |= (1 << INT0);  // Enable INT0 interrupt


    sei();  // Enable global interrupts
    int_timer1();

    while (1) {
        // Read LDR value
        adc_value = adc_read(LDR_PIN);  
        uint8_t num_leds = (adc_value * NUM_LEDS) / 1024;  
        // Scale voltage value


        // Update LED display
        if (num_leds > 0) {
            PORTC = (1 << num_leds) - 1;  
        } else {
            PORTC = 0;  
        }
        if(flag== 1)
        {
            char serialString[60] = {};
            int voltage = (int)(((adc_value * 5) / 1023.0) * 1000);
            sprintf(serialString, "\nLight Intensity Level = %d.%3d mV", voltage / 1000, voltage % 1000);
            serial0_print_string(serialString);
            flag =0;
        }
        if(flag2== 1)
        {
            char serialString[60] = {};
            sprintf(serialString, "\nFalling Edges in Last Second: %d", edge_count);
            serial0_print_string(serialString);

            char serialString2[60] = {};
            int voltage = (int)(((average_ADC() * 5) / 1023.0) * 1000);
            sprintf(serialString2, "\nThe Average Voltage of  the Photoresistor: %d.%3d mV", voltage / 1000, voltage % 1000);
            serial0_print_string(serialString2);
        }
        _delay_ms(1000);  // Small delay for stability
    }
    return 0;
}

// External Interrupt Service Routine (ISR) for INT0 (Button Press)
ISR(INT0_vect) {
    edge_count++;  // Count the number of button presses (falling edges)
    flag = 1;
}

// Timer1 Compare Match Interrupt - Fires every 1 second
ISR(TIMER1_COMPA_vect) {
    flag2 = 1;
    edge_count = 0;  // Reset count after printing
}
