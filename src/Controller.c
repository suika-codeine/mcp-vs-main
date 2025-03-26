//include this .c file's header file
#include "Controller.h"
#include "adc.h"  // Include ADC library
#define MAX_DISTANCE   30   // Maximum possible distance
#define RANGE_SENSOR_CHANNEL 0  // ADC Channel 0 (A0)

int main(void) {
    DDRA = 0xFF;  // Set PORTC as output for the DC Motor
    PORTA = 0x00; // Start with the Motor off
    adc_init();  // Initialize ADC system
    serial0_init();
    char serialString[60]={};

    while (1) {
        uint16_t adc_value = adc_read(RANGE_SENSOR_CHANNEL );  
        //uint8_t distance = (adc_value * MAX_DISTANCE) / 1023;        
        if (adc_value > 250) {
            PORTA |= (1 << PA3) ;  // Turn on DC Motor
        } else {
            PORTA &= ~(1 << PA3);    // Turn off DC Motor
        }
        sprintf(serialString, "\nDistance = %d cm", adc_value);
        serial0_print_string(serialString);
        _delay_ms(150);  // Small delay to stabilize readings
    }
    return 0;
}
