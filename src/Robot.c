#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"

#define RANGE_LEFT_CHANNEL 0
#define RANGE_RIGHT_CHANNEL 1
#define SERVO_MIN_PULSE 2000
#define SERVO_MAX_PULSE 4000

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {  
    adc_init();

    // Set PB5 (OC1A) as output for servo
    DDRB |= (1 << PB5);

    // Configure Timer1 for Fast PWM Mode 14 (ICR1 as TOP)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
    ICR1 = 39999;
    OCR1A = 3000;

    serial2_init();  // UART to controller
    serial0_init();  // USB debug
    sei();

    uint8_t received[2];
    uint8_t rangeL, rangeR;

    while (1) {
        // Receive joystick data
        if (serial2_available()) {
            serial2_get_data(received, 2);
            uint8_t joystickX = received[0];

            // Control servo
            OCR1A = map(joystickX, 0, 255, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

            // Read range sensors (ADC0 and ADC1)
            rangeL = adc_read(RANGE_LEFT_CHANNEL) >> 2;
            rangeR = adc_read(RANGE_RIGHT_CHANNEL) >> 2;

            // Send both sensor values back to controller
            serial2_write_bytes(2, rangeL, rangeR);
        }

        _delay_ms(10);
    }

    return 0;
}
