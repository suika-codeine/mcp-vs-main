#include <avr/io.h>
#include <util/delay.h>
#include "Controller.h"
#include "adc.h"

#define SERVO_MIN_PULSE 2000    // 1 ms pulse width
#define SERVO_MAX_PULSE 4000    // 2 ms pulse width

#define JOYSTICK_X_CHANNEL 0    // ADC0 = A0
#define JOYSTICK_Y_CHANNEL 1    // ADC1 = A1

// Simple map function (similar to Arduino)
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void timer1_pwm_init() {
    DDRB |= (1 << PB5); // Set PB5 (OC1A) as output

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 40000;  // 20 ms period (50Hz)

    OCR1A = 3000;  // Center servo initially
}

void timer3_pwm_init() {
    DDRE |= (1 << PE3); // Set PE3 (OC3A) as output 000001000 

    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // Prescaler 8
    ICR3 = 40000;   //20 ms period (50Hz)

    OCR3A = 3000;  // Center servo initially
}

int main(void) {
    adc_init();           // Initialise ADC
    timer1_pwm_init();    // Servo 1 on OC1A (Pin 11)
    timer3_pwm_init();    // Servo 2 on OC3A (Pin 5)

    while (1) {
        // Read both joystick channels
        uint16_t joyX = adc_read(JOYSTICK_X_CHANNEL); // A0
        uint16_t joyY = adc_read(JOYSTICK_Y_CHANNEL); // A1

        // Map joystick positions to servo pulse widths
        uint16_t pulse1 = map(joyX, 0, 1023, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        uint16_t pulse2 = map(joyY, 0, 1023, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

        // Update each servo
        OCR1A = pulse1;  // Servo 1
        OCR3A = pulse2;  // Servo 2

        _delay_ms(20); // Refresh rate ~50Hz
    }

    return 0;
}
