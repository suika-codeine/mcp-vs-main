#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"  // Replaces timer.h, serial0.h, serial2.h for communication and timing

#define LDR_LEFT_CHANNEL 0
#define LDR_RIGHT_CHANNEL 1
#define SERVO_MIN 2000
#define SERVO_MAX 4000

// Arduino-style map function
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void timer1_init() {
    DDRB |= (1 << PB5);  // OC1A output pin
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler 8
    ICR1 = 40000;        // 20ms period for 50Hz PWM
    OCR1A = 3000;        // Start centered
}

int main(void) {
    adc_init();
    timer1_init();
    serial2_init();       // Communicate with controller
    serial0_init();       // USB serial debug
    milliseconds_init();

    uint32_t current_ms = 0, last_send_ms = 0;
    uint8_t databyte1 = 0;
    uint8_t databyte2 = 0;
    uint8_t receivedData[5];
    char serial_string[60] = {0};

    sei(); // Enable global interrupts

    while (1) {
        current_ms = milliseconds_now();

        // Read sensors (reduce to 8-bit scale)
        databyte1 = adc_read(LDR_LEFT_CHANNEL) >> 2;
        databyte2 = adc_read(LDR_RIGHT_CHANNEL) >> 2;

        // Send data to controller every 100 ms
        if ((current_ms - last_send_ms) >= 100) {
            serial2_write_bytes(2, databyte1, databyte2);
            last_send_ms = current_ms;
        }

        // Receive joystick data (X, Y) from controller
        if (serial2_available()) {
            serial2_get_data(receivedData, 2);
            uint8_t joystickX = receivedData[0];
            // Optional: use Y if needed: uint8_t joystickY = receivedData[1];

            OCR1A = map(joystickX, 0, 255, SERVO_MIN, SERVO_MAX);

            sprintf(serial_string, "\nJoystick X: %3u | LDR L: %3u R: %3u", joystickX, databyte1, databyte2);
            serial0_print_string(serial_string);
        }
    }
}