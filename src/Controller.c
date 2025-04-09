#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"   // Uses shared serial file for UART and timer functions

#define START_BYTE 0xFF
#define END_BYTE   0xFE

#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1

volatile uint8_t serial_state = 0;
volatile uint8_t sensor_type = 0;
volatile uint8_t sensor_L = 0;
volatile uint8_t sensor_R = 0;
volatile uint8_t new_data = 0;

void display_status() {
    if (sensor_L > 200 && sensor_R > 200)
        serial0_print_string("Robot is STOPPED\n");
    else if (sensor_L < 150 && sensor_R < 150)
        serial0_print_string("Moving FORWARD\n");
    else if (sensor_L < sensor_R)
        serial0_print_string("Turning LEFT\n");
    else
        serial0_print_string("Turning RIGHT\n");
}

// USART Receive ISR (Used for receiving sensor packet from robot)
ISR(USART0_RX_vect) {
    uint8_t byte = UDR0;

    switch (serial_state) {
        case 0:
            if (byte == START_BYTE) serial_state = 1;
            break;
        case 1:
            sensor_type = byte;
            serial_state = 2;
            break;
        case 2:
            sensor_L = byte;
            serial_state = 3;
            break;
        case 3:
            sensor_R = byte;
            serial_state = 4;
            break;
        case 4:
            if (byte == END_BYTE) new_data = 1;
            serial_state = 0;
            break;
        default:
            serial_state = 0;
    }
}

int main(void) {
    adc_init();
    serial0_init();   // Use UART0 for debug + robot comms
    milliseconds_init();
    sei();

    uint32_t last_send = 0;
    uint32_t now = 0;

    while (1) {
        now = milliseconds_now();

        if (now - last_send >= 100) {
            // Read joystick
            uint16_t x = adc_read(JOYSTICK_X_CHANNEL);
            uint16_t y = adc_read(JOYSTICK_Y_CHANNEL);

            // Scale to 8-bit and send over serial
            serial2_write_bytes(2, x >> 2, y >> 2);
            last_send = now;
        }

        if (new_data) {
            display_status();
            new_data = 0;
        }
    }
}