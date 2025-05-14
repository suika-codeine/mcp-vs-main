#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

// ===== Serial Protocol Constants =====
#define START_BYTE 0xFF
#define END_BYTE 0xFE

// ===== ADC Channels for Joystick =====
#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1

// ===== Global Variables for Receiving Sensor Data from Robot =====
volatile uint8_t serial_state = 0;
volatile uint8_t sensor_type = 0;
volatile uint8_t sensor_L = 0;
volatile uint8_t sensor_R = 0;
volatile uint8_t new_data = 0;

// ===== Initialize USART0 for Serial Communication =====
void serial0_init(void) {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud rate at 16 MHz
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Enable RX, TX, and interrupt
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit character size
}

// ===== Send a Single Byte Over Serial =====
void serial0_send(uint8_t byte) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
    UDR0 = byte; // Send byte
}

// ===== Send a 5-byte Packet with Joystick Data to the Robot =====
void serial0_send_packet(uint8_t x, uint8_t y) {
    serial0_send(START_BYTE);
    serial0_send(0x01);     // Command ID for joystick
    serial0_send(x);
    serial0_send(y);
    serial0_send(END_BYTE);
}

// ===== Print Status Message Based on Received Sensor Values =====
void display_status() {
    if (sensor_L > 200 && sensor_R > 200)
        serial0_send_string("Robot is STOPPED\n");
    else if (sensor_L < 150 && sensor_R < 150)
        serial0_send_string("Moving FORWARD\n");
    else if (sensor_L < sensor_R)
        serial0_send_string("Turning LEFT\n");
    else
        serial0_send_string("Turning RIGHT\n");
}

// ===== Send a Full String Over Serial =====
void serial0_send_string(const char *s) {
    while (*s) {
        serial0_send(*s++);
    }
}

// ===== USART Receive Interrupt Handler (Reads 5-byte Packet) =====
ISR(USART_RX_vect) {
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

// ===== MAIN PROGRAM (Controller) =====
int main(void) {
    adc_init();         // Initialize ADC for joystick
    serial0_init();     // Initialize serial
    sei();              // Enable global interrupts

    while (1) {
        // Read joystick ADC (10-bit ➝ 8-bit scale)
        uint16_t x = adc_read(JOYSTICK_X_CHANNEL);
        uint16_t y = adc_read(JOYSTICK_Y_CHANNEL);

        // Send joystick position to robot
        serial0_send_packet(x >> 2, y >> 2);

        // If sensor data was received from robot, interpret it
        if (new_data) {
            display_status(); // Print interpreted message
            new_data = 0;
        }

        _delay_ms(200); // Communication refresh rate
    }
}