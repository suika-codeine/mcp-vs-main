#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"
#include "hd44780.h"
#include <string.h>
#include <stdio.h>

#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1
#define JOYSTICK_Z_CHANNEL [?]

int main(void) {
    adc_init();         // Initialize ADC to read analog signals
    serial2_init();     // USART2: for wireless communication with the robot
    lcd_init();         // Initialize LCD display
    sei();              // Enable global interrupts if needed

    uint16_t x, y, z;
    uint8_t x_val, y_val, z_val;
    char line[17];
    char rx_buffer[8];
    uint8_t rx_index = 0;
    char received;

    uint8_t battery_low = 0;  // <--- New flag

    while (1) {
        // === 1. Check for LOW BATTERY warning ===
        if(serial2_available())
        {
            serial2_get_data(1,rx_buffer);
            if(rx_buffer[0] ==0){
                battery_low = 1;
            }

        }

        // === 2. Display LOW BATTERY if triggered ===
        if (battery_low) {
            lcd_clrscr();
            lcd_goto(0);
            lcd_puts("! LOW BATTERY !");
            _delay_ms(2000);  // Stay visible 3s
            // lcd_clrscr();
            continue;         // Skip joystick update
        }

        // === 3. Read joystick analog values ===
        x = adc_read(JOYSTICK_X_CHANNEL);
        y = adc_read(JOYSTICK_Y_CHANNEL);
        z = adc_read(JOYSTICK_Z_CHANNEL);
        x_val = x >> 2;
        y_val = y >> 2;
        z_val = z >> 2;

        // === 4. Send X/Y over serial to robot ===
        serial2_write_bytes(3, x_val, y_val, z_val);

        // === 5. Display on LCD ===
        // lcd_clrscr();
        // lcd_goto(0);
        // snprintf(line, 17, "X:%3u Y:%3u", x_val, y_val);
        // lcd_puts(line);

        _delay_ms(100);
    }

    return 0;
}
