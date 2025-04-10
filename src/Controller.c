#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"
#include "hd44780.h"

#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1

uint8_t estimate_distance_cm(uint8_t adc_val) {
    uint16_t raw_adc = adc_val << 2;
    if (raw_adc < 10) raw_adc = 10;  // prevent divide by 0
    return 4800 / raw_adc; // rough scaling to cm
}

int main(void) {
    adc_init();
    serial2_init();
    lcd_init();
    sei();

    uint16_t x, y;
    uint8_t sensorData[2];
    char line[17];

    while (1) {
        // 1. Read joystick
        x = adc_read(JOYSTICK_X_CHANNEL);
        y = adc_read(JOYSTICK_Y_CHANNEL);
        uint8_t x_val = x >> 2;
        uint8_t y_val = y >> 2;

        // 2. Send joystick to robot
        serial2_write_bytes(2, x_val, y_val);

        // 3. Wait for range sensor response
        if (serial2_available()) {
            serial2_get_data(sensorData, 2);

            uint8_t distL = estimate_distance_cm(sensorData[0]);
            uint8_t distR = estimate_distance_cm(sensorData[1]);

            // 4. Display distances
            lcd_clrscr();
            lcd_goto(0);
            snprintf(line, 17, "L:%2ucm R:%2ucm", distL, distR);
            lcd_puts(line);

            lcd_goto(64);  // second row
            if (distL > 35 && distR > 35)
                lcd_puts("Robot: STOPPED ");
            else if (distL < 20 && distR < 20)
                lcd_puts("Moving FORWARD ");
            else if (distL < distR)
                lcd_puts("Turning RIGHT   ");
            else
                lcd_puts("Turning LEFT  ");
        }

        _delay_ms(200);
    }

    return 0;
}
