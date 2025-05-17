#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"
#include "hd44780.h"




#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1
#define JOYSTICK_X1_CHANNEL 14




int main(void) {
    adc_init();         // Initialize ADC to read analog signals
    serial2_init();     // USART2: for wireless communication with the robot
    lcd_init();         // Initialize LCD display
    sei();              // Enable global interrupts (if needed for serial)




    uint16_t x, y, x1;      // Raw 10-bit ADC values
    uint8_t x_val, y_val, x_val1;
    char line[17];
    




    while (1) {
        // 1. Read analog joystick values from ADC
        x = adc_read(JOYSTICK_X_CHANNEL);   // Read X-axis (horizontal movement)
        y = adc_read(JOYSTICK_Y_CHANNEL);   // Read Y-axis (forward/backward)


        x1 = adc_read(JOYSTICK_X1_CHANNEL);   // Read X-axis (horizontal movement)
        




        // 2. Convert to 8-bit format (0–255) expected by the robot
        x_val = x >> 2;     // Map 10-bit (0–1023) to 8-bit (0–255)
        y_val = y >> 2;


       // 3. Convert to 8-bit format (0–255) expected by the robot fo9r the servo
        x_val1 = x1 >> 2;     // Map 10-bit (0–1023) to 8-bit (0–255)
       
        // 4. Send X and Y joystick values over serial to the robot
        serial2_write_bytes(3, x_val, y_val, x_val1);




        // 4. Optional: Display values on LCD for user feedback
        lcd_clrscr();
        lcd_goto(0);
        snprintf(line, 17, "X:%3u Y:%3u", x_val, y_val);
        lcd_puts(line);




        _delay_ms(100); // Short delay to avoid flooding the serial line
    }




    return 0;
}
