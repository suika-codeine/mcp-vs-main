//Example ATmega2560 Project
//File: Controller.h
//Author: Robert Howie (original), Gemini (modifications)

#ifndef CONTROLLER_H_ // Use CONTROLLER_H_ for controller-specific header
#define CONTROLLER_H_

// Include standard libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>

// Include header files for private libraries
#include "serial.h"         // Minimal serial lib
#include "adc.h"            // Minimal ADC lib
#include "milliseconds.h"   // Milliseconds timekeeping lib
#include "hd44780.h"        // LCD lib

// Constants
#define BUILD_DATE __TIME__ " " __DATE__"\n"

// Define controller modes
typedef enum {
    CONTROLLER_MODE_AUTONOMOUS,
    CONTROLLER_MODE_REMOTE_CONTROL
} ControllerMode;

// Global variables (declared extern to be defined in Controller.c)
extern volatile ControllerMode currentControllerMode;
extern volatile uint8_t robot_front_cm;
extern volatile uint8_t robot_right_cm;
extern volatile uint8_t robot_left_cm;
extern volatile uint16_t robot_beacon_freq_mHz;
extern volatile ControllerMode robot_reported_mode;


// Function Prototypes for Controller.c
void init_mode_toggle_button(void);
bool is_mode_toggle_button_pressed(void);
void process_incoming_robot_data(void);
void send_mode_toggle_command(void);
void send_joystick_and_servo_data(uint8_t x_val, uint8_t y_val, uint8_t servo_val);
void update_lcd_display(void);


#endif /* CONTROLLER_H_ */
