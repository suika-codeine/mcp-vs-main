//Example ATmega2560 Project
//File: Robot.h
//Author: Robert Howie (original), Gemini (modifications)

#ifndef ROBOT_H_ // Use ROBOT_H_ for robot-specific header
#define ROBOT_H_

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

// Constants
#define BUILD_DATE __TIME__ " " __DATE__"\n"

// Define robot modes
typedef enum {
    MODE_AUTONOMOUS,
    MODE_REMOTE_CONTROL
} RobotMode;

// Global variable for robot mode (declared extern to be defined in Robot.c)
extern volatile RobotMode currentRobotMode;
extern volatile uint8_t received_x_val;
extern volatile uint8_t received_y_val;
extern volatile uint8_t received_servo_val;


// Function Prototypes for Robot.c
void pwm_init(void);
void motor_init(void);
void set_motor_speeds(int16_t lm, int16_t rm);

void servo_init(void);
void set_servo_position(uint8_t angle_8bit);

uint16_t average_ADC(uint8_t channel);
uint16_t read_ldr(void);
uint16_t adc_to_cm_front(uint16_t adc);
uint16_t adc_to_cm_side(uint16_t adc);

uint8_t beacon_detected(void);
uint16_t measure_beacon_frequency_mHz(void);

void go_forward(void);
void rotate_right(void);
void rotate_left(void);
void stop(void);
void reset_speed(void);
void slow_speed(void);
void turn_left(void);
void turn_right(void);
void turn_180(void);

void battery_led_init(void);
void check_battery_and_update_led(void);

// Communication Protocol Functions
void send_sensor_data(uint8_t front_cm, uint8_t right_cm, uint8_t left_cm);
void send_beacon_frequency(uint16_t freq_mHz);
void send_mode_status(RobotMode mode);
void process_incoming_serial_data(void);


#endif /* ROBOT_H_ */