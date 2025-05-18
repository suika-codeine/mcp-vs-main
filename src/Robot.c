#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "serial.h"
#include "adc.h"

#define F_CPU 16000000UL

// ADC Channels
#define FRONT_SENSOR_CHANNEL 0   // A0
#define LEFT_SENSOR_CHANNEL  1   // A1
#define RIGHT_SENSOR_CHANNEL 2   // A2
#define LDR_CHANNEL           3  // A3
#define BATTERY_CHANNEL       4  // A4 (voltage divider)

// Battery monitor
#define BATTERY_LOW_ADC       593     // Corresponds to 7.0V with 3.3k/4.7k divider
#define BATTERY_LED_PIN       PB7     // LED connected to PB7

// Beacon detection
#define THRESHOLD 50
#define SAMPLE_DURATION_MS 2000
#define DEBOUNCE_DELAY_MS 5

// Movement thresholds
#define MOTOR_SPEED      80
#define FRONT_THRESHOLD  300
#define LR_THRESHOLD     180

static int16_t lm = 0;
static int16_t rm = 0;

// ------------------------ Motor + PWM Setup ------------------------

void pwm_init() {
    DDRB |= (1 << DDB5) | (1 << DDB6);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 10000;
}

void motor_init() {
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3);
    pwm_init();
}

void set_motor_speeds(int16_t lm, int16_t rm) {
    OCR1A = (int32_t)abs(lm) * 10000 / 126;
    OCR1B = (int32_t)abs(rm) * 10000 / 126;

    if (lm >= 0) {
        PORTD |= (1 << PD2);
        PORTD &= ~(1 << PD3);
    } else {
        PORTD &= ~(1 << PD2);
        PORTD |= (1 << PD3);
    }

    if (rm >= 0) {
        PORTD |= (1 << PD0);
        PORTD &= ~(1 << PD1);
    } else {
        PORTD &= ~(1 << PD0);
        PORTD |= (1 << PD1);
    }
}

// ------------------------ ADC Utilities ------------------------

uint16_t average_ADC(uint8_t channel) {
    const uint8_t samples = 10;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += adc_read(channel);
    }
    return sum / samples;
}

uint16_t read_ldr() {
    return adc_read(LDR_CHANNEL);
}

// ------------------------ Sensor to cm Conversion ------------------------

uint16_t adc_to_cm_front(uint16_t adc) {
    if (adc < 20) return 80;  // too far
    return 12343 / (adc + 11);  // for GP2Y0A21YK (10–80cm)
}

uint16_t adc_to_cm_side(uint16_t adc) {
    if (adc < 30) return 30;  // too far
    return 6787 / (adc - 3) - 4;  // for GP2D120XJ00F (4–30cm)
}

// ------------------------ Beacon Detection ------------------------

uint8_t beacon_detected() {
    uint16_t samples = 50;
    uint16_t high_count = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (read_ldr() > THRESHOLD)
            high_count++;
        _delay_ms(2);
    }

    return (high_count > 10 && high_count < 40);
}

uint16_t measure_beacon_frequency_mHz() {
    uint8_t last_state = 0;
    uint16_t edge_count = 0;
    uint32_t time_elapsed = 0;
    uint16_t light_value;
    uint8_t current_state;

    _delay_ms(100);

    while (time_elapsed < SAMPLE_DURATION_MS) {
        light_value = read_ldr();
        current_state = (light_value > THRESHOLD) ? 1 : 0;

        if (current_state != last_state) {
            edge_count++;
            last_state = current_state;
            _delay_ms(DEBOUNCE_DELAY_MS);
            time_elapsed += DEBOUNCE_DELAY_MS;
        } else {
            _delay_ms(2);
            time_elapsed += 2;
        }
    }

    return edge_count * 250;
}

// ------------------------ Movement Wrappers ------------------------

void go_forward()     { set_motor_speeds(MOTOR_SPEED, MOTOR_SPEED); }
void rotate_right()   { set_motor_speeds(MOTOR_SPEED, -MOTOR_SPEED); }
void rotate_left()    { set_motor_speeds(-MOTOR_SPEED, MOTOR_SPEED); }
void stop()           { set_motor_speeds(0, 0); }

void turn_left() {
    stop();
    rotate_left();
    _delay_ms(750);
}

void turn_right() {
    stop();
    rotate_right();
    _delay_ms(750);
}

void turn_180() {
    stop();
    rotate_left();
    _delay_ms(1500);
}

// ------------------------ Battery LED Monitor ------------------------

void battery_led_init() {
    DDRB |= (1 << BATTERY_LED_PIN);  // Set PB7 as output
}

void check_battery_and_update_led() {
    uint16_t battery_adc = adc_read(BATTERY_CHANNEL);

    if (battery_adc < BATTERY_LOW_ADC) {
        PORTB |= (1 << BATTERY_LED_PIN);  // Turn LED on
    } else {
        PORTB &= ~(1 << BATTERY_LED_PIN); // Turn LED off
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "Battery ADC: %u\n", battery_adc);
    serial0_print_string(msg);
}

// ------------------------ Main ------------------------

int main(void) {
    serial0_init();
    adc_init();
    motor_init();
    battery_led_init();
    sei();

    char msg[64];
    uint16_t front, right, left;
    bool wallInFront, wallOnLeft, wallOnRight;

    while (1) {
        front = average_ADC(FRONT_SENSOR_CHANNEL);
        right = average_ADC(RIGHT_SENSOR_CHANNEL);
        left  = average_ADC(LEFT_SENSOR_CHANNEL);

        wallInFront = (front > FRONT_THRESHOLD);
        wallOnLeft  = (left  > LR_THRESHOLD);
        wallOnRight = (right > LR_THRESHOLD);

        //Print formatted cm values
        uint16_t front_cm = adc_to_cm_front(front);
        uint16_t right_cm = adc_to_cm_side(right);
        uint16_t left_cm  = adc_to_cm_side(left);

        snprintf(msg, sizeof(msg), "FRONT: %2ucm | RIGHT: %2ucm | LEFT: %2ucm\n", front_cm, right_cm, left_cm);
        serial0_print_string(msg);

        // Battery check
        check_battery_and_update_led();

        // Beacon check
        if (beacon_detected()) {
            stop();
            _delay_ms(100);

            uint16_t beacon_freq_mHz = measure_beacon_frequency_mHz();
            snprintf(msg, sizeof(msg), "Beacon Freq: %u.%03u Hz\n",
                     beacon_freq_mHz / 1000,
                     beacon_freq_mHz % 1000);
            serial0_print_string(msg);
            _delay_ms(500);
        }

        // Movement logic
        if (wallInFront) {
            stop();
            if (wallOnLeft && wallOnRight) {
                // turn_180(); // Optional: Uncomment to escape dead end
            } else {
                if (wallOnLeft) {
                    turn_right();
                }
                if (wallOnRight) {
                    turn_left();
                }
            }
        } else {
            go_forward();
        }

        _delay_ms(200);
    }

    return 0;
}
