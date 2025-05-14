#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "serial.h"
#include "adc.h"

#define F_CPU 16000000UL

#define DEAD_ZONE 10
#define OBSTACLE_THRESHOLD 230
#define MOTOR_SPEED 70

#define FRONT_SENSOR_CHANNEL 0
#define LEFT_SENSOR_CHANNEL  1
#define RIGHT_SENSOR_CHANNEL 2
#define LDR_CHANNEL          3

#define SERVO_MIN_PULSE 200  // ~1ms pulse at 16 MHz with prescaler 64
#define SERVO_MAX_PULSE 400  // ~2ms pulse

#define SAMPLE_DURATION_MS 2000
#define DEBOUNCE_DELAY_MS 10

static int16_t lm = 0;
static int16_t rm = 0;

// ------------------------ Mapping Function ------------------------
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ------------------------ Motor + PWM (Timer1) ------------------------
void pwm_init() {
    DDRB |= (1 << DDB5) | (1 << DDB6);  // PB5 (OC1A), PB6 (OC1B)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 10000; // 2kHz PWM for motors
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

// ------------------------ Servo PWM (Timer2 on PB4 / OC2A) ------------------------
void servo_init() {
    DDRB |= (1 << PB4); // PB4 (OC2A) as output

    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, non-inverted
    TCCR2B = (1 << WGM22) | (1 << CS22); // Prescaler 64
    OCR2A = 300; // Centered
    ICR1 = 39999; // Keep Timer1 unaffected
    // Note: TOP for Timer2 is fixed at 255 in Fast PWM
}

// ------------------------ ADC & Beacon ------------------------
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

uint8_t beacon_detected() {
    uint16_t samples = 50;
    uint16_t high_count = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (read_ldr() > 50)
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
        current_state = (light_value > 50) ? 1 : 0;

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

// ------------------------ Movement ------------------------
void go_forward()   { set_motor_speeds(MOTOR_SPEED, MOTOR_SPEED); }
void turn_right()   { set_motor_speeds(MOTOR_SPEED, -MOTOR_SPEED); }
void turn_left()    { set_motor_speeds(-MOTOR_SPEED, MOTOR_SPEED); }
void go_backward()  { set_motor_speeds(-MOTOR_SPEED, -MOTOR_SPEED); }
void stop()         { set_motor_speeds(0, 0); }

// ------------------------ Main ------------------------
int main(void) {
    serial0_init();
    serial2_init();  // Joystick
    adc_init();
    motor_init();
    servo_init();
    sei();

    char msg[64];
    uint8_t data[3]; // joystick x, y, servo

    while (1) {
        if (serial2_available()) {
            serial2_get_data(data, 3);
            int16_t fc = (int16_t)data[1] - 126;
            int16_t rc = (int16_t)data[0] - 126;
            uint8_t servo_raw = data[2];

            if (abs(fc) < DEAD_ZONE) fc = 0;
            if (abs(rc) < DEAD_ZONE) rc = 0;

            lm = fc - rc;
            rm = fc + rc;
            set_motor_speeds(lm, rm);

            // Servo on Timer2 (PB4/OC2A)
            OCR2A = map(servo_raw, 0, 255, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

            sprintf(msg, "\nfc: %4d | rc: %4d | lm: %4d | rm: %4d | servo: %3d", fc, rc, lm, rm, OCR2A);
            serial0_print_string(msg);
        } else {
            // Autonomous mode
            uint16_t front = average_ADC(FRONT_SENSOR_CHANNEL);
            uint16_t left = average_ADC(LEFT_SENSOR_CHANNEL);
            uint16_t right = average_ADC(RIGHT_SENSOR_CHANNEL);

            if (beacon_detected()) {
                stop();
                _delay_ms(100);
                uint16_t freq = measure_beacon_frequency_mHz();
                sprintf(msg, "Beacon Freq: %u.%03u Hz\n", freq / 1000, freq % 1000);
                serial0_print_string(msg);
                _delay_ms(500);
            }

            if (front > OBSTACLE_THRESHOLD) {
                if (right < OBSTACLE_THRESHOLD) {
                    turn_right();
                } else if (left < OBSTACLE_THRESHOLD) {
                    turn_left();
                } else {
                    go_backward();
                    _delay_ms(500);
                    turn_left();
                }
            } else {
                if (right > OBSTACLE_THRESHOLD) {
                    turn_right();
                } else {
                    go_forward();
                }
            }

            _delay_ms(200);
        }
    }

    return 0;
}
