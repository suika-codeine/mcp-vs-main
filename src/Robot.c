#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h> // Required for sei() and cli()
#include "serial.h"
#include "adc.h"
#include "milliseconds.h" // For precise timing for beacon detection

#define F_CPU 16000000UL

// --- ADC Channels ---
#define FRONT_SENSOR_CHANNEL 0   // A0
#define LEFT_SENSOR_CHANNEL  1   // A1
#define RIGHT_SENSOR_CHANNEL 2   // A2
#define LDR_CHANNEL           3  // A3
#define BATTERY_CHANNEL       4  // A4 (voltage divider)

// --- Battery Monitor ---
#define BATTERY_LOW_ADC       1000     // Corresponds to 7.0V with 3.3k/4.7k divider
#define BATTERY_LED_PIN       PB7     // LED connected to PB7

// --- Beacon Detection ---
#define THRESHOLD 50
#define SAMPLE_DURATION_MS 2000
#define DEBOUNCE_DELAY_MS 5
#define BEACON_COOLDOWN_PERIOD_MS 5000 // 5 seconds cooldown for beacon detection

// --- Movement Thresholds ---
#define MOTOR_SPEED     80
#define FRONT_THRESHOLD 250
#define LR_THRESHOLD    200
#define FRONT_CLOSE     150

// --- Servo Control ---
#define SERVO_DDR   DDRB
#define SERVO_PORT  PORTB
#define SERVO_PIN   PB6 // Assuming PB6 is used for servo PWM (OC1B)

// --- Global Variables for Robot State ---
static int16_t lm = 0;
static int16_t rm = 0;
static uint8_t currentSpeed = MOTOR_SPEED;
static uint32_t lastBeaconDetectionTime = 0; // To implement cooldown for beacon detection

// Define robot modes
typedef enum {
    MODE_AUTONOMOUS,
    MODE_REMOTE_CONTROL
} RobotMode;

volatile RobotMode currentRobotMode = MODE_AUTONOMOUS; // Default mode is autonomous

// Variables to store received joystick and servo data
volatile uint8_t received_x_val = 127; // Centered
volatile uint8_t received_y_val = 127; // Centered
volatile uint8_t received_servo_val = 127; // Centered

// --- Motor + PWM Setup ---

/**
 * @brief Initializes PWM for motor control.
 * Uses Timer1 in Fast PWM mode (Mode 14) with ICR1 as TOP.
 * OC1A and OC1B are used for Left and Right motor speed control.
 */
void pwm_init() {
    // Set OC1A (PB5) and OC1B (PB6) as output
    DDRB |= (1 << DDB5) | (1 << DDB6); // PB5 for OC1A, PB6 for OC1B

    // Configure Timer1 for Fast PWM mode (Mode 14)
    // WGM13:10 = 1110 (Fast PWM, TOP = ICR1)
    TCCR1A = (1 << WGM11); // WGM11 set
    TCCR1B = (1 << WGM13) | (1 << WGM12); // WGM13, WGM12 set

    // Clear OC1A/OC1B on Compare Match, set at BOTTOM (non-inverting mode)
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

    // Set TOP value for Timer1. This determines the PWM frequency.
    // With a prescaler of 8 (CS11) and F_CPU = 16MHz,
    // a TOP of 10000 gives a frequency of 16MHz / (8 * 10000) = 200 Hz.
    ICR1 = 10000;

    // Set prescaler to 8 and start Timer1
    TCCR1B |= (1 << CS11);
}

/**
 * @brief Initializes motor control pins and PWM.
 * Sets up direction pins for motors and calls pwm_init().
 */
void motor_init() {
    // Set motor direction pins as output (PD0, PD1, PD2, PD3)
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3);
    pwm_init();
}

/**
 * @brief Sets the speed and direction of the left and right motors.
 * @param lm Left motor speed (-126 to 126).
 * @param rm Right motor speed (-126 to 126).
 */
void set_motor_speeds(int16_t lm_speed, int16_t rm_speed) {
    // Convert speed (0-126) to PWM duty cycle (0-ICR1)
    // abs(lm_speed) * ICR1 / 126
    OCR1A = (uint16_t)((int32_t)abs(lm_speed) * ICR1 / 126); // Left motor PWM
    OCR1B = (uint16_t)((int32_t)abs(rm_speed) * ICR1 / 126); // Right motor PWM

    // Set direction for left motor
    if (lm_speed >= 0) { // Forward
        PORTD |= (1 << PD2); // IN1 (Left Motor)
        PORTD &= ~(1 << PD3); // IN2 (Left Motor)
    } else { // Backward
        PORTD &= ~(1 << PD2);
        PORTD |= (1 << PD3);
    }

    // Set direction for right motor
    if (rm_speed >= 0) { // Forward
        PORTD |= (1 << PD0); // IN3 (Right Motor)
        PORTD &= ~(1 << PD1); // IN4 (Right Motor)
    } else { // Backward
        PORTD &= ~(1 << PD0);
        PORTD |= (1 << PD1);
    }
}

// --- Servo Control ---

/**
 * @brief Initializes PWM for servo control.
 * Uses Timer1 (same as motors) but with specific OCR1B output for servo.
 * The servo is connected to PB6 (OC1B).
 */
void servo_init() {
    // Already initialized in pwm_init() for OC1B.
    // Just ensure the pin is set as output.
    SERVO_DDR |= (1 << SERVO_PIN);
}

/**
 * @brief Sets the servo position.
 * @param angle_8bit An 8-bit value (0-255) representing the servo angle.
 * This will be mapped to a suitable PWM pulse width.
 * For a typical servo, 0-255 might map to 0.5ms to 2.5ms pulse width.
 */
void set_servo_position(uint8_t angle_8bit) {
    // Map 0-255 to a suitable OCR1B value for servo control.
    // Standard servo pulse width: 1ms (0 degrees) to 2ms (180 degrees)
    // With ICR1 = 10000 (200 Hz PWM), 1ms pulse = 2000 counts, 2ms pulse = 4000 counts.
    // Let's map 0-255 to 1000 (0.5ms) to 5000 (2.5ms) for a wider range.
    // This mapping might need calibration based on your specific servo.
    OCR1B = (uint16_t)(1000 + (uint32_t)angle_8bit * 4000 / 255);
}

// --- ADC Utilities ---

/**
 * @brief Reads the average of multiple ADC samples from a given channel.
 * @param channel The ADC channel to read from.
 * @return The averaged 10-bit ADC value.
 */
uint16_t average_ADC(uint8_t channel) {
    const uint8_t samples = 10;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += adc_read(channel);
        _delay_us(100); // Small delay between samples
    }
    return sum / samples;
}

/**
 * @brief Reads the LDR sensor value.
 * @return The 10-bit ADC value from the LDR.
 */
uint16_t read_ldr() {
    return adc_read(LDR_CHANNEL);
}

// --- Sensor to cm Conversion ---

/**
 * @brief Converts a raw ADC value from the front IR sensor to centimeters.
 * This is a linearization for the GP2Y0A21YK (10-80cm) sensor.
 * @param adc The 10-bit ADC value.
 * @return Distance in centimeters.
 */
uint16_t adc_to_cm_front(uint16_t adc) {
    if (adc < 20) return 80;  // Too far, return max range
    return 12343 / (adc + 11);  // Empirical formula for GP2Y0A21YK
}

/**
 * @brief Converts a raw ADC value from side IR sensors to centimeters.
 * This is a linearization for the GP2D120XJ00F (4-30cm) sensor.
 * @param adc The 10-bit ADC value.
 * @return Distance in centimeters.
 */
uint16_t adc_to_cm_side(uint16_t adc) {
    if (adc < 30) return 30;  // Too far, return max range
    return 6787 / (adc - 3) - 4;  // Empirical formula for GP2D120XJ00F
}

// --- Beacon Detection ---

/**
 * @brief Checks if a beacon is detected by analyzing LDR readings.
 * Looks for significant fluctuations in light intensity.
 * @return True if a beacon is likely detected, false otherwise.
 */
uint8_t beacon_detected() {
    uint16_t samples = 50;
    uint16_t high_count = 0;
    uint16_t low_count = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (read_ldr() > THRESHOLD) {
            high_count++;
        } else {
            low_count++;
        }
        _delay_ms(2); // Small delay between samples
    }
    // A beacon is detected if there's a mix of high and low readings,
    // indicating a flashing light, not just steady light or darkness.
    return (high_count > 10 && low_count > 10);
}

/**
 * @brief Measures the frequency of a detected beacon.
 * Counts rising/falling edges of the LDR signal over a sample duration.
 * @return The measured beacon frequency in milliHertz (mHz).
 */
uint16_t measure_beacon_frequency_mHz() {
    uint8_t last_state = 0;
    uint16_t edge_count = 0;
    uint32_t start_time = milliseconds_now();
    uint32_t current_time;
    uint16_t light_value;
    uint8_t current_state;

    _delay_ms(100); // Settle time

    last_state = (read_ldr() > THRESHOLD) ? 1 : 0; // Initial state

    while ((milliseconds_now() - start_time) < SAMPLE_DURATION_MS) {
        light_value = read_ldr();
        current_state = (light_value > THRESHOLD) ? 1 : 0;

        if (current_state != last_state) {
            edge_count++;
            last_state = current_state;
            _delay_ms(DEBOUNCE_DELAY_MS); // Debounce
        }
        _delay_ms(1); // Small delay to allow time to pass
    }

    // Frequency is (number of edges / 2) / sample duration in seconds
    // Since edge_count is total edges (rising + falling), divide by 2 for cycles.
    // Convert SAMPLE_DURATION_MS to seconds (divide by 1000).
    // Result in Hz, then multiply by 1000 for mHz.
    // (edge_count / 2) / (SAMPLE_DURATION_MS / 1000) * 1000
    // = edge_count * 500 / SAMPLE_DURATION_MS * 1000
    // = edge_count * 500000 / SAMPLE_DURATION_MS
    if (SAMPLE_DURATION_MS > 0) {
        return (uint32_t)edge_count * 500000 / SAMPLE_DURATION_MS;
    } else {
        return 0;
    }
}

// --- Movement Wrappers ---

/**
 * @brief Sets the robot to move forward at current speed.
 */
void go_forward()     { set_motor_speeds(currentSpeed, currentSpeed); }

/**
 * @brief Sets the robot to rotate right at current speed.
 */
void rotate_right()   { set_motor_speeds(currentSpeed, -currentSpeed); }

/**
 * @brief Sets the robot to rotate left at current speed.
 */
void rotate_left()    { set_motor_speeds(-currentSpeed, currentSpeed); }

/**
 * @brief Stops both motors.
 */
void stop()           { set_motor_speeds(0, 0); }

/**
 * @brief Resets the motor speed to the default MOTOR_SPEED.
 */
void reset_speed() {
    currentSpeed = MOTOR_SPEED;
}

/**
 * @brief Reduces the motor speed for cautious movement.
 */
void slow_speed() {
    currentSpeed = MOTOR_SPEED - 10; // Slightly reduce speed
    if (currentSpeed < 0) currentSpeed = 0; // Ensure speed doesn't go negative
}

/**
 * @brief Turns the robot left for a fixed duration.
 */
void turn_left() {
    stop();
    _delay_ms(50); // Small pause before turning
    reset_speed();
    rotate_left();
    _delay_ms(950); // Adjust this delay for desired turn angle
    stop();
}

/**
 * @brief Turns the robot right for a fixed duration.
 */
void turn_right() {
    stop();
    _delay_ms(50); // Small pause before turning
    reset_speed();
    rotate_right();
    _delay_ms(950); // Adjust this delay for desired turn angle
    stop();
}

/**
 * @brief Performs a 180-degree turn by rotating left for a longer duration.
 */
void turn_180() {
    stop();
    _delay_ms(50); // Small pause before turning
    reset_speed();
    rotate_left();
    _delay_ms(1500); // Adjust this delay for desired 180-degree turn
    stop();
}

// --- Battery LED Monitor ---

/**
 * @brief Initializes the battery low indicator LED pin.
 */
void battery_led_init() {
    DDRB |= (1 << BATTERY_LED_PIN);  // Set PB7 as output
}

/**
 * @brief Checks battery voltage and updates the LED.
 * Also prints battery ADC value to serial for debugging.
 */
void check_battery_and_update_led() {
    uint16_t battery_adc = adc_read(BATTERY_CHANNEL);

    if (battery_adc < BATTERY_LOW_ADC) {
        PORTB |= (1 << BATTERY_LED_PIN);  // Turn LED on (low battery)
    } else {
        PORTB &= ~(1 << BATTERY_LED_PIN); // Turn LED off (battery OK)
    }

    // For debugging on robot's serial monitor [COMMENT OUT]
    // char msg[32];
    // snprintf(msg, sizeof(msg), "Battery ADC: %u\n", battery_adc);
    // serial0_print_string(msg);
}

// --- Communication Protocol ---

// Command types
#define CMD_SET_MODE        0x01
#define CMD_JOYSTICK_DATA   0x02
#define CMD_SERVO_DATA      0x03

// Report types
#define RPT_SENSOR_DATA     0x10
#define RPT_BEACON_FREQ     0x11
#define RPT_MODE_STATUS     0x12

/**
 * @brief Sends sensor data to the remote controller.
 * @param front_cm Front sensor distance in cm.
 * @param right_cm Right sensor distance in cm.
 * @param left_cm Left sensor distance in cm.
 */
void send_sensor_data(uint8_t front_cm, uint8_t right_cm, uint8_t left_cm) {
    // Packet format: [Start Byte] [Num Bytes] [Report Type] [Front CM] [Right CM] [Left CM] [End Byte]
    // Num Bytes = 3 (Report Type, Front CM, Right CM, Left CM)
    serial2_write_bytes(4, RPT_SENSOR_DATA, front_cm, right_cm, left_cm);
}

/**
 * @brief Sends beacon frequency data to the remote controller.
 * @param freq_mHz Beacon frequency in milliHertz.
 */
void send_beacon_frequency(uint16_t freq_mHz) {
    // Packet format: [Start Byte] [Num Bytes] [Report Type] [Freq MSB] [Freq LSB] [End Byte]
    // Num Bytes = 3 (Report Type, Freq MSB, Freq LSB)
    serial2_write_bytes(3, RPT_BEACON_FREQ, (uint8_t)(freq_mHz >> 8), (uint8_t)(freq_mHz & 0xFF));
}

/**
 * @brief Sends the current robot mode to the remote controller.
 * @param mode The current RobotMode.
 */
void send_mode_status(RobotMode mode) {
    // Packet format: [Start Byte] [Num Bytes] [Report Type] [Mode] [End Byte]
    // Num Bytes = 2 (Report Type, Mode)
    serial2_write_bytes(2, RPT_MODE_STATUS, (uint8_t)mode);
}


/**
 * @brief Processes incoming serial data from the remote controller.
 * This function should be called periodically in the main loop.
 */
void process_incoming_serial_data() {
    if (serial2_available()) {
        uint8_t data[6]; // Max 6 data bytes
        uint8_t num_bytes; // This will store the actual number of data bytes received (excluding start/end/numBytes)

        // It's crucial to know how many bytes to expect for serial2_get_data.
        // The serial.c ISR sets serial2DataReady when a full packet is received,
        // and numBytes is the total length including start/end bytes.
        // We need to retrieve the actual payload size from the second byte of the packet.
        // A better approach would be to modify serial_get_data to return the actual payload.
        // For now, let's assume we read up to 6 bytes and parse based on the command.

        // Temporarily disable interrupts while reading volatile global variables
        cli();
        // Access the global volatile variables directly as serial2_get_data might not be flexible enough
        // to get the numBytes from the second byte of the packet before getting the data.
        // This is a workaround given the current serial.c implementation.
        // A more robust serial library would allow peeking at the numBytes or passing it.
        uint8_t temp_serial2DataByte1 = serial2DataByte1;
        uint8_t temp_serial2DataByte2 = serial2DataByte2;
        uint8_t temp_serial2DataByte3 = serial2DataByte3;
        uint8_t temp_serial2DataByte4 = serial2DataByte4;
        uint8_t temp_serial2DataByte5 = serial2DataByte5;
        uint8_t temp_serial2DataByte6 = serial2DataByte6;
        bool temp_serial2DataReady = serial2DataReady;
        serial2DataReady = false; // Clear the flag after reading
        sei();

        if (temp_serial2DataReady) {
            uint8_t command_type = temp_serial2DataByte1; // First data byte is command type

            switch (command_type) {
                case CMD_SET_MODE:
                    // Expected payload: [Mode (0=Autonomous, 1=Remote)]
                    if (temp_serial2DataByte2 == 0) {
                        currentRobotMode = MODE_AUTONOMOUS;
                        stop(); // Stop motors when switching modes
                        serial0_print_string("Switched to Autonomous Mode\n");
                    } else if (temp_serial2DataByte2 == 1) {
                        currentRobotMode = MODE_REMOTE_CONTROL;
                        stop(); // Stop motors when switching modes
                        serial0_print_string("Switched to Remote Control Mode\n");
                    }
                    send_mode_status(currentRobotMode); // Confirm mode change to controller
                    break;

                case CMD_JOYSTICK_DATA:
                    // Expected payload: [X_val] [Y_val]
                    if (currentRobotMode == MODE_REMOTE_CONTROL) {
                        received_x_val = temp_serial2DataByte2;
                        received_y_val = temp_serial2DataByte3;
                    }
                    break;

                case CMD_SERVO_DATA:
                    // Expected payload: [Servo_val]
                    if (currentRobotMode == MODE_REMOTE_CONTROL) {
                        received_servo_val = temp_serial2DataByte2;
                    }
                    break;

                default:
                    serial0_print_string("Unknown command received.\n");
                    break;
            }
        }
    }
}


// --- Main ---

int main(void) {
    // Initialize all necessary peripherals
    serial0_init();         // For debugging output to PC
    serial2_init();         // For XBee communication with remote controller
    adc_init();
    motor_init();
    servo_init();           // Initialize servo
    battery_led_init();
    milliseconds_init();    // Initialize milliseconds timer
    sei();                  // Enable global interrupts

    char msg[64]; // Buffer for serial messages

    uint16_t front_adc, right_adc, left_adc;
    uint16_t front_cm, right_cm, left_cm;
    bool wallInFront, wallOnLeft, wallOnRight, frontWallClose;

    uint32_t lastSensorReportTime = 0;
    const uint32_t sensorReportInterval = 500; // Report sensor data every 500ms

    // Initial mode status report
    send_mode_status(currentRobotMode);

    while (1) {
        // Always process incoming serial data regardless of mode
        process_incoming_serial_data();

        // Add debug output for raw ADC values CHECK HERE
        snprintf(msg, sizeof(msg), "ADC: F:%u R:%u L:%u\n", front_adc, right_adc, left_adc);
        serial0_print_string(msg);

        // --- Sensor Readings ---
        front_adc = average_ADC(FRONT_SENSOR_CHANNEL);
        right_adc = average_ADC(RIGHT_SENSOR_CHANNEL);
        left_adc  = average_ADC(LEFT_SENSOR_CHANNEL);

        front_cm = adc_to_cm_front(front_adc);
        right_cm = adc_to_cm_side(right_adc);
        left_cm  = adc_to_cm_side(left_adc);

        // Update wall detection flags
        wallInFront = (front_adc > FRONT_THRESHOLD);
        wallOnLeft  = (left_adc  > LR_THRESHOLD);
        wallOnRight = (right_adc > LR_THRESHOLD);
        frontWallClose = (front_adc > FRONT_CLOSE);

        // Periodically send sensor data to the controller
        if (milliseconds_now() - lastSensorReportTime >= sensorReportInterval) {
            send_sensor_data((uint8_t)front_cm, (uint8_t)right_cm, (uint8_t)left_cm);
            lastSensorReportTime = milliseconds_now();
        }

        // --- Battery Check ---
        check_battery_and_update_led();

        // --- Mode-specific Logic ---
        if (currentRobotMode == MODE_AUTONOMOUS) {
            // --- Autonomous Movement Logic ---
            // if (frontWallClose) {
            //     slow_speed();
            // } else {
            //     reset_speed();
            // }

            if (wallInFront) {
                stop();
                // Prioritize turns based on wall detection, no 180-degree turn
                if (wallOnLeft) {
                    // If wall on left, turn right
                    turn_right();
                } else if (wallOnRight) {
                    // If wall on right, turn left
                    turn_left();
                } else {
                    // If only wall in front (no side walls), default to turning right
                    turn_right();
                }
            } else {
                go_forward(); // No immediate wall, keep going forward
            }

            // --- Autonomous Beacon Check ---
            if ((milliseconds_now() - lastBeaconDetectionTime) >= BEACON_COOLDOWN_PERIOD_MS) {
                if (beacon_detected()) {
                    stop();
                    _delay_ms(100); // Small delay to ensure robot is stopped

                    uint16_t beacon_freq_mHz = measure_beacon_frequency_mHz();
                    snprintf(msg, sizeof(msg), "Beacon Freq: %u.%03u Hz\n",
                            beacon_freq_mHz / 1000,
                            beacon_freq_mHz % 1000);
                    serial0_print_string(msg); // Print to robot's serial for debugging

                    send_beacon_frequency(beacon_freq_mHz); // Send to controller
                    _delay_ms(500); // Pause after beacon detection
                    lastBeaconDetectionTime = milliseconds_now(); // Reset cooldown timer
                }
            }

        } else if (currentRobotMode == MODE_REMOTE_CONTROL) {
            // --- Remote Control Movement Logic ---
            // Map 0-255 joystick values to motor speeds (-126 to 126)
            // Y-axis for forward/backward, X-axis for turning
            int16_t y_motor = (int16_t)received_y_val - 127; // -127 to 128
            int16_t x_motor = (int16_t)received_x_val - 127; // -127 to 128

            // Simple tank drive mixing
            int16_t left_motor_speed = y_motor + x_motor;
            int16_t right_motor_speed = y_motor - x_motor;

            // Clamp motor speeds to valid range (-126 to 126)
            if (left_motor_speed > 126) left_motor_speed = 126;
            if (left_motor_speed < -126) left_motor_speed = -126;
            if (right_motor_speed > 126) right_motor_speed = 126;
            if (right_motor_speed < -126) right_motor_speed = -126;

            set_motor_speeds(left_motor_speed, right_motor_speed);

            // --- Remote Control Servo Logic ---
            set_servo_position(received_servo_val);

            // For debugging on robot's serial monitor
            snprintf(msg, sizeof(msg), "RC Mode: X:%u Y:%u Servo:%u\n", received_x_val, received_y_val, received_servo_val);
            serial0_print_string(msg);
        }

        _delay_ms(50); // Main loop delay to prevent busy-waiting and allow serial processing
    }

    return 0;
}