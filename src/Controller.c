#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h> // For boolean types
#include <stdio.h>   // For snprintf
#include <avr/interrupt.h> // For sei()
#include "adc.h"
#include "serial.h"
#include "hd44780.h" // LCD library
#include "milliseconds.h" // For timing

// --- Joystick ADC Channels ---
#define JOYSTICK_X_CHANNEL    0
#define JOYSTICK_Y_CHANNEL    1
#define JOYSTICK_SERVO_CHANNEL 14 // Assuming this is for the servo control input

// --- Mode Toggle Button ---
#define MODE_TOGGLE_BUTTON_PORT PORTD
#define MODE_TOGGLE_BUTTON_DDR  DDRD
#define MODE_TOGGLE_BUTTON_PIN  PD1 

// --- Communication Protocol ---
// Command types (sent from Controller to Robot)
#define CMD_SET_MODE        0x01
#define CMD_JOYSTICK_DATA   0x02
#define CMD_SERVO_DATA      0x03

// Report types (received from Robot by Controller)
#define RPT_SENSOR_DATA     0x10
#define RPT_BEACON_FREQ     0x11
#define RPT_MODE_STATUS     0x12

// --- Global Variables for Controller State ---
typedef enum {
    CONTROLLER_MODE_AUTONOMOUS,
    CONTROLLER_MODE_REMOTE_CONTROL
} ControllerMode;

volatile ControllerMode currentControllerMode = CONTROLLER_MODE_AUTONOMOUS; // Initial mode

// Variables to store received data from robot
volatile uint8_t robot_front_cm = 0;
volatile uint8_t robot_right_cm = 0;
volatile uint8_t robot_left_cm = 0;
volatile uint16_t robot_beacon_freq_mHz = 0;
volatile ControllerMode robot_reported_mode = CONTROLLER_MODE_AUTONOMOUS;

static uint32_t lastButtonPressTime = 0;
const uint32_t buttonDebounceDelay = 200; // milliseconds

// --- Function Prototypes ---
void init_mode_toggle_button(void);
bool is_mode_toggle_button_pressed(void);
void process_incoming_robot_data(void);
void send_mode_toggle_command(void);
void send_joystick_and_servo_data(uint8_t x_val, uint8_t y_val, uint8_t servo_val);
void update_lcd_display(void);


// --- Main ---
int main(void) {
    // Initialize peripherals
    adc_init();         // Initialize ADC to read analog signals
    serial0_init();     // For debugging output to PC
    serial2_init();     // USART2: for wireless communication with the robot (XBee)
    lcd_init();         // Initialize LCD display
    milliseconds_init(); // Initialize milliseconds timer for debouncing and timing
    init_mode_toggle_button(); // Initialize the mode toggle button
    sei();              // Enable global interrupts

    uint16_t x_raw, y_raw, servo_raw;      // Raw 10-bit ADC values
    uint8_t x_val, y_val, servo_val;       // Converted 8-bit values (0-255)

    uint32_t lastSendTime = 0;
    const uint32_t sendInterval = 100; // Send data every 100ms

    // Initial LCD display setup
    lcd_clrscr(); // Clear the LCD screen
    lcd_puts("Init..."); // Display "Init..." on LCD
    _delay_ms(500); // Delay for 500ms

    // Send initial mode status to robot
    send_mode_toggle_command(); // Send the current mode to the robot

    while (1) {
        // 1. Read analog joystick and servo values from ADC
        x_raw = adc_read(JOYSTICK_X_CHANNEL);   // Read X-axis (horizontal movement)
        y_raw = adc_read(JOYSTICK_Y_CHANNEL);   // Read Y-axis (forward/backward)
        servo_raw = adc_read(JOYSTICK_SERVO_CHANNEL); // Read servo control input

        // 2. Convert to 8-bit format (0–255)
        x_val = x_raw >> 2;     // Map 10-bit (0–1023) to 8-bit (0–255)
        y_val = y_raw >> 2; // Map 10-bit (0–1023) to 8-bit (0–255)
        servo_val = servo_raw >> 2; // Map 10-bit (0–1023) to 8-bit (0–255)

        // 3. Check for mode toggle button press
        if (is_mode_toggle_button_pressed()) { // Check if the mode toggle button is pressed
            if (milliseconds_now() - lastButtonPressTime > buttonDebounceDelay) { // Check for debounce delay
                // Toggle mode
                if (currentControllerMode == CONTROLLER_MODE_AUTONOMOUS) { // If current mode is autonomous
                    currentControllerMode = CONTROLLER_MODE_REMOTE_CONTROL; // Switch to remote control mode
                } else { // If current mode is remote control
                    currentControllerMode = CONTROLLER_MODE_AUTONOMOUS; // Switch to autonomous mode
                }
                send_mode_toggle_command(); // Send the new mode to the robot
                lastButtonPressTime = milliseconds_now(); // Update last press time
            }
        }

        // 4. Send data to robot based on current mode
        if (milliseconds_now() - lastSendTime >= sendInterval) { // Check if enough time has passed since last send
            if (currentControllerMode == CONTROLLER_MODE_REMOTE_CONTROL) { // If in remote control mode
                // Only send joystick and servo data if in remote control mode
                send_joystick_and_servo_data(x_val, y_val, servo_val); // Send joystick and servo data to the robot
            }
            // Always send mode toggle command if it was just pressed (handled by send_mode_toggle_command)
            lastSendTime = milliseconds_now(); // Update last send time
        }

        // 5. Process incoming data from the robot
        process_incoming_robot_data(); // Process any data received from the robot

        // 6. Update LCD display
        update_lcd_display(); // Update the information shown on the LCD

        _delay_ms(50); // Short delay to avoid flooding the serial line and allow other tasks
    }

    return 0;
}

// --- Function Implementations ---

/**
 * @brief Initializes the mode toggle button pin as input with pull-up.
 */
void init_mode_toggle_button(void) {
    MODE_TOGGLE_BUTTON_DDR &= ~(1 << MODE_TOGGLE_BUTTON_PIN); // Set as input
    MODE_TOGGLE_BUTTON_PORT |= (1 << MODE_TOGGLE_BUTTON_PIN); // Enable pull-up resistor
}

/**
 * @brief Checks if the mode toggle button is pressed.
 * Assumes active low (button pulls pin to GND).
 * @return True if button is pressed, false otherwise.
 */
bool is_mode_toggle_button_pressed(void) {
    return !((PIND >> MODE_TOGGLE_BUTTON_PIN) & 0x01); // Read pin state
}

/**
 * @brief Sends a command to the robot to set its operational mode.
 */
void send_mode_toggle_command(void) {
    // Packet format: [Start Byte] [Num Bytes] [Command Type] [Mode] [End Byte]
    // Num Bytes = 2 (Command Type, Mode)
    serial2_write_bytes(2, CMD_SET_MODE, (uint8_t)currentControllerMode); // Send mode command to robot
}

/**
 * @brief Sends joystick (X, Y) and servo data to the robot.
 * @param x_val 8-bit X-axis joystick value.
 * @param y_val 8-bit Y-axis joystick value.
 * @param servo_val 8-bit servo control value.
 */
void send_joystick_and_servo_data(uint8_t x_val, uint8_t y_val, uint8_t servo_val) {
    // Packet for joystick data: [Start Byte] [Num Bytes] [Command Type] [X_val] [Y_val] [End Byte]
    // Num Bytes = 3 (Command Type, X_val, Y_val)
    serial2_write_bytes(3, CMD_JOYSTICK_DATA, x_val, y_val); // Send joystick X and Y data

    // Packet for servo data: [Start Byte] [Num Bytes] [Command Type] [Servo_val] [End Byte]
    // Num Bytes = 2 (Command Type, Servo_val)
    serial2_write_bytes(2, CMD_SERVO_DATA, servo_val); // Send servo data
}

/**
 * @brief Processes incoming serial data from the robot.
 * Updates global variables with received sensor data, beacon frequency, or mode status.
 */
void process_incoming_robot_data(void) {
    if (serial2_available()) { // Check if data is available on serial2
        char serial_output_buffer[32]; // Buffer for serial output

        // Temporarily disable interrupts while reading volatile global variables
        cli(); // Disable global interrupts
        uint8_t temp_serial2DataByte1 = serial2DataByte1; // Get first data byte
        uint8_t temp_serial2DataByte2 = serial2DataByte2; // Get second data byte
        uint8_t temp_serial2DataByte3 = serial2DataByte3; // Get third data byte
        uint8_t temp_serial2DataByte4 = serial2DataByte4; // Get fourth data byte
        bool temp_serial2DataReady = serial2DataReady; // Check if serial data is ready
        serial2DataReady = false; // Clear the flag after reading
        sei(); // Enable global interrupts

        if (temp_serial2DataReady) { // If serial data is ready
            uint8_t report_type = temp_serial2DataByte1; // First data byte is report type

            switch (report_type) {
                case RPT_SENSOR_DATA: // If report is sensor data
                    // Expected payload: [Front CM] [Right CM] [Left CM]
                    robot_front_cm = temp_serial2DataByte2; // Store front sensor data
                    robot_right_cm = temp_serial2DataByte3; // Store right sensor data
                    robot_left_cm  = temp_serial2DataByte4; // Store left sensor data
                    // Print sensor data to serial monitor
                    snprintf(serial_output_buffer, sizeof(serial_output_buffer), "Sensors: F:%u R:%u L:%u cm\n",
                             robot_front_cm, robot_right_cm, robot_left_cm);
                    serial0_print_string(serial_output_buffer);
                    break;

                case RPT_BEACON_FREQ: // If report is beacon frequency
                    // Expected payload: [Freq MSB] [Freq LSB]
                    robot_beacon_freq_mHz = ((uint16_t)temp_serial2DataByte2 << 8) | temp_serial2DataByte3; // Store beacon frequency
                    // Print beacon frequency to serial monitor
                    snprintf(serial_output_buffer, sizeof(serial_output_buffer), "Beacon Freq: %u.%03u Hz\n",
                             robot_beacon_freq_mHz / 1000, robot_beacon_freq_mHz % 1000);
                    serial0_print_string(serial_output_buffer);
                    break;

                case RPT_MODE_STATUS: // If report is mode status
                    // Expected payload: [Mode]
                    robot_reported_mode = (ControllerMode)temp_serial2DataByte2; // Store reported mode
                    break;

                default:
                    // Removed the general "Unknown report" debug print
                    break;
            }
        }
    }
}

/**
 * @brief Updates the LCD display with current status and robot data.
 */
void update_lcd_display(void) {
    char line1[17]; // Buffer for the first line of LCD
    char line2[17]; // Buffer for the second line of LCD

    lcd_clrscr(); // Clear the LCD screen

    // Line 1: Current Controller Mode and Robot's Reported Mode
    snprintf(line1, sizeof(line1), "C:%s R:%s", // Format string for line 1
             (currentControllerMode == CONTROLLER_MODE_AUTONOMOUS) ? "AUTO" : "RC", // Display "AUTO" or "RC" for controller mode
             (robot_reported_mode == CONTROLLER_MODE_AUTONOMOUS) ? "AUTO" : "RC"); // Display "AUTO" or "RC" for robot reported mode
    lcd_goto(0); // Go to the first line
    lcd_puts(line1); // Put the formatted string on LCD

    // Line 2: Sensor Data or Beacon Frequency based on Robot's mode
    if (robot_reported_mode == CONTROLLER_MODE_AUTONOMOUS) { // If robot is in autonomous mode
        // Display sensor data in autonomous mode
        snprintf(line2, sizeof(line2), "F:%u R:%u L:%u", // Format string for line 2 to display sensor data
                 robot_front_cm, robot_right_cm, robot_left_cm); // Display front, right, and left sensor data
    } else { // If robot is in remote control mode
        // Display beacon frequency (if detected) or joystick values in remote control mode
        if (robot_beacon_freq_mHz > 0) { // If a beacon frequency is detected
            snprintf(line2, sizeof(line2), "Bcn Freq:%u.%03u", // Format string for line 2 to display beacon frequency
                     robot_beacon_freq_mHz / 1000, robot_beacon_freq_mHz % 1000); // Display beacon frequency
        } else {
            // In RC mode, if no beacon detected, display joystick values for feedback
            // For simplicity, we'll just show a general RC message if no beacon.
            snprintf(line2, sizeof(line2), "RC Active"); // Display "RC Active"
        }
    }
    lcd_goto(0x40); // Go to second line
    lcd_puts(line2); // Put the formatted string on LCD
}
