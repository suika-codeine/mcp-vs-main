#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"

int main(void)
{
    //Main function initialisation
    serial2_init();
    milliseconds_init();
    uint32_t current_ms, last_send_ms;
    uint8_t databyte1 = 0;
    uint8_t databyte2 = 0;
    uint8_t recievedData[2]; //recieved data array
    char serial_string[60] = {0}; //String used for printing to terminal

    while(1)
    {
       //main loop
       current_ms = milliseconds_now();
       
       //sending section
       if( (current_ms-last_send_ms) >= 100) //sending rate controlled here
       {
        //Arbitrary process to update databytes
        databyte1++;
        databyte2+=2;

        //Function takes the number of bytes to send followed by the databytes as arguments
        serial2_write_bytes(2, databyte1, databyte2);
        last_send_ms = current_ms;
       }
       if(serial2_available()) //Returns true if new data available on serial buffer
       {
        //Function takes the array to return data to and the number of bytes to be read
        serial2_get_data(recievedData,2);
        sprintf(serial_string,"\nData 1: %3u, Data2: %3u", recievedData[0],recievedData[1]); //Format string
        serial0_print_string(serial_string); //Print string to usb serial
       }
    }
}