### MCP - Visual Studio Code version

This directory contains all that is needed to set up and get started with the visual studio code version of the microcontroller project.

To get started follow the directions below
 - Download a zip of the repository which will contain the code and libraries used during the project
 - Extract the zip to a location you will work from for the project
 - If Visual Studio **Code** is not installed on your device go to https://code.visualstudio.com/Download and install Visual Studio Code (note not Visual Studio)
 - Open Visual Studio Code and select the extensions tab on the left (ctrl + shift + x) and install all of these which are not already installed; PlatformIO IDE, C/C++, C/C++ extension Pack, Serial Monitor and CMake.
 - After extensions are installed restart Visual Studio Code
 - Select the PlatformIO tab on the left, select open project and select the mcp-vs folder which contains the platformio.ini file, it should end in "MCP_VS-main\MCP_VS-main\mcp-vs"
 - Inside explorer (ctrl + shift + r) select the platformio.ini file and change the upload port to the port assigned to your arduino
   - This can be found by running "mode" in the terminal
 - Add code to the Controller.c or Robot.c files then click build or upload under the controller or robot tasks in the platformIO tab
- 1. git add -A
- 2. git commit -m "comment"
- 3. git push
- 4. git pull

Lab 1:
//Example ATmega2560 Project
//File: ATmega2560Project.c
//An example file for second year mechatronics project

//include this .c file's header file
#include "Controller.h"

//static function prototypes, functions only called in this file

int main(void)
{
  DDRC = 0xFF;//put PORTA into output mode
  PORTC = 0; 
  while(1)//main loop
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      PORTC = (1<<i); 
      _delay_ms(500);
    }

      for (uint8_t i = 7; i < 255; i--)
    {
      PORTC=(1<<i);
      _delay_ms(500);
    }    
    //_delay_ms(500);     //500 millisecond delay
    //PORTA |= (1<<PA3);  // note here PA3 is just an alias for the number 3
    //PORTA &= ~(1<<PA4);
                        // this line is equivalent to PORTA = PORTA | 0b00001000   which writes a HIGH to pin 3 of PORTA
    //_delay_ms(500); 
    //PORTA &= ~(1<<PA3); // this line is equivalent to PORTA = PORTA & (0b11110111)  which writes a HIGH to pin 3 of PORTA
    //PORTA |= (1<<PA4);
  }
  return(1);
}//end main 