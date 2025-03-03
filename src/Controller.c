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