/********************************************************************************
 Platform: Atmega2560 Development Board.
 Experiment: PWM generation using timer for L293D driver 
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 08th May 2012
 AVR Studio Version 4.17, Build 666

 This experiment demonstrates L293D driver for  motor control application 
 using PWM 

 Concepts covered: Simple motion control

 There are two components to the motion control:
 1. Direction control using pins PORTL2,5,6,7
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B of timer 5.
  
 Connection Details: 
 
 input side: 	
 L293D Header 		PORTL con	   
 (10pin header)     (10pin header)
 ------------------------------------------------
 pin8(INPUT1)		pin1(PL7)       
 pin7(INPUT2)		pin2(PL6)       
 pin4(Inhibit1)		pin4(PL3) (PWM using timer 5)
 pin6(INPUT3)		pin6(PL5)		
 pin3(INPUT4)		pin3(PL2)		
 pin5(Inhibit2)		pin5(PL4) (PWM using timer 5)
 -------------------------------------------------
 
 L293D output side:

 L293D con 		 LED con	
(7pin berg strip)	(10 pin header)   
 ----------------------------------
 pin1(GND)		gnd of any connctor       
 pin2(VINB)		Refer to section 3.9 in the product manual     
 pin3(VINE)	    Refer to section 3.9 in the product manual		
 pin4(A)		pin1(LED1)	
 pin5(B)		pin2(LED2)	
 pin6(C)		pin3(LED3)     	
 pin7(D)		pin4(LED4)     		
 -----------------------------------------
  
 Note: Refer to the section 3.9 from the hardware manual for detailed connections.

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600Hz
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 2.22 in the hardware manual)
 
*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

void motion_pin_config (void)
{
 DDRL = DDRL | 0xE4;    //set direction of the PORTL2.5,6,7 pins as output
 PORTL = PORTL & 0x18;  //set initial value of the PORTL2.5,6,7 pins to logic 0
 DDRL = DDRL | 0x18;    //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18;  //PL3 and PL4 pins are for velocity control using PWM
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();
}

// TIMER5 initialize - prescale:1024
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// timer5 value: 56.250Hz
void timer5_init(void)
{
 TCCR5B = 0x00; //stop
 TCNT5H = 0xFC; //setup
 TCNT5L = 0x00;
 OCR5AH = 0x03;
 OCR5AL = 0xFF;
 OCR5BH = 0x03;
 OCR5BL = 0xFF;
 TCCR5A = 0xAB;
 TCCR5C = 0x00;
 TCCR5B = 0x04; //start Timer
}

// Function for robot velocity control
void set_PWM_value(unsigned char value) 	//set 8 bit PWM value
{
/* OCR5AH = 0x00;
 OCR5AL = value;
 OCR5BH = 0x00;
 OCR5BL = value; */
 OCR5A = value;
 lcd_print(1,1,value,5);
 OCR5B = value;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortLRestore = 0;

 PortLRestore = PORTL; 			// reading the PORTL's original status
 PortLRestore &= 0x18; 			// setting lower direction nibbel to 0
 PortLRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTL status
 PORTL = PortLRestore; 			// setting the command to the port
}

void forward (void) 		//both inputs forward
{
  motion_set(0xA0);
}

void back (void) 			//both inputs backward
{
  motion_set(0x44);
}

void left (void) 			//input12 backward, input34 forward
{
  motion_set(0x84);
}

void right (void) 			//input34 backward, input12 forward
{
  motion_set(0x60);
}

void soft_left (void) 		//input12 stationary, input34 forward
{
 motion_set(0x80);
}

void soft_right (void)      //input12 forward, input34 stationary
{
 motion_set(0x20);
}

void soft_left_2 (void)     //input12 backward, input34 stationary
{
 motion_set(0x40);
}

void soft_right_2 (void)    //input12 stationary, input34 backward
{
 motion_set(0x04);
}

void stop (void)            // both input stationary
{
 motion_set(0x00);
}
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;      //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;    // all the LCD pins are set to logic 0 except PORTC 7
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 timer5_init();
 lcd_port_config();
 sei(); //Enables the global interrupts
}


//Main 
int main()
{
     
  
  init_devices();
  lcd_init();
  lcd_set_4bit();
  
  unsigned char pwm_value = 200;
  set_PWM_value(pwm_value); 

  while(1)
  {
	 forward();              //Logic Input to L293D for forward direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above
 
	 back();                 //Logic Input to L293D for back direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 left();                 //Logic Input to L293D for left direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 right();                //Logic Input to L293D for right direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 soft_left();            //Logic Input to L293D for soft left direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 soft_right();           //Logic Input to L293D for soft right direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 soft_right_2();         //Logic Input to L293D for soft right 2 direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above

	 soft_left_2();          //Logic Input to L293D for soft left 2 direction
	 _delay_ms(2000);        //we can use other direction control function as above

	 stop();                 //Logic Input to L293D for stop direction
	 _delay_ms(250);         //we can use other direction control function as above
  }
}

