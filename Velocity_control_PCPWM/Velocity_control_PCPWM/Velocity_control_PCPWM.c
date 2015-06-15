/*/********************************************************************************
 Written by: Vinod Desai,Sachitanand Malewar NEX Robotics Pvt. Ltd.
 Edited by: e-Yantra team
 AVR Studio Version 6

 Date: 19th October 2012

 This experiment demonstrates robot velocity control using PWM.

 Concepts covered:  Use of timer to generate PWM for velocity control

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 
  
 Connection Details:  	L-1---->PL0;		L-2---->PL1;
   						R-1---->PL2;		R-2---->PL3;
   						PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 


 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current 
	surge which can reset the microcontroller because of sudden fall in voltage. 
	It is a good practice to stop the motors for at least 0.5seconds before changing 
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

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

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRL = DDRL | 0xE4;    //set direction of the PORTL2.5,6,7 pins as output
	PORTL = PORTL & 0x18;  //set initial value of the PORTL2.5,6,7 pins to logic 0
	DDRL = DDRL | 0x18;    //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18;  //PL3 and PL4 pins are for velocity control using PWM
}

//Function to initialize ports
void init_ports()
{
 motion_pin_config();
}

// Timer 5 initialized in Phase correct PWM mode for velocity control
// Prescale:1024
// PWM 10bit phase_correct, TOP=0x03FF
// Timer Frequency:
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFC;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x03;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x03;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x03;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xAB;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=1, WGM50=1} Along With (WGM53=0,WGM52=0) in TCCR5B for Selecting Phase_correct PWM 10-bit Mode*/
	
	TCCR5B = 0x05;	//WGM52=0; CS52=1, CS51=0, CS50=1 (Prescaler=1024)
}

// Function for robot velocity control
void velocity (unsigned int left_motor, unsigned int right_motor)
{   
	OCR5A = (unsigned int)left_motor;
	OCR5B = (unsigned int)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortLRestore = 0;

 //Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortLRestore = PORTL; 			// reading the PORTA's original status
 PortLRestore &= 0x18; 			// setting lower direction nibbel to 0
 PortLRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
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

void init_devices (void) //use this function to initialize all devices
{
 cli(); //disable all interrupts
 init_ports();
 timer5_init();
 sei(); //re-enable interrupts
}

//Main Function
int main()
{
	init_devices();
	  
	while(1)
	{
		motion_set(0x20);
		_delay_ms(500);
		
		motion_set(0x40);
		_delay_ms(500);
		
		stop();
		_delay_ms(500);
		//velocity (255, 255); //Smaller the value lesser will be the velocity.Try different valuse between 0 to 1023
		//forward(); //both wheels forward
		//_delay_ms(3000);

		//stop();						
		//_delay_ms(500);
	
		//velocity (1023, 1023);
		//back(); //both wheels backward						
		//_delay_ms(500);

		//stop();						
		//_delay_ms(500);
		
		
		/*left(); //Left wheel backward, Right wheel forward
		_delay_ms(1000);
		
		stop();						
		_delay_ms(500);
		
		/*right(); //Left wheel forward, Right wheel backward
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_left(); //Left wheel stationary, Right wheel forward
		_delay_ms(1000);
		
		stop();						
		_delay_ms(500);

		soft_right(); //Left wheel forward, Right wheel is stationary
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_left_2(); //Left wheel backward, right wheel stationary
		_delay_ms(1000);

		stop();						
		_delay_ms(500);

		soft_right_2(); //Left wheel stationary, Right wheel backward
		_delay_ms(1000);

		stop();						
		_delay_ms(1000); */
	}
}

