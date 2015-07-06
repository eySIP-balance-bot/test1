
/*
 * eYSIP-2015
 * PC Controlled Two Wheel Balanced Bot
 * Author List: B Suresh, Ramiz Hussain, Devendra Kr Jangid
 * Mentors: Piyush Manavar, Saurav Shandilya
 * Filename: balancebot.c
 * Functions:uart0_init(), ISR(USART0_RX_vect), motion_pin_config (), port_init(), timer5_init(), motion_set (unsigned char), init_devices1 (), init_devices1 (),
			 forward(), back(), stop(), set_PWM_value(int), SetTunings(double,double,double), Compute(), main()
 * Global Variables:Input, Output, errSum, Iterm, dErr, lastErr, kp, ki, kd, data, para, para_flag, error, Setpoint, spinSpeed
 *
 * The orientation information is taken from ADXL345,L3G4200D of GY-80 using I2C protocol
 * PID Controller is used to calculate the output and this output is mapped to the velocity of the motors
 * Zigbee is used Tuning and Controlling using PC
 */

#define F_CPU 14745600     //Defining frequency of microcontroller ATmega2560

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "adxl.h"        //includes header files for accelerometer,gyroscope and timer
#include "gyro.h"


#define THRESHOLD   20	//minimum pwm for the motors to start
//#define factorA     0   //offset for motor A:to equate motor speeds
#define factorB     30  //offset for motor B:to equate motor speeds--in our case motor B was slower 
#define spin        25  //pwm difference required for taking turns
#define DELAY       0   //additional delay in every cycle if required

/*working variables*/
double Input, Output;					//PID input,output
double errSum=0,Iterm=0,dErr=0,lastErr; 
double kp, ki, kd;						//PID tuning parameters
unsigned char data;                     //to store received data from UDR0
double para=0;							//the parameter that is being changed using Zigbee:temporary variable to store the data
int para_flag=0;						//flag variable to indicate the variable that is being changed
double error =0; 						//PID error
double Setpoint=0,spinSpeed=0;          //Balanced angle of the bot;variable for PWM difference in turning






//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00;                //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F;                // 14745600 Hz set baud rate lo
	UBRR0H = 0x00;                //set baud rate hi
	UCSR0B = 0x98;
}


//Interrupt subroutine(ISR) for Zigbee 
//PARAMETERS    	-->  CHARACTER(sent through Zigbee)		para_flag
//	Kp 		    	--		7									1
//	Ki				--		8									2
//	Kd		    	--	    9									3
//Setpoint			--		0									4
//increment by 0.1	--		3
//decrement by 0.1	--		6
//increment by 1	--		1
//decrement by 1	--		4
//increment by 5	--		2
//decrement by 5	--		5
//Forward			--		w
//Backward			--		x
//left				--		a
//right				--		d
//balance			--		s

ISR(USART0_RX_vect)
{
	data = UDR0;
	if (data == 55)
	{
		para = kp;				//assigning kp to para
		para_flag=1;			//setting up the flag so as to increment and decrement later
		
	}
	else if (data == 56)
	{
		para = ki;
		para_flag=2;
	}
	else if (data == 57)
	{
		para = kd;
		para_flag=3;
	}
	else if (data == 48)
	{
		para = Setpoint;
		para_flag=4;
	}
	else if (data == 97)
	{
		spinSpeed=spin;			//turning right
		lcd_cursor(2,10);		//printing on Zigbee input
		lcd_string("R");
		
	}
	else if (data == 115)
	{
		spinSpeed=0;			//balanced state without moving in either direction
		Setpoint=0;
		lcd_cursor(2,8);
		lcd_string("O O");
	}
	else if (data == 100)
	{
		spinSpeed=-1*spin;		//turning left
		lcd_cursor(2,10);
		lcd_string("L");
	}
	else if (data == 119)
	{
		Setpoint = 1;			//changing setpoint to +1 deg to move forward
		lcd_cursor(2,8);
		lcd_string("F");
	}
	else if (data == 120)
	{
		Setpoint = -1;			//changing setpoint to +1 deg to move forward
		lcd_cursor(2,8);
		lcd_string("B");
	}
	
	if (data==49)
	{
		para++;
	}
	else if (data==50)
	{
		para += 5;
	}
	else if (data==52)
	{
		para--;
	}
	else if (data==53)
	{
		para -= 5;
	}
	else if (data == 51)
	{
		para += 0.1;
	}
	else if (data == 54)
	{
		para -=0.1;
	}
	
	
	if (para_flag == 1)
	{
		kp = para;					//assigning kp to parameter
		lcd_print(1,1,kp*10,4);		//display on change of parameter
	}
	else if (para_flag == 2)
	{
		ki = para;
		lcd_print(1,6,ki*10,4);
	}
	else if (para_flag == 3)
	{
		kd = para;
		lcd_print(1,11,kd*10,4);
	}
	else if (para_flag == 4)
	{
		Setpoint = para;
		pr_int(2,1,Setpoint*10,3);
	}
	
}





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
// WGM: 5) PWM 8bit	 fast, TOP=0x00FF
// timer5 value: 56.250Hz
void timer5_init(void)
{
	TCCR5B = 0x00; //stop
	TCNT5H = 0xFF; //setup
	TCNT5L = 0x01;
	OCR5AH = 0x00;
	OCR5AL = 0xFF;
	OCR5BH = 0x00;
	OCR5BL = 0xFF;
	TCCR5A = 0xA1;
	TCCR5C = 0x00;
	TCCR5B = 0x0D; //start Timer
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

void stop (void)            // both input stationary
{
	motion_set(0x00);
}

void init_devices1 (void)
{
	cli(); //Clears the global interrupts
	
	lcd_port_config();  // configure the LCD port
	lcd_set_4bit();
	lcd_init();
	port_init();
	timer5_init();
	sei(); //Enables the global interrupts
}

//Function to set tuning parameters of PID
void SetTunings(double Kp, double Ki, double Kd)   
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

// Function for robot velocity control
/*
*
* Function Name: set_PWM_value(int)
* Input: Output of PID
* Output: None
* Logic: the variable value is clamped taking care of motor offsets and spin speed 
* Example Call: set_PWM_value(155);
*
*/
void set_PWM_value(int value) 	//set 8 bit PWM value
{
	OCR5AH = 0x00;
	OCR5BH = 0x00;
	
	if (value <= (255-factorB-spinSpeed))	//clamping motor speed
	{
		value=value+factorB+spinSpeed;
	}
	else if (value<0)
	{
		value=0;
	}
	OCR5BL=value;
	
	
	if (value>255+spinSpeed)
	{
		value=255-spinSpeed;
	}
	else if (value<0)
	{
		value=0;
	}
	OCR5AL=value;
}

/*
*
* Function Name: Compute()
* Input: error, Input, Setpoint, Iterm, kp, ki, kd, dErr, lastErr :Global Variables
* Output: Output(PID output)
* Logic: Works on the global variables to compute PID output,the integral terms is also clamped,
* 			the differential term is calculated at larger time interval of 10 ms since the difference between subsequent readings of 
*			error is very small
* Example Call: Compute()
*
*/
void Compute()                          //Function for PID controller
{
	
	/*Compute all the working error variables*/
	error = Input - Setpoint;
	Iterm += ki*0.01*error;         //Taking the sum of all previous errors to implement integral part of PID;'0.01' is added to increase the resolution while tuning 
	if (Iterm >= 255)             	//Clamping the Integral part
	{
		Iterm = 255;
	}
	else if (Iterm <= -255)
	{
		Iterm = -255;
	}
	
	if(millis(1)>=10)           	//condition to take differences after regular larger interval of time:this is to take the differential term more larger 
	{
		dErr= (error - lastErr);   	//Differential term of PID
		lastErr=error;
		start_timer4();
	}
	else
	{
		dErr=0;
	}
	
	Output = kp*error+ Iterm + kd*0.1*dErr;                      //Compute PID Output
}

/*The data sent by Zigbee is one byte at a time.We send the 2 entities(input,output) by making it into a packet.First byte is the marker for the 
purpose of recognition in PC.The second byte is the input/angle which is signed,so we add 100 to it and send.The third byte is Output of PID which ranges from 
-255 to 255,so we divide it by 2 and add 127.

To get the original values on the PC end we can do the following:
First byte:0xFF           (marker used)
Second byte:<received_data>-100;
Third byte:(<received_data>/2)-127;
*/

int main(void)          //Main program starts from here
{
	
	double acc_Angle;			//angle from accelerometer
	double gyro_Angle;			//digital reading from gyroscope
	double filt_Angle=0;		//filtered angle from complimentary filter
	unsigned int pwm_value;
	init_adxl();               	//Initialise accelerometer
	init_gyro();               	//Initialise gyroscope
	init_devices1();
	uart0_init();              	//Initialize UART0 for serial communication
	start_timer4();            	//Timer4 for timing calculations
	
	SetTunings(11.1,9.3,6.3);	//SETTING TUNING PARAMETERS
	lcd_print(1,1,kp*10,4);		//Initial printing of values on lcd
	lcd_print(1,6,ki*10,4);
	lcd_print(1,11,kd*10,4);
	
	while(1)
	{
		
		acc_Angle = 0.1*acc_angle();        			//Accelerometer angle:the output of acc_Angle is in tenth of a degree,so multiplication factor of 0.1 is required
		gyro_Angle=gyro_Rate();           				//Angular rate from Gyroscope
		filt_Angle = comp_filter(acc_Angle,gyro_Angle); //Filtered angle after passing through Complementary filter
		Input=filt_Angle;                              	//Input for error calculation of PID
		Compute();                                  	//Calling PID
		if (Output>0)                               	//Mapping PID output to velocity of motors
		{
			pwm_value = (Output+THRESHOLD);				//clamping output
			if(pwm_value>=255)
			{
				
				pwm_value=255;
			}
			set_PWM_value(pwm_value);
			forward();									//moving in the same direction as the error to counter the falling
		}
		else if(Output<0)
		{
			pwm_value = (-Output+THRESHOLD);
			if(pwm_value>=255)
			{
				
				pwm_value=255;
			}
			set_PWM_value(pwm_value);
			back();
		}
		
		UDR0=0xFF;										//marker to recognize the packet--this should be unique compared to the remaining packet 
		_delay_ms(1);									//additional delay to enable clear transmission
		UDR0=(uint8_t)(filt_Angle+100);					//sending the input angle to PC:+100 added to transmit even negative values 
		_delay_ms(1);
		uint8_t op=(Output/2)+127;						 
		UDR0=op;										//sending PID output:+127 added to transmit values upto -255
		_delay_ms(DELAY);



	}
}
