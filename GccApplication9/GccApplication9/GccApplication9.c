#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
//#include "lcd.c"
#include "adxl.h"

#define Setpoint 0

double lastTime=0 ,lastTime2=0 ,lastTime3=0;
double Input, Output;
double errSum=0, lastErr=0 ,lastErr2=0 ,lastErr3=0;
double kp, ki, kd;
/*working variables
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd; */
int SampleTime = 100 ; //100 msec
void Compute()
{
	//unsigned long now = millis();
	int timeChange = millis();
	if(timeChange>=SampleTime)
	{
		/*Compute all the working error variables*/
		double error = Setpoint - Input;
		errSum = (error*timeChange)+(lastErr*lastTime)+(lastErr2*lastTime2);
		double dErr =(error-lastErr)/timeChange;
		
		/*Compute PID Output*/
		Output = kp * error + ki * errSum + kd * dErr;
		
		/*Remember some variables for next time*/
		lastErr2 = lastErr;
		lastErr = error;
		lastTime2 = lastTime;
		lastTime = timeChange;
	}
}

void SetTunings(double Kp, double Ki, double Kd)
{
	//double SampleTimeInSec = ((double)SampleTime)/1000;
	//kp = Kp;
	//ki = Ki * SampleTimeInSec;
	//kd = Kd / SampleTimeInSec;
	kp=Kp;
	ki=Ki;
	kd=Kd;
}

/*void SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime / (double)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}*/

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

// Function for robot velocity control
void set_PWM_value(unsigned char value) 	//set 8 bit PWM value
{
	OCR5AH = 0x00;
	OCR5AL = value;
	OCR5BH = 0x00;
	OCR5BL = value;
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


/*void SetTunings(double Kp, double Ki, double Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}*/

int main(void)
{
	//int acc_Angle;
	unsigned char pwm_value = 0;   // variable for velocity control 
	init_adxl();
	init_devices1();
	SetTunings(1,1,1);
	start_timer4();
	while(1)
	{
		Input = (double)acc_angle();
		Compute();
		pr_int(1,1,Input,3);
		pr_int(2,1,Output,5);
	}
	//_delay_ms(10);
	//acc_Angle=millis();
	//lcd_print(1,1,acc_Angle,5);
	/*while(0)
	{
		acc_Angle=acc_angle();
		Input=(double)acc_Angle;
		Compute();
		if (Output>0)
		{
			set_PWM_value(18+Output/100);
			forward();
		} 
		else
		{
			set_PWM_value(18+Output/-100);
			back();
		}
		
		pr_int(2,1,acc_Angle,3);
		pr_int(1,1,Output,5);
	} */	
}