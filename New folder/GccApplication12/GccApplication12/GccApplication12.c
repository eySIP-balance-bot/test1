//Project-PC Controlled Two Wheel Balanced Bot
//Team members-B Suresh,Ramiz Hussain,Devendra Kr Jangid
//This includes ATmega2560 Development board,Gy-80 IMU module

#define F_CPU 14745600     //Defining frequency of microcontroller ATmega2560

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "adxl.h"        //includes header files for accelerometer,gyroscope and timer
#include "gyro.h"


#define THRESHOLD 0
#define factorA 0.8
#define factorB 1
#define spin 25
#define DELAY 0

/*working variables*/
double Input, Output;
double errSum=0,Iterm=0,dErr=0,lastErr;
double kp, ki, kd;
unsigned char data;                     //to store received data from UDR0
double para=0;
int para_flag=0;
double error =0;
double Setpoint=0,spinSpeed=0;       //Balanced angle of the bot
//int frequency=0;





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
	UBRR0L = 0x5F;                // 14745600 Hzset baud rate lo
	UBRR0H = 0x00;                //set baud rate hi
	UCSR0B = 0x98;
}

ISR(USART0_RX_vect)
{
	data = UDR0;
	if (data == 55)
	{
		para = kp;
		para_flag=1;
		
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
		spinSpeed=spin;
		
	}
	else if (data == 115)
	{
		spinSpeed=0;
	}
	else if (data == 100)
	{
		spinSpeed=-1*spin;
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
		kp = para;
		lcd_print(1,1,kp*10,4);
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

void SetTunings(double Kp, double Ki, double Kd)   //Function to set tuning parameters of PID
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

void pid_right(void)
{
	OCR5BH=0xFF;
	OCR5AL = 0xFF;
	right();
	_delay_ms(5);
}

void pid_left(void)
{
	OCR5AH = 0xFF;
	OCR5BL = 0xFF;
	left();
	_delay_ms(5);
}

// Function for robot velocity control
void set_PWM_value(unsigned char value) 	//set 8 bit PWM value
{
	OCR5AH = 0x00;
//	OCR5AL = value;  //motor A
	OCR5BH = 0x00;
//	OCR5BL = value;
	value=value*factorB+spinSpeed;
	if (value>255)
	{
		value=255;
	}
	else if (value<0)
	{
		value=0;
	}
	OCR5BL=value;
		
	value=value*factorA-spinSpeed;
	if (value>255)
	{
		value=255;
	}
	else if (value<0)
	{
		value=0;
	}
	OCR5AL=value;
}
void Compute()                          //Function for PID controller
{
	
	/*Compute all the working error variables*/
	error = Input - Setpoint;
	Iterm += ki*0.01*error;            //Taking the sum of all previous errors to implement integral part of PID
	if (Iterm >= 255)                 //Clamping te Integral part
	{
		Iterm = 255;
	}
	else if (Iterm <= -255)
	{
		Iterm = -255;
	}
	
	if(millis(1)>=10)           //condition to take differences after regular interval of time.
	{
		dErr= (error - lastErr);   //Differential term of PID
		lastErr=error;
		start_timer4();
	}
	else
	{
		dErr=0;
	}
	
	Output = kp*error+ Iterm + kd*0.1*dErr;                      //Compute PID Output
}


int main(void)          //Main program starts from here
{   
	
	double acc_Angle;
	double gyro_Angle;
	double filt_Angle=0;
	unsigned int pwm_value;
	init_adxl();               //Initialise accelerometer
	init_gyro();               //Initialise gyroscope
	init_devices1();
	uart0_init();              //Initailize UART0 for serial communiaction
	start_timer4();            //Timer4 for timing calculations
	
	SetTunings(9.1,8,5);
	lcd_print(1,1,kp*10,4);
	lcd_print(1,6,ki*10,4);
	lcd_print(1,11,kd*10,4);
	
	while(1)
	{
		
		acc_Angle = 0.1*acc_angle();        //Accelerometer angle
		gyro_Angle=gyro_Rate();           //Angular rate from Gyroscope
		filt_Angle = comp_filter(acc_Angle,gyro_Angle);  //Filtered angle after passing through Complementary filter
		Input=filt_Angle;                              //Input for error calculation of PID
		Compute();                                  //Calling PID 
		if (Output>0)                               //Mapping PID output to velocity of motors
		{
			pwm_value = (Output+THRESHOLD);
			if(pwm_value>=255)
			{
				
				pwm_value=255;
			}
			set_PWM_value(pwm_value);
			forward();
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
// 		else if(Input==0)
// 		{
// 			stop();
// 		}
		
		//_delay_ms(20); 
		UDR0=0xFF;
		_delay_ms(1);
		UDR0=(uint8_t)(filt_Angle+100);
		_delay_ms(1);
		uint8_t op=(Output/2)+127;
		UDR0=op;
	    _delay_ms(DELAY);



	}
}
