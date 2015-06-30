#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "adxl.h"
#include "gyro.h"

#define Setpoint 0



/*working variables*/
double lastTime=0 ,lastTime2=0 ,lastTime3=0;
double Input, Output;
double errSum=0, lastErr=0 ,lastErr2=0 ,lastErr3=0,Iterm=0,lastErr4=0,lastErr5=0;
double kp, ki, kd;
unsigned char data; //to store received data from UDR0
double para=0;
int para_flag=0;
double error =0;


void Compute()
{
	/*How long since we last calculated*/
	//double timeChange = (double)millis();
	
	/*Compute all the working error variables*/
	error = Input - Setpoint;
// 	if (error==0)
// 	{
// 		errSum=0;
// 	}

	//errSum += error;
	//Iterm += error+lastErr+lastErr2+lastErr3+lastErr4+lastErr5;
	Iterm += ki*0.1*error;
	if (Iterm >= 255)
	{
		Iterm = 255;
	}
	else if (Iterm <= -255)
	{
		Iterm = -255;
	}
	
	
	double dErr = (error - lastErr);
	
	/*Compute PID Output*/
	Output = kp*error + Iterm + kd*0.1*dErr;
	
	if (Output >= 255)
	{
		Output = 255;
	}
	else if (Output <= -255)
	{
		Output = -255;
	}
	
	/*Remember some variables for next time*/
	lastErr5 = lastErr4;
	lastErr4 = lastErr3;
	lastErr3 = lastErr2;
	lastErr2=lastErr;
	lastErr = error;
	
}


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

ISR(USART0_RX_vect)
{
	data = UDR0;
	//UDR0=data;
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

// Function for robot velocity control
void set_PWM_value(unsigned char value) 	//set 8 bit PWM value
{
	OCR5AH = 0x00;
	if(value<=245)
	{
	OCR5AL = value +10;  //motor  is slower
	}
	else 
	{
		OCR5AL = 255;
	}	
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

void SetTunings(double Kp, double Ki, double Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

int main(void)
{
	double acc_Angle;
	int gyro_Angle;
	int filt_Angle;
	unsigned int pwm_value;
	init_adxl();
	init_gyro();
	init_devices1();
	uart0_init(); //Initailize UART1 for serial communiaction
	//start_timer4();
	
	SetTunings(1,0,0);
	lcd_print(1,1,kp*10,4);
	lcd_print(1,6,ki*10,4);
	lcd_print(1,11,kd*10,4);
	
	while(1)
	{
		
		acc_Angle=0.1*acc_angle();
		//pr_int(2,1,acc_Angle,3);
		gyro_Angle=gyro_Rate();
		filt_Angle=comp_filter(acc_Angle,gyro_Angle);
		//pr_int(2,10,acc_Angle,3);
		//UDR0=(int8_t)acc_angle();
		
		
		
		Input=filt_Angle;
		//pr_int(1,1,Input,3);
		if (error*lastErr < 0)
		{
			stop();
			_delay_ms(5);
		}
		Compute();
		if (Output>0)
		{
			//set_PWM_value(min((Output*Output),255));
			pwm_value = (Output)+35;
			if(pwm_value>=255)
			{
				
				pwm_value=255;
				//stop();
				//_delay_ms(50);
				//set_PWM_value(255);
				//forward();
				//_delay_ms(100);
			}
			
				
			//pr_int(2,1,pwm_value,3);
			set_PWM_value(pwm_value);
			forward();
			//_delay_ms(10);
			//pwm_value=0;
		}
		else if(Output<0)
		{
			//set_PWM_value(min(Output*Output,255));
			pwm_value = (-Output)+35;
			if(pwm_value>=255)
			{
				
				pwm_value=255;
				//stop();
				//_delay_ms(50);
				//set_PWM_value(pwm_value);
				//back();
				//_delay_ms(100);
			}
			
				
			//pr_int(2,6,pwm_value,3);
			set_PWM_value(pwm_value);
			back();
			//_delay_ms(10);
			//pwm_value=0;
		}
		else if(Output==0)
		{
			stop();
			//_delay_ms(10);
		}
		
		_delay_ms(20); 
		//pr_int(2,1,Output,3);
		UDR0=0xFF;
		_delay_ms(1);
		UDR0=(uint8_t)(filt_Angle+100);
		_delay_ms(1);
		uint8_t op=(Output/2)+127;
		UDR0=op;
		
		//UDR0=(Output/2)+127;
	}
	
}
