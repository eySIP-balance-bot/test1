
/********************************************************************************
 Platform: ATMEGA2560 Development Board
 Experiment: Serial communication
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 08th May 2012
 AVR Studio Version 4.17, Build 666

 Concepts covered: Two wire(I2C) interfacing with DS1037 
 
 This program demonstrate the interfacing of External RTC (DS1307) with the microcontroller via I2C bus.
 Values from the RTC are displayed on the LCD.

 Hardware Setup: (Ref. Fig. 3.21 from chapter 3)
 Connect the jumpers at SCL and SDA lines at the I2C Header to interface DS1307 with the microcontroller.
 For more details refer to section 3.11

 Refer product manual for more detailed description.

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: ATMEGA2560
 	Frequency: 14745600Hz
 	Optimization: -O0 (For more information refer to the section below figure 2.22 in the product manual)

 2. Include lcd.c file in the same project file.

*********************************************************************************/
#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "adxl.h"


#define	SLAVE_W	0xD2             // Write address for DS1307 selection for writing	
#define	SLAVE_R	0xD3             // Write address for DS1307 selection for reading  

//------------------------------------------------------------------------------
// define Gyroscope register addresses
//------------------------------------------------------------------------------

#define XL   0x28      
#define XH	 0x29
#define YL	 0x2A
#define YH   0x2B
#define ZL	 0x2C
#define ZH   0x2D
#define  K   10


//------------------------------------------------------------------------------
//Function to configure LCD port
// void lcd_port_config (void)
// {
//  DDRC = DDRC | 0xF7;      //all the LCD pin's direction set as output
//  PORTC = PORTC & 0x80;    // all the LCD pins are set to logic 0 except PORTC 7
// }

//------------------------------------------------------------------------------
// I2C Peripheral Function Prototypes
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// I2C initilise
//------------------------------------------------------------------------------

//TWI initialize
// bit rate:72
// void twi_init(void)
// {
//  TWCR = 0x00;   //disable twi
//  TWBR = 0x10; //set bit rate
//  TWSR = 0x00; //set prescale
//  TWAR = 0x00; //set slave address
//  TWCR = 0x04; //enable twi
// }

//------------------------------------------------------------------------------
// Procedure:	write_byte 
// Inputs:		data out, address
// Outputs:		none
// Description:	Writes a byte to the RTC given the address register 
//------------------------------------------------------------------------------
void write_byte_gyro(unsigned char data_out,unsigned char address)
{
 TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);       // send START condition  
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);                                    

 TWDR = SLAVE_W;                                     // load SLA_W into TWDR Register
 TWCR  = (1<<TWINT) | (0<<TWSTA) | (1<<TWEN);      // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWDR = address;                                   // send address of register byte want to access register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWDR = data_out;                       // convert the character to equivalent BCD value and load into TWDR
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of data byte
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 _delay_ms(10);

 TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);       // send STOP condition
}

//------------------------------------------------------------------------------
// Procedure:	read_byte 
// Inputs:		address
// Outputs:		none
// Description:	read a byte from the RTC from send the address register 
//------------------------------------------------------------------------------
unsigned char read_byte_gyro(unsigned char address)
{  
 unsigned char rtc_recv_data;

 
TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);      // send START condition  
while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 //_delay_ms(10);

 

 TWDR = SLAVE_W;									   // load SLA_W into TWDR Register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 //_delay_ms(10); 

 TWDR = address;                                   // send address of register byte want to access register
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
// _delay_ms(10);
 


 TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);       // send RESTART condition
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 //_delay_ms(10);


 
 TWDR = SLAVE_R;									   // load SLA_R into TWDR Register
 TWCR  = (1<<TWINT) | (0<<TWSTA) | (1<<TWEN);      // clear TWINT flag to start tramnsmission of slave address 
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 //_delay_ms(10);
 
 

 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to read the addressed register
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 rtc_recv_data = TWDR;
 //_delay_ms(10);
 

 TWDR = 00;                                        // laod the NO-ACK value to TWDR register 
 TWCR  = (1<<TWINT) | (1<<TWEN);                   // clear TWINT flag to start tramnsmission of NO_ACK signal
 while(!(TWCR & (1<<TWINT)));                      // wait for TWINT Flag set
 //_delay_ms(10);
  
 return(rtc_recv_data);                            // return the read value to called function
}


// initialise the devices 
// void init_devices()
// {
//  cli();              // disable all interrupts 
//  //lcd_port_config();  // configure the LCD port 
//  twi_init();         // configur the I2cC, i.e TWI module 
//  sei();              // re-enable interrupts
//  //all peripherals are now initialized
// }

// int sign(uint16_t c) /* get negative values*/
// {
// 	if (c>32767)
// 	{
// 		c -= 65536;
// 	    return c;
// 	} 
// 	else
// 	{
// 		return c;
// 	}
// }


// 
// void pr_int(int a,int b,int c,int d) /* get negative values*/
// {
// 	if (c<0)
// 	{
// 		lcd_cursor(a,b);
// 		lcd_string("-");
// 		lcd_print(a,b+1,abs(c),d);
// 	}
// 	else
// 	{
// 		lcd_cursor(a,b);
// 		lcd_string("+");
// 		lcd_print(a,b+1,c,d);
// 	}
// }
//-------------------------------------------------------------------------------
// Main Programme start here.
//-------------------------------------------------------------------------------
int main(void)
{   
  uint16_t x_byte = 0,y_byte = 0,z_byte = 0;
  uint8_t x_byte1 = 0,x_byte2 = 0,y_byte1 = 0,y_byte2 = 0,z_byte1 = 0,z_byte2 = 0;
  double gy_angle =0,gy_sum=0;
  int16_t x_ang=0;

 //init_devices();
 init_adxl();
//  lcd_set_4bit();                // set the LCD in 4 bit mode
//  lcd_init();                    // initialize the LCD with its commands
//  display_clear();               // clear the LCD
 write_byte_gyro(0x0F,0x20);       //Normal mode of control reg.1
 //write_byte(0x8,0x2D);
start_timer4();

 
while(1)
{
	   
	   
	  x_byte1 = read_byte_gyro(XL);
	  //lcd_print(1,1,x_byte1,3);
	   
	   x_byte2 = read_byte_gyro(XH);
	   //lcd_print(2,1,x_byte2,3);
	   
	   y_byte1 = read_byte_gyro(YL);
	   //lcd_print(1,6,y_byte1,3);
	   
	   y_byte2 = read_byte_gyro(YH);
	   //lcd_print(2,6,y_byte2,3);
	   
	   z_byte1 = read_byte_gyro(ZL);
	   //lcd_print(1,10,z_byte1,3);
	   
	   z_byte2 = read_byte_gyro(ZH);
	   //lcd_print(2,10,z_byte2,3);
	   
	   x_byte = x_byte2;   // to print 10 bit integer value on LCD
	   x_byte = (x_byte << 8);
	   x_byte |= x_byte1;
	   x_ang = sign(x_byte);
	   //x_ang /=100;
	   
	   //lcd_print(1,1,x_byte,5);
	   //pr_int(1,1,x_ang,3);
	   
	   /*y_byte = y_byte2;
	   y_byte = (y_byte << 8);
	   y_byte |= y_byte1;
	   //lcd_print(2,5,y_byte,5);
	   //pr_int(2,4,y_byte,5);
	   
	   z_byte = z_byte2;
	   z_byte = (z_byte << 8);
	   z_byte |= z_byte1;
	   //lcd_print(1,10,z_byte,5);
	   //pr_int(1,9,z_byte,5);*/
	   double timechange=micros();
	   timechange /= 1000;
	   gy_angle = (x_ang*timechange);
	   //pr_int(2,10,gy_angle,3);
	   gy_sum +=gy_angle;
	   gy_sum /= 100*K;
	   
	    //gy_sum = gy_angle+x_ang;
        pr_int(2,1,gy_sum,3);
	    pr_int(1,1,acc_angle(),3);
		//gy_angle = gy_sum;
		//_delay_ms(100);
	  
	  }
}

//--------------------------------------------------------------------------------




