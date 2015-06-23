
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

void init_gyro(void)
{   
 

	 write_byte_gyro(0x0F,0x20);       //Normal mode of control reg.1
}

//Complementary filter
float comp_filter(float newAngle,  float newRate) 
{   
	static float filterAngle;
	float dt=0.01;

	float filterTerm0;
	float filterTerm1;
	float filterTerm2;
	float timeConstant;

	timeConstant=30; // default 1.0

	filterTerm0 = (newAngle - filterAngle) * timeConstant * timeConstant;
	filterTerm2 += filterTerm0 * dt;
	filterTerm1 = filterTerm2 + ((newAngle - filterAngle) * 2 * timeConstant) + newRate;
	filterAngle = (filterTerm1 * dt) + filterAngle;

	return filterAngle; // This is actually the current angle, but is stored for the next iteration
}
//-------------------------------------
// Main Programme start here.
//-------------------------------------------------------------------------------
float gyro_Rate(void)
{   
  uint16_t x_byte = 0;
  uint8_t x_byte1 = 0,x_byte2 = 0;
  double gy_angle =0,gy_sum=0;
  int16_t x_ang=0;
  int filt_ang=0;
 
	   
	   
	  x_byte1 = read_byte_gyro(XL);
	  //lcd_print(1,1,x_byte1,3);
	   
	   x_byte2 = read_byte_gyro(XH);
	   //lcd_print(2,1,x_byte2,3);
	   
	   x_byte = x_byte2;   // to print 10 bit integer value on LCD
	   x_byte = (x_byte << 8);
	   x_byte |= x_byte1;
	   x_ang = sign(x_byte);
	   x_ang /=100;
	   return x_ang;
}


//--------------------------------------------------------------------------------




