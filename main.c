#include "stm32f0xx.h"                  // Device header
#include<stdio.h>

// function prototypes
void I2C_1_Start(void);
void I2C_1_Write(unsigned char DATA);
void delay (void);
void LCD_Write(unsigned char DATA, unsigned char command);

//#define E   1<<0
//#define RS  1<<1
//#define DB4 1<<2
//#define DB5 1<<3
//#define DB6 1<<4
//#define DB7 1<<5

struct IO_Expander {
	
	unsigned char E:1;
	unsigned char RS:1;
	unsigned char DB:4;
	unsigned char LED:1;
	unsigned char NC:1; // not used
	
} port;

//	int sw1 = 0;

//slave address of IO expander
char slave_add = 0x20;
static const struct IO_Expander IO_Clear;

int speed = 0;

//main loop
int main(void){
	
	delay();
	delay();
	delay();
	delay();

	
	// AF4 for I2C1
	// SDA PA10
	// SCL PA9
	
	// enable port a clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// enable i2c clock (PE BIT?)
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	// select the alternate function reg for both ios (use the AF reg later to choose i2c)
	GPIOA->MODER	|= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1; // mask bit 1, to set into AF mode

	// set output type to open drain
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	
	// set gpio alternate function reg
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL9_Pos; // select AF4 (i2c) on pa9
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL10_Pos; // select AF4 (i2c) on pa10
	
	// i2c configuration, set up using table from table 74 in referene manual
	
	I2C1->CR1 		&= ~(I2C_CR1_PE); // turn off the peripheral
	I2C1->CR1 		|= I2C_CR1_RXIE; // receive interrupt enable
	I2C1->CR2 		|= I2C_CR2_AUTOEND; // set the auto end mode so we don't need to send the stop bit
	I2C1->TIMINGR |= ((0x01) << I2C_TIMINGR_PRESC_Pos) | ((0xC7) << I2C_TIMINGR_SCLL_Pos) | ((0xC3) << I2C_TIMINGR_SCLH_Pos) | ((0x2) << I2C_TIMINGR_SDADEL_Pos) | ((0x4) << I2C_TIMINGR_SCLDEL_Pos);
	I2C1->CR2 		|= slave_add << I2C_CR2_SADD_Pos << 1; // write the slave address
	I2C1->CR1 		|= I2C_CR1_PE; // turn on the peripheral

	LCD_Write(0xFF,0x1);
	LCD_Write(0x0,0x1);
  LCD_Write(0x3,0x1);
	LCD_Write(0x3,0x1);
	LCD_Write(0x3,0x1);
	LCD_Write(0x2,0x1);
	
	//LCD_Write(0x28,0x1);
	//LCD_Write(0x0C,0x1);
	//LCD_Write(0x06,0x1);
	//LCD_Write(0x80,0x1);
	
	//LCD_Write(0x1, 0x1);
	
//  LCD_Write(0x20,0x0);
//  LCD_Write(0x00,0x0);
//	LCD_Write(0x40,0x0);
//	LCD_Write(0x00,0x0);
//	LCD_Write(0x80,0x0);
//	LCD_Write(0x00,0x0);

//I2C_1_Write(0);
//I2C_1_Write(1<<2);
//I2C_1_Write(1<<3);
//I2C_1_Write(1<<4);
//I2C_1_Write(1<<5);
//I2C_1_Write(0);
//LCD_Write(0x28,1);
//LCD_Write(0x0E,1);
//LCD_Write(0x2, 1);
//LCD_Write(0x6, 1);
//LCD_Write(0x1, 1);

LCD_Write('S', 0x0);
LCD_Write('P', 0x0);
LCD_Write('E', 0x0);
LCD_Write('E', 0x0);
LCD_Write('D', 0x0);
LCD_Write(' ', 0x0);



	while(1){

	  if(speed == 9){
			speed = 0;
		} else {
			speed++;
		}
	  
		LCD_Write('0' + speed, 0x0); // turn off LED
		delay();
		LCD_Write(0x10, 0x1);

		//LCD_Write('r', 0x0);
		//LCD_Write('e', 0x0);
		
		//LCD_Write('Y', 0x0);
		//LCD_Write('o', 0x0);
		//LCD_Write('u', 0x0);
		
		//LCD_Write('O', 0x0);
		//LCD_Write('K', 0x0);
		//LCD_Write('?', 0x0);
		//delay();
		//delay();
		//delay();
		//delay();
		//delay();
		//delay();
		//LCD_Write(0x1, 0x1);
//		LCD_Write('j', 0x0); // turn off LED
//		LCD_Write('a', 0x0); // turn off LED
		
	}
}

// sends the start bit
void I2C_1_Start(void) {
	
	I2C1->CR2 |= I2C_CR2_START; // send start bit
  while(!(I2C1->ISR & ~(I2C_ISR_TC_Msk)));// //wait for start bit, to go low, once the start bit and slave address is sent
	
}

// write data to the 
void I2C_1_Write(unsigned char DATA) {
	
 I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos; // send only one byte
 I2C1->TXDR = DATA; // send the data
 I2C1->CR2 |= I2C_CR2_START; // send start bit
 delay();

}

// write to the lcd
void LCD_Write(unsigned char DATA, unsigned char command) {

	unsigned char byte = 0;
	
	// sending a command so RS is low
	if(command == 0x1){
		
		port.RS = 0;
		
	} else {
		
		port.RS = 1;
		
	}
	
	// send the higher nibble
	port.DB = DATA >> 4; // write to the IO expander
	
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte); // send to i2c

	port.E = 0x1; // assert the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte);

	port.E = 0x0; // remove the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte);
	
	
	// send the lower nibble
	port.DB = DATA; // write to the IO expander
	
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte); // send to i2c

	port.E = 0x1; // assert the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte);

	port.E = 0x0; // remove the E line
	byte = port.E << 0 | port.RS << 1 | port.DB << 2 | port.LED << 6 | port.NC << 7;
	I2C_1_Write(byte);
	
}

void delay (void) 				//create simple delay loop
{
	int d;
	for (d=0; d<20000; d++) {
		__asm("NOP");
	}
}
