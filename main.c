#include "stm32f0xx.h"                  // Device header

// function prototypes
void I2C_1_Start(void);
void I2C_1_Write(unsigned char DATA);
void delay (int ms);
void LCD_Write(unsigned char DATA, unsigned char command);
void ADC_EN(void);

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

int speed, throttle = 0;

//main loop
int main(void){
	
	delay(400); //wait 15ms
	
	// AF4 for I2C1
	// SDA PA10
	// SCL PA9
	
	// enable port a clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// enable i2c clock (PE BIT?)
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	//ADC
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable the ADC
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{
	 /* For robust implementation, add here time-out management */
	}
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	
	ADC1->CHSELR = ADC_CHSELR_CHSEL2;
	ADC->CCR |= ADC_CCR_VREFEN;
	
	ADC_EN();
	
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

LCD_Write(0x33,1);
LCD_Write(0x32,1);
//LCD_Write(0x28,1); // This seems to screw things up
LCD_Write(0x01,1);
LCD_Write(0x0C,1);
LCD_Write(0x06,1);
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
		delay(1000);
		LCD_Write(0x10, 0x1);
		
		ADC1->CR |= ADC_CR_ADSTART;
		
		while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* Wait end of conversion */
		{
 /* For robust implementation, add here time-out management */
		}
		throttle = ADC1->DR;
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
 delay(3);

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

// 1ms delay
void delay (int ms) 				//create simple delay loop
{
	int d;
	for (d=0; d<(ms*1000); d++) {
		__asm("NOP");
	}
}

void ADC_EN(void) {
if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
{
 ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
}
ADC1->CR |= ADC_CR_ADEN; /* (3) */
while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
{
 /* For robust implementation, add here time-out management */
}
	
}