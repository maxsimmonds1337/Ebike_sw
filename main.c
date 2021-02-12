#include "stm32f0xx.h"                  // Device header

// function prototypes
void I2C_1_Start(void);
void I2C_1_Write(unsigned char DATA);
void delay (void);


//	int sw1 = 0;

//slave address of IO expander
char slave_add = 0x40;

int var = 0;

//main loop
int main(void){
	
	// AF4 for I2C1
	// SDA PA10
	// SCL PA9
	
	// enable port a clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// enable i2c clock (PE BIT?)
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	// select the alternate function reg for both ios (use the AF reg later to choose i2c)
	//GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10)) | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->MODER	|= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;//GPIO_MODER_MODER9_1 << GPIO_MODER_MODER9_Pos; // mask bit 1, to set into AF mode
	//GPIOA->MODER	|= GPIO_MODER_MODER10_1 << GPIO_MODER_MODER10_Pos; // mask bit 1, to set into AF mode 

	// set output type to open drain
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	
	// set gpio alternate function reg
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL9_Pos; // select AF4 (i2c) on pa9
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL10_Pos; // select AF4 (i2c) on pa10
	
	// i2c configuration, set up using table from table 74 in referene manual
	
	I2C1->CR1 &= ~(I2C_CR1_PE); // turn off the peripheral
	I2C1->CR1 |= I2C_CR1_RXIE; // receive interrupt enable
	I2C1->CR2 |= I2C_CR2_AUTOEND; // set the auto end mode so we don't need to send the stop bit
	I2C1->TIMINGR |= 0x00201D2B; // taken from the excel spreadsheet
	//((0x01) << I2C_TIMINGR_PRESC_Pos) | ((0xC7) << I2C_TIMINGR_SCLL_Pos) | ((0xC3) << I2C_TIMINGR_SCLH_Pos) | ((0x2) << I2C_TIMINGR_SDADEL_Pos) | ((0x4) << I2C_TIMINGR_SCLDEL_Pos);
	I2C1->CR2 |= slave_add << I2C_CR2_SADD_Pos; // write the slave address
	I2C1->CR1 |= I2C_CR1_PE; // turn on the peripheral
	
	//I2C1->TIMINGR = (uint32_t)0x00B01A4B; /* (1) */
	//I2C1->CR1 = I2C_CR1_PE; /* (2) */
	//I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (slave_add << 1);

//	//mode switch 1 is on PB7
//	//enable the GPIO PORT D CLK
//	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//	
//	//Configure the port B pin 7 as an input
//	GPIOB->MODER	|= GPIO_MODER_MODER7;
		
	while(1){
	  
		delay();
		var = GPIOA->MODER;
		//sw1 = GPIOB->IDR; // mask the input data reg
		//I2C_1_Start(); // send the start pulse
		I2C_1_Write('A'); // send the letter A
		
	}
}

// sends the start bit
void I2C_1_Start(void) {
	
	I2C1->CR2 |= I2C_CR2_START; // send start bit
  while(!(I2C1->ISR & ~(I2C_ISR_TC_Msk)));// //wait for start bit, to go low, once the start bit and slave address is sent
	
}

// write data to the 
void I2C_1_Write(unsigned char DATA) {

// I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos; // send only one byte
// I2C1->TXDR = 0x1; // send the data
// I2C1->CR2 |= I2C_CR2_START; // send start bit
	
	if ((I2C1->ISR & I2C_ISR_TXE) == I2C_ISR_TXE) {
		I2C1->TXDR = 0xA; /* Byte to send */
		I2C1->CR2 |= I2C_CR2_START; /* Go */
	}

}

void delay (void) 				//create simple delay loop
{
	int d;
	for (d=0; d<100; d++) {
		__asm("NOP");
	}
}
