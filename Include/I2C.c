 #include "I2C.h"
 #include "stm32f0xx.h"                  // Device header
 
 /* I2C 1 INIT Function
 
 Initialise the i2c 1, this is using pa9 and 10
 this does the following:
 
 * Enables the gpio A clock
 * Enables the i2c clock
 * Selects the alternate function for ports pa9 and pa10
 * Selects alternate function 4, i2c mode, for ports pa9 and pa10
 * Sets the outputs of pa9 and pa 10 to open drain, required for i2c
 * Turns off the i2c
 * sets the recieve interupt, auto end mode (so a stop bit does not need to be manually sent)
 * Sets the speed, as per table 74 in the reference manual
 * Write the slave address, defined in the header file
 * Enable the peripheral 
 
 */
 
//slave address of IO expander
char slave_add = 0x20;
struct IO_Expander port;
 
void I2C_1_init(void) {

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable port a clock
	// enable i2c clock (PE BIT?)
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// select the alternate function reg for both ios (use the AF reg later to choose i2c)
	GPIOA->MODER	|= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1; // mask bit 1, to set into AF mode
	// set gpio alternate function reg
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL9_Pos; // select AF4 (i2c) on pa9
	GPIOA->AFR[1] |= (0x04) << GPIO_AFRH_AFSEL10_Pos; // select AF4 (i2c) on pa10
		// set output type to open drain
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
	// i2c configuration, set up using table from table 74 in referene manual
	I2C1->CR1 		&= ~(I2C_CR1_PE); // turn off the peripheral
	I2C1->CR1 		|= I2C_CR1_RXIE; // receive interrupt enable
	I2C1->CR2 		|= I2C_CR2_AUTOEND; // set the auto end mode so we don't need to send the stop bit
	I2C1->TIMINGR |= ((0x01) << I2C_TIMINGR_PRESC_Pos) | ((0xC7) << I2C_TIMINGR_SCLL_Pos) | ((0xC3) << I2C_TIMINGR_SCLH_Pos) | ((0x2) << I2C_TIMINGR_SDADEL_Pos) | ((0x4) << I2C_TIMINGR_SCLDEL_Pos);
	I2C1->CR2 		|= slave_add << I2C_CR2_SADD_Pos << 1; // write the slave address
	I2C1->CR1 		|= I2C_CR1_PE; // turn on the peripheral
	
}

// sends the start bit
void I2C_1_start(void) {
	
	I2C1->CR2 |= I2C_CR2_START; // send start bit
  while(!(I2C1->ISR & ~(I2C_ISR_TC_Msk)));// //wait for start bit, to go low, once the start bit and slave address is sent
	
}

// write data to the 
void I2C_1_write(unsigned char DATA) {
	
 I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos; // send only one byte
 I2C1->TXDR = DATA; // send the data
 I2C1->CR2 |= I2C_CR2_START; // send start bit
 delay(4);

}
