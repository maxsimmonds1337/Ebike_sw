#include "stm32f0xx.h"                  // Device header
#include <string.h>

// function prototypes
void I2C_1_Start(void);
void I2C_1_Write(unsigned char DATA);
void delay (int ms);
void delay_us(int us);
void LCD_write_str(char *string);
void LCD_Write(unsigned char DATA, unsigned char command);
void ADC_EN(void);

const float K_proportional = 1.0;
const float K_integral = 1;

const int hall_counts_per_rot = 23; // number of counts per one wheel rotation
const float wheel_diameter = 1;//  0.662; //meters
const float pi = 3.1412; // pi
const float cir = pi * wheel_diameter; // circumfrenece of the wheel
const float mps_mph = 2.23694; // m/s to m/h conversion

//define some vars
volatile unsigned char LED_ON; // this is used to toggle the LED
volatile unsigned int HAL_count, HAL_count_frozen, timer_count = 0; // used for counting

int Vin, Vin_largest,adc_reg = 0; // this reads the input voltage

// bldc io

int dir, nSLEEP, nFAULT, nBRAKE = 0;

// used for delay
int a,D = 0;

double speed, speed_largest,speed_constant = 0.0; // var to hold the speed

#define throttle_scale 0.80586  // 12 bit adc, 3v3 is full scale

struct IO_Expander {
	
	unsigned char E:1;
	unsigned char RS:1;
	unsigned char DB:4;
	unsigned char LED:1;
	unsigned char NC:1; // not used
	
} port;

//slave address of IO expander
char slave_add = 0x20;

char display_str[16];
static const struct IO_Expander IO_Clear;

float throttle = 0.0;

int digits[4]; // this will store the digits
int ADC_Result[2];
volatile int duty_cycle, old_duty_cycle = 0;
volatile float errorIntegral = 0.0;


//main loop
int main(void){
	
	volatile int i, val = 0;
	
	delay(400); //wait 15ms
	
	// AF4 for I2C1
	// SDA PA10
	// SCL PA9
	
	/***** I2C INIT *********/
	// enable port a clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
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
	
	/********* ADC INIT *******/
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable the ADC
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{
	 /* For robust implementation, add here time-out management */
	}
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	
	ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL2;
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (3) */

	
	ADC->CCR |= ADC_CCR_VREFEN;
	ADC_EN();

	
	/***** LCD INIT **********/
	LCD_Write(0x28,1); // ensure you're using 5v if using two lines displays
	LCD_Write(0x01,1);
	LCD_Write(0x0C,1);
	LCD_Write(0x06,1);
	
	/**** TIM3 INIT ******/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable the timer
	TIM3->PSC = 1000; 
	TIM3->ARR = 100; //125ms delay
	TIM3->DIER |= TIM_DIER_UIE; //enable timer interrupts
	NVIC_EnableIRQ(TIM3_IRQn); // enable the IRQ
	TIM3->CR1 |= TIM_CR1_CEN; //enable the counter
	
	/**** TIM6 INIT ******/
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; // enable the timer
	TIM14->PSC = 24; 
	TIM14->DIER |= TIM_DIER_UIE; //enable timer interrupts
	NVIC_EnableIRQ(TIM14_IRQn); // enable the IRQ
	TIM14->CR1 |= TIM_CR1_CEN; //enable the counter
	
	/****** INT INIT ********/
	//EXTI is on syscfg clk, so needs to be enabled
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable port b
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;	//link line 0 with Port B (so we're using PB0 as the interrupt)
	
	//Configure the interupts
	EXTI->IMR |= EXTI_IMR_MR0; // set line 0 to be enabled
	EXTI->RTSR	|= EXTI_RTSR_TR0; // set it to trigger on a rising edge
	
	NVIC_SetPriority(EXTI0_1_IRQn, 0x03);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	
	
	/**** TIM 1 INIT********/
	//TIM1_CH4 is the pwm
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // enable the clock
	//TIM1->PSC |= 0x1; // set the prescaler to 1, this gives 125ns ticks
	TIM1->ARR = 400; // this gives an interrupt frequency of 20khz
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0; // write 110 to the output compare mode, setting pwm1
	TIM1->CCER |= TIM_CCER_CC4E; // enable channel 4
	
	GPIOA->MODER	|= GPIO_MODER_MODER11_1;
	GPIOA->AFR[1] |= (0x02) << GPIO_AFRH_AFSEL11_Pos; // set port a 11 to alternate function 2 (tim1_ch4)
	
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;

	strcpy(display_str, "Speed ");
	LCD_write_str(display_str);
	
	LCD_Write(0xC0, 0x1); // set the ddram address to row 2 col 1

	strcpy(display_str, "Throttle = ");
	LCD_write_str(display_str);
	
	/****** set up the bldc io *******/
	
	
	// outputs
	
	dir = 0x0; // 1 is backwards, 0 is forwards
	nSLEEP = 0x1; // 0 is sleep
	
	//dir - pa0
	GPIOA->MODER |= GPIO_MODER_MODER15_0; // set pa1 as an output
	GPIOA->ODR |= (dir << GPIO_ODR_15); // set the direction of the motor
	
	//sleep - pa12
	GPIOA->MODER |= GPIO_MODER_MODER12_0; // set pa1 as an output
	GPIOA->ODR |= (nSLEEP << GPIO_ODR_12); // 
	
	//brake - pa3
	GPIOA->MODER |= GPIO_MODER_MODER3_0; // set pa1 as an output
	GPIOA->ODR |= (nBRAKE << GPIO_ODR_3); // 
	
	// inputs
	//nFAULT
	
	while(1){
		
		delay(10);
	
		LCD_Write(0x86, 0x1); // set the ddram address to row 2 col 1
		speed = ((HAL_count_frozen * cir/hall_counts_per_rot) / 0.5) * mps_mph; // calculate the speed, in mph
	
		// TODO make this a function
		for(i = 0; i < 2; i++) {
			digits[i] = (int)speed % 10;
			speed /= 10;
		}
		
		LCD_Write('0' + digits[1], 0x0);
		LCD_Write('0' + digits[0], 0x0);
		
		strcpy(display_str, " MPH");
		LCD_write_str(display_str);
		
		LCD_Write(0xC0+0xB, 0x1); // set the ddram address to row 2 col 1
		throttle /= 4;
		for(i = 0; i < 3; i++) {
			digits[i] = (int)throttle % 10;
			throttle /= 10;
		}
		
		LCD_Write('0' + digits[2], 0x0);
		LCD_Write('0' + digits[1], 0x0);
		LCD_Write('0' + digits[0], 0x0);
		LCD_Write('%', 0x0);
		
		LCD_Write(0x2, 0x1);
//		
		
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
 delay(4);

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

//print a string function
void LCD_write_str(char *string) {
	
	int i = 0; // used for looping
	
	for(i = 0; i < strlen(display_str); i++) {
		LCD_Write(*(string+i), 0x0);
	}
		
	
}

// 1ms delay
void delay (int ms) 				//create simple delay loop
{
	int d;
	for (d=0; d<(ms*1000); d++) {
		__asm("NOP");
	}
}

// 1ms delay
void delay_us (int us) 				//create simple delay loop
{
	int d;
	for (d=0; d<(us); d++) {
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

//interrupt service routine for the timer
int TIM3_IRQHandler(void) {

	int i = 0;
  
	TIM3->SR = 0;		// clear the status reg
	// start ADC acq.
	
	for (i=0; i < 2; i++) {
			ADC1->CR |= ADC_CR_ADSTART;

		while ((ADC1->ISR & ADC_ISR_EOC) == 0){ /* Wait end of conversion */
			/* For robust implementation, add here time-out management */
		}
		
		ADC_Result[i] = ADC1->DR; /* Store the ADC conversion result */
	
	}
	
	//while ((ADC1->ISR & ADC_ISR_EOC) == 0); // Wait end of conversion
	
	// read the ADC DR and return
	throttle = ADC1->DR;
	throttle /= (4095.0);
	throttle *= 400.0;
	
	duty_cycle = (unsigned int)(throttle); // set duty cycle
	
	if( (duty_cycle - old_duty_cycle) > 1 ) {
		duty_cycle = old_duty_cycle + 1;
	}

	if( (duty_cycle - old_duty_cycle) < -1 ) {
		duty_cycle = old_duty_cycle - 1;
	}

	TIM1->CCR4 = 405 - (int)duty_cycle;
	old_duty_cycle = (int)duty_cycle;
	
	return throttle;
}

//interrupt service routine for the timer
int TIM14_IRQHandler(void) {
  TIM14->SR = 0;		// clear the status reg
	
	HAL_count_frozen = HAL_count; // freeze the counter
	HAL_count = 0;// reset the counter
	
	return HAL_count_frozen; // return the value for processing outside the IRQ
}

//interrupt service routine, for counting pulses
void EXTI0_1_IRQHandler(void) {
	if(EXTI->PR & (GPIOB->IDR) ) {
		//clear the progress reg
		EXTI->PR = 0x1;
		
		//LED_ON = !LED_ON; // toggle the led
		
		HAL_count++; // inc the counter
	}
}

//throttle *= throttle_scale;
//		
