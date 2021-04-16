#include "stm32f0xx.h"                  // Device header
#include "ADC.h"

// enable the ADC
void ADC_init(void) {
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable the ADC
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{
	 /* For robust implementation, add here time-out management */
	}
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	ADC1->CHSELR = ADC_CHSELR_CHSEL2;
	ADC->CCR |= ADC_CCR_VREFEN;
	
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

// read the ADC
int ADC_read(void) {
	
	// return the adc value
	return ADC1->DR;
	
}
