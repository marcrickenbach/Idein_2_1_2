/*	IDEIN 2.1.2
 * 	Author: Marc Rickenbach
 * 	October 14, 2022
 *
 * 	Note: move away from HAL
 *
*/

#include <stdint.h>
#include "stm32l010x4.h"
#include "stm32l0xx.h"
#include "stdbool.h"
#include "main.h"

#include "stdlib.h"

#define ADC_READY 			(1U<<0)
#define LIGHT_RESOLUTION	0
#define RED_THRESHOLD 		adc_value[0]
#define GREEN_THRESHOLD 	adc_value[1]
#define BLUE_THRESHOLD 		adc_value[2]
#define IR_THRESHOLD 		adc_value[3]
#define RED_HIGH 			GPIOA->ODR |= (1U<<5)
#define RED_LOW 			GPIOA->ODR &= ~(1U<<5)
#define GREEN_HIGH 			GPIOA->ODR |= (1U<<6)
#define GREEN_LOW 			GPIOA->ODR &= ~(1U<<6)
#define BLUE_HIGH 			GPIOA->ODR |= (1U<<7)
#define BLUE_LOW 			GPIOA->ODR &= ~(1U<<7)
#define IR_HIGH 			GPIOB->ODR |= (1U<<1)
#define IR_LOW				GPIOB->ODR &= ~(1U<<1)

// Handlers
void NMI_Handler() { }
void HardFault_Handler() { while (1); }
void MemManage_Handler() { while (1); }
void BusFault_Handler() { while (1); }
void UsageFault_Handler() { while (1); }
void SVC_Handler() { }
void DebugMon_Handler() { }
void PendSV_Handler() { }
void SysTick_Handler() {}


// Function Prototypes
void clk_init(void);
void gpio_init(void);
void i2c_init(void);
void adc_init(void);
void tim2_init(void);
void dma_channel1_init(void);
void dma_config(uint32_t src, uint32_t dst, uint32_t len);
void i2c_tx (uint8_t reg, uint8_t *TX_data, uint8_t size);
void i2c_rx (uint8_t reg, uint8_t *RX_data, uint8_t size);
void configure_light_sensor ();
void read_sensor ();
void comparator_circuit ();

uint8_t flag_bank = 0;
uint32_t adc_value[4], adc_buffer[4];
bool MAIN_TEST = 0;
bool MEAS_RATE = 0;
bool GAIN_TEST = 0;

bool red_read =  0;
bool red_write = 0;
bool green_read = 0;
bool green_write = 0;
bool blue_read = 0;
bool blue_write = 0;
bool ir_read = 0;
bool ir_write = 0;

uint8_t TEST = 0;


// CHIP ADDRESSES
const uint16_t RGB_ADDRESS = 0x52;

// CONFIG ADDRESSES
const uint16_t MAIN_CTRL = 0x00;
const uint16_t LS_MEAS_RATE = 0x04;
const uint16_t LS_GAIN = 0x05;

// COLOR REGISTERS
uint8_t RED_REGISTER = 0x13;
uint8_t GREEN_REGISTER = 0xD;
uint8_t BLUE_REGISTER = 0x10;
uint8_t IR_REGISTER = 0xA;

// COMMANDS
uint8_t RGB_CONFIG = 0x06;

// READ DATA
// raw data
uint8_t red_val[3];
uint8_t blue_val[3];
uint8_t green_val[3];
uint8_t ir_val[3];

// compiled data
uint32_t RED_TOTAL = 0;
uint32_t GREEN_TOTAL = 0;
uint32_t BLUE_TOTAL = 0;
uint32_t IR_TOTAL = 0;

uint32_t whileTester = 0;

int main(void) {

	clk_init();
	gpio_init();
	tim2_init();
	adc_init();
	dma_channel1_init();
	i2c_init();
	dma_config((uint32_t)&ADC1->DR, (uint32_t)adc_buffer, 4);	// configure and start the DMA from ADC

	configure_light_sensor ();

	while(1){
		if(flag_bank & ADC_READY) {
			read_sensor();
			comparator_circuit();
			whileTester++;
			whileTester = whileTester % 1000;
			flag_bank &= ~ ADC_READY;
		}

	}

}


void clk_init(void) {	// checked, ok
	RCC->CR 		|= 	(1U<<0);				// Enable HSI_16
	while (!(RCC->CR & 	(1U<<2))){};			// Wait for HSI_RDY Flag, then we can keep going
	RCC->APB1ENR 	|= 	(1U<<28);				// Set the Power Enable CLock
	PWR->CR 		|= 	(1U<<11);				// Set the VOS, Voltage Scaling Range, to Range 1 (01), bits 12:11.
	PWR->CR 		&= 	~(1U<<12);
	RCC->APB2ENR 	|= 	(1U<<0);				// Set SYSCFG Enable
	FLASH->ACR		= 	0x40;					// PreRead is Enabled (bit 6)
	RCC->CFGR 		|= 	(1U<<7);				// AHB, divided by 2[1000]
	RCC->CFGR 		&= 	~(1U<<10);				// APB1 not divided, 0xx bits 10-8
	RCC->CFGR 		&= 	~(1U<<13);				// APB2 not divided, 0xx bits 13-11
	RCC->CFGR 		|= 	(1U<<18);				// Set PLL Multiplication factor to 4 [0001] in bits 21:18
	RCC->CFGR 		|= 	(1U<<22);				// Set PLL Division factor to 2 [01] in bits 23:22
	RCC->CR 		|= 	(1U<<24);				// RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	while (!(RCC->CR & (1U<<25))){};			// Wait for PLL_RDY Flag, then continue on.
	RCC->CFGR 		|= 	(1U<<0);				// Select the PLL as the System Clock (SYSCLK) Source, [11] in bits 1:0
	RCC->CFGR 		|= 	(1U<<1);
	while (!(RCC->CFGR & (3U<<2))){};			// Wait for System Clock Switch Status Flag to read PLL [11] in bits 3:2

	RCC->APB1ENR |= (1U<<21);					// Enable I2C1 Clock

}



void gpio_init(void) {	// checked, ok
	RCC->IOPENR 	|= 	(1U<<0);				// Enable GPIO A and B Clocks
	RCC->IOPENR 	|= 	(1U<<1);
	GPIOA->MODER 	= 	0xebeb54ff;
	GPIOA->OTYPER 	= 	0x600;
	GPIOA->OSPEEDR 	= 	0xc3cfc00;
	GPIOA->PUPDR 	=	0x24000000;
	GPIOB->MODER 	= 	0xfffffff7;
	GPIOB->OTYPER 	= 	0x0;
	GPIOB->OSPEEDR 	= 	0xc;
	GPIOB->PUPDR 	=	0x0;
	GPIOA->AFR[1]	|= (1U<<4);
	GPIOA->AFR[1]	|= (1U<<8);
}



void tim2_init(void){ // checked, ok
	RCC->APB1ENR	|= 	(1U<<0);			// Enable Clock Access to TIM 2
	TIM2->PSC 		= 	100;				// Set Pre-Scaler Value
	TIM2->ARR 		= 	100;				// Set Auto-Reload Value
	TIM2->CR1 		&= 	~(1U<<4);			// Set Direction, 0 is up-counter
	TIM2->CR1 		|= 	(1U<<7);			// Auto-reload preload enable
	TIM2->CNT 		= 	0;					// Clear Counter
	TIM2->CR2 		|= 	(1U<<5);			// Sets Timer to update event on trigger output
	TIM2->CR1 		|= 	(1U<<0);			// Enable Timer
	TIM2->DIER 		|= 	(1U<<0);			// Enable TIM Interrupt
	NVIC_EnableIRQ(TIM2_IRQn);				// Enable TIM Interrupt in NVIC
}


void adc_init() {	//checked, ok, fixed channel select
	RCC->APB2ENR 	|= 	(1U<<9);			//Enable Clock Access to ADC
	ADC1->CHSELR 	= 	0xF; 				// ADC Channel Selection Register: ADC_CHSELR
	ADC1->IER		|= 	(1U<<4);			// ADC Overrun interrupt enable
	ADC1->CR 		|= 	(1U<<0);			// ADC Enable
	ADC1->CFGR1 	= 	0x11483;	 		// ADC CONFIGURATION REGISTER 1: ADC1->CFGR1
	ADC1->CFGR2 	&= 	~(1U<<0);			// Over-sampling Mode Disabled
	ADC1->SMPR 		= 	0x7;				// ADC SAMPLING TIME REGISTER: ADC1->SMPR
	ADC1->CR 		|= 	(1U<<2);			// Start ADC
}


void dma_channel1_init(void) {
	RCC->AHBENR |= (1U<<0);					// Enable Clock Access to the DMA
	DMA1_Channel1->CCR &= ~(1U<<0);			// Disable the DMA Channel 1, CxS 0000 so we can configure it
	while(DMA1_Channel1->CCR & (1U<<0)){}
	DMA1->IFCR |= (1U<<0);					// Clear all interrupt flags for Channel 1, CxS 0000
	DMA1_Channel1->CCR &= ~(1U<<12);		// Set Channel Priority to High
	DMA1_Channel1->CCR |= (1U<<13);
	DMA1_Channel1->CCR &= ~(1U<<4);			// Set Data Transfer Direction, read from Peripheral (ADC)
	DMA1_Channel1->CCR |= (1U<<5);			// Set Circular Mode
	DMA1_Channel1->CCR &= ~(1U<<6);			// Set Peripheral Increment Mode to disabled
	DMA1_Channel1->CCR |= (1U<<7);			// Set Memory Increment Mode to enabled
	DMA1_Channel1->CCR &= ~(1U<<10);		// Set Memory Data Size, 32 bit
	DMA1_Channel1->CCR |= (1U<<11);
	DMA1_Channel1->CCR &= ~(1U<<8);			// Set Peripheral Size to 32 bit
	DMA1_Channel1->CCR |= (1U<<9);
	DMA1_Channel1->CCR |= (1U<<1);			// Enable Interrupted at Full Transfer, TCIE, enabled
	DMA1_Channel1->CCR |= (1U<<2);
	DMA1_Channel1->CCR |= (1U<<3);
}


void i2c_init(void) {
	GPIOA->OTYPER |= (1U<<9) | (1U<<10);	// Open Drain on I2C pins, PA9, PA10
	GPIOA->OSPEEDR |= (3U<<18) | (3U<<20);	// High Speed, Pins PA9, PA10
	GPIOA->PUPDR |= (1U<<18) | (1U<<20);	// Pull-Up on Pins PA9, PA10
	RCC->APB1RSTR |= (1U<<21);				// Resets I2C1
	RCC->APB1RSTR &= ~(1U<<21);				// Takes out of Reset State I2C1
	I2C1->CR1 = 0;							// clear
	I2C1->CR2 = 0;
	I2C1->TIMINGR = 0x0010061A;				// Set Timing Register Properties
	I2C1->OAR1 = 0;
	I2C1->OAR2 = 0;
	I2C1->CR2 |=  (0x52<<1); 				// Set Target address
	I2C1->CR1 &= ~(1U<<12); 				// Analog Filter Enable
	I2C1->CR1 |= (0U<<8);					// Digital Filter Disable
	I2C1->CR1 &= ~(1U<<17);					// Clock stretching disabled
	I2C1->CR2 &= ~(1U<<11);					// ADD10 to 0 for 7-bit, 1 for 10-bit Addressing Mode (pg612)
	SYSCFG->CFGR2 |= (1U<<12);				// Fast Mode is Enabled
	I2C1->CR1 |= (1U<<0);					// SET PE bit in I2C_CR1
}





//////////////////////////////////////////////////////////////////////
//// DMA-ADC FUNCTION
//////////////////////////////////////////////////////////////////////

void dma_config(uint32_t src, uint32_t dst, uint32_t len) {
	DMA1_Channel1->CPAR = src;			// Set source buffer
	DMA1_Channel1->CMAR = dst;			// Set the destination buffer
	DMA1_Channel1->CNDTR = len;			// Set the data length
	DMA1_Channel1->CCR |= (1U<<0);		// Activate the Channel
}

//////////////////////////////////////////////////////////////////////
//// IRQ HANDLER FUNCTIONS
//////////////////////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
	for (int i=0; i<4; i++) {		// Transfer our ADC readings from the DMA buffer to our local variables for use.
		adc_value[i] = (adc_buffer[i] & 0xFFF)>>4;
		if (adc_value[i] >= 253) adc_value[i] = 255;
		if (adc_value[i] <= 20) adc_value[i] = 40;
	}

	flag_bank |= (1U<<0);			// ADC ready flag goes up so we can enter our main loop

	TIM2->SR &= ~(1U<<0);			// Clear the Interrupt Flag
}



//////////////////////////////////////////////////////////////////////
//// IDEIN MAIN FUNCTION
//////////////////////////////////////////////////////////////////////

void comparator_circuit () {

    if (RED_TOTAL > RED_THRESHOLD) {
        RED_HIGH;
    } else {
        RED_LOW;
    }

    if (GREEN_TOTAL > GREEN_THRESHOLD) {
        GREEN_HIGH;
    } else {
        GREEN_LOW;
    }

    if (BLUE_TOTAL > BLUE_THRESHOLD) {
        BLUE_HIGH;
    } else {
        BLUE_LOW;
    }

    if (IR_TOTAL > IR_THRESHOLD) {
        IR_HIGH;
    } else {
        IR_LOW;
    }
}


//////////////////////////////////////////////////////////////////////
//// IDEIN SENSOR FUNCTIONS
//////////////////////////////////////////////////////////////////////

void configure_light_sensor () {
    uint8_t datacheck = 0;

    uint8_t datatowrite_ctrl = 0x06;    // Main Control
    i2c_tx (MAIN_CTRL, &datatowrite_ctrl, 1);
    for (int i=0; i<10000; i++){};
    i2c_rx (MAIN_CTRL, &datacheck, 1);
    if (datatowrite_ctrl == datacheck) {
    	TEST |= (1U<<0);
    }

    uint8_t datatowrite_meas = 0x22;    // Measurement and Rate Parameters
    i2c_tx (LS_MEAS_RATE, &datatowrite_meas, 1);
    for (int i=0; i<10000; i++){};
    i2c_rx (LS_MEAS_RATE, &datacheck, 1);
    if (datatowrite_meas == datacheck) {
    	TEST |= (1U<<1);
    }

    uint8_t datatowrite_gain = 0x02;    // Gain setting
    i2c_tx (LS_GAIN, &datatowrite_gain, 1);
    for (int i=0; i<10000; i++){};
    i2c_rx (LS_GAIN, &datacheck, 1);
    if (datatowrite_gain == datacheck) {
    	TEST |= (1U<<2);
    }

    if (TEST == 7) {
    	RED_HIGH;
    	GREEN_HIGH;
    	BLUE_HIGH;
    	IR_HIGH;
    	for (int i=0; i<500000; i++){};
    	RED_LOW;
    	GREEN_LOW;
    	BLUE_LOW;
    	IR_LOW;
    	for (int i=0; i<500000; i++){};
    	RED_HIGH;
    	GREEN_HIGH;
    	BLUE_HIGH;
    	IR_HIGH;
    	for (int i=0; i<500000; i++){};
    	RED_LOW;
    	GREEN_LOW;
    	BLUE_LOW;
    	IR_LOW;
    	for (int i=0; i<500000; i++){};

    }
}


void read_sensor () {
	i2c_rx (RED_REGISTER, red_val, 3); 	// RED
	RED_TOTAL = (red_val[2] | red_val[1] | red_val[0])<<LIGHT_RESOLUTION;

	i2c_rx (GREEN_REGISTER, green_val, 3);	// GREEN
	GREEN_TOTAL = (green_val[2] | green_val[1] | green_val[0])<<LIGHT_RESOLUTION;

	i2c_rx (BLUE_REGISTER, blue_val, 3);	// BLUE
	BLUE_TOTAL = (blue_val[2] | blue_val[1] | blue_val[0])<<LIGHT_RESOLUTION;

	i2c_rx (IR_REGISTER, ir_val, 3);		// IR
	IR_TOTAL = (ir_val[2] | ir_val[1] | ir_val[0])<<LIGHT_RESOLUTION;
}


//////////////////////////////////////////////////////////////////////
//// I2C READ AND WRITE
//////////////////////////////////////////////////////////////////////

void i2c_tx (uint8_t reg, uint8_t *TX_data, uint8_t size) {

	while(((I2C1->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY)));

	uint8_t *pBufferData = TX_data;
	uint8_t transferSize = size;

	I2C1->CR1 &= ~I2C_CR1_PE; 										// Disable Peripheral
	I2C1->CR2 &= ~(I2C_CR2_NBYTES); 								// Clear Nbytes
	I2C1->CR2 |= ((transferSize+1)<<16);							// Set Nbytes to transfer size and one for register address
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN); 								// Set Write Request
	I2C1->CR1 |= I2C_CR1_PE; 										// Enable Peripheral
	I2C1->CR2 |= I2C_CR2_AUTOEND;									// Enable AutoEnd
	I2C1->CR2 |= (1U<<13);											// Start

	while(!((I2C1->ISR & I2C_ISR_TXIS) == (I2C_ISR_TXIS))); 		// Wait for TX buffer to be empty
	I2C1->TXDR = reg; 												// Send register address

    while(transferSize > 0) {
    	while(!((I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE))); 		// Wait for TX buffer to be empty
    	I2C1->TXDR = *pBufferData; 									// Send bytes to Data Register
    	pBufferData++;
    	transferSize--;
    }

    while(!((I2C1->ISR & I2C_ISR_STOPF) == (I2C_ISR_STOPF)));
    I2C1->ICR |= (1U<<5);

}



void i2c_rx (uint8_t reg, uint8_t *RX_data, uint8_t size) {

	while(((I2C1->ISR & I2C_ISR_BUSY) == (I2C_ISR_BUSY)));

	uint8_t *pBufferData = RX_data;
	uint8_t transferSize = size;

	I2C1->CR1 &= ~I2C_CR1_PE; 										// Disable Peripheral
	I2C1->CR2 &= ~(I2C_CR2_NBYTES); 								// Clear Nbytes
	I2C1->CR2 |= (1U<<16); 											// Set Nbytes to 1 for register address
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN); 								// Set Write Request
	I2C1->CR1 |= I2C_CR1_PE; 										// Enable Peripheral
    I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	I2C1->CR2 |= (1U<<13);											// Start

	while(!((I2C1->ISR & I2C_ISR_TXIS) == (I2C_ISR_TXIS))); 		// Wait for TX buffer to be empty
	I2C1->TXDR = reg; 												// Send register address
	while(!((I2C1->ISR & I2C_ISR_TC) == (I2C_ISR_TC))); 			// Wait for transfer complete

    I2C1->CR2 &= ~(I2C_CR2_NBYTES); 								// Clear Nbytes
    I2C1->CR2 |= (transferSize)<<I2C_CR2_NBYTES_Pos; 				// Set Nbytes to transfer size
    I2C1->CR2 |= (I2C_CR2_RD_WRN);									// Set Read Request

    I2C1->CR2 |= (1U<<13);											// Start

    while (transferSize > 0) {

    	while(!((I2C1->ISR & I2C_ISR_RXNE) == (I2C_ISR_RXNE)));

    	*pBufferData = (uint8_t)I2C1->RXDR;
    	pBufferData++;
    	transferSize--;
    	I2C1->CR2 |= I2C_CR2_AUTOEND;
    }

    I2C1->CR2 &= ~I2C_CR2_AUTOEND;
    I2C1->CR2 |= (1U<<14);
    while(!((I2C1->ISR & I2C_ISR_STOPF) == (I2C_ISR_STOPF)));
    I2C1->ICR |= (1U<<5);
}

