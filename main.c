#include "stm32f4xx.h"
#include "stdbool.h"

#define GPIOAEN		(1UL<<0)
#define UART2EN		(1UL<<17)
#define SYS_FREQ	16000000
#define APB1_CLK	SYS_FREQ
#define USART2_BAUD	115200

void ADC1_Initialise (void);
void ADC1_Start_Conversion(void);
uint32_t ADC_Read(void);

void uart2_txrx_init(void);
static void uart_set_Baudrate(USART_TypeDef *USARTx, uint32_t Periph_Clk, uint32_t Baudrate);
static uint16_t compute_usartdiv(uint32_t Periph_Clk, uint32_t Baudrate);
void uart2_write(int ch);
void uart2_writeInt(uint16_t integer);

void spi1_init(void);
void timer4_enocoder_mode_init(void);
void timer1_centrealigned_mode_init(void);

uint16_t Calc_APP_Torque_Request(uint16_t MeasuredAnalogueCounts, uint16_t MaxCounts, uint16_t MinCounts);
void Calc_CaptureCompareRegisterUVW(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR, uint16_t TorqueRequestPercent, uint16_t positionFeedbackMechanical);
void Temp_Calc_CaptureCompareRegisterUVW(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR, uint16_t TorqueRequestPercent, uint16_t positionFeedbackMechanical, uint16_t* positionFeedbackElectrical, uint16_t* appliedVectorAngleRevolution, uint16_t* appliedVectorAngleSector);
void Update_CaptureCompare(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR);
bool MotorSynchronise();


uint32_t uCTemp = 0x0000;
uint16_t APP_1 = 0x0000;
uint16_t APP_2 = 0x0000;
uint8_t uartcharacter = 0x00;
uint16_t spicharacter = 0x3FA6;
uint16_t encoderFeedback = 0x0000;
uint16_t timer1_cnt = 0x0000;
uint16_t APP_TorquePercent = 0x0000;
uint16_t uCompareRegister;
uint16_t vCompareRegister;
uint16_t wCompareRegister;
uint16_t positionFeedbackElectrical;
uint16_t appliedVectorAngleRevolution;
uint16_t appliedVectorAngleSector;
bool bsynchronised = false;



int main(void)
{
 	ADC1_Initialise();
	ADC1_Start_Conversion();
	uart2_txrx_init();
	//spi1_init();
	timer4_enocoder_mode_init();
	timer1_centrealigned_mode_init();

	while (1)
	{
		bsynchronised = MotorSynchronise();
		encoderFeedback = TIM4->CNT;
		while(bsynchronised)
		{
			ADC1_Start_Conversion();
			APP_1 = ADC_Read();
			APP_TorquePercent = Calc_APP_Torque_Request(APP_1, 1900, 500);

			//Here will be where we get encoder position and calculate where to apply a new vector.
			//I am using simulated encoder feedback from uart to determine which way the motor will spin initially.

			encoderFeedback = TIM4->CNT;
			//Calc_CaptureCompareRegisterUVW(&uCompareRegister, &vCompareRegister, &wCompareRegister, APP_TorquePercent, encoderFeedback);
			Temp_Calc_CaptureCompareRegisterUVW(&uCompareRegister, &vCompareRegister, &wCompareRegister, APP_TorquePercent, encoderFeedback, &positionFeedbackElectrical, &appliedVectorAngleRevolution, &appliedVectorAngleSector);
			Update_CaptureCompare(&uCompareRegister, &vCompareRegister, &wCompareRegister);

			//encoderFeedback = (uint16_t)uartcharacter;

			//uart2_write("e");
			uart2_writeInt(encoderFeedback);
			//uart2_write(" ");

			//uart2_write("a");
			uart2_writeInt(APP_TorquePercent);
			//uart2_write(" ");

			uart2_write(0x75);
			uart2_writeInt(uCompareRegister);
			//uart2_write(" ");

			uart2_write(0x76);
			uart2_writeInt(vCompareRegister);
			//uart2_write(" ");

			uart2_write(0x77);
			uart2_writeInt(wCompareRegister);
			//uart2_write(" ");

			uart2_write(0x0D);
			uart2_write(0x0A);
		}
	}
}

void ADC1_Initialise (void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0;

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Pin PC1 = ADC_IN11 (11th channel)
	ADC1->SQR3 |= ADC_SQR3_SQ1_3 | ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_0; // Set 11th channel as 1st conversion
	ADC1->SQR1 &= 0x0;
	ADC1->SQR1 |= 0x00 << ADC_SQR1_L_Pos; // Set the number of conversions to 1

	// Set sampling time greater than sampling time. 10us at 8MHz is 80 cycles.
	ADC1->SMPR1 |= ADC_SMPR1_SMP18_2;
	ADC1->SMPR1 |= ADC_SMPR1_SMP10_2;
	ADC1->SMPR1 |= ADC_SMPR1_SMP11_2;

	ADC1->CR2 |= ADC_CR2_ADON;	//Set ADON bit in ADC_CR2 Register
}

void ADC1_Start_Conversion(void)
{
	// Start the ADC conversion by setting the SWSTART bit (or by external trigger)
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint32_t ADC_Read(void)
{
	while(!(ADC1->SR & ADC_SR_EOC)){}
	return (ADC1->DR);
}

void uart2_txrx_init(void)
{
	/******** Configure UART GPIO pins ***********/

	// Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;

	// Set PA2 to alternate function mode
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	// Set PA3 to alternate function mode
	GPIOA->MODER &= ~(1U<<6);
	GPIOA->MODER |= (1U<<7);

	// Set PA2 to alternate function type UART_TX
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	// Set PA3 to alternate function type UART_RX
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &= ~(1U<<15);

	/************ Configure UART Module ************/

	// Enable Clock Access to UART2
	RCC->APB1ENR |= UART2EN;

	// Configure Baud rate
	uart_set_Baudrate(USART2, APB1_CLK, USART2_BAUD);

	// Configure the transfer direction
	USART2->CR1 |= (1U<<3);
	USART2->CR1 |= (1U<<2);

	// Configure Receive Interrupt
	USART2->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART2_IRQn);

	// Enable UART module
	USART2->CR1 |= (1U<<13);
}

static void uart_set_Baudrate(USART_TypeDef *USARTx, uint32_t Periph_Clk, uint32_t Baudrate)
{
	USARTx->BRR = compute_usartdiv(Periph_Clk, Baudrate);
}

static uint16_t compute_usartdiv(uint32_t Periph_Clk, uint32_t Baudrate)
{
	return ((Periph_Clk<<4)/16/Baudrate);
}

void uart2_write(int ch)
{
	while(!(USART2->SR & (1U<<7))){}

	USART2->DR = (ch & 0xFF);
}
void uart2_writeInt(uint16_t integer)
{
	while(!(USART2->SR & (1U<<7))){}
	USART2->DR = (integer>>8 & 0xFF);
	while(!(USART2->SR & (1U<<7))){}
	USART2->DR = (integer & 0xFF);
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & 1U<<5)
	{
		uartcharacter =  USART2->DR;
		uart2_write(uartcharacter+0x05);

		//Enable
		SPI1->CR1 |= SPI_CR1_SPE;
		GPIOA->BSRR |= GPIO_BSRR_BR4;
		SPI1->DR = spicharacter;
		while(!(SPI1->SR & SPI_SR_TXE_Msk)){}
		while(SPI1->SR & SPI_SR_BSY_Msk){}
		GPIOA->BSRR |= GPIO_BSRR_BS4;
		SPI1->CR1 &= ~SPI_CR1_SPE;

	}
}

void spi1_init(void)
{
	//SPI1_CS(NSS) 	-> PA4 COnfigured as GPIO, not NSS.
	//SPI1_SCK 		-> PA5
	//SPI1_MISO 	-> PA6
	//SPI1_MOSI 	-> PA7

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//Pins as mode alternate function. Except pin 4, which is output.
	GPIOA->MODER &= ~GPIO_MODER_MODER4_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER4_0;

	GPIOA->OTYPER &= ~GPIO_OTYPER_OT4_Msk; // Push/Pull
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk; // Disable pull up or down resistors
	GPIOA->BSRR |= GPIO_BSRR_BS4; // Set the PA4 bit (Chip Select)

	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;

	//Alternate function set to 5 (AF5) 0101
	//GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
	//GPIOA->AFR[0] |= GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2;

	//Enable SPI1 on the APB2 bus
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	//Configure baud rate as 4MHz (fpclk/4). AS5147U can operate up to 10MHz.
	SPI1->CR1 &= ~SPI_CR1_BR_Msk;
	SPI1->CR1 |= SPI_CR1_BR_0;

	//Set CPOL=0 and CPHA=1
	SPI1->CR1 &= ~SPI_CR1_CPOL_Msk;
	SPI1->CR1 |= SPI_CR1_CPHA;

	// Set unidirectional mode, i.e. full duplex
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE_Msk;

	//MSB first
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST_Msk;

	//Master configuration
	SPI1->CR1 |= SPI_CR1_MSTR;

	//16bit data frames
	SPI1->CR1 |= SPI_CR1_DFF;

	//Software slave select
	SPI1->CR1 |= SPI_CR1_SSM_Msk;
	//SPI1->CR1 &= ~SPI_CR1_SSI_Msk;

	SPI1->CR2 |= SPI_CR2_SSOE_Msk;
}

void timer4_enocoder_mode_init(void)
{
	//Set PB6 and PB7 to alternate mode. Alternate function 2.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER &= ~GPIO_MODER_MODER6_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->MODER &= ~GPIO_MODER_MODER7_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;

	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_1;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_1;

	//Enable CLock to Timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	//Define the timer as input
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM4->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;


	//Count on TI2 edge only
	TIM4->SMCR &= ~TIM_SMCR_SMS_Msk;
	TIM4->SMCR |= TIM_SMCR_SMS_0;

	TIM4->ARR = 0x07FF;

	TIM4->CR1 |= TIM_CR1_CEN;
}

void timer1_centrealigned_mode_init(void)
{
	//This is to configure the timer for providing the switching signals to the fets.
	//A Space Vector PWM methodology is implemented.
	//CH1	PA8 	AF1
	//CH1N	PA7 	AF1
	//CH2	PA9 	AF1
	//CH2N	PB0		AF1
	//CH3	PA10 	AF1
	//CH3N	PB1		AF1

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOB->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER0_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER10_Msk;
	GPIOA->MODER |= GPIO_MODER_MODER10_1;
	GPIOB->MODER &= ~GPIO_MODER_MODER1_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER1_1;

	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_0;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_0;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9_0;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL0_0;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL10_0;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL1_0;


	//
	//Enable CLock to Timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	// Centre aligned mode with interrupt flags for both up and down comparisons.
	TIM1->CR1 |= (TIM_CR1_CMS_1 | TIM_CR1_CMS_0);

	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S_Msk; // OC1 output mode
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;		//Output compare preload enable
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; //OC1REF is active when count matched the compare register
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
	TIM1->CCMR1 &= ~TIM_CCMR1_CC2S_Msk; // OC2 output mode
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;		//Output compare preload enable
	TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk; //OC2REF is active when count matched the compare register
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
	TIM1->CCMR2 &= ~TIM_CCMR2_CC3S_Msk; // OC3 output mode
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;		//Output compare preload enable
	TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk; //OC3REF is active when count matched the compare register
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;

	TIM1->CCER |= TIM_CCER_CC1E; //Capture/compare output enable
	TIM1->CCER &= ~TIM_CCER_CC1P; //OC1 Active High polarity
	TIM1->CCER |= TIM_CCER_CC1NE; //Capture/compare output complementary enable
	TIM1->CCER &= ~TIM_CCER_CC1NP; //OC1 complementary Active High polarity
	TIM1->CCER |= TIM_CCER_CC2E; //Capture/compare output enable
	TIM1->CCER &= ~TIM_CCER_CC2P; //OC2 Active High polarity
	TIM1->CCER |= TIM_CCER_CC2NE; //Capture/compare output complementary enable
	TIM1->CCER &= ~TIM_CCER_CC2NP; //OC2 complementary Active High polarity
	TIM1->CCER |= TIM_CCER_CC3E; //Capture/compare output enable
	TIM1->CCER &= ~TIM_CCER_CC3P; //OC3 Active High polarity
	TIM1->CCER |= TIM_CCER_CC3NE; //Capture/compare output complementary enable
	TIM1->CCER &= ~TIM_CCER_CC3NP; //OC3 complementary Active High polarity

	TIM1->ARR = 0x0320; //Auto Reload value 800

	TIM1->CCR1 = 0x0190; //400
	TIM1->CCR2 = 0x0190; //200
	TIM1->CCR3 = 0x0190; //600

	TIM1->BDTR |= (TIM_BDTR_DTG_2 | TIM_BDTR_DTG_1 | TIM_BDTR_DTG_0); //Added approximately 437ns of deadtime as not to rely on the gate driver function.
	TIM1->BDTR |= TIM_BDTR_MOE;	//Main output enable


	// Counter Enable
	TIM1->CR1 |= TIM_CR1_CEN;
}

uint16_t Calc_APP_Torque_Request(uint16_t MeasuredAnalogueCounts, uint16_t MaxCounts, uint16_t MinCounts)
{
	bool inputsHealthy = false;
	uint16_t calculatedTorque = 0x00;

	// Check Inputs
	if((MaxCounts - MinCounts) > 0)
	{
		inputsHealthy = true;
	}
	if(MeasuredAnalogueCounts < MinCounts)
	{
		MeasuredAnalogueCounts = MinCounts;
	}

	if(inputsHealthy)
	{
		calculatedTorque = ((MeasuredAnalogueCounts - MinCounts)*100)/(MaxCounts - MinCounts);

		// Limit Output to between 0 and 100
		if(calculatedTorque > 100)
		{
			calculatedTorque = 100;
		}
		if(calculatedTorque < 0)
		{
			calculatedTorque = 0;
		}
	}

	return calculatedTorque;
}

void Calc_CaptureCompareRegisterUVW(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR, uint16_t TorqueRequestPercent, uint16_t positionFeedbackMechanical)
{
	//Position feedback is 0 - 2023 counts for 1 mechanical revolution.
	//A 7 pole pair motor requires the feedback divided by 7 to determine 1 electrical revolution.
	uint16_t oneElectricalRevolution = 289;
	uint16_t ninetyElectricalDegrees = 72; //The number of counts for 90 electrical degrees.
	int16_t positionFeedbackElectrical = positionFeedbackMechanical;
	uint16_t appliedVectorAngleRevolution = 0x0000;
	uint16_t appliedVectorAngleSector = 0x0000;
	uint16_t appliedVectorSector = 0x0001;
	uint32_t appliedVectorAnglePercentageArray[48] = {100,98,96,94,91,89,87,85,83,81,79,77,74,72,70,68,66,64,62,60,57,55,53,51,49,47,45,43,40,38,36,34,32,30,28,26,23,21,19,17,15,13,11,9,6,4,2,0};


	//A sector divides the the electrical revolution into 6 segments.
	//There are 48 counts for the resolution of 60 degrees.
	uint16_t oneElectricalSector = 48;



	//The following loop determines ("unwinds") the mechanical revolutions to just one electrical revolution. Will always be between 0 and 289.
	while(positionFeedbackElectrical > oneElectricalRevolution)
	{
		positionFeedbackElectrical = positionFeedbackElectrical - oneElectricalRevolution;
	}


	//Apply the 90 degree advance to determine new applied vector. Will always be between 0 and 289.
	appliedVectorAngleRevolution = positionFeedbackElectrical + ninetyElectricalDegrees;
	if(appliedVectorAngleRevolution > oneElectricalRevolution)
	{
		appliedVectorAngleRevolution = appliedVectorAngleRevolution - oneElectricalRevolution;
	}
	appliedVectorAngleSector = appliedVectorAngleRevolution;


	//Determine what sector the applied vector will be in. This determines whether the high side or low side FETs will be majority on.
	//appliedVectorAngleSector will provide a count (0 to 47) which corresponds to the angle.
	while(appliedVectorAngleSector > oneElectricalSector)
	{
		appliedVectorAngleSector = appliedVectorAngleSector - oneElectricalSector;
		appliedVectorSector++;
	}
	//At this stage, the angle of where to apply the vector within the electrical revolution and subsequent sector are known.
	//We now need to calculate the applied vector based on the torque request.
	//No third harmonic injection will not be used at this stage.
	switch(appliedVectorSector)
	{
	case 1:
		*uCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*wCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*vCCR = (*uCCR - *wCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *wCCR;
		break;

	case 2:
		*vCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*wCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*uCCR = (*vCCR - *wCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *wCCR;
		break;

	case 3:
		*vCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*uCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*wCCR = (*vCCR - *uCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *uCCR;
		break;

	case 4:
		*wCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*uCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*vCCR = (*wCCR - *uCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *uCCR;
		break;

	case 5:
		*wCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*vCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*uCCR = (*wCCR - *vCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *vCCR;
		break;

	case 6:
		*uCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*vCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*wCCR = (*uCCR - *vCCR) * appliedVectorAnglePercentageArray[appliedVectorAngleSector] / 100 + *vCCR;
		break;

	default:
		*uCCR = 400;
		*vCCR = 400;
		*wCCR = 400;
		break;

	}


	//The following loop determines what sector the motor is in and number of counts in sector.
	//while(positionFeedbackSector > oneElectricalSector)
	//{
	//	positionFeedbackSector = positionFeedbackSector - oneElectricalSector;
	//	sector++;
	//}



}

void Temp_Calc_CaptureCompareRegisterUVW(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR, uint16_t TorqueRequestPercent, uint16_t positionFeedbackMechanical, uint16_t* positionFeedbackElectrical, uint16_t* appliedVectorAngleRevolution, uint16_t* appliedVectorAngleSector)
{
	//Position feedback is 0 - 2023 counts for 1 mechanical revolution.
	//A 7 pole pair motor requires the feedback divided by 7 to determine 1 electrical revolution.
	uint16_t oneElectricalRevolution = 289;
	uint16_t ninetyElectricalDegrees = 72; //The number of counts for 90 electrical degrees.
	//uint16_t positionFeedbackElectrical = positionFeedbackMechanical;
	//uint16_t appliedVectorAngleRevolution = 0x0000;
	//uint16_t appliedVectorAngleSector = 0x0000;
	uint16_t appliedVectorSector = 0x0001;
	uint32_t appliedVectorAnglePercentageArray[48] = {100,98,96,94,91,89,87,85,83,81,79,77,74,72,70,68,66,64,62,60,57,55,53,51,49,47,45,43,40,38,36,34,32,30,28,26,23,21,19,17,15,13,11,9,6,4,2,0};


	//A sector divides the the electrical revolution into 6 segments.
	//There are 48 counts for the resolution of 60 degrees.
	uint16_t oneElectricalSector = 48;

	*positionFeedbackElectrical = positionFeedbackMechanical;

	//The following loop determines ("unwinds") the mechanical revolutions to just one electrical revolution. Will always be between 0 and 289.
	while(*positionFeedbackElectrical >= oneElectricalRevolution)
	{
		*positionFeedbackElectrical = *positionFeedbackElectrical - oneElectricalRevolution;
	}


	//Apply the 90 degree advance to determine new applied vector. Will always be between 0 and 289.
	*appliedVectorAngleRevolution = *positionFeedbackElectrical + ninetyElectricalDegrees;
	if(*appliedVectorAngleRevolution >= oneElectricalRevolution)
	{
		*appliedVectorAngleRevolution = *appliedVectorAngleRevolution - oneElectricalRevolution;
	}
	*appliedVectorAngleSector = *appliedVectorAngleRevolution;


	//Determine what sector the applied vector will be in. This determines whether the high side or low side FETs will be majority on.
	//appliedVectorAngleSector will provide a count (0 to 47) which corresponds to the angle.
	while(*appliedVectorAngleSector >= oneElectricalSector)
	{
		*appliedVectorAngleSector = *appliedVectorAngleSector - oneElectricalSector;
		appliedVectorSector++;
	}
	//At this stage, the angle of where to apply the vector within the electrical revolution and subsequent sector are known.
	//We now need to calculate the applied vector based on the torque request.
	//No third harmonic injection will not be used at this stage.
	switch(appliedVectorSector)
	{
	case 1:
		*uCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*wCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*vCCR = *uCCR - (*uCCR - *wCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100;
		break;

	case 2:
		*vCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*wCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*uCCR = (*vCCR - *wCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100 + *wCCR;
		break;

	case 3:
		*vCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*uCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*wCCR = *vCCR - (*vCCR - *uCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100;
		break;

	case 4:
		*wCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*uCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*vCCR = (*wCCR - *uCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100 + *uCCR;
		break;

	case 5:
		*wCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*vCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*uCCR = *wCCR - (*wCCR - *vCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100;
		break;

	case 6:
		*uCCR = 400 + (TorqueRequestPercent * 346) / 100;
		*vCCR = 400 - (TorqueRequestPercent * 346) / 100;

		*wCCR = (*uCCR - *vCCR) * appliedVectorAnglePercentageArray[*appliedVectorAngleSector] / 100 + *vCCR;
		break;

	default:
		*uCCR = 400;
		*vCCR = 400;
		*wCCR = 400;
		break;

	}


	//The following loop determines what sector the motor is in and number of counts in sector.
	//while(positionFeedbackSector > oneElectricalSector)
	//{
	//	positionFeedbackSector = positionFeedbackSector - oneElectricalSector;
	//	sector++;
	//}



}

void Update_CaptureCompare(uint16_t* uCCR, uint16_t* vCCR, uint16_t* wCCR)
{
	TIM1->CCR1 = *uCCR;
	TIM1->CCR2 = *vCCR;
	TIM1->CCR3 = *wCCR;
}

bool MotorSynchronise()
{
	bool bsynchronisedState = false;
	uint32_t i = 0;
	//PC13;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk; //PC13 as input

	if((GPIOC->IDR & GPIO_IDR_ID13) == 0x00)
	{
		TIM1->CCR1 = 0x0226;	//450
		TIM1->CCR2 = 0x00FA;	//350
		TIM1->CCR3 = 0x00FA;		//350

		for (i = 0; i < 0x007A1200; i++) // Gives me a delay of 0.5 seconds for the motor to move to position. 0x007A1200
		{

		}
		TIM4->CNT = 0x0000;
		bsynchronisedState = true;
	}

	return bsynchronisedState;
}
