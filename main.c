#include <stm32f10x.h>

void RCCPLL_Init(void);
void LED_Init(void);
void CRC_Init(void);
void UART_Init(void);
void SPI_Init(void);
void Timer_Init(void);

//void USART1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
//void DMA1_Channel2_IRQHandler(void);
void SPI1_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

typedef struct{
	uint16_t Mass[1001][4];
	uint8_t CRC8;
}Message_struct;

static Message_struct Rx = {0}, Tx = {0};

static uint8_t SPIRx; 
static uint16_t SPIRxCnt = 0;

static uint8_t check = 0;

int main(void)
{
	RCCPLL_Init();
	
	for (uint16_t i = 0; i < 1001; i++)
	{
		for (uint8_t j = 0; j < 4; j++)
		{
			if (j & 1)
				Tx.Mass[i][j] = 0xAAAA;
			else
				Tx.Mass[i][j] = 0x5555;
		}
	}
	LED_Init();
	UART_Init();
	CRC_Init();
	SPI_Init();
	Timer_Init();
	
	while(1)
	{
		__WFI();
	}
}

/* Опредение функции задания тактовой частоты */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																		//Включение внешнего тактового сигнала
	while(!(RCC->CR & RCC_CR_HSERDY))														//Ожидание готовности работы внешнего тактового сигнала
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;				//Конфигурирование ФАПЧ с умножением входной частоты на 9 до 72МГц и деление частоты APB1 на 2 до 36 МГц согласно reference manual
	RCC->CR |= RCC_CR_PLLON;																		//Включение ФАПЧ
	while(!(RCC->CR & RCC_CR_PLLRDY))														//Ожидание готовности работы ФАПЧ
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																//Переключение тактового сигнала на ФАПЧ
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))											//Ожидание готовности переключения
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																		//Отключение внутреннего тактового генератора
}

void LED_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;												 	//Включение портов А и С
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;													//Включение таймера 3
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);					//Инициализация светодиода PC13
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->ODR |= GPIO_ODR_ODR13;
	
	NVIC_EnableIRQ(TIM3_IRQn);																	//Switching-on timer 4 interrupt
	TIM3->DIER = TIM_DIER_UIE;																	//Switching-on timer 4 counter update interrupt
	TIM3->PSC = 35999;																					//Timer 4 counter prescaler is 36
	TIM3->ARR = 99;																							//Timer 4 start counter period (36 MHz / 36 / 65000 / 6 * 60 = 150 BLDC start PRM)
	TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_OPM;											//Configure timer 4 for auro-reload preload
}

void CRC_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN;
	
	CRC->CR = CRC_CR_RESET;
	
	DMA1_Channel1->CCR = 	DMA_CCR1_MEM2MEM | DMA_CCR1_MSIZE_0 | 
												DMA_CCR1_PSIZE_1 | DMA_CCR1_MINC | 
												DMA_CCR1_DIR | DMA_CCR1_TEIE | 
												DMA_CCR1_TCIE;
	DMA1_Channel1->CNDTR = (sizeof(Message_struct) >> 1) - 1;
	DMA1_Channel1->CPAR = CRC_BASE;
	DMA1_Channel1->CMAR = (uint32_t)&Tx;
	DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* Определение функции инициализации UART */
void UART_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;	//Включение порта А и интерфейса USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |						//Инициализация порта под USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |
								GPIO_CRH_CNF10_1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// Interrupt version
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
//	NVIC_EnableIRQ(USART1_IRQn);																//Включение прерывания по USART1
//	
//	USART1->BRR = 0x271;																				//Подстройка бодрейта 115200 (Взято готовое из reference manual)
//	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 
//								USART_CR1_PEIE | /*USART_CR1_TXEIE |*/  				//Включение прерывания по приёму данных и ключение приёмника и передатчика
//								USART_CR1_RXNEIE | USART_CR1_TE | 
//								USART_CR1_RE;
//	USART1->CR2 = USART_CR2_STOP_1;
//	USART1->CR3 = USART_CR3_EIE;
//	USART1->CR1 |= USART_CR1_UE;																//Включение USART1
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// DMA version
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
//	USART1->BRR = 0x271;																				//Подстройка бодрейта 115200 (Взято готовое из reference manual)
	USART1->BRR = 0x4E;																						//Подстройка бодрейта на 921600(923.076) (Взято готовое из reference manual) 
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 
								USART_CR1_TE | USART_CR1_RE;			 						//Включение прерывания по приёму данных и ключение приёмника и передатчика
	USART1->CR2 = USART_CR2_STOP_1;
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR |
								USART_CR3_EIE;
	USART1->CR1 |= USART_CR1_UE;																//Включение USART1
	
	DMA1_Channel5->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC |
												DMA_CCR1_CIRC | DMA_CCR1_TEIE |
												DMA_CCR1_TCIE;
	DMA1_Channel5->CNDTR = sizeof(Message_struct);
	DMA1_Channel5->CPAR = USART1_BASE + 0x4;
	DMA1_Channel5->CMAR = (uint32_t)&Rx;
	DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	
	DMA1_Channel5->CCR |= DMA_CCR1_EN;
	
	DMA1_Channel4->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC | 
												DMA_CCR1_DIR | DMA_CCR1_TEIE | 
												DMA_CCR1_TCIE;
	DMA1_Channel4->CNDTR = sizeof(Message_struct);
	DMA1_Channel4->CPAR = USART1_BASE + 0x4;
	DMA1_Channel4->CMAR = (uint32_t)&Tx;
	DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}

void SPI_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5 |
									GPIO_CRL_CNF6 | GPIO_CRL_MODE6 |
									GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 |
								GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6 |
								GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	NVIC_EnableIRQ(SPI1_IRQn);
	
	SPI1->SR = 0;
	SPI1->CR1 = SPI_CR1_BR_2 | SPI_CR1_BR_0 |
							SPI_CR1_MSTR | SPI_CR1_CPHA |
							SPI_CR1_CPOL;
	SPI1->CR2 = SPI_CR2_RXNEIE | SPI_CR2_TXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;
	
//	DMA1_Channel2->CCR = 	DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_TEIE |
//												DMA_CCR1_TCIE;
//	DMA1_Channel2->CNDTR = sizeof(Message_struct);
//	DMA1_Channel2->CPAR = SPI1_BASE + 0xC;
//	DMA1_Channel2->CMAR = (uint32_t)&SPIMassRx;
//	DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
//	
//	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
//	
//	DMA1_Channel2->CCR |= DMA_CCR1_EN;
	
	DMA1_Channel3->CCR = 	DMA_CCR1_MINC | DMA_CCR1_DIR | 
												DMA_CCR1_TEIE | DMA_CCR1_TCIE;
	DMA1_Channel3->CNDTR = sizeof(Message_struct);
	DMA1_Channel3->CPAR = SPI1_BASE + 0xC;
	DMA1_Channel3->CMAR = (uint32_t)&Rx;
	DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void Timer_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;													//Switching-on timer 4 module
	
	NVIC_EnableIRQ(TIM4_IRQn);																	//Switching-on timer 4 interrupt
	TIM4->DIER = TIM_DIER_UIE;																	//Switching-on timer 4 counter update interrupt
	TIM4->PSC = 35999;																					//Timer 4 counter prescaler is 36
	TIM4->ARR = 999;																						//Timer 4 start counter period (36 MHz / 36 / 65000 / 6 * 60 = 150 BLDC start PRM)
	TIM4->CR1 = TIM_CR1_ARPE;																		//Configure timer 4 for auro-reload preload
	TIM4->CR1 |= TIM_CR1_CEN;
}

/* Определение функции прерывания по UART */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
//void USART1_IRQHandler(void)
//{
//	uint16_t StatusReg = USART1->SR;
//	uint16_t ErrBlank;
//	if (StatusReg & USART_SR_RXNE)															//Опрос регистра статуса по наличию данных в буфере
//	{
//		if (StatusReg & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
//			ErrBlank = (uint16_t)USART1->DR;
//		else
//			UART_Data = (uint8_t)USART1->DR;
//	}
//}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF1)
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF1 | /*DMA_ISR_HTIF1 |*/ DMA_ISR_TCIF1)){
			case DMA_ISR_TEIF1:
				DMA1->IFCR = DMA_IFCR_CTEIF1;
			break;
			
			case DMA_ISR_HTIF1:
				DMA1->IFCR = DMA_IFCR_CHTIF1;
			break;
			
			case DMA_ISR_TCIF1:
				DMA1->IFCR = DMA_IFCR_CTCIF1;
				if (DMA1_Channel1->CMAR == (uint32_t)&Tx)
				{
					Tx.CRC8 = (uint8_t)CRC->DR;
					DMA1_Channel4->CNDTR = sizeof(Message_struct);
					DMA1_Channel4->CCR |= DMA_CCR4_EN;
				}
				else
				{
					Rx.CRC8 = (uint8_t)CRC->DR;
					if (Rx.CRC8 == Tx.CRC8)
					{
						GPIOC->ODR &= ~GPIO_ODR_ODR13;
						TIM3->CR1 |= TIM_CR1_CEN;
						DMA1_Channel3->CNDTR = sizeof(Message_struct);
						DMA1_Channel3->CCR |= DMA_CCR3_EN;
					}
				}
				CRC->CR = CRC_CR_RESET;
				DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
				DMA1_Channel1->CNDTR = (sizeof(Message_struct) >> 1) - 1;
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF1;
	}
}

//void DMA1_Channel2_IRQHandler(void)
//{
//	if (DMA1->ISR & DMA_ISR_GIF2)
//	{
//		switch (DMA1->ISR & (DMA_ISR_TEIF2 | /*DMA_ISR_HTIF2 |*/ DMA_ISR_TCIF2)){
//			case DMA_ISR_TEIF2:
//				DMA1->IFCR = DMA_IFCR_CTEIF2;
//			break;
//			
//			case DMA_ISR_HTIF2:
//				DMA1->IFCR = DMA_IFCR_CHTIF2;
//			break;
//			
//			case DMA_ISR_TCIF2:
//				DMA1->IFCR = DMA_IFCR_CTCIF2;
//				for (uint16_t i = 0; i < sizeof(Message_struct); i++)
//				{
//					uint16_t BUF = *(uint8_t*)((uint32_t)&SPIMassRx + i);
//					if (BUF != *(uint8_t*)((uint32_t)&Tx + i))
//					{
//						check = 1;
//						break;
//					}
//				}
//			break;
//			
//			default:
//				
//			break;
//		}
//		DMA1->IFCR = DMA_IFCR_CGIF2;
//	}
//}

void SPI1_IRQHandler(void)
{
	if (SPI1->SR & SPI_SR_RXNE)
	{
		SPIRx = (uint8_t)SPI1->DR;
		if (SPIRx != *(uint8_t*)((uint32_t)&Tx + SPIRxCnt))
		{
			check = 1;
			SPIRxCnt = 0;
			SPI1->CR2 &= ~SPI_CR2_RXNEIE; 
		}
		else
			SPIRxCnt++;
		if (SPIRxCnt == sizeof(Message_struct))
			SPIRxCnt = 0;
	}
}

void DMA1_Channel3_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF3)
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF3 | /*DMA_ISR_HTIF3 |*/ DMA_ISR_TCIF3)){
			case DMA_ISR_TEIF3:
				DMA1->IFCR = DMA_IFCR_CTEIF3;
			break;
			
			case DMA_ISR_HTIF3:
				DMA1->IFCR = DMA_IFCR_CHTIF3;
			break;
			
			case DMA_ISR_TCIF3:
				DMA1->IFCR = DMA_IFCR_CTCIF3;
				DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF3;
	}
}

void DMA1_Channel4_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF4)
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF4 | /*DMA_ISR_HTIF4 |*/ DMA_ISR_TCIF4)){
			case DMA_ISR_TEIF4:
				DMA1->IFCR = DMA_IFCR_CTEIF4;
			break;
			
			case DMA_ISR_HTIF4:
				DMA1->IFCR = DMA_IFCR_CHTIF4;
			break;
			
			case DMA_ISR_TCIF4:
				DMA1->IFCR = DMA_IFCR_CTCIF4;
				DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF4;
	}
}

void DMA1_Channel5_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF5)
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF5 | /*DMA_ISR_HTIF5 |*/ DMA_ISR_TCIF5)){
			case DMA_ISR_TEIF5:
				DMA1->IFCR = DMA_IFCR_CTEIF5;
			break;
			
			case DMA_ISR_HTIF5:
				DMA1->IFCR = DMA_IFCR_CHTIF5;
			break;
			
			case DMA_ISR_TCIF5:
				DMA1->IFCR = DMA_IFCR_CTCIF5;
				DMA1_Channel1->CMAR = (uint32_t)&Rx;	
				DMA1_Channel1->CCR |= DMA_CCR5_EN;
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF5;
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF)
	{
		TIM3->SR = ~TIM_SR_UIF;
		GPIOC->ODR |= GPIO_ODR_ODR13;
	}
}

void TIM4_IRQHandler(void)
{
	if (TIM4->SR & TIM_SR_UIF)
	{
	TIM4->SR = ~TIM_SR_UIF;
	DMA1_Channel1->CMAR = (uint32_t)&Tx;	
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	}
}
