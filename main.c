////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*																																																																						*/
/*	������������ ���������� STM32F103C8T6 �� �������� ����� Bluepill																																					*/
/*	PA6 - ���� SPI																																																														*/
/*	PA7 - ����� SPI																																																														*/
/*	PA9 - ����� UART																																																													*/
/*	PA10 - ���� UART																																																													*/
/*	PC13 - LED																																																																*/
/*																																																																						*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stm32f10x.h>

// �������������� �������
void RCCPLL_Init(void);																					//������� �������� �������
void LED_Init(void);																						//������������� ������ ����������
void CRC_Init(void);																						//������������� ����� CRC
void UART_Init(void);																						//������������� UART
void SPI_Init(void);																						//������������� SPI
void Timer_Init(void);																					//������������� ������� �� 500��

// �������������� ����������	
void DMA1_Channel1_IRQHandler(void);														//�� DMA CRC
void SPI1_IRQHandler(void);																			//�� SPI
void DMA1_Channel3_IRQHandler(void);														//�� �������� DMA SPI 
void DMA1_Channel4_IRQHandler(void);														//�� �������� DMA UART
void DMA1_Channel5_IRQHandler(void);														//�� ����� DMA UART
void TIM3_IRQHandler(void);																			//�� ������� �������� ����������
void TIM4_IRQHandler(void);																			//�� ������ �� �������� UART

//��������� ��������� UART
typedef struct{
	uint16_t Mass[1001][4];																				//������ 0-1000 4 ���������
	uint8_t CRC8;																									//CRC
}Message_struct;

//���������� ����������
static Message_struct Rx = {0}, Tx = {0};												//������ �������� � ����������� UART

static uint8_t SPIRx; 																					//����� �������� SPI
static uint16_t SPIRxCnt = 0;																		//������� ������ SPI

static uint8_t check = 0;																				//�������� ����������� 

//�������� ���������
int main(void)
{
	RCCPLL_Init();																								//������� �������� ������� (72���)
	
	for (uint16_t i = 0; i < 1001; i++)														//���������� ���������
	{
		for (uint8_t j = 0; j < 4; j++)
		{
			if (j & 1)
				Tx.Mass[i][j] = 0xAAAA;
			else
				Tx.Mass[i][j] = 0x5555;
		}
	}
	LED_Init();																										//������������� ������ ����������
	UART_Init();																									//������������� UART
	CRC_Init();																										//������������� ����� CRC
	SPI_Init();																										//������������� SPI
	Timer_Init();																									//������������� ������� �� 500��
	
	while(1)
		__WFI();																										//�������� ���� ��������� (�������� ����������)
}

/* ��������� ������� ������� �������� ������� */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																			//��������� �������� ��������� �������
	while(!(RCC->CR & RCC_CR_HSERDY))															//�������� ���������� ������ �������� ��������� �������
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;					//���������������� ���� � ���������� ������� ������� �� 9 �� 72��� � ������� ������� APB1 �� 2 �� 36 ��� �������� reference manual
	RCC->CR |= RCC_CR_PLLON;																			//��������� ����
	while(!(RCC->CR & RCC_CR_PLLRDY))															//�������� ���������� ������ ����
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																	//������������ ��������� ������� �� ����
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))												//�������� ���������� ������������
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																			//���������� ����������� ��������� ����������
}

/* ����������� ������� ������������� ������ ���������� */
void LED_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;												 		//��������� ����� �
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;														//��������� ������� 3
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);						//����� ��������� ��� PC13
	GPIOC->CRH |= GPIO_CRH_MODE13_1;															//��������� ����� � push-pull
	GPIOC->ODR |= GPIO_ODR_ODR13;																	//����� ������ ����������
	
	NVIC_EnableIRQ(TIM3_IRQn);																		//��������� ���������� ��� ������� 3
	TIM3->DIER = TIM_DIER_UIE;																		//��������� ���������� �� ���������� �������� �������
	TIM3->PSC = 35999;																						//��������� ������������
	TIM3->ARR = 199;																							//��������� ������� (��������� ����� ���������� �� 100�� - 72MHz/36000/200=10Hz - 100 ��)
	TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_OPM;												//������������ ���������� � ����������� ������ �������
}

/* ����������� ������� ������������� CRC */
void CRC_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN;					//��������� ������������ ������ CRC � DMA
	
	CRC->CR = CRC_CR_RESET;																				//����� ����� CRC
	
/* ��������� ����� DMA CRC */
	DMA1_Channel1->CCR = 	DMA_CCR1_MEM2MEM | DMA_CCR1_PSIZE_1 |		//����� ������ � �������, ��� ����������, ������ �������� 1 ����, ������� 4 �����
												DMA_CCR1_MINC | DMA_CCR1_DIR |					//��������� ��������� ������, ��� ���������� ��������� ������, �� ��������� �����, ������ �� ������
												DMA_CCR1_TEIE | DMA_CCR1_TCIE;					//���������� �� ������ � ��������� ��������
	DMA1_Channel1->CNDTR = sizeof(Message_struct) - 2;						//������ �� ������� ��������� ��� ����������� ����� (��� ������ CRC �������� 2 ����� � ������)
	DMA1_Channel1->CPAR = CRC_BASE;																//����� ����� 
	DMA1_Channel1->CMAR = (uint32_t)&Tx;													//����� ������
	DMA1->IFCR = 	DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 |							//����� ������ ���������� �� ������� ������ DMA 
								DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);														//��������� ���������� �� ������� ������ DMA
}

/* ����������� ������� ������������� UART */
void UART_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;															//��������� ������������ ����� DMA
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;		//��������� ����� � � ���������� USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |							//����� ����� ��� USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |							//���� PA9 ����� � �������������� �������� Push-pull � ������������ ���������
								GPIO_CRH_CNF10_1;																//���� PA10 ���� � �������������� �������� Push-pull

	USART1->BRR = 0x4E;																						//���������� �������� �� 921600(923.076)(����������� �������� ��� �������� ��������� ������ 500��)
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 									//����� 9 ��� � ������ �������� � �������� ��������
								USART_CR1_TE | USART_CR1_RE;			 							//��������� ����������� � �������� UART
	USART1->CR2 = USART_CR2_STOP_1;																//������������� 2 �������
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;								//������������� DMA �� �������� � ����																	
	USART1->CR1 |= USART_CR1_UE;																	//��������� USART1
	
/* ��������� ����� DMA �� ������ �������� USART1 */	
	DMA1_Channel5->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC |					//������� ���������, ��������� ������
												DMA_CCR1_CIRC | DMA_CCR1_TEIE |					//��������� �����, ���������� �� ������
												DMA_CCR1_TCIE;													//��������� �����
	DMA1_Channel5->CNDTR = sizeof(Message_struct);								//������ �� ������� ���������
	DMA1_Channel5->CPAR = USART1_BASE + 0x4;											//�������� ����� �������� ������ USART1
	DMA1_Channel5->CMAR = (uint32_t)&Rx;													//������� ����� ������ ���������
	DMA1->IFCR = 	DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 |							//����� ������ ���������� �� ������ ������ DMA 
								DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);														//��������� ���������� �� ������ ������ DMA
	
	DMA1_Channel5->CCR |= DMA_CCR1_EN;														//��������� ����� DMA
	
/* ��������� ����� DMA �� ������ ����������� USART1 */	
	DMA1_Channel4->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC |					//������� ���������, ��������� ������
												DMA_CCR1_DIR | DMA_CCR1_TEIE |					//����������� � ���������, ���������� �� ������
												DMA_CCR1_TCIE;													//��������� ��������
	DMA1_Channel4->CNDTR = sizeof(Message_struct);								//������ �� ������� ���������
	DMA1_Channel4->CPAR = USART1_BASE + 0x4;											//������� ����� �������� ������ USART1
	DMA1_Channel4->CMAR = (uint32_t)&Tx;													//�������� ����� ������ ���������
	DMA1->IFCR = 	DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 |							//����� ������ ���������� �� ���������� ������ DMA 
								DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);														//��������� ���������� �� ���������� ������ DMA
}

/* ����������� ������� ������������� SPI */
void SPI_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;															//��������� ������������ ����� DMA
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;			//��������� ������������ ����� � � SPI1
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5 |							//����� ������ PA5, PA6, PA7
									GPIO_CRL_CNF6 | GPIO_CRL_MODE6 |
									GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 |							//������������� ������ �� �������������� ������� Push-Pull
								GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6 |
								GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	NVIC_EnableIRQ(SPI1_IRQn);																		//��������� ���������� �� SPI1
	
	SPI1->SR = 0;																									//����� ���� ������ SPI
	SPI1->CR1 = SPI_CR1_BR_2 | SPI_CR1_BR_0 |											//������� 72MHz/64
							SPI_CR1_MSTR | SPI_CR1_CPOL |											//������� ���������� � ���������� ���������� 
							SPI_CR1_CPHA;																			//� ���� �������
	SPI1->CR2 = SPI_CR2_RXNEIE | SPI_CR2_TXDMAEN;									//��������� �������� �� DMA � �������� �� ����������
	SPI1->CR1 |= SPI_CR1_SPE;																			//��������� SPI

/* ��������� ����� DMA �� ������ ����������� SPI1 */
	DMA1_Channel3->CCR = 	DMA_CCR1_MINC | DMA_CCR1_DIR | 					//��������� ������, ����������� � ���������
												DMA_CCR1_TEIE | DMA_CCR1_TCIE;					//���������� �� ������ � �� ��������� ��������
	DMA1_Channel3->CNDTR = sizeof(Message_struct);								//������ �� ������� ���������
	DMA1_Channel3->CPAR = SPI1_BASE + 0xC;												//������� ����� �������� ������ SPI1
	DMA1_Channel3->CMAR = (uint32_t)&Rx;													//�������� ����� ������ ���������
	DMA1->IFCR = 	DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 |							//����� ������ ���������� �� �������� ������ DMA 
								DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/* ����������� ������� ������������� ������� 500�� */
void Timer_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;														//��������� ������������ ������� 4
	
	NVIC_EnableIRQ(TIM4_IRQn);																		//��������� ���������� �� ������� 4
	
	TIM4->DIER = TIM_DIER_UIE;																		//��������� ���������� �� ���������� �������� ������� 
	TIM4->PSC = 35999;																						//��������� ������������
	TIM4->ARR = 999;																							//��������� ������� �� 500 �� (72MHz/36000/1000=2Hz - 500��)
	TIM4->CR1 = TIM_CR1_ARPE;																			//������������ ���������� ������� ��� ���������� ��������
	TIM4->CR1 |= TIM_CR1_CEN;																			//��������� ����� �������
}

/* ����������� ���������� �� DMA CRC */
void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF1)																	//�������� ����� �� ���������� DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF1 | DMA_ISR_TCIF1)){				//������ ������� ����������
			case DMA_ISR_TEIF1:																				//���������� �� ������ DMA
				DMA1->IFCR = DMA_IFCR_CTEIF1;														//����� ����� ����������
			break;
			
			case DMA_ISR_TCIF1:																				//���������� �� ��������� �������� � ���� CRC
				DMA1->IFCR = DMA_IFCR_CTCIF1;														//����� ����� ����������
				if (DMA1_Channel1->CMAR == (uint32_t)&Tx)								//���� ������ ��� ����������� �� ���������� ������ � �������� 
				{
					Tx.CRC8 = (uint8_t)CRC->DR;														//������� CRC
					DMA1_Channel4->CNDTR = sizeof(Message_struct);				//������������� ������� ������ ����������� USART1 ��� DMA
					DMA1_Channel4->CCR |= DMA_CCR4_EN;										//��������� �������� USART1 �� DMA
				}
				else																										//���� ������ ��� ����������� �� ��������� ����� ������
				{
					if (Rx.CRC8 == (uint8_t)CRC->DR)											//�������� ����������� �����
					{
						GPIOC->ODR &= (uint32_t)~GPIO_ODR_ODR13;						//��������� ����������
						TIM3->CR1 |= TIM_CR1_CEN;														//��������� �������
						DMA1_Channel3->CNDTR = sizeof(Message_struct);			//������������� ������� ������ ����������� SPI1 ��� DMA
						DMA1_Channel3->CCR |= DMA_CCR3_EN;									//��������� �������� SPI1 �� DMA
					}
				}
				CRC->CR = CRC_CR_RESET;																	//����� ����� CRC
				DMA1_Channel1->CCR &= (uint32_t)~DMA_CCR1_EN;						//���������� ������ DMA �� CRC
				DMA1_Channel1->CNDTR = sizeof(Message_struct) - 2;			//������������� ������� ������ ��� CRC DMA
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF1;																//����� ������ ���������� �� ������� ������ DMA
	}
}

/* ����������� ���������� �� SPI1 */
void SPI1_IRQHandler(void)
{
	if (SPI1->SR & SPI_SR_RXNE)																		//�������� ������������� ������ �������� SPI1
	{
		SPIRx = (uint8_t)SPI1->DR;																	//������ ������ �� ������ �������� SPI1
		if (SPIRx != *(uint8_t*)((uint32_t)&Tx + SPIRxCnt))					//��������� �������� ������ �� SPI � ��������������� ������� 
		{
			check = 1;																								//��� ������������ ��������� �������� �����������
			SPIRxCnt = 0;																							//����� �������� �����
			SPI1->CR2 &= ~SPI_CR2_RXNEIE; 														//����� �������� �� ���������� �� ����� SPI1
		}
		else
			SPIRxCnt++;																								//��������� �������� ����� SPI
		if (SPIRxCnt == sizeof(Message_struct))											//���� ������ ��������� ���� ���������
			SPIRxCnt = 0;																							//����� �������� �� �����
	}
}

/* ����������� ���������� �� �������� SPI1 DMA */
void DMA1_Channel3_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF3)																	//�������� ����� �� ���������� DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF3 | DMA_ISR_TCIF3)){				//������ ������� ����������
			case DMA_ISR_TEIF3:																				//���������� �� ������ DMA
				DMA1->IFCR = DMA_IFCR_CTEIF3;														//����� ����� ����������
			break;
			
			case DMA_ISR_TCIF3:																				//���������� �� ��������� �������� SPI1
				DMA1->IFCR = DMA_IFCR_CTCIF3;														//����� ����� ����������													
				DMA1_Channel3->CCR &= (uint32_t)~DMA_CCR3_EN;						//���������� ������ DMA �� �������� SPI1
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF3;																//����� ������ ���������� �� �������� ������ DMA
	}
}

/* ����������� ���������� �� �������� USART1 DMA */
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF4)																	//�������� ����� �� ���������� DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF4 | DMA_ISR_TCIF4)){				//������ ������� ����������
			case DMA_ISR_TEIF4:																				//���������� �� ������ DMA
				DMA1->IFCR = DMA_IFCR_CTEIF4;														//����� ����� ����������
			break;
			
			case DMA_ISR_TCIF4:																				//���������� �� ��������� �������� USART1
				DMA1->IFCR = DMA_IFCR_CTCIF4;														//����� ����� ����������	
				DMA1_Channel4->CCR &= (uint32_t)~DMA_CCR4_EN;						//���������� ������ DMA �� �������� USART1
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF4;																//����� ������ ���������� �� ��������� ������ DMA
	}
}

/* ����������� ���������� �� �������� USART1 DMA */
void DMA1_Channel5_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF5)																	//�������� ����� �� ���������� DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF5 | DMA_ISR_TCIF5)){				//������ ������� ����������
			case DMA_ISR_TEIF5:																				//���������� �� ������ DMA
				DMA1->IFCR = DMA_IFCR_CTEIF5;														//����� ����� ����������
			break;

			case DMA_ISR_TCIF5:																				//���������� �� ��������� ����� USART1
				DMA1->IFCR = DMA_IFCR_CTCIF5;														//����� ����� ����������	
				DMA1_Channel1->CMAR = (uint32_t)&Rx;										//������������� ������ ������ �������� ��� DMA CRC
				DMA1_Channel1->CCR |= DMA_CCR1_EN;											//��������� ������ DMA CRC
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF5;																//����� ������ ���������� �� ������ ������ DMA
	}
}

/* ����������� ���������� �� ������� 3 */
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF)																		//�������� ����� �� ���������� ��������
	{
		TIM3->SR = (uint16_t)~TIM_SR_UIF;														//����� ����� �� ���������� ��������
		GPIOC->ODR |= GPIO_ODR_ODR13;																//���������� ����������
	}
}

/* ����������� ���������� �� ������� 4 */
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & TIM_SR_UIF)																		//�������� ����� �� ���������� ��������
	{
		TIM4->SR = (uint16_t)~TIM_SR_UIF;														//����� ����� �� ���������� ��������
		DMA1_Channel1->CMAR = (uint32_t)&Tx;												//������������� ������ ������ ����������� ��� DMA CRC
		DMA1_Channel1->CCR |= DMA_CCR1_EN;													//��������� ������ DMA CRC
	}
}
