#include <stm32f10x.h>

void RCCPLL_Init(void);
void CRC_Init(void);
void UART_Init(void);

void USART1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);

static uint16_t Mass[1001][4] = {0};
static uint8_t UART_Data = 0;

static uint32_t TestCRCMass[8] = {0xDEADBEEF, 0xAE864AFE, 0x12345678, 0x9ABCDEF0, 0xAAAAAAAA, 0x55555555, 0xCCCCCCCC, 0x33333333};
static uint8_t TestUSARTMass[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
static uint32_t CRC32 = 0;

int main(void)
{
	RCCPLL_Init();
	UART_Init();
	CRC_Init();
	
	while(1)
	{
		__WFI();
	}
}

/* ��������� ������� ������� �������� ������� */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																		//��������� �������� ��������� �������
	while(!(RCC->CR & RCC_CR_HSERDY))														//�������� ���������� ������ �������� ��������� �������
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;				//���������������� ���� � ���������� ������� ������� �� 9 �� 72��� � ������� ������� APB1 �� 2 �� 36 ��� �������� reference manual
	RCC->CR |= RCC_CR_PLLON;																		//��������� ����
	while(!(RCC->CR & RCC_CR_PLLRDY))														//�������� ���������� ������ ����
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																//������������ ��������� ������� �� ����
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))											//�������� ���������� ������������
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																		//���������� ����������� ��������� ����������
}

void CRC_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN;
	
	CRC->CR = CRC_CR_RESET;
	
	DMA1_Channel1->CCR = 	DMA_CCR1_MEM2MEM | DMA_CCR1_MSIZE_1 | 
												DMA_CCR1_PSIZE_1 | DMA_CCR1_MINC | 
												DMA_CCR1_DIR | DMA_CCR1_TEIE | 
												DMA_CCR1_TCIE;
	DMA1_Channel1->CNDTR = 8;
	DMA1_Channel1->CPAR = CRC_BASE;
	DMA1_Channel1->CMAR = (uint32_t)&TestCRCMass;
	DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
}

/* ����������� ������� ������������� UART */
void UART_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;	//��������� ����� � � ���������� USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |						//������������� ����� ��� USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |
								GPIO_CRH_CNF10_1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// Interrupt version
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
//	NVIC_EnableIRQ(USART1_IRQn);																//��������� ���������� �� USART1
//	
//	USART1->BRR = 0x271;																				//���������� �������� 115200 (����� ������� �� reference manual)
//	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 
//								USART_CR1_PEIE | /*USART_CR1_TXEIE |*/  				//��������� ���������� �� ����� ������ � �������� �������� � �����������
//								USART_CR1_RXNEIE | USART_CR1_TE | 
//								USART_CR1_RE;
//	USART1->CR2 = USART_CR2_STOP_1;
//	USART1->CR3 = USART_CR3_EIE;
//	USART1->CR1 |= USART_CR1_UE;																//��������� USART1
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// DMA version
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	USART1->BRR = 0x271;																				//���������� �������� 115200 (����� ������� �� reference manual)
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 
								USART_CR1_PEIE | USART_CR1_TE |				 				//��������� ���������� �� ����� ������ � �������� �������� � �����������
								USART_CR1_RE;
	USART1->CR2 = USART_CR2_STOP_1;
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR |
								USART_CR3_EIE;
	USART1->CR1 |= USART_CR1_UE;																//��������� USART1
	
	DMA1_Channel4->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC | 
												DMA_CCR1_DIR | DMA_CCR1_TEIE | 
												DMA_CCR1_TCIE;
	DMA1_Channel4->CNDTR = 10;
	DMA1_Channel4->CPAR = USART1_BASE + 0x4;
	DMA1_Channel4->CMAR = (uint32_t)&TestUSARTMass;
	DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	
	DMA1_Channel4->CCR |= DMA_CCR1_EN;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
}

/* ����������� ������� ���������� �� UART */
void USART1_IRQHandler(void)
{
	uint16_t StatusReg = USART1->SR;
	uint16_t ErrBlank;
	if (StatusReg & USART_SR_RXNE)															//����� �������� ������� �� ������� ������ � ������
	{
		if (StatusReg & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
			ErrBlank = (uint16_t)USART1->DR;
		else
			UART_Data = (uint8_t)USART1->DR;
	}
}

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
				CRC32 = CRC->DR;
				CRC->CR = CRC_CR_RESET;	
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF1;
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

			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF4;
	}
}
