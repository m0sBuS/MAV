////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*																																																																						*/
/*	Используемое устройство STM32F103C8T6 на макетной плате Bluepill																																					*/
/*	PA6 - Вход SPI																																																														*/
/*	PA7 - Выход SPI																																																														*/
/*	PA9 - Выход UART																																																													*/
/*	PA10 - Вход UART																																																													*/
/*	PC13 - LED																																																																*/
/*																																																																						*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stm32f10x.h>

// Декларирование функций
void RCCPLL_Init(void);																					//Задание тактовой частоты
void LED_Init(void);																						//Инициализация вывода светодиода
void CRC_Init(void);																						//Инициализация блока CRC
void UART_Init(void);																						//Инициализация UART
void SPI_Init(void);																						//Инициализация SPI
void Timer_Init(void);																					//Инициализация таймера на 500мс

// Декларирование прерываний	
void DMA1_Channel1_IRQHandler(void);														//По DMA CRC
void SPI1_IRQHandler(void);																			//По SPI
void DMA1_Channel3_IRQHandler(void);														//По передаче DMA SPI 
void DMA1_Channel4_IRQHandler(void);														//По передаче DMA UART
void DMA1_Channel5_IRQHandler(void);														//По приёму DMA UART
void TIM3_IRQHandler(void);																			//По таймеру моргания светодиода
void TIM4_IRQHandler(void);																			//По таймер на передачу UART

//Структура сообщения UART
typedef struct{
	uint16_t Mass[1001][4];																				//данные 0-1000 4 параметра
	uint8_t CRC8;																									//CRC
}Message_struct;

//Объявление переменных
static Message_struct Rx = {0}, Tx = {0};												//Буферы приёмника и передатчика UART

static uint8_t SPIRx; 																					//Буфер приёмника SPI
static uint16_t SPIRxCnt = 0;																		//Счетчик данных SPI

static uint8_t check = 0;																				//Контроль целостности 

//Основная программа
int main(void)
{
	RCCPLL_Init();																								//Задание тактовой частоты (72МГц)
	
	for (uint16_t i = 0; i < 1001; i++)														//Заполнение сообщения
	{
		for (uint8_t j = 0; j < 4; j++)
		{
			if (j & 1)
				Tx.Mass[i][j] = 0xAAAA;
			else
				Tx.Mass[i][j] = 0x5555;
		}
	}
	LED_Init();																										//Инициализация вывода светодиода
	UART_Init();																									//Инициализация UART
	CRC_Init();																										//Инициализация блока CRC
	SPI_Init();																										//Инициализация SPI
	Timer_Init();																									//Инициализация таймера на 500мс
	
	while(1)
		__WFI();																										//Основной цикл программы (Ожидание прерываний)
}

/* Опредение функции задания тактовой частоты */
void RCCPLL_Init(void)
{
	RCC->CR |= RCC_CR_HSEON;																			//Включение внешнего тактового сигнала
	while(!(RCC->CR & RCC_CR_HSERDY))															//Ожидание готовности работы внешнего тактового сигнала
		__NOP();
	RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2;					//Конфигурирование ФАПЧ с умножением входной частоты на 9 до 72МГц и деление частоты APB1 на 2 до 36 МГц согласно reference manual
	RCC->CR |= RCC_CR_PLLON;																			//Включение ФАПЧ
	while(!(RCC->CR & RCC_CR_PLLRDY))															//Ожидание готовности работы ФАПЧ
		__NOP();
	RCC->CFGR |= RCC_CFGR_SW_PLL;																	//Переключение тактового сигнала на ФАПЧ
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL))												//Ожидание готовности переключения
		__NOP();
	RCC->CR &= ~RCC_CR_HSION;																			//Отключение внутреннего тактового генератора
}

/* Определение функции инициализация вывода светодиода */
void LED_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;												 		//Включение порта С
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;														//Включение таймера 3
	
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);						//Сброс регистров для PC13
	GPIOC->CRH |= GPIO_CRH_MODE13_1;															//Медленный выход с push-pull
	GPIOC->ODR |= GPIO_ODR_ODR13;																	//Сброс вывода светодиода
	
	NVIC_EnableIRQ(TIM3_IRQn);																		//Включение прерывания для таймера 3
	TIM3->DIER = TIM_DIER_UIE;																		//Включение прерывания по обновлению счетчика таймера
	TIM3->PSC = 35999;																						//Установка предделителя
	TIM3->ARR = 199;																							//Установка периода (светодиод будет загораться на 100мс - 72MHz/36000/200=10Hz - 100 мс)
	TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_OPM;												//Автозагрузка параметров и однократная работа таймера
}

/* Определение функции инициализации CRC */
void CRC_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN;					//Включение тактирования блоков CRC и DMA
	
	CRC->CR = CRC_CR_RESET;																				//Сброс блока CRC
	
/* Настройка блока DMA CRC */
	DMA1_Channel1->CCR = 	DMA_CCR1_MEM2MEM | DMA_CCR1_PSIZE_1 |		//Режим работы с памятью, без приоритета, размер выходной 1 байт, входной 4 байта
												DMA_CCR1_MINC | DMA_CCR1_DIR |					//Инкремент выходного буфера, без инкремента выходного буфера, не кольцевой режим, чтение из памяти
												DMA_CCR1_TEIE | DMA_CCR1_TCIE;					//Прерывание по ошибке и окончанию передачи
	DMA1_Channel1->CNDTR = sizeof(Message_struct) - 2;						//Размер по размеру сообщения без контрольной суммы (Без сжатия CRC занимает 2 байта в памяти)
	DMA1_Channel1->CPAR = CRC_BASE;																//Адрес входа 
	DMA1_Channel1->CMAR = (uint32_t)&Tx;													//Адрес Выхода
	DMA1->IFCR = 	DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 |							//Сброс флагов прерываний по первому каналу DMA 
								DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);														//Включение прерываний по первому каналу DMA
}

/* Определение функции инициализации UART */
void UART_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;															//Включение тактирования блока DMA
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;		//Включение порта А и интерфейса USART1
	
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 |							//Сброс порта под USART1
									GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 |							//Порт PA9 выход с альтернативной функцией Push-pull с максимальной скоростью
								GPIO_CRH_CNF10_1;																//Порт PA10 вход с альтернативной функцией Push-pull

	USART1->BRR = 0x4E;																						//Подстройка бодрейта на 921600(923.076)(оптимальная скорость под передачу сообщения каждые 500мс)
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | 									//Длина 9 бит с учетом четности и контроль четности
								USART_CR1_TE | USART_CR1_RE;			 							//Включение передатчика и приёмника UART
	USART1->CR2 = USART_CR2_STOP_1;																//Использование 2 стопбит
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;								//Использование DMA на передачу и приём																	
	USART1->CR1 |= USART_CR1_UE;																	//Включение USART1
	
/* настройка блока DMA по каналу приёмника USART1 */	
	DMA1_Channel5->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC |					//Средний приоритет, инкремент памяти
												DMA_CCR1_CIRC | DMA_CCR1_TEIE |					//Кольцевой режим, прерывание по ошибке
												DMA_CCR1_TCIE;													//Окончанию приёма
	DMA1_Channel5->CNDTR = sizeof(Message_struct);								//Размер по размеру сообщения
	DMA1_Channel5->CPAR = USART1_BASE + 0x4;											//Выходной адрес регистра данных USART1
	DMA1_Channel5->CMAR = (uint32_t)&Rx;													//Входной адрес буфера сообщения
	DMA1->IFCR = 	DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 |							//Сброс флагов прерываний по пятому каналу DMA 
								DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);														//Включения прерываний по пятому каналу DMA
	
	DMA1_Channel5->CCR |= DMA_CCR1_EN;														//Включение блока DMA
	
/* настройка блока DMA по каналу передатчика USART1 */	
	DMA1_Channel4->CCR = 	DMA_CCR1_PL_1 | DMA_CCR1_MINC |					//Средний приоритет, инкремент памяти
												DMA_CCR1_DIR | DMA_CCR1_TEIE |					//Направление в периферию, прерывание по ошибке
												DMA_CCR1_TCIE;													//Окончанию передачи
	DMA1_Channel4->CNDTR = sizeof(Message_struct);								//Размер по размеру сообщения
	DMA1_Channel4->CPAR = USART1_BASE + 0x4;											//Входной адрес регистра данных USART1
	DMA1_Channel4->CMAR = (uint32_t)&Tx;													//Выходной адрес буфера сообщения
	DMA1->IFCR = 	DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 |							//Сброс флагов прерываний по четвертому каналу DMA 
								DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);														//Включения прерываний по четвертому каналу DMA
}

/* Определение функции инициализации SPI */
void SPI_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;															//Включение тактирования блока DMA
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;			//Включение тактирования порта А и SPI1
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5 |							//Сброс портов PA5, PA6, PA7
									GPIO_CRL_CNF6 | GPIO_CRL_MODE6 |
									GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5 |							//Инициализация портов на альтернативную функцию Push-Pull
								GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6 |
								GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7;
	
	NVIC_EnableIRQ(SPI1_IRQn);																		//Включение прерывания по SPI1
	
	SPI1->SR = 0;																									//Сброс всех флагов SPI
	SPI1->CR1 = SPI_CR1_BR_2 | SPI_CR1_BR_0 |											//Бодрейт 72MHz/64
							SPI_CR1_MSTR | SPI_CR1_CPOL |											//Ведущее устройство с изменением полярности 
							SPI_CR1_CPHA;																			//И фазы сигнала
	SPI1->CR2 = SPI_CR2_RXNEIE | SPI_CR2_TXDMAEN;									//Включение передачи по DMA и приёмника по прерыванию
	SPI1->CR1 |= SPI_CR1_SPE;																			//Включение SPI

/* Настройка блока DMA по каналу передатчика SPI1 */
	DMA1_Channel3->CCR = 	DMA_CCR1_MINC | DMA_CCR1_DIR | 					//Инкремент памяти, направление в периферию
												DMA_CCR1_TEIE | DMA_CCR1_TCIE;					//Прерывание по ошибке и по окончанию передачи
	DMA1_Channel3->CNDTR = sizeof(Message_struct);								//Размер по размеру сообщения
	DMA1_Channel3->CPAR = SPI1_BASE + 0xC;												//Входной адрес регистра данных SPI1
	DMA1_Channel3->CMAR = (uint32_t)&Rx;													//Выходной адрес буфера сообщения
	DMA1->IFCR = 	DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 |							//Сброс флагов прерываний по третьему каналу DMA 
								DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/* Определение функции инициализации таймера 500мс */
void Timer_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;														//Включение тактирования таймера 4
	
	NVIC_EnableIRQ(TIM4_IRQn);																		//Включение прерывания по таймеру 4
	
	TIM4->DIER = TIM_DIER_UIE;																		//Включение прерывания по обновлению счетчика таймера 
	TIM4->PSC = 35999;																						//Установка предделителя
	TIM4->ARR = 999;																							//Установка периода на 500 мс (72MHz/36000/1000=2Hz - 500мс)
	TIM4->CR1 = TIM_CR1_ARPE;																			//Автозагрузка параметров таймера при обновлении счетчика
	TIM4->CR1 |= TIM_CR1_CEN;																			//Включение счета таймера
}

/* Определение прерывания по DMA CRC */
void DMA1_Channel1_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF1)																	//Проверка флага по прерыванию DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF1 | DMA_ISR_TCIF1)){				//Разбор причины прерывания
			case DMA_ISR_TEIF1:																				//Прерывание по ошибке DMA
				DMA1->IFCR = DMA_IFCR_CTEIF1;														//Сброс флага прерывания
			break;
			
			case DMA_ISR_TCIF1:																				//Прерывание по окончанию передачи в блок CRC
				DMA1->IFCR = DMA_IFCR_CTCIF1;														//Сброс флага прерывания
				if (DMA1_Channel1->CMAR == (uint32_t)&Tx)								//Если запрос был инициирован по подготовке данных к отправке 
				{
					Tx.CRC8 = (uint8_t)CRC->DR;														//Счетние CRC
					DMA1_Channel4->CNDTR = sizeof(Message_struct);				//Предустановка размера буфера передатчика USART1 для DMA
					DMA1_Channel4->CCR |= DMA_CCR4_EN;										//Включение передачи USART1 по DMA
				}
				else																										//Если запрос был инициирован по окончанию приёма данных
				{
					if (Rx.CRC8 == (uint8_t)CRC->DR)											//Проверка контрольной суммы
					{
						GPIOC->ODR &= (uint32_t)~GPIO_ODR_ODR13;						//Включение светодиода
						TIM3->CR1 |= TIM_CR1_CEN;														//Включение таймера
						DMA1_Channel3->CNDTR = sizeof(Message_struct);			//Предустановка размера буфера передатчика SPI1 для DMA
						DMA1_Channel3->CCR |= DMA_CCR3_EN;									//Включение передачи SPI1 по DMA
					}
				}
				CRC->CR = CRC_CR_RESET;																	//Сброс блока CRC
				DMA1_Channel1->CCR &= (uint32_t)~DMA_CCR1_EN;						//Выключение канала DMA по CRC
				DMA1_Channel1->CNDTR = sizeof(Message_struct) - 2;			//Предустановка размера буфера для CRC DMA
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF1;																//Сброс флагов прерывания по первому каналу DMA
	}
}

/* Определение прерывания по SPI1 */
void SPI1_IRQHandler(void)
{
	if (SPI1->SR & SPI_SR_RXNE)																		//Проверка заполненности буфера приёмника SPI1
	{
		SPIRx = (uint8_t)SPI1->DR;																	//Чтение данных из буфера приёмника SPI1
		if (SPIRx != *(uint8_t*)((uint32_t)&Tx + SPIRxCnt))					//Сравнение принятых данных из SPI с первоначальными данными 
		{
			check = 1;																								//При несовпадении выставить контроль целостности
			SPIRxCnt = 0;																							//Сброс счетчика приёма
			SPI1->CR2 &= ~SPI_CR2_RXNEIE; 														//Сброс запросов по прерыванию по приёму SPI1
		}
		else
			SPIRxCnt++;																								//Инкремент счетчика приёма SPI
		if (SPIRxCnt == sizeof(Message_struct))											//Если принят последний байт сообщения
			SPIRxCnt = 0;																							//Сброс счетчика по приёму
	}
}

/* Определение прерывания по передаче SPI1 DMA */
void DMA1_Channel3_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF3)																	//Проверка флага по прерыванию DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF3 | DMA_ISR_TCIF3)){				//Разбор причины прерывания
			case DMA_ISR_TEIF3:																				//Прерывание по ошибке DMA
				DMA1->IFCR = DMA_IFCR_CTEIF3;														//Сброс флага прерывания
			break;
			
			case DMA_ISR_TCIF3:																				//Прерывание по окончанию передачи SPI1
				DMA1->IFCR = DMA_IFCR_CTCIF3;														//Сброс флага прерывания													
				DMA1_Channel3->CCR &= (uint32_t)~DMA_CCR3_EN;						//Выключение канала DMA по передаче SPI1
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF3;																//Сброс флагов прерывания по третьему каналу DMA
	}
}

/* Определение прерывания по передаче USART1 DMA */
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF4)																	//Проверка флага по прерыванию DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF4 | DMA_ISR_TCIF4)){				//Разбор причины прерывания
			case DMA_ISR_TEIF4:																				//Прерывание по ошибке DMA
				DMA1->IFCR = DMA_IFCR_CTEIF4;														//Сброс флага прерывания
			break;
			
			case DMA_ISR_TCIF4:																				//Прерывание по окончанию передачи USART1
				DMA1->IFCR = DMA_IFCR_CTCIF4;														//Сброс флага прерывания	
				DMA1_Channel4->CCR &= (uint32_t)~DMA_CCR4_EN;						//Выключение канала DMA по передаче USART1
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF4;																//Сброс флагов прерывания по четвёртому каналу DMA
	}
}

/* Определение прерывания по передаче USART1 DMA */
void DMA1_Channel5_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_GIF5)																	//Проверка флага по прерыванию DMA
	{
		switch (DMA1->ISR & (DMA_ISR_TEIF5 | DMA_ISR_TCIF5)){				//Разбор причины прерывания
			case DMA_ISR_TEIF5:																				//Прерывание по ошибке DMA
				DMA1->IFCR = DMA_IFCR_CTEIF5;														//Сброс флага прерывания
			break;

			case DMA_ISR_TCIF5:																				//Прерывание по окончанию приёма USART1
				DMA1->IFCR = DMA_IFCR_CTCIF5;														//Сброс флага прерывания	
				DMA1_Channel1->CMAR = (uint32_t)&Rx;										//Предустановка адреса буфера приёмника для DMA CRC
				DMA1_Channel1->CCR |= DMA_CCR1_EN;											//Включение канала DMA CRC
			break;
			
			default:
				
			break;
		}
		DMA1->IFCR = DMA_IFCR_CGIF5;																//Сброс флагов прерывания по пятому каналу DMA
	}
}

/* Определение прерывания по таймеру 3 */
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF)																		//Проверка флага по обновлению счетчика
	{
		TIM3->SR = (uint16_t)~TIM_SR_UIF;														//Сброс флага по обновлению счетчика
		GPIOC->ODR |= GPIO_ODR_ODR13;																//Выключение светодиода
	}
}

/* Определение прерывания по таймеру 4 */
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & TIM_SR_UIF)																		//Проверка флага по обновлению счетчика
	{
		TIM4->SR = (uint16_t)~TIM_SR_UIF;														//Сброс флага по обновлению счетчика
		DMA1_Channel1->CMAR = (uint32_t)&Tx;												//Предустановка адреса буфера передатчика для DMA CRC
		DMA1_Channel1->CCR |= DMA_CCR1_EN;													//Включение канала DMA CRC
	}
}
