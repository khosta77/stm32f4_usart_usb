#include "../system/include/cmsis/stm32f4xx.h"

#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200
#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
    // 0. Вкл. тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // 1. Назначили пины на альтернативный режим работы
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    // 2. Альтернативные функции
    GPIOA->AFR[0] |= (0x77 << 8);

    // Включаем светодиоды
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void USART_init() {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = MYBRR;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART2->CR2 |= (USART_CR2_STOP_1);
    USART2->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    USART2->CR1 |= USART_CR1_UE;
}

#define SIZE 10

uint8_t usart2_rx_array[SIZE];
uint8_t usart2_tx_array[SIZE];

uint8_t usart2_tx_status = 0x00;
uint8_t usart2_rx_status = 0xFF;

uint8_t usart2_mrk_rx = 0x00;
uint8_t usart2_mrk_tx = 0x00;

void DMA1_Stream6_IRQHandler(void) {  // TX
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        GPIOD->ODR ^= GPIO_ODR_OD12;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        usart2_tx_status = 0x00;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
}

void DMA1_Stream5_IRQHandler(void) {  // RX
    if ((DMA1->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5) {
        GPIOD->ODR ^= GPIO_ODR_OD13;
        usart2_rx_status = 0x00;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
        DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
    }
}

void DMA_init() {
    // 0. Включили тактирование DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream6->CR) & DMA_SxCR_EN){;}
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}

    // 1. Задание настройки
    // - (0x4 << 25) - 4-ый канал
    // - DMA_SxCR_MINC - увеличенный объем памяти
    // - DMA_SxCR_TCIE - прерывания по приему/передачи
    // - DMA_SxCR_CIRC (for rx) - циклическая работа
    DMA1_Stream6->CR |= ((0x4 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE);
    DMA1_Stream5->CR |= ((0x4 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE);

    // 2. Устанавливаем размер ячейки 8-бит и периферийных данных 8-бит
    DMA1_Stream6->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
    DMA1_Stream5->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);

    // 3. Включаем режим работы
    DMA1_Stream6->CR |= (0x01<<6);  // Из памяти в перефирию
    DMA1_Stream5->CR &= ~(3UL<<6);  // Из переферии в память

    // 4. Количество элементов данных, подлежащих передаче
    DMA1_Stream6->NDTR = SIZE;
    DMA1_Stream5->NDTR = SIZE;

    // 5. Задаем адрес переферии
    DMA1_Stream6->PAR = (uint32_t)(&USART2->DR);
    DMA1_Stream5->PAR = (uint32_t)(&USART2->DR);

    // 6. Задаем адрес памяти
    DMA1_Stream6->M0AR = (uint32_t)&usart2_tx_array[0];
    DMA1_Stream5->M0AR = (uint32_t)&usart2_rx_array[0];

    // 7. Настройка прерываний
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 5);

    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_SetPriority(DMA1_Stream5_IRQn, 4);

    // 8. Включаем DMA на прием данных
    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

void USART_DMA_init() {
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
}

void usart_read() {
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
    usart2_rx_status = 0xFF;
    DMA1_Stream5->NDTR = SIZE;
    DMA1_Stream5->CR |= DMA_SxCR_MINC;
    DMA1_Stream5->CR |= DMA_SxCR_EN; 
}

void usart_write() {
    while (usart2_tx_status != 0x00) {;}

    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream6->CR) & DMA_SxCR_EN){;}
    
    usart2_tx_status = 0xFF;
    DMA1_Stream6->NDTR = SIZE;
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void pull() {
    uint8_t buffer[SIZE];
    for (int i = 0; i < SIZE; i++)
        buffer[i] = (1 + usart2_rx_array[i]);

    for (int i = 0; i < SIZE; i++)
        usart2_tx_array[i] = buffer[i];
}

int main(void) {
    for (int i = 0; i < SIZE; i++)
        usart2_tx_array[i] = 0;
    for (int i = 0; i < SIZE; i++)
        usart2_rx_array[i] = 0;
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
    while(1) {
        if ((usart2_rx_status == 0x00) && (usart2_tx_status == 0x00)) {
            GPIOD->ODR |= GPIO_ODR_OD14;

            pull();

            usart_write();
            usart_read();
            GPIOD->ODR &= ~GPIO_ODR_OD14;
        }
    }
}


