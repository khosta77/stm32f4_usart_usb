#include "../system/include/cmsis/stm32f4xx.h"

#if 0
#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200

#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
    // 0. Вкл. тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // 1. Назначили пины на альтернативный режим работы
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    // 2. Альтернативные функции
    GPIOA->AFR[0] |= (0x77 << 8);
    GPIOA->AFR[1] |= (0x77 << 4);

    // Включаем светодиоды
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0);
}

void USART_init() {
    //// включаем USART2 PA2, PA3
    // 0. Вкл тактирование
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // 1. Задали частоту работы
    USART2->BRR = MYBRR;
    // 2. Настроили на чтение и запись
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    // 3. Установили STOP бит
    USART2->CR2 |= (USART_CR2_STOP_1);
    // 4. Включили DMA для USART
    USART2->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    // 5. Вкл USART
    USART2->CR1 |= USART_CR1_UE;

    //// Включаем USART1 PA9, PA10
    // Аналогично, как и для USART2, только USART2->USART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = MYBRR;
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    USART1->CR2 |= (USART_CR2_STOP_1);
    USART1->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    USART1->CR1 |= USART_CR1_UE;
}

#define SIZE 20

uint8_t usart1_receive_buffer[(2 * SIZE)];
uint8_t usart2_receive_buffer[(2 * SIZE)];
uint8_t mrk = 0x00;

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        GPIOD->ODR ^= GPIO_ODR_OD12;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        mrk = 0xA0;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
}


void DMA2_Stream2_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF2) == DMA_LISR_TCIF2) {
        GPIOD->ODR ^= GPIO_ODR_OD13;
        //DMA1_Stream6->CR |= DMA_SxCR_EN;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        while ((DMA2_Stream2->CR) & DMA_SxCR_EN){;}
        mrk = 0x0A;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
    }
}

void DMA_init() {
    // DMA1_Stream6 это USART2_TX
    // DMA2_Stream2 это USART1_RX

    // 0. Включили тактирование DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // 1. Выбираем канал
    DMA1_Stream6->CR |= (0x4 << 25);  // Канал 4
    DMA2_Stream2->CR |= (0x4 << 25);  // Канал 4
    
    // 2. Устанавливаем размер ячейки (8-бит(можно 16-бит, 32-бит))
    DMA1_Stream6->CR &= ~DMA_SxCR_MSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE;

    // 3. Установите размер периферийных данных равным 8 битам
    DMA1_Stream6->CR &= ~DMA_SxCR_PSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE;

    // 4. Установите режим увеличения объема памяти
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA2_Stream2->CR |= DMA_SxCR_MINC;

    // 5. Включаем режим работы
    DMA1_Stream6->CR |= (0x01<<6);  // Из памяти в перефирию
    DMA2_Stream2->CR &= ~(3UL<<6);  // Из переферии в память

    // 6. Включить циклическую запись
    //DMA1_Stream6->CR |= DMA_SxCR_CIRC;
    DMA2_Stream2->CR |= DMA_SxCR_CIRC;

    // 7. Включаем прерывания
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;
    DMA2_Stream2->CR |= DMA_SxCR_TCIE;

    // 8. Количество элементов данных, подлежащих передаче	
    DMA1_Stream6->NDTR = SIZE;
    DMA2_Stream2->NDTR = SIZE;

    // 9. Задаем адрес переферии
    DMA1_Stream6->PAR = (uint32_t)(&USART2->DR);
    DMA2_Stream2->PAR = (uint32_t)(&USART1->DR);

    // 10. Задаем адрес памяти
    DMA1_Stream6->M0AR = (uint32_t)&usart2_receive_buffer[0];
    DMA2_Stream2->M0AR = (uint32_t)&usart1_receive_buffer[0];

    // 11. Настройка прерываний
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 5);

    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream2_IRQn, 4);

    // 12. Включаем DMA
    //DMA1_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
    // Включение DMA происходит только после того, как мы инициализировали GPIO и USART
    // И не надо его выключать и включать после, он уже работает в цикле.
}

void USART_DMA_init() {
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
}

void write_dma(uint8_t *df, uint16_t size) {
    for (uint16_t i = 0; i < size; i++)
        usart2_receive_buffer[i] = df[i];
}

void read_dma(uint8_t *df, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *(df + i) = *(usart1_receive_buffer + i);
    }
}

int main(void) {
    SystemCoreClockUpdate();
    uint8_t temp[(2 * SIZE)];
    for (int i = 0; i < (2 * SIZE); i++) {
        temp[i] = i;
    }
    write_dma(&temp[0], (2 * SIZE));

    GPIO_init();
    USART_init();
    DMA_init();
//    DMA1_Stream6->CR |= DMA_SxCR_EN;
//    DMA2_Stream2->CR |= DMA_SxCR_EN;

    // инициализируем массив который будет отдан на запись
    //uint8_t temp[(2 * SIZE)];
    //for (int i = 0; i < (2 * SIZE); i++) {
      //  temp[i] = (i % 254);
    //}
    uint8_t temp2[(2 * SIZE)];
    while(1) {
        if (mrk == 0x0A) {
            for (uint16_t i = 0; i < SIZE; i++)
                usart2_receive_buffer[i] = (usart1_receive_buffer[i] + 1);

            mrk = 0x00;
            //DMA2_Stream2->CR |= DMA_SxCR_EN;
            DMA1_Stream6->CR |= DMA_SxCR_EN;
        }
        if (mrk == 0xA0) {
            DMA2_Stream2->CR |= DMA_SxCR_EN;
            mrk = 0x00;
        }
        //write_dma(&temp[0], SIZE);

        //for (int i = 0; i < 10000; i++);
        //read_dma(&temp2[0], SIZE);
    }
}
#endif


#if 1
#define SIZE 20
#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200
#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0] |= (0x77 << 8);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void USART_init() {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = MYBRR;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    USART2->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    USART2->CR1 |= USART_CR1_UE;
}

uint8_t usart2_mrk = 0x00;
uint8_t usart2_rx_array[SIZE];
uint8_t usart2_tx_array[SIZE];


void DMA1_Stream6_IRQHandler(void) {  // TX
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        //GPIOD->ODR ^= GPIO_ODR_OD12;
        usart2_mrk = 0xA0;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
}

void DMA1_Stream5_IRQHandler(void) {  // RX
    if ((DMA1->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5) {
        //GPIOD->ODR ^= GPIO_ODR_OD13;
        usart2_mrk = 0x0A;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
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
    DMA1_Stream5->CR |= ((0x4 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_CIRC);

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

void USART2_init() {
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
}

void init() {
    USART2_init();
}

void if_message_in() {
    if (usart2_mrk == 0x0A) {
        GPIOD->ODR |= GPIO_ODR_OD12;
        for (uint16_t i = 0; i < SIZE; i++)
            usart2_tx_array[i] = (usart2_rx_array[i] + 1);
        usart2_mrk = 0x00;
        DMA1_Stream6->CR |= DMA_SxCR_EN;
        GPIOD->ODR &= ~GPIO_ODR_OD12;
    } 
}

void if_message_out() {
    if (usart2_mrk == 0xA0) {
        GPIOD->ODR |= GPIO_ODR_OD13;
        usart2_mrk = 0x00;
        DMA1_Stream5->CR |= DMA_SxCR_EN;
        GPIOD->ODR &= ~GPIO_ODR_OD13;
    }
}

void loop() {
    if_message_in();
    if_message_out();
}

int main(void) {
    init();
    while(1) {
        loop(); 
    }
}

#endif
