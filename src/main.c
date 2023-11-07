#include "../system/include/cmsis/stm32f4xx.h"
#include <string.h>

#define SIZE 20
#define SIZE_CMD 2
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
    DMA1_Stream5->CR |= ((0x4 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_CIRC);

    // 2. Устанавливаем размер ячейки 8-бит и периферийных данных 8-бит
    DMA1_Stream6->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
    DMA1_Stream5->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);

    // 3. Включаем режим работы
    DMA1_Stream6->CR |= (0x01<<6);  // Из памяти в перефирию
    DMA1_Stream5->CR &= ~(3UL<<6);  // Из переферии в память

    // 4. Количество элементов данных, подлежащих передаче
    DMA1_Stream6->NDTR = SIZE_CMD;
    DMA1_Stream5->NDTR = SIZE_CMD;

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

void cmd_check() {
    if (usart2_rx_array[0] != usart2_rx_array[1]) {  // Если команды не одинаковые - произошла ошибка чтения
        GPIOD->ODR |= (GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
        while(1){;}
    }
}

void nop() {
    GPIOD->ODR |= GPIO_ODR_OD12;
    for (uint32_t t = 0; t < 0xFFFFF; t++);
    GPIOD->ODR &= ~GPIO_ODR_OD12;
}

void send_display_info() {
    char disinfo[] = "ST7735 128x160 RGB565;ILI9341-240x320-RGB565";
    memcpy(&usart2_tx_array[0], disinfo, strlen(disinfo) + 1);
    usart2_mrk = 0x00;
    DMA1_Stream6->NDTR = 128;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void send_display_range() {
    uint16_t x = 240;
    uint16_t y = 360;
    uint8_t range[4] = { (uint8_t)((x >> 8) & 0xFF), (uint8_t)(x & 0xFF),
                         (uint8_t)((y >> 8) & 0xFF), (uint8_t)(y & 0xFF)};
    usart2_tx_array[0] = range[1];
    usart2_tx_array[1] = range[0];
    usart2_tx_array[2] = range[3];
    usart2_tx_array[3] = range[2];

    usart2_mrk = 0x00;
    DMA1_Stream6->NDTR = 4;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void print_h_line() {
    GPIOD->ODR |= GPIO_ODR_OD12;
    for (uint32_t t = 0; t < 0xFFFFF; t++);
    GPIOD->ODR &= ~GPIO_ODR_OD12;

    usart2_mrk = 0x00;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

void message_in() {
    cmd_check();
    uint8_t cmd = usart2_rx_array[0];
    switch (cmd) {
        case 0x00:
            nop();
            break;
        case 0x01:
            send_display_info();
            break;
        case 0x02:
            send_display_range();
            break;
        case 0x03:
            print_h_line();
            break;
        default: {
            GPIOD->ODR |= (GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
            for (uint32_t t = 0; t < 0xFFFFF; t++);
            GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
            usart2_mrk = 0x00;
            DMA1_Stream5->CR |= DMA_SxCR_EN;
        }
    }
    
  //  for (uint16_t i = 0; i < SIZE_CMD; i++)
    //    usart2_tx_array[i] = cmd;
//    usart2_mrk = 0x00;
//    DMA1_Stream5->CR |= DMA_SxCR_EN;
//    DMA1_Stream6->CR |= DMA_SxCR_EN;
//    GPIOD->ODR &= ~GPIO_ODR_OD12; 
}

void message_out() {
    //GPIOD->ODR |= GPIO_ODR_OD13;
    usart2_mrk = 0x00;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
    //GPIOD->ODR &= ~GPIO_ODR_OD13;
}

void loop() {
    if (usart2_mrk == 0x0A)
        message_in();
    if (usart2_mrk == 0xA0)
        message_out();
}

int main(void) {
    init();
    while(1) {
        loop(); 
    }
}


