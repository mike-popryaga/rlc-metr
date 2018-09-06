/**
 * @defgroup: RLC Meter V6
 * @file:     SPI_communication.c
 *
 * @brief:    Module for SPI communication setup
 *            Hardware RLC meter for STM32F100
 *
 * @data:     August, 2013
 * @author:   Neekeetos
 *
 */

///=============================================================================
/// Includes
///=============================================================================


#include "SPI_communication.h"
#include "main.h"
#include "uart.h"



//void begin_SPI(int);

void setup_SPI(void) {

	SPI_InitTypeDef spi;
	GPIO_InitTypeDef port;

	SPI_StructInit(&spi);

	/*
	 *
	 // Ќастройка SPI
RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // ¬ключить тактирование
SPI1->CR1 = 0x0000; // ќчистить первый управл¤ющий регистр
SPI1->CR1 |= SPI_CR1_DFF; // Ѕит11 ‘ормат данных 0-8бит 1-16бит
SPI1->CR1 |= SPI_CR1_SSM; // Ѕит9 SSM Ц выбирает источник сигнала NSS (0 Ч с внешнего вывода, 1 Ч программно);
SPI1->CR1 |= SPI_CR1_SSI; // Ѕит8 SSI Ц если SSM = 1 определ¤ет значение NSS;
SPI1->CR1 |= SPI_CR1_LSBFIRST; // Ѕит7 LSBFIRST Ц задаЄт способ передачи (0 - старшим, 1 Ч младшим разр¤дом вперЄд);
SPI1->CR1 |= SPI_CR1_SPE; // Ѕит6 SPE - работа SPI (1 Ц вкл. 0 Ц откл.)
//Ѕит3-5 BR[2:0]; Ч делитель скорости обмена fPCLK/x (000:2, 001:4, 010:8, 011:16, 100:32, 101:64, 110:128, 111:256)
SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; // «адать скорость fPCLK/x
SPI1->CR1 |= SPI_CR1_MSTR; // MSTR - делает модуль ведущим(1)/ведомым(0);
SPI1->CR1 |= SPI_CR1_CPOL; // CPOL - задаЄт пол¤рность тактового сигнала;
SPI1->CR1 |= SPI_CR1_CPHA; // CPHA - задаЄт фазу тактового сигнала 0-\ 1-/;
SPI1->CR2 = 0x0000; // ќчистить второй управл¤ющий регистр
	 */
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_2Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &spi);
	SPI1->CR2 = 0x0000;
	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = LCD_SCK_PIN | LCD_SDA_PIN;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOB, &port);
    #ifdef DEBUG_PRINT
    uart_tx("\r\nSPI has been initialized", 1);
    #endif
	begin_SPI(1);
}

void begin_SPI(int test_var) {
	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_SendData(SPI1, test_var);
}


