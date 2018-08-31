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

#define LCD_GPIO		GPIOB
#define LCD_RESET_PIN	GPIO_Pin_6
#define LCD_SCK_PIN		GPIO_Pin_3
#define LCD_SDA_PIN		GPIO_Pin_4
#define LCD_CS_PIN		GPIO_Pin_5

//void begin_SPI(int);

void setup_SPI(void) {

	SPI_InitTypeDef spi;
	GPIO_InitTypeDef port;

	SPI_StructInit(&spi);

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

	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = LCD_SCK_PIN | LCD_SDA_PIN;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &port);
    #ifdef DEBUG_PRINT
    uart_tx("\r\nSPI has been initialized", 1);
    #endif
	begin_SPI(1);
}

void begin_SPI(int test_var) {
	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_SendData(SPI1, test_var);
}


