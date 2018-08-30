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

#define CS_Pin GPIO_Pin_3
#define CS_Pin_Port GPIOA

#define MOSI_Pin GPIO_Pin_5
#define MOSI_Pin_Port GPIOA

#define SCK_Pin  GPIO_Pin_6
#define SCK_Pin_Port GPIOA



//begin_SPI();


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
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &spi);


	GPIO_StructInit(&port);
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = SCK_Pin | MOSI_Pin | CS_Pin;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &port);

    port.GPIO_Mode = GPIO_Mode_IPD;
    port.GPIO_Pin = GPIO_Pin_0;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);

    uart_tx("\r\nSPI has been initialized", 1);
	//begin_SPI(1);
}
void begin_SPI(int test_var) {
	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_SendData(SPI1, test_var);
}


