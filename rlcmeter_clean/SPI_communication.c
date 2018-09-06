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
	 // ��������� SPI
RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // �������� ������������
SPI1->CR1 = 0x0000; // �������� ������ ���������� �������
SPI1->CR1 |= SPI_CR1_DFF; // ���11 ������ ������ 0-8��� 1-16���
SPI1->CR1 |= SPI_CR1_SSM; // ���9 SSM � �������� �������� ������� NSS (0 � � �������� ������, 1 � ����������);
SPI1->CR1 |= SPI_CR1_SSI; // ���8 SSI � ���� SSM = 1 ��������� �������� NSS;
SPI1->CR1 |= SPI_CR1_LSBFIRST; // ���7 LSBFIRST � ����� ������ �������� (0 - �������, 1 � ������� ������� �����);
SPI1->CR1 |= SPI_CR1_SPE; // ���6 SPE - ������ SPI (1 � ���. 0 � ����.)
//���3-5 BR[2:0]; � �������� �������� ������ fPCLK/x (000:2, 001:4, 010:8, 011:16, 100:32, 101:64, 110:128, 111:256)
SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; // ������ �������� fPCLK/x
SPI1->CR1 |= SPI_CR1_MSTR; // MSTR - ������ ������ �������(1)/�������(0);
SPI1->CR1 |= SPI_CR1_CPOL; // CPOL - ����� ��������� ��������� �������;
SPI1->CR1 |= SPI_CR1_CPHA; // CPHA - ����� ���� ��������� ������� 0-\ 1-/;
SPI1->CR2 = 0x0000; // �������� ������ ���������� �������
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


