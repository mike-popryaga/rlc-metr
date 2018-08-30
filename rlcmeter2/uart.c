#include "uart.h"
#include "stm32f10x.h"
//-------------------------------------------------------------------
void initUART()
{
	USART1->CR1 |= USART_CR1_UE; //�������� USART1
	USART1->CR1 &= ~USART_CR1_M; //����� ����� - 8 ���
	USART1->CR2 &= ~USART_CR2_STOP; //1 ����-���
	USART1->BRR = 278;//@32//139;//115200 @160000 //104; //115200 @ 12M
	USART1->CR1 |= USART_CR1_TE; //��������� �������� ������
//	USART1->CR1 |= USART_CR1_RE; //��������� ����� ������
}
//-------------------------------------------------------------------
void uart_tx(const char* str, int wait)
{
	while(*str != 0 )
	{
	uartSendByte(*str++);
	}

}
//-------------------------------------------------------------------
void uartSendByte(uint8_t c) {
	while(!(USART1->SR & USART_SR_TC));
	  USART1->DR = c;

}
//-------------------------------------------------------------------
