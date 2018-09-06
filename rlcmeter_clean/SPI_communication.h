#ifndef SPI_H
#define SPI_H
#include "main.h"
/*
#define CS_Pin GPIO_Pin_5
#define CS_Pin_Port GPIOB

#define MOSI_Pin GPIO_Pin_4
#define MOSI_Pin_Port GPIOB

#define SCK_Pin  GPIO_Pin_3
#define SCK_Pin_Port GPIOB
*/
#define LCD_GPIO		GPIOB
#define LCD_RESET_PIN	GPIO_Pin_6
#define LCD_SCK_PIN		GPIO_Pin_3
#define LCD_SDA_PIN		GPIO_Pin_4
#define LCD_CS_PIN		GPIO_Pin_5


void setup_SPI(void);
void begin_SPI(int);


#endif /* SPI_H */
