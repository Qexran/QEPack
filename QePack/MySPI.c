#include "main.h"                  // Device header

void MySPI_W_SS(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, BitValue);
}

void MySPI_W_SCK(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, BitValue);
}

void MySPI_W_MOSI(GPIO_PinState BitValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, BitValue);
}

uint8_t MySPI_R_MISO(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
}

void MySPI_Init(void)
{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_6;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	MySPI_W_SS((GPIO_PinState)1);
	MySPI_W_SCK((GPIO_PinState)0);
}

void MySPI_Start(void)
{
	MySPI_W_SS((GPIO_PinState)0);
}

void MySPI_Stop(void)
{
	MySPI_W_SS((GPIO_PinState)1);
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;
	
	for (i = 0; i < 8; i ++)
	{
		MySPI_W_MOSI((GPIO_PinState)(ByteSend & (0x80 >> i)));
		MySPI_W_SCK((GPIO_PinState)1);
		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}
		MySPI_W_SCK((GPIO_PinState)0);
	}
	
	return ByteReceive;
}
