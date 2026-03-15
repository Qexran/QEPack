

#ifndef _TEST_H_
#define _TEST_H_



#define LED1_ON()       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED1_OFF()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#define LED_ON                  GPIO_PIN_RESET
#define LED_OFF                 GPIO_PIN_SET
#define LED_UPDATE(LED_STATUS)  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LED_STATUS)


void vLedInit(void);
void vKeyInit(void);
void vUartInit(void);
void vOLEDInit(void);
void vUtrasonicInit(void);
void vUltrasonicInit(void);
void ServoDeviceInit(void);
void vAdcInit(void);
void vHMC5883Init(void);
#endif

