
#include "stm32f4xx_hal.h"
#include "ADXL3557.h"

#ifndef iocontrol
#define iocontrol

void ADXL_init(void);

#define UART_PORT		huart1;
extern UART_HandleTypeDef UART_PORT;

#endif
