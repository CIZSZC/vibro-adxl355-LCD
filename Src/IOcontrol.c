#include "IOcontrol.h"
#include "SSD1306.h"
#include <string.h>

void ADXL_init(void)
{
char buffer[100];
uint8_t pocitadlo=0;
sprintf(buffer,"Inicializace!\n");
  HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);


  while(b[0]!=0xAD)
	{
  ADXL355_init();
		
	LCD_print("INIT ADXL",Font_11x18,White,23,23,true,true);
		
	HAL_Delay(100);
	SPI_Recv_Multi2(DEVID_AD,4);
  sprintf(buffer,"DEVID_AD:0x%02X\n",b[0]);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
  sprintf(buffer,"DEVID_MST:0x%02X\n",b[1]);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
  sprintf(buffer,"PARTID:0x%02X\n",b[2]);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
  sprintf(buffer,"REVID:0x%02X\n",b[3]);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
	pocitadlo++;
	if (pocitadlo>10)
	{
	  LCD_print("FAIL",Font_11x18,White,50,45,false,true);
		while(1)
		{}
	}
	}
	
  LCD_print("INIT OK",Font_11x18,White,23,23,true,true);
	
  sprintf(buffer,"Inicializace OK!\n");
  HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);

	SPI_Recv_Single(FIFO_ENTRIES);
	sprintf(buffer,"zasobnik:%d\n",(single[0])&0x0F);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
	
	SPI_Recv_Single(LOWPASS_FILTER);
	sprintf(buffer,"filterrr:%d\n",single[0]);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
	
	
}
