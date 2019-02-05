#include "ADXL3557.h"

void ADXL355_init(void)
{
CS_ON();
HAL_Delay(100);
CS_OFF();
HAL_Delay(100);
SPI_Send1(POWER_RESET, 0x52);

HAL_Delay(100);
SPI_Send1(LOWPASS_FILTER,0x00);//05h: no-highpass, ODR=2kHz
SPI_Send1(RANGE,RANGE_2G);      //01h: kousoku-mode, +/-8g

SPI_Send1(POWER_CTL,0x00);      //00h: measure mode
}

void SPI_Send1(unsigned char adr, unsigned char b0)
{
    uint8_t spibuffer[3];
    spibuffer[0]=(adr<<1);
    spibuffer[1]=b0;
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,2,1);
    CS_ON();
}

void SPI_Send(unsigned char adr, unsigned char b0)
{
    uint8_t spibuffer[3];
    spibuffer[0]=(adr);
    spibuffer[1]=b0;
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,2,1);
    CS_ON();
}

void SPI_Send2(unsigned char adr, unsigned char b0, unsigned char b1)
{
    uint8_t spibuffer[4];
    spibuffer[0]=adr;
    spibuffer[1]=(b1<<4)|b0;
    spibuffer[1]=(spibuffer[1]<<1);
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,2,1);
    CS_ON();
}

void SPI_Recv_Multi2(int adr, int n)
{
    uint8_t spibuffer[2];
     uint8_t spibuffer1[2];
   spibuffer[0]=((adr <<1)|ADXL355_READ);
   spibuffer1[0]=0xAA;
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,1,1);
    HAL_SPI_Transmit(&hspi1,spibuffer1,1,1);
    CS_ON();
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,1,1);
    HAL_SPI_Receive(&hspi1,&b[0],n, 1);
    CS_ON();

}

void SPI_Recv_Single(int adr)
{
    uint8_t spibuffer[2];
    uint8_t spibuffer1[2];
    spibuffer[0]=((adr <<1)|ADXL355_READ);
   spibuffer1[0]=0xAA;
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,1,1);
    HAL_SPI_Transmit(&hspi1,spibuffer1,1,1);
    CS_ON();
    CS_OFF();
    HAL_SPI_Transmit(&hspi1,spibuffer,1,1);
    HAL_SPI_Receive(&hspi1,&single[0],1, 1);
    CS_ON();

}

int32_t Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = (ui32SensorData  >> 4);
   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){

         i32Conversion = (ui32SensorData | 0xFFF00000);

   }
   else{
         i32Conversion = ui32SensorData;
   }

   return i32Conversion;
}

