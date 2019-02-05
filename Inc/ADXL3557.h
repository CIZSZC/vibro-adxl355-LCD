
#include "stm32f4xx_hal.h"

#ifndef adxl3557
#define adxl3557

#define POWER_CTL 0x2D
#define POWER_CTL_OFF 0x01
#define POWER_CTL_ON ~POWER_CTL_OFF
#define POWER_CTL_DRDY_OFF 0x04
#define POWER_CTL_DRDY_ON ~POWER_CTL_DRDY_OFF
#define POWER_CTL_TEMP_OFF 0x02
#define POWER_CTL_TEMP_ON ~POWER_CTL_TEMP_OFF
#define POWER_RESET 0x2F

#define RANGE 0x2C
#define RANGE_MASK 0x03
#define RANGE_2G 0x01
#define RANGE_4G 0x02
#define RANGE_8G 0x03

#define RANGE_2G_SCALE 256000.0f
#define RANGE_4G_SCALE 128000.0f
#define RANGE_8G_SCALE 64000.0f
#define SELECTED_RANGE_SCALE RANGE_2G_SCALE

#define HIGHPASS_FILTER_OFF 0b0000
#define HIGHPASS_FILTER_9_88 0b0010
#define HIGHPASS_FILTER_0_62 0b0011
#define HIGHPASS_FILTER_0_15 0b0100
#define HIGHPASS_FILTER_0_03 0b0101
#define HIGHPASS_FILTER_0_009 0b0110

#define LOWPASS_FILTER 0x28
#define LOWPASS_FILTER_MASK 0x0F
#define LOWPASS_FILTER_4000 0b0000
#define LOWPASS_FILTER_2000 0b0001
#define LOWPASS_FILTER_1000 0b0010
#define LOWPASS_FILTER_500 0b0011
#define LOWPASS_FILTER_250 0b0100
#define LOWPASS_FILTER_125 0b0101
#define LOWPASS_FILTER_62_5 0b0110
#define LOWPASS_FILTER_31_25 0b0111
#define LOWPASS_FILTER_15_625 0b1000
#define LOWPASS_FILTER_7_813 0b1001
#define LOWPASS_FILTER_3_906 0b1010

#define XDATA3 0x08
#define XDATA2 0x09
#define XDATA1 0x0A
#define YDATA3 0x0B
#define YDATA2 0x0C
#define YDATA1 0x0D
#define ZDATA3 0x0E
#define ZDATA2 0x0F
#define ZDATA1 0x10

#define TEMP2 0x06
#define TEMP1 0x07
#define FIFO_ENTRIES 0x05
#define FIFO_DATA 0x11
#define TEMP_START TEMP2
#define TEMP_LENGTH 2

#define AXIS_START XDATA3
#define AXIS_LENGTH 9

#define STATUS 0x04
#define STATUS_MASK_DATARDY 0x01
#define STATUS_MASK_NVMBUSY 0x10

#define DEVID_AD                 0x00
#define DEVID_MST                0x01
#define PARTID                   0x02
#define REVID                    0x03
#define STATUS                   0x04

#define ADXL355_WRITE         0x00
#define ADXL355_READ          0x1

#define SCALE_TEMPL 9.05
#define SCALE_TEMPH 19.21

#define CS_ON()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET)
#define CS_OFF()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET)


#define ADXL3557_SPI_PORT		hspi1;
extern SPI_HandleTypeDef ADXL3557_SPI_PORT;
extern uint8_t single[2];
extern uint8_t b[16];

void SPI_Send2(unsigned char adr, unsigned char b0, unsigned char b1);
void SPI_Send1(unsigned char adr, unsigned char b0);
void ADXL355_init(void);
void SPI_Recv_Multi2(int adr, int n);
int32_t Data_Conversion (uint32_t ui32SensorData);
void SPI_Send(unsigned char adr, unsigned char b0);
void SPI_Recv_Single(int adr);
#endif

