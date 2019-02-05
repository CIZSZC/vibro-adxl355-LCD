
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "FFT_radix2.h"
#include "SSD1306.h"
#include "fonts.h"
#include "arm_math.h"
#ifndef signal_processing
#define signal_processing

#define IIR_b0 0.9994098235120858
#define IIR_b1 -0.9994098235120858
#define IIR_b2 0
#define IIR_a1 0.9988196470208914
#define IIR_a2 0

#define M_PI 3.14159265358979323846
#define M_RADIAN 57.2957795131
#define ACC_CONSTANT 9806.65

#define SAMPLES                 512            /* 1024 real party and 1024 imaginary parts */
#define FFT_SIZE                SAMPLES / 2 

extern float Axcomp,Aycomp,Azcomp;
struct Comp;
extern Comp *sig, *sig0, *f;
#define UART_PORT		huart1;
extern UART_HandleTypeDef UART_PORT;



float IIR_filter(float input_sample,char channel);
void gravity_compenzation(int32_t Axreal,int32_t Ayreal,int32_t Azreal);
void G_to_MS(void);
void RPM_calculation(float x, bool enable_FFT);
#endif

