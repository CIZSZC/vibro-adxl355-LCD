#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#ifndef fft_radix2
#define fft_radix2

typedef struct Comp {
    /* comp of the form: a + bi */
    double a, b;
} Comp;

#define UART_PORT		huart1;
extern UART_HandleTypeDef UART_PORT;

Comp comp_create(double a, double b);
Comp comp_add(Comp c1, Comp c2);
Comp comp_sub(Comp c1, Comp c2);
Comp comp_mul(Comp c1, Comp c2);
void comp_print(Comp comp);

#define UART_PORT		huart1;
extern UART_HandleTypeDef UART_PORT;

/* const double PI = acos(-1); */
#define PI 3.141592653589793
#define SQR(x) ((x) * (x))

Comp comp_euler(double x);
#define comp_mul_self(c, c2);
void dft(const Comp *sig, Comp *f, int n, int inv);
void fft(const Comp *sig, Comp *f, int s, int n, int inv);
void print_result(const Comp *sig, const Comp *sig0, int n);
void test_dft(const Comp *sig, Comp *f, Comp *sig0, int n);
void test_fft(const Comp *sig, Comp *f, Comp *sig0, int n);



#endif
