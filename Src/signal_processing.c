//#include "signal_processing.h"
#include "ADXL3557.h"
#include "signal_processing.h"


float IIR_filter(float input_sample,char channel) //slo by to predelat na vypocty s pevnou desetinnou carkou, ale zatim neni nouze o vykon
{

static float w1[3],w2[3],w0[3],accumulator;
uint8_t pointer=0;
	
if (channel=='x') pointer=0;
else if (channel=='y') pointer=1;
else if (channel=='z') pointer=2;
		
		accumulator  = w2[pointer] * IIR_a2;
		accumulator += w1[pointer] * IIR_a1;
		accumulator += input_sample ;

		w0[pointer] = accumulator ;
	
		// Run feedforward part of filter
		accumulator  = w0[pointer] * IIR_b0;
		accumulator += w1[pointer] * IIR_b1;
		accumulator += w2[pointer] * IIR_b2;

		w2[pointer] = w1[pointer];		// Shuffle history buffer
		w1[pointer] = w0[pointer];

		// Write output
		return accumulator;
}

void gravity_compenzation(int32_t Axreal,int32_t Ayreal,int32_t Azreal)
{
//float pitch, roll, beta;
//	pitch=atanf(Axreal/__sqrtf(powf(Ayreal,2)+powf(Azreal,2)));
//	roll=atanf(Ayreal/__sqrtf(powf(Axreal,2)+powf(Azreal,2)));
//	beta=atanf(Azreal/__sqrtf(powf(Ayreal,2)+powf(Axreal,2)));
//	
//	Axcomp=(float)(Axreal/RANGE_2G_SCALE)-sinf(pitch);
//	Aycomp=(float)(Ayreal/RANGE_2G_SCALE)-sinf(roll);
//	Azcomp=(float)(Azreal/RANGE_2G_SCALE)-sinf(beta);
	
	Axcomp=(float)(Axreal/SELECTED_RANGE_SCALE);
	Aycomp=(float)(Ayreal/SELECTED_RANGE_SCALE);
	Azcomp=(float)(Azreal/SELECTED_RANGE_SCALE);
	
	Axcomp=IIR_filter(Axcomp,'x');
	Aycomp=IIR_filter(Aycomp,'y');
	Azcomp=IIR_filter(Azcomp,'z');
	
//	sprintf(buffer,"%.1f,%.1f,%.1f\n\r",pitch*M_RADIAN,roll*M_RADIAN,beta*M_RADIAN);
//	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
	
//	sprintf(buffer,"%.1f,%.1f\n\r",Axreal/RANGE_2G_SCALE,Axcomp);
//	HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
//	HAL_Delay(500);
}

void G_to_MS(void)
{


Axcomp*=ACC_CONSTANT;
Aycomp*=ACC_CONSTANT;
Azcomp*=ACC_CONSTANT;

}



void RPM_calculation(float x, bool enable_FFT)
{
	 arm_cfft_radix4_instance_f32 S;
	 arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
static bool reset=true;
	static float buffer[SAMPLES];
	float bufferOutput[SAMPLES];
  static uint16_t stack_full=0;
  float32_t maxValue;                /* Max FFT value is stored here */
  uint32_t maxIndex;                /* Index in Output array where max value is */

//	sig0 = (Comp *)malloc(sizeof(Comp) * (size_t)FFT_SIZE);
//	f = (Comp *)malloc(sizeof(Comp) * (size_t)FFT_SIZE);

//if(reset)
//	 for (uint16_t i = 0; i < FFT_SIZE; i++)
//	    {
//			sig[i].a = 0;
//	        sig[i].b = 0;
//		 }
//reset= false;
//sig[stack_full].a=(double)x/100;

  for(uint16_t i=0;i<SAMPLES;i+=2)
  {
	  buffer[i]=x;
	  buffer[i+1]=0;
  }

//	    for (uint16_t i = 0; i < FFT_SIZE; i++)
//	    comp_print(sig[i]);

	//if ((FFT_SIZE-1)>stack_full) stack_full++;
	//else
	//{
		//sprintf(buffer,"ZACATEK\n\r");
		//HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
		//reset=true;
		//stack_full=0;
		//for (uint16_t i = 0; i < FFT_SIZE-1; i++)
	    //comp_print(sig[i]);

		//test_fft(sig, f, sig0, FFT_SIZE);
	//}

	arm_cfft_radix4_f32(&S, buffer);
	arm_cmplx_mag_f32(buffer, bufferOutput, FFT_SIZE);
	arm_max_f32(bufferOutput, FFT_SIZE, &maxValue, &maxIndex);

	//sprintf(buffer,"%d\n",(uint16_t)maxValue);
	//HAL_UART_Transmit(&huart1,(uint8_t *)buffer,strlen(buffer), 1);
	//if(enable_FFT) test_fft(sig, f, sig0, FFT_SIZE);
}
