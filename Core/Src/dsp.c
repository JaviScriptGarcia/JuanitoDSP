// *****************************************************************************
// File: dsp.c
// Description: Source code of parametrizable filters
// *****************************************************************************

// *****************************************************************************
// Includes

#include <stdio.h>
#include <stdint.h>
// #include <arm_math.h>
#include <math.h>
#include "dsp.h"

// *****************************************************************************
// Defines 

#define BLOCK_SIZE            32
#define NUM_TAPS              29


// *****************************************************************************
// Variables 

//FIR struct init
// static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

// FIR Coefficients buffer generated using fir1() MATLAB function. fir1(28, 6/24)
// const float32_t firCoeffs32[NUM_TAPS] = {
//   -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
//   -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
//   +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
//   +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
// };

// const int16_t triangle_wave[] ={
// 		1024, 0, 2046, 0, 3072, 0, 4096, 0, 5120, 0, 6144, 0, 7168, 0, 8192, 0,
// 		9216, 0, 10240, 0, 11264, 0, 12288, 0, 13312, 0, 14336, 0, 15360, 0, 16384, 0,
// 		17408, 0, 18432, 0, 19456, 0, 20480, 0, 21504, 0, 22528, 0, 23552, 0, 24576, 0,
// 		25600, 0, 26624, 0, 27648, 0, 28672, 0, 29696, 0, 30720, 0, 31744, 0, 32767, 0,
// 		31744, 0, 30720, 0, 29696, 0, 28672, 0, 27648, 0, 26624, 0, 25600, 0, 24576, 0,
// 		23552, 0, 22528, 0, 21504, 0, 20480, 0, 19456, 0, 18432, 0, 17408, 0, 16384, 0,
// 		15360, 0, 14336, 0, 13312, 0, 12288, 0, 11264, 0, 10240, 0, 9216, 0, 8192, 0,
// 		7168, 0, 6144, 0, 5120, 0, 4096, 0, 3072, 0, 2046, 0, 1024, 0, 0, 0, -1024, 0, -2048, 0, -3072, 0, -4096, 0, -5120, 0, -6144, 0, -7168, 0,
// 			-8192, 0, -9216, 0, -10240, 0, -11264, 0, -12288, 0, -13312, 0, -14336, 0, -15360, 0,
// 			-16384, 0, -17408, 0, -18432, 0, -19456, 0, -20480, 0, -21504, 0, -22528, 0, -23552, 0,
// 			-24576, 0, -25600, 0, -26624, 0, -27648, 0, -28672, 0, -29696, 0, -30720, 0, -31744, 0,
// 			-32768, 0, -31744, 0, -30720, 0, -29696, 0, -28672, 0, -27648, 0, -26624, 0, -25600, 0,
// 			-24576, 0, -23552, 0, -22528, 0, -21504, 0, -20480, 0, -19456, 0, -18432, 0, -17408, 0,
// 			-16384, 0, -15360, 0, -14336, 0, -13312, 0, -12288, 0, -11264, 0, -10240, 0, -9216, 0,
// 			-8192, 0, -7168, 0, -6144, 0, -5120, 0, -4096, 0, -3072, 0, -2046, 0, -1024
// };

// *****************************************************************************
// Functions 

// *****************************************************************************
tErrorCode DSP_SecondOrderIIR(int16_t *buf, uint16_t nSamples, 
                             tInstanceIIR *inst, tCoeffsIIR coeff)
// *****************************************************************************
// Description: Basic second order filter for left channel. Filter
// structure is direct form II
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *instance: pointer to structure storing feedfoward and feedback samples
//   coeffs: structure containing filter coefficients
//   Returns: output sample
// *****************************************************************************
{
	uint16_t i = 0; // Index used in loop
    int32_t t;      // Temporal variable
	int32_t y;      // Work with 32 bits to prevent overflow

	if ((NULL == buf) || (NULL == inst)) return RES_ERROR;

	for (i = 0; i < nSamples; i++)
	{
      t = buf[i] - coeff.a1 * (*inst).t1 - coeff.a2 * (*inst).t2;
	  // Difference equation
	  y = t * coeff.b0 + coeff.b1 * (*inst).t1 + 
	  coeff.b2 * (*inst).t2;

      buf[i] = (int16_t)y;
      // Update feedback and feedfoward components
	  (*inst).t2 = (*inst).t1;
	  (*inst).t1 = t;
	}

	return RES_OK;
}



// *****************************************************************************
tErrorCode DSP_UpdateFilterInstances(tParamConfig pCfg, tCoeffsIIR *coeff)
// *****************************************************************************
// Description: Updates the filter coefficients with the new parameter config
// received from master
// Parameters: 
// Returns: 
// *****************************************************************************
{
	// Calculate filter coefficients based on the selected filter form and filter parameters (fcutoff etc.)
	return RES_OK;
}

// *****************************************************************************
void DSP_TestFilterInstances(tCoeffsIIR *coeffs)
// *****************************************************************************
// Description: FIlter coefficients to a test value
// received from master
// Parameters: 
// Returns: 
// *****************************************************************************
{
  (*coeffs).b0 =   0.003916123487156427;
  (*coeffs).b1 =  0.007832246974312854;
  (*coeffs).b2 =  0.003916123487156427;
  (*coeffs).a1 =  -1.8153396116625289;
  (*coeffs).a2 =  0.8310041056111546;
}


// *****************************************************************************
void DSP_Init(tInstanceIIR *inst, tCoeffsIIR *coeffs)
// *****************************************************************************
// Description: Initializes both coefficients and instances values with 0
// Parameters: none
// Returns: nothing
// *****************************************************************************
{
	uint8_t i = 0;
	
	for (i = 0; i<MAX_FILTERS; i++)
	{
      inst[i].t1 = 0;
      inst[i].t2 = 0;

	  coeffs[i].b0 = 0;
      coeffs[i].b1 = 0;
      coeffs[i].b2 = 0;
      coeffs[i].a1 = 0;
	  coeffs[i].a2 = 0;
	}
	
}



