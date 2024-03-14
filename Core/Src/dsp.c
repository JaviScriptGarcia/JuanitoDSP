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
tErrorCode DSP_SecondOrderLP(int16_t *buf, uint16_t nSamples, 
                             tInstanceIIR *inst, tCoeffsIIR coeff)
// *****************************************************************************
// Description: Basic second order lowpass filter for left channel. Filter
// structure is direct form II
// Parameters: 
//   *buf: pointer to the buffer containing data to be proccessed
//   nSamples: number of samples to proccess
//   *instance: pointer to structure storing feedfoward and feedback samples
//   coeffs: structure containing filter coefficients
//   Returns: output sample
// *****************************************************************************
{
	uint16_t i = 0; // Index used in loop
	int16_t x; // backup for the current input sample

	if ((NULL == buf) || (NULL == inst)) return RES_ERROR;

	for (i = 0; i < nSamples; i++)
	{
	  // Copy the value of the current input sample
	  x = buf[i];

	  // Difference equation
	  buf[i] = (*inst).x1 * coeff.cx1 + (*inst).x2 * coeff.cx2 + 
	  (*inst).y1 * coeff.cy1 - (*inst).y2 * coeff.cy2;

      // Update feedback and feedfoward components
	  (*inst).x2 = (*inst).x1;
	  (*inst).x1 = x;
	  (*inst).y2 = (*inst).y1;
	  (*inst).y1 = buf[i];
	}

	// ly =  lu1 * 0.0003964 + lu2 * 0.0003911 + ly1 * 1.96 - ly2 * 0.9607;
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
  (*coeffs).cx1 = 0.0003964;
  (*coeffs).cx2 = 0.0003911;
  (*coeffs).cy1 = 1.96;
  (*coeffs).cy2 = 0.9607;
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
      inst[i].x1 = 0;
      inst[i].x2 = 0;
      inst[i].y1 = 0;
      inst[i].y2 = 0;

	  coeffs[i].cx1 = 0;
      coeffs[i].cx2 = 0;
      coeffs[i].cy1 = 0;
      coeffs[i].cy2 = 0;
	}
	
}



