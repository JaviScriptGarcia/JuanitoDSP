// *****************************************************************************
// File: dsp.c
// Description: Source code of parametrizable filters
// *****************************************************************************

// *****************************************************************************
// Includes

#include <stdio.h>
#include <stdint.h>
#include <arm_math.h>
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

// *****************************************************************************
// Functions 

// *****************************************************************************
tErrorCode DSP_IIR_f32(int16_t *buf, uint32_t nSamples, 
                             tInstanceIIR *inst)
// *****************************************************************************
// Description: Basic second order filter for left channel. Filter
// structure is direct form II. Float arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *instance: pointer to structure storing coefficients and delayed samples
// Returns: output sample
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == inst)) return RES_ERROR_PARAM;
    float t;        // Temporal variable
    float y;        // Work with 32 bits to prevent overflow
    float *pDelays = &((*inst).delays[0]); // Pointer to state buffer

    for ( ; nSamples > 0; buf++, nSamples--)
    {
      // Transposed form 
      //   y = (*inst).coeffs[0] * buf[i] + (*inst).delays[0];
      //   (*inst).delays[0] = (*inst).coeffs[1] * buf[i] + (*inst).coeffs[3] * y 
      //   + (*inst).delays[1];
      //   (*inst).delays[1] = (*inst).coeffs[2] * buf[i] + (*inst).coeffs[4] * y;
      //   buf[i] = (int16_t)y;

      // Non-transposed form
      t = *buf + (*inst).coeffs[3] * *pDelays + (*inst).coeffs[4] * *(pDelays+1);
      y = t * (*inst).coeffs[0] + (*inst).coeffs[1] * *pDelays + 
	  (*inst).coeffs[2] * *(pDelays+1);
      *buf = (int16_t)y;

      // Update feedback and feedfoward components
      *(pDelays+1) = *pDelays;
      *pDelays = t;
    }

	return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_IIR_f32_arm(int16_t *buf, uint32_t nSamples, 
                            arm_biquad_cascade_df2T_instance_f32 *s)
// *****************************************************************************
// Description: Basic second order filter for left channel. Filter
// structure is direct form II
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *instance: pointer to structure storing coefficients and delayed samples
//   Returns: output sample
// *****************************************************************************
{
	#ifdef USE_LIBRARY
	if ((NULL == buf) || (NULL == s)) return RES_ERROR_PARAM;
    float32_t auxBuf[nSamples];

    arm_q15_to_float((q15_t*)buf, auxBuf, nSamples);
    arm_biquad_cascade_df2T_f32(s, auxBuf, auxBuf, nSamples);
    arm_float_to_q15(auxBuf, (q15_t*)buf, nSamples);
	return RES_OK;
    #else 
	return RES_ERROR_CONFIG;
	#endif
}

// *****************************************************************************
tErrorCode DSP_UpdateFilterInstances(tParamConfig *pCfg, tInstanceIIR *inst, 
                                     arm_biquad_cascade_df2T_instance_f32 *s)
// *****************************************************************************
// Description: Updates the filter coefficients with the new parameter config
// received from master. Adapted to work with CMSIS-DSP filters from 
// Audio-EQ-Cookbook.txt, by Robert Bristow-Johnson
// Parameters: 
//   *pCfg: pointer to the structure containing design parameters
//   *inst: pointer to the structure containing array for coefficients
//   *s: pointer to the structure containing pointers for CMSIS-DSP filters
// Returns: 
// *****************************************************************************
{
  // Calculate filter coefficients based on the selected filter form and filter parameters (fcutoff etc.)
  if ((NULL == pCfg) || (NULL == s) || (NULL == inst)) 
  return RES_ERROR_PARAM;

  uint8_t i; 
  for (i = 0; i < MAX_FILTERS; i++)
  {
    float w0 = 2 * PI * (*pCfg).freq / SAMPLE_RATE;
    double alpha = sin(w0) / (2 * (*pCfg).q);
    float gain =  pow(10.0,(double)((*pCfg).gain/40));
	float a0 = 0;

    switch ((*pCfg).type)
	{

	  case LOWPASS: 
	                   a0 =              1 + alpha;
		(*inst).coeffs[3] =     (2 * cos(w0)) / a0;  // a1
		(*inst).coeffs[4] =     - (1 - alpha) / a0;  // a2
        (*inst).coeffs[0] = ((1 - cos(w0))/2) / a0;  // b0
		(*inst).coeffs[1] =  2 * (*inst).coeffs[0];  // b1
		(*inst).coeffs[2] =      (*inst).coeffs[0];  // b2
		break;

	  case HIGHPASS:
                       a0 =              1 + alpha;
	    (*inst).coeffs[3] =  - (-2 * cos(w0)) / a0;  // a1
	    (*inst).coeffs[4] =     - (1 - alpha) / a0;  // a2
        (*inst).coeffs[0] = ((1 + cos(w0))/2) / a0;  // b0
	    (*inst).coeffs[1] = -2 * (*inst).coeffs[0];  // b1
	    (*inst).coeffs[2] =      (*inst).coeffs[0];  // b2
		break;

	  case PEAK:
	                   a0 =          1 + (alpha / gain);
		(*inst).coeffs[3] =            - (-2 * cos(w0)) / a0;  // a1
		(*inst).coeffs[4] = - (1 - (alpha / gain)) / a0;  // a2
        (*inst).coeffs[0] =   (1 + (alpha * gain)) / a0;  // b0
		(*inst).coeffs[1] =              - (*inst).coeffs[3];  // b1
		(*inst).coeffs[2] =   (1 - (alpha * gain)) / a0;  // b2
        break;

	  case LOWSHELF:
	                   a0 = (1 + gain) + (gain - 1) * cos(w0) + 
	                        2 * alpha * sqrt(gain);
		(*inst).coeffs[3] = 2 * ((gain - 1) + (gain + 1) * cos(w0)) / a0;
		(*inst).coeffs[4] = -((1 + gain) + (gain - 1) * cos(w0) - 
				            2 * alpha * sqrt(gain)) / a0;        
        (*inst).coeffs[0] = gain * ((gain + 1) - (gain - 1) * cos(w0) +
		                    2 * sqrt(gain) * alpha) / a0;
		(*inst).coeffs[1] = 2 * gain * ((gain - 1) - (gain + 1) * 
		                    cos(w0)) / a0;
		(*inst).coeffs[2] = gain * ((gain + 1) - (gain - 1) * cos(w0) -
		                    2 * sqrt(gain) * alpha) / a0;
        break;

	  case HIGHSHELF:
	  	               a0 = (1 + gain) - (gain - 1) * cos(w0) + 
	                        2 * alpha * sqrt(gain);
		(*inst).coeffs[3] = -2 * ((gain - 1) - (gain + 1) * cos(w0)) / a0;
		(*inst).coeffs[4] = -((1 + gain) - (gain - 1) * cos(w0) - 
				            2 * alpha * sqrt(gain)) / a0;
        (*inst).coeffs[0] = gain * ((gain + 1) + (gain - 1) * cos(w0) +
		                    2 * sqrt(gain) * alpha) / a0;
		(*inst).coeffs[1] = -2 * gain * ((gain - 1) + (gain + 1) * 
		                    cos(w0)) / a0;
		(*inst).coeffs[2] = gain * ((gain + 1) + (gain - 1) * cos(w0) -
		                    2 * sqrt(gain) * alpha) / a0;
        break;

	  default:
	    return RES_ERROR_PARAM;
	}

    #ifdef USE_LIBRARY
    // Copy coefficients to arm_math instance	
	arm_biquad_cascade_df2T_init_f32(s, 1, inst->coeffs, inst->delays);
    #endif

	inst++;
	s++;
	pCfg++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Uint24ToInt16(uint32_t *inBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts received samples from unsigned 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of array positions to convert 
// Returns: error code
// *****************************************************************************
{
  if ((NULL == inBuf)) 
  {return RES_ERROR_PARAM;}
  
  for ( ; 0 < nSamples; nSamples--)
  {
  	*inBuf++ = (*inBuf)>>8; //Convert from 24 unsigned to 16 signed.
  	*inBuf++ = (*inBuf)>>8; //Convert from 24 unsigned to 16 signed.
  }

    return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_DecodePCM(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                         uint16_t nSamples)
// *****************************************************************************
// Description: Decodes a PCM-formatted buffer into two buffers of half length
// Parameters:
//   inBuf: Pointer to the beginning input buffer position to be decoded
//   bufL: Pointer to the buffer where the left channel should be written
//   bufR: Pointer to the buffer where the right channel should be written
//   nSamples: Number of array positions to be decoded 
// Returns: error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == bufL) || (NULL == bufR) || (nSamples < 2)) 
  {return RES_ERROR_PARAM;}
  
  for ( ; 0 < nSamples; nSamples--)
  {
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
  }

    return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                               uint16_t nSamples)
// *****************************************************************************
// Description: Encodes a PCM-formatted buffer into two buffers of half length
// Parameters:
//   outBuf: Pointer to the beginning output buffer position to start encoding
//   bufL: Pointer to the buffer where the left channel should be written
//   bufR: Pointer to the buffer where the right channel should be written
//   nSamples: Number of array positions to be decoded 
// Returns: error code
// *****************************************************************************
{
  if ((NULL == outBuf) || (NULL == bufL) || (NULL == bufR) || (nSamples < 2)) 
  {return RES_ERROR_PARAM;}
  
  for ( ; 0 < nSamples; nSamples--)
  {
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
  }

    return RES_OK;
}

// *****************************************************************************
void DSP_TestFilters(tParamConfig *pCfg)
// *****************************************************************************
// Description: Set filter parameters for testing purposes
// Parameters: 
// Returns: nothing
// *****************************************************************************
{
  (*pCfg).freq =   6000;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  HIGHSHELF;
  pCfg++;
  (*pCfg).freq =   6000;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  HIGHSHELF;
  pCfg++;
  (*pCfg).freq =   150;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  LOWSHELF;
  pCfg++;
  (*pCfg).freq =   150;
  (*pCfg).q =  1;
  (*pCfg).gain = 5;
  (*pCfg).type =  LOWSHELF;
}

// *****************************************************************************
void DSP_Init(tInstanceIIR *inst)
// *****************************************************************************
// Description: Initializes both coefficients and instances values with 0
// Parameters: none
// Returns: nothing
// *****************************************************************************
{
  uint8_t i = 0;
	
  for (i = 0; i<MAX_FILTERS; i++)
  {
    (*inst).delays[0] = 0;
    (*inst).delays[1] = 0;
   
    (*inst).coeffs[0] = 0;
    (*inst).coeffs[1] = 0;
    (*inst).coeffs[2] = 0;
    (*inst).coeffs[3] = 0;
    (*inst).coeffs[4] = 0;
    inst++;
  }
}



