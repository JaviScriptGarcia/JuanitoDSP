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
tErrorCode DSP_IIR_f32(float *buf, uint32_t nSamples, 
                             tInstanceIIR *inst)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *instance: pointer to structure storing coefficients and delayed samples
// Returns: output sample
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == inst)) return RES_ERROR_PARAM;
    float t;        // Temporal variable   
    float *pDelays = &((*inst).delays[0]); // Pointer to state buffer

    for ( ;0 < nSamples--; buf++)
    {
      // Transposed form 
      //   y = (*inst).coeffs[0] * buf[i] + (*inst).delays[0];
      //   (*inst).delays[0] = (*inst).coeffs[1] * buf[i] + (*inst).coeffs[3] * y 
      //   + (*inst).delays[1];
      //   (*inst).delays[1] = (*inst).coeffs[2] * buf[i] + (*inst).coeffs[4] * y;
      //   buf[i] = (int16_t)y;

      // Non-transposed form
      t = *buf + (*inst).coeffs[3] * *pDelays + (*inst).coeffs[4] * *(pDelays+1);
      *buf = t * (*inst).coeffs[0] + (*inst).coeffs[1] * *pDelays + 
	    (*inst).coeffs[2] * *(pDelays+1);

      // Update feedback and feedfoward components
      *(pDelays+1) = *pDelays;
      *pDelays = t;
    }

	return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts passed int16_t buffer into float 32 using arm_math.h
// Parameters: 
//   *inBuf: pointer to buffer containing q15 data
//   *outBuf: pointer to buffer that will store f32 data
//    nSamples: number of samples to convert
// Returns: error code
// *****************************************************************************
{
	#ifdef USE_LIBRARY
	if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;
  int16_t auxBuf[nSamples];
  memcpy(auxBuf, inBuf, sizeof(int16_t)*nSamples);
  arm_q15_to_float(auxBuf, outBuf, nSamples);
	return RES_OK;
    #else 
	return RES_ERROR_CONFIG;
	#endif
}

// *****************************************************************************
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts passed int16_t buffer into float 32 using arm_math.h
// Parameters: 
//   *inBuf: pointer to buffer containing f32 data
//   *outBuf: pointer to buffer that will store q15 data
//    nSamples: number of samples to convert
// Returns: error code
// *****************************************************************************
{
	#ifdef USE_LIBRARY
	if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;
  arm_float_to_q15(inBuf, outBuf, nSamples);
	return RES_OK;
  #else 
	return RES_ERROR_CONFIG;
	#endif
}

// *****************************************************************************
tErrorCode DSP_IIR_f32_arm(float *buf, uint32_t nSamples, 
                           arm_biquad_cascade_df2T_instance_f32 *s)
// *****************************************************************************
// Description: Basic second order filter for left channel. Filter
// structure is direct form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *instance: pointer to structure storing coefficients and delayed samples
//   Returns: error code
// *****************************************************************************
{
	#ifdef USE_LIBRARY
	if ((NULL == buf) || (NULL == s)) return RES_ERROR_PARAM;
  arm_biquad_cascade_df2T_f32(s, buf, buf, nSamples);
	return RES_OK;
  #else 
	return RES_ERROR_CONFIG;
	#endif
}

// *****************************************************************************
tErrorCode DSP_Int24ToInt16(uint32_t *inBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  while(0 < nSamples--)
  {
    *inBuf >>= 8; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_DecodePCM(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                         uint32_t nSamples)
// *****************************************************************************
// Description: Decodes a PCM stereo buffer into two buffers of half length
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
  
  while(0 < nSamples--)
  {
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
  }

    return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                         uint32_t nSamples)
// *****************************************************************************
// Description: Encodes a two buffers into a stereo PCM buffer of double length
// Parameters:
//   outBuf: Pointer to the beginning output buffer position
//   bufL: Pointer to the beginning position of the left channel buffer
//   bufR: Pointer to the beginning position of the right channel buffer
//   nSamples: Number of samples to encode
// Returns: error code
// *****************************************************************************
{
  if ((NULL == outBuf) || (NULL == bufL) || (NULL == bufR) || (nSamples < 2)) 
  {return RES_ERROR_PARAM;}
  
  while(0 < nSamples--)
  {
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
  }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_UpdateFilterInstances(tParamConfig *pCfg, tInstanceIIR *inst, 
                                     arm_biquad_cascade_df2T_instance_f32 *s)
// *****************************************************************************
// Description: Updates the filter coefficients with the new parameter config
// received from master. Adapted to work with CMSIS-DSP filters from 
// Audio EQ Cookbook, by Robert Bristow-Johnson
// Parameters: 
//   *pCfg: pointer to the structure containing design parameters
//   *inst: pointer to the structure containing array for coefficients
//   *s: pointer to the structure containing pointers for CMSIS-DSP filters
// Returns: 
// *****************************************************************************
{
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

    // Assign filter channel to instance
    (*inst).channel = (*pCfg).channel;

	  inst++;
	  s++;
	  pCfg++;
  }
  return RES_OK;
}

// *****************************************************************************
void DSP_TestFilters(tParamConfig *pCfg)
// *****************************************************************************
// Description: Set filter parameters for testing purposes
// Parameters: 
//   *pCfg: Pointer to structure containing filter parameters
// Returns: nothing
// *****************************************************************************
{
  (*pCfg).freq =   6000;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  HIGHSHELF;
  (*pCfg).channel =  CHANNEL_0;
  pCfg++;
  (*pCfg).freq =   6000;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  HIGHSHELF;
  (*pCfg).channel =  CHANNEL_1;
  pCfg++;
  (*pCfg).freq =   150;
  (*pCfg).q =  1;
  (*pCfg).gain =  5;
  (*pCfg).type =  LOWSHELF;
  (*pCfg).channel =  CHANNEL_0;
  pCfg++;
  (*pCfg).freq =   150;
  (*pCfg).q =  1;
  (*pCfg).gain = 5;
  (*pCfg).type =  LOWSHELF;
  (*pCfg).channel =  CHANNEL_1;
}

// *****************************************************************************
void DSP_Init(tParamConfig *pCfg, tInstanceIIR *inst,
              arm_biquad_cascade_df2T_instance_f32 *s)
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
  	(*pCfg).channel = CHANNEL_NONE;
  	(*inst).channel = CHANNEL_NONE;
    arm_biquad_cascade_df2T_init_f32(s, 1, inst->coeffs, inst->delays);
    inst++;
    pCfg++;
    s++;
  }
}