// *****************************************************************************
// File: dsp.c
// Description: Source code of parametrizable filters
// *****************************************************************************

// *****************************************************************************
// Includes

#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <arm_math.h>
#include <math.h>
#include "dsp.h"

// *****************************************************************************
// Defines 
// FIR filters
#define BLOCK_SIZE            32
#define NUM_TAPS              29

// Frequency response plot parameters
#define PLOT_RESOLUTION       31
#define PLOTX_START           20.0    //Hz
#define PLOTX_END             20000.0 //Hz

// Macros
#define c2d(a) (a/((double)SAMPLE_RATE/2)) // Convert to discrete frequency

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

// *****************************************************************************
tErrorCode DSP_IIR_f32(float *buf, uint32_t nSamples, tInstanceIIRf32 *inst)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *inst: pointer to structure storing coefficients and delayed samples
// Returns: output sample
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == inst)) return RES_ERROR_PARAM;
    float t;                          // Temporal variable   
    float *pDelays = &(inst->delays); // Pointer to state buffer
    float *pCoef = &(inst->coeffs);   // Pointer to filter coefficients

    for ( ;0 < nSamples--; buf++)
    {
      // Direct form
      t = *buf + pCoef[3] * pDelays[0] + pCoef[4] * pDelays[1];
      *buf = t * pCoef[0] + pCoef[1] * pDelays[0] + pCoef[2] * pDelays[1];

      // Update feedback and feedfoward components
      pDelays[1] = pDelays[0];
      pDelays[0] = t;
    }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_IIR_q15(int16_t *buf, uint32_t nSamples, tInstanceIIRq15 *inst)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form I. Fixed point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: Number of samples to process
//   *inst: Pointer to structure storing coefficients and delayed samples
// Returns: Error code
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == inst)) return RES_ERROR_PARAM;
    int32_t acc;                     // 32 bit Accumulator   
    int16_t *pDelays = inst->delays; // Pointer to state buffer

    // Reserve 32 bit variables to perform operations without overflow
    int32_t coef[5] = {inst->coeffs[0], inst->coeffs[2], inst->coeffs[3],
                       inst->coeffs[4], inst->coeffs[5]};

    for ( ;0 < nSamples--; buf++)
    {

      acc = (coef[0] * *buf);
      acc += (coef[1] * pDelays[0]);
      acc += (coef[2] * pDelays[1]);
      acc += (coef[3] * pDelays[2]);
      acc += (coef[4] * pDelays[3]);
      acc >>= (15 - inst->postShift);
      if (acc > SHRT_MAX) acc = SHRT_MAX;
      else if (acc < SHRT_MIN) acc = SHRT_MIN;
      
      // Update feedback and feedfoward components
      pDelays[1] = pDelays[0]; // x_2 = x_1
      pDelays[0] = *buf;       // x_1 = x
      pDelays[3] = pDelays[2]; // y_2 = y_1
      pDelays[2] = acc;        // y_1 = y

      *buf = acc;
    }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_IIR_q31(int32_t *buf, uint32_t nSamples, tInstanceIIRq31 *inst)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *inst: pointer to structure storing coefficients and delayed samples
// Returns: Error code
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == inst)) return RES_ERROR_PARAM;
    int64_t acc;                     // 64 bit Accumulator   
    int32_t *pDelays = inst->delays; // Pointer to state buffer

    // Reserve 64 bit variables to perform operations without overflow
    int64_t coef[5] = {inst->coeffs[0], inst->coeffs[1], inst->coeffs[2],
                       inst->coeffs[3], inst->coeffs[4]};

    for ( ;0 < nSamples--; buf++)
    {

      acc = (coef[0] * *buf);
      acc += (coef[1] * pDelays[0]);
      acc += (coef[2] * pDelays[1]);
      acc += (coef[3] * pDelays[2]);
      acc += (coef[4] * pDelays[3]);
      acc >>= (31 - inst->postShift);
      if (acc > LONG_MAX) acc = LONG_MAX;
      else if (acc < LONG_MIN) acc = LONG_MIN;
      
      // Update feedback and feedfoward components
      pDelays[1] = pDelays[0]; // x_2 = x_1
      pDelays[0] = *buf;       // x_1 = x
      pDelays[3] = pDelays[2]; // y_2 = y_1
      pDelays[2] = acc;        // y_1 = y

      *buf = acc;
    }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_IIR_f32_arm(float *buf, uint32_t nSamples, 
                           arm_biquad_cascade_df2T_instance_f32 *s)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to the buffer containing data to be processed
//   nSamples: Number of samples to process
//   *s: Pointer to structure storing coefficients and delayed samples
//   Returns: Error code
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
tErrorCode DSP_IIR_q15_arm(int16_t *buf, uint32_t nSamples, 
                           arm_biquad_casd_df1_inst_q15 *q)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form I. Fixed point arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *q: pointer to structure storing coefficients and delayed samples
//   Returns: error code
// *****************************************************************************
{
  #ifdef USE_LIBRARY
  if ((NULL == buf) || (NULL == q)) return RES_ERROR_PARAM;
  arm_biquad_cascade_df1_q15(q, buf, buf, nSamples);
  return RES_OK;
  #else 
  return RES_ERROR_CONFIG;
  #endif
}

// *****************************************************************************
tErrorCode DSP_IIR_q31_arm(int32_t *buf, uint32_t nSamples, 
                           arm_biquad_casd_df1_inst_q31 *r)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form I. Fixed point arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to the buffer containing data to be processed
//   nSamples: Number of samples to process
//   *r: Pointer to structure storing coefficients and delayed samples
//   Returns: error code
// *****************************************************************************
{
  #ifdef USE_LIBRARY
  if ((NULL == buf) || (NULL == r)) return RES_ERROR_PARAM;
  arm_biquad_cascade_df1_q31(r, buf, buf, nSamples);
  return RES_OK;
  #else 
  return RES_ERROR_CONFIG;
  #endif
}

// *****************************************************************************
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts passed int16_t buffer into float 32. Only for ARM MCUs.
// Parameters: 
//   *inBuf: Pointer to buffer containing q15 data
//   *outBuf: Pointer to buffer that will store f32 data
//    nSamples: Number of samples to convert
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;

  int16_t auxBuf[nSamples];
  // arm_shift_q15(inBuf, 0, auxBuf, nSamples); // Performs similar to memmove 
  memmove(auxBuf, inBuf, sizeof(int16_t)*nSamples); 
  arm_q15_to_float(auxBuf, outBuf, nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts passed float buffer into int16_t. Only for ARM MCUs.
// Parameters: 
//   *inBuf: Pointer to buffer containing f32 data
//   *outBuf: Pointer to buffer that will store q15 data
//    nSamples: Number of samples to convert
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;

  arm_float_to_q15(inBuf, outBuf, nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_q31_to_q15_arm(int32_t *inBuf, int16_t *outBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts passed int32_t buffer into int16_t. Only for ARM MCUs.
// Parameters: 
//   *inBuf: Pointer to buffer containing q31 data
//   *outBuf: Pointer to buffer that will store q15 data
//    nSamples: Number of samples to convert
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;

  arm_q31_to_q15(inBuf, outBuf, nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Int24ToInt16(uint32_t *inBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
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
tErrorCode DSP_Int24ToInt32(uint32_t *inBuf, uint32_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  while(0 < nSamples--)
  {
    *inBuf <<= 8; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_DecodePCM_Int16(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
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
tErrorCode DSP_DecodePCM_Int32(uint32_t *inBuf, int32_t *bufL, int32_t *bufR, 
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
tErrorCode DSP_Gain_f32_arm(float *buf, float gain, int32_t nSamples)
// *****************************************************************************
// Description: Linear gain function for f32 arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to the buffer to proccess
//   gain: Linear gain to be applied
//   nSamples: Number of samples to proccess
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf)) return RES_ERROR_PARAM;
  if (0 > gain) return RES_ERROR_MATH;

  arm_scale_f32(buf, gain, buf, nSamples);
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Gain_q15_arm(int16_t *buf, int16_t gain, int8_t postShift, 
                            int32_t nSamples)
// *****************************************************************************
// Description: Linear gain function for q15 arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to the buffer to proccess
//   gain: Linear gain to be applied
//   postShift: bitshift for q(15-postShift) format
//   nSamples: Number of samples to proccess
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf)) return RES_ERROR_PARAM;
  if ((0 > postShift) || (0 > gain)) return RES_ERROR_MATH;

  arm_scale_q15(buf, gain, postShift, buf, nSamples);
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Gain_q31_arm(int32_t *buf, int32_t gain, int8_t postShift, 
                            int32_t nSamples)
// *****************************************************************************
// Description: Linear gain function for q15 arithmetic. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to the buffer to proccess
//   gain: Linear gain to be applied
//   postShift: Bitshift for q(31-postShift) format
//   nSamples: Number of samples to proccess
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf)) return RES_ERROR_PARAM;
  if ((0 > postShift) || (0 > gain)) return RES_ERROR_MATH;

  arm_scale_q31(buf, gain, postShift, buf, nSamples);
  return RES_OK;
}

// *****************************************************************************
static tErrorCode DSP_PlotMagnitude(tInstanceIIRf32 *instf32, double *hLinear, 
                                    tGeneralChannel channel)
// *****************************************************************************
// Description: Plots the magnitude response of the combination of all filters
// used in one channel.
// Parameters: 
//   *instf32: Pointer to the IIR floating point filters instance
//   *hLinear: Pointer to the magnitude response vector in linear scale
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == instf32) || (NULL == hLinear)) return RES_ERROR_PARAM;
  double frPoints[PLOT_RESOLUTION];
  double phi, dFreq, exp, magnitude2;
  float *coeff; // Discrete frequency normalized to the unit circle
  uint8_t i,j;
  
  // Initialize values with 0dB gain
  for (i = 0; i < PLOT_RESOLUTION; i++)
  {
    hLinear[i] = (double)1;
  }
  
  // Compute frequency points in hertz and calculate amplitude for each one
  for (j = 0; j < PLOT_RESOLUTION; j++)
  {
    exp = ((double)j)/PLOT_RESOLUTION;
    frPoints[j] = PLOTX_START * pow((PLOTX_END/PLOTX_START), exp); // Get Hz
    dFreq = c2d(frPoints[j]); // Hz to digital frequency
    phi = pow(sin(dFreq*PI/2), 2);
    for (i = 0; i < MAX_FILTERS; i++)
    {
      if (instf32->channel == channel)
      {
        coeff = instf32->coeffs;
        magnitude2 = 
        (pow((coeff[0] + coeff[1] + coeff[2])/2, 2) - phi * (4 * coeff[0] *
        coeff[2] * (1 - phi) + coeff[1] * (coeff[0] + coeff[2])))
                                         / 
        (pow((1 - coeff[3] - coeff[4])/2, 2) - phi * (4 * (-coeff[4]) * 
        (1 - phi) - coeff[3] * (1 - coeff[4])));
        *hLinear *= sqrt(magnitude2);
      }
      instf32++;
    }
    instf32 -= MAX_FILTERS;
    hLinear++;
  }

  return RES_OK;
}

// *****************************************************************************
static tErrorCode DSP_UpdateNormGain(tGain *normGain, double *fResponse, 
                              uint8_t nPoints, tGeneralChannel channel)
// *****************************************************************************
// Description: Updates given normalization block with the needed gain 
// adjustement to obtain a final magnitude response that peaks at 0 dB for
// passed channel.
// Parameters: 
//   *normGain: Pointer to the normalization gain array
//   *fResponse: Pointer to the channel's magnitude response
//   channel: Channel of the given magnitude response
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == normGain) || (NULL == fResponse)) return RES_ERROR_PARAM;

  uint8_t i;
  double maxValue = 0;
  double inversef32 = 0;
  int16_t inverseq15 = 0;
  int32_t inverseq31 = 0;
  while (0 < nPoints--)
  {
    if (maxValue < *fResponse++) maxValue = *fResponse; // Get FR amplitude peak
  }

  inversef32 = 1.0 / maxValue; // Get inverse gain
  
  if (1 < inversef32) 
  {
    inverseq15 = (int16_t)(inversef32 * SHRT_MAX/2);
    inverseq31 = (int32_t)(inversef32 * LONG_MAX/2);
    normGain->postShift = 1;
  }
  else if (0 < inversef32) 
  {
    inverseq15 = (int16_t)(inversef32 * SHRT_MAX);
    inverseq31 = (int32_t)(inversef32 * LONG_MAX);
    normGain->postShift = 0;
  }
  else return RES_ERROR_MATH;

  normGain->f32 = inversef32;
  normGain->q15 = inverseq15; 
  normGain->q31 = inverseq31; 
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_UpdateIIRInstances(tParamConfig *pCfg, 
                                  tInstanceIIRf32 *instf32,
                                  tInstanceIIRq15 *instq15,
                                  tInstanceIIRq31 *instq31, 
                                  arm_biquad_cascade_df2T_instance_f32 *s,
                                  arm_biquad_casd_df1_inst_q15 *q,
                                  arm_biquad_casd_df1_inst_q31 *r,
                                  tGain *normGain)
// *****************************************************************************
// Description: Updates the filter coefficients with the new parameter config
// received from master. Adapted to work with CMSIS-DSP filters from 
// Audio EQ Cookbook, by Robert Bristow-Johnson. Updates normalization gain.
// Parameters: 
//   *pCfg: Pointer to the structure containing design parameters
//   *instf32: pointer to the f32 structure containing array for coefficients
//   *instq15: Pointer to the q15 structure containing array for coefficients
//   *instq31: Pointer to the q31 structure containing array for coefficients
//   *s: Pointer to the f32 structure containing pointers for CMSIS-DSP filters
//   *q: Pointer to the q15 structure containing pointers for CMSIS-DSP filters
//   *r: Pointer to the q31 structure containing pointers for CMSIS-DSP filters
//   *normGain: Pointer to the normalization gain array
// Returns: Error code 
// *****************************************************************************
{
  if ((NULL == pCfg) || (NULL == s) || (NULL == instf32) || (NULL == instq15)) 
  return RES_ERROR_PARAM;
  
  uint8_t i; 
  float w0 = 0;
  double a = 0;
  float A = 0;
  float a0 = 0;
  float *coeff; // Pointer to filter coefficient array

    for (i = 0; i < MAX_FILTERS; i++)
  {
    w0 = 2 * PI * pCfg->freq / SAMPLE_RATE;
    a = sin(w0) / (2 * pCfg->q);
    A =  pow(10.0,(double)(pCfg->gain/40));
    a0 = 0;
    coeff = instf32->coeffs; // Pointer to filter coefficient array

    switch (pCfg->type)
    {
  
      case LOWPASS: 
              a0 =                  1 + a;
        coeff[3] =     (2 * cos(w0)) / a0;  // a1
        coeff[4] =         - (1 - a) / a0;  // a2
        coeff[0] = ((1 - cos(w0))/2) / a0;  // b0
        coeff[1] =           2 * coeff[0];  // b1
        coeff[2] =               coeff[0];  // b2
      break;
  
      case HIGHPASS:
              a0 =                  1 + a;
        coeff[3] =  - (-2 * cos(w0)) / a0;  // a1
        coeff[4] =         - (1 - a) / a0;  // a2
        coeff[0] = ((1 + cos(w0))/2) / a0;  // b0
        coeff[1] =          -2 * coeff[0];  // b1
        coeff[2] =               coeff[0];  // b2
      break;
  
      case PEAK:
              a0 =           1 + (a / A);
        coeff[3] = - (-2 * cos(w0)) / a0;  // a1
        coeff[4] =  - (1 - (a / A)) / a0;  // a2
        coeff[0] =    (1 + (a * A)) / a0;  // b0
        coeff[1] =            - coeff[3];  // b1
        coeff[2] =    (1 - (a * A)) / a0;  // b2
      break;
  
      case LOWSHELF:
              a0 =            (1 + A) + (A - 1) * cos(w0) + 2 * a * sqrt(A); 
        coeff[3] =                   2 * ((A - 1) + (A + 1) * cos(w0)) / a0; 
        coeff[4] =   - ((1 + A) + (A - 1) * cos(w0) - 2 * a * sqrt(A)) / a0;         
        coeff[0] = A * ((A + 1) - (A - 1) * cos(w0) + 2 * sqrt(A) * a) / a0; 
        coeff[1] =               2 * A * ((A - 1) - (A + 1) * cos(w0)) / a0; 
        coeff[2] = A * ((A + 1) - (A - 1) * cos(w0) - 2 * sqrt(A) * a) / a0; 
      break;
  
      case HIGHSHELF:
              a0 =            (1 + A) - (A - 1) * cos(w0) + 2 * a * sqrt(A);
        coeff[3] =                  -2 * ((A - 1) - (A + 1) * cos(w0)) / a0;
        coeff[4] =   - ((1 + A) - (A - 1) * cos(w0) - 2 * a * sqrt(A)) / a0;
        coeff[0] = A * ((A + 1) + (A - 1) * cos(w0) + 2 * sqrt(A) * a) / a0;
        coeff[1] =             - 2 * A * ((A - 1) + (A + 1) * cos(w0)) / a0;
        coeff[2] = A * ((A + 1) + (A - 1) * cos(w0) - 2 * sqrt(A) * a) / a0;
      break;
  
      default:
        return RES_ERROR_PARAM;
    }
    
    // Set CMSIS-DSP library instances
    arm_biquad_cascade_df2T_init_f32(s, 1, instf32->coeffs, instf32->delays);

    // Compute fixed-point coefficients
    if ((coeff[0] > 1) || (coeff[1] > 1) || (coeff[2] > 1) || (coeff[3] > 1) ||
        (coeff[4] > 1) || (coeff[0] < -1) || (coeff[1] < -1) ||
        (coeff[2] < -1) || (coeff[3] < -1) || (coeff[4] < -1))
    {
     instq15->coeffs[0] = (int16_t)(coeff[0] * SHRT_MAX/2);
     instq15->coeffs[2] = (int16_t)(coeff[1] * SHRT_MAX/2);
     instq15->coeffs[3] = (int16_t)(coeff[2] * SHRT_MAX/2);
     instq15->coeffs[4] = (int16_t)(coeff[3] * SHRT_MAX/2);
     instq15->coeffs[5] = (int16_t)(coeff[4] * SHRT_MAX/2);
     instq15->postShift = 1;
     arm_biquad_cascade_df1_init_q15(q, 1, instq15->coeffs, instq15->delays, 1);
     
     instq31->coeffs[0] = (int32_t)(coeff[0] * LONG_MAX/2);
     instq31->coeffs[1] = (int32_t)(coeff[1] * LONG_MAX/2);
     instq31->coeffs[2] = (int32_t)(coeff[2] * LONG_MAX/2);
     instq31->coeffs[3] = (int32_t)(coeff[3] * LONG_MAX/2);
     instq31->coeffs[4] = (int32_t)(coeff[4] * LONG_MAX/2);
     instq31->postShift = 1;
     arm_biquad_cascade_df1_init_q31(r, 1, instq31->coeffs, instq31->delays, 1);
    }
    else 
    {
     instq15->coeffs[0] = (int16_t)(coeff[0] * SHRT_MAX);
     instq15->coeffs[1] = (int16_t)(coeff[1] * SHRT_MAX);
     instq15->coeffs[2] = (int16_t)(coeff[2] * SHRT_MAX);
     instq15->coeffs[3] = (int16_t)(coeff[3] * SHRT_MAX);
     instq15->coeffs[4] = (int16_t)(coeff[4] * SHRT_MAX);
     instq15->postShift = 0;
     arm_biquad_cascade_df1_init_q15(q, 1, instq15->coeffs, instq15->delays, 0);
     
     instq31->coeffs[0] = (int32_t)(coeff[0] * LONG_MAX);
     instq31->coeffs[1] = (int32_t)(coeff[1] * LONG_MAX);
     instq31->coeffs[2] = (int32_t)(coeff[2] * LONG_MAX);
     instq31->coeffs[3] = (int32_t)(coeff[3] * LONG_MAX);
     instq31->coeffs[4] = (int32_t)(coeff[4] * LONG_MAX);
     instq31->postShift = 0;
     arm_biquad_cascade_df1_init_q31(r, 1, instq31->coeffs, instq31->delays, 0);
    }
    

    // Assign filter channel to instances
    instf32->channel = pCfg->channel;
    instq15->channel = pCfg->channel;
    instq31->channel = pCfg->channel;

    instq15++;
    instq31++;
    instf32++;
    q++;
    r++;
    s++;
    pCfg++;
  }

  instq15 -= MAX_FILTERS;
  instf32 -= MAX_FILTERS;
  
  double fResponse[PLOT_RESOLUTION];  // Store frequency response temporarily
  uint8_t j;
  for (j = CHANNEL_0; j < MAX_CHANNELS; j++)
  {
    if (RES_OK != DSP_PlotMagnitude(instf32, fResponse, j)) return RES_ERROR;
    if (RES_OK != DSP_UpdateNormGain(normGain, fResponse, PLOT_RESOLUTION, j)) 
    return RES_ERROR;
    normGain++;
  }
  return RES_OK;
}

// *****************************************************************************
void DSP_TestFilters(tParamConfig *pCfg)
// *****************************************************************************
// Description: Set filter parameters for testing purposes
// Parameters: 
//   *pCfg: Pointer to structure containing filter parameters
// Returns: Nothing
// *****************************************************************************
{
  pCfg->freq =  1000;
  pCfg->q =  0.707;
  pCfg->gain =  0;
  pCfg->type =  HIGHPASS;
  pCfg->channel =  CHANNEL_0;
  pCfg++;
  pCfg->freq =  1000;
  pCfg->q =  0.707;
  pCfg->gain =  0;
  pCfg->type =  HIGHPASS;
  pCfg->channel =  CHANNEL_1;
  pCfg++;
  pCfg->freq =  200;
  pCfg->q =  2;
  pCfg->gain =  15;
  pCfg->type =  PEAK;
  pCfg->channel =  CHANNEL_0;
  pCfg++;
  pCfg->freq =  200;
  pCfg->q =  2;
  pCfg->gain = 15;
  pCfg->type =  PEAK;
  pCfg->channel =  CHANNEL_1;
  // pCfg++;
  // pCfg->freq =  200;
  // pCfg->q =  2;
  // pCfg->gain =  8;
  // pCfg->type =  PEAK;
  // pCfg->channel =  CHANNEL_0;
  // pCfg++;
  // pCfg->freq =  200;
  // pCfg->q =  2;
  // pCfg->gain = 8;
  // pCfg->type =  PEAK;
  // pCfg->channel =  CHANNEL_1;
}

// *****************************************************************************
void DSP_Init(tParamConfig *pCfg, tInstanceIIRf32 *instf32, 
              tInstanceIIRq15 *instq15, arm_biquad_cascade_df2T_instance_f32 *s,
              arm_biquad_casd_df1_inst_q15 *q, tGain *gain)
// *****************************************************************************
// Description: Initializes both coefficients and instances values with 0
// Parameters: none
// Returns: nothing
// *****************************************************************************
{
  uint8_t i;
  
  for (i = 0; i<MAX_FILTERS; i++)
  {
    instf32->delays[0] = 0;
    instf32->delays[1] = 0;
   
    instf32->coeffs[0] = 0;
    instf32->coeffs[1] = 0;
    instf32->coeffs[2] = 0;
    instf32->coeffs[3] = 0;
    instf32->coeffs[4] = 0;
    instf32->channel = CHANNEL_NONE;
    arm_biquad_cascade_df2T_init_f32(s, 1, instf32->coeffs, instf32->delays);

    instq15->delays[0] = 0;
    instq15->delays[1] = 0;
    instq15->delays[2] = 0;
    instq15->delays[3] = 0;
   
    instq15->coeffs[0] = 0;
    instq15->coeffs[1] = 0;
    instq15->coeffs[2] = 0;
    instq15->coeffs[3] = 0;
    instq15->coeffs[4] = 0;
    instq15->coeffs[5] = 0;
    instq15->channel = CHANNEL_NONE;
    arm_biquad_cascade_df1_init_q15(q, 1, instq15->coeffs, instq15->delays, 0); 

    pCfg->channel = CHANNEL_NONE;
    pCfg->gain = 0;

    instf32++;
    instq15++;
    pCfg++;
    s++;
  }

  for (i = 0; i < MAX_CHANNELS; i++)
  {
    gain->f32 = 0;
    gain->q15 = 0;
    gain++;
  }
}