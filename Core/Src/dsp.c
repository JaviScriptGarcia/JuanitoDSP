// *****************************************************************************
// File: dsp.c
// Description: Source code of parametrizable filters
// *****************************************************************************

// *****************************************************************************
// Includes

#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#ifndef _ARM_MATH_H
#include <arm_math.h>
#endif
#include "uty.h"
#include "dsp.h"
#include "coeffs.h"
#include "conv.h"

// *****************************************************************************
// Defines 

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

// *****************************************************************************
tErrorCode DSP_Convolution_q15_arm(int16_t *buf, uint16_t nSamples, 
                                   tConvq15 *ir)
// *****************************************************************************
// Description: Performs a convolution using CMSIS-DSP functions. Only for ARM.
// Parameters: 
//   *buf: Pointer to the input signal buffer.
//   *ir: Pointer to the IR to be used.
//   nSamples: Number of samples to process.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == buf) || (NULL == ir) || (4 > nSamples)) return RES_ERROR_PARAM;
  
  uint16_t sizeIR = ir->size;
  int16_t *tail = ir->tail;
  int16_t inLen = nSamples + sizeIR;
  int16_t input[inLen]; 

  // Build input signal from actual and past samples
  memcpy(input, tail, sizeof(int16_t) * sizeIR);
  memcpy(input + sizeIR, buf, sizeof(int16_t) * nSamples);

  // Save last samples for next convolution
  if (sizeIR < nSamples)
  {
    memcpy(tail, &buf[nSamples - sizeIR], sizeof(int16_t) * sizeIR);
  }
  else
  {
    memmove(tail, tail + nSamples, sizeof(int16_t) * (sizeIR - nSamples));
    memcpy(tail + (sizeIR - nSamples), buf, sizeof(int16_t) * nSamples);
  }
  
  if (ARM_MATH_SUCCESS != arm_conv_partial_fast_q15(input, inLen, ir->coeffs, 
                          sizeIR, buf - sizeIR, sizeIR, nSamples))
  {return RES_ERROR;}

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_FIR_f32(float *buf, uint16_t nSamples, tFIRf32 *fir)
// *****************************************************************************
// Description: Basic FIR filter function. Convolutes input signal with 
// stored impulse response. FIR size must be greater than 4.
// Parameters: 
//   *buf: Pointer to buffer with samples to process
//   nSamples: Number of samples to process
//   *fir: Pointer to structure storing impulse repsonse of FIR filter
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf) || (NULL == fir)) return RES_ERROR_PARAM;
  if (4 > fir->size) return RES_ERROR_MATH;

  float acc = 0;
  uint16_t n, k;
  float *coeff = fir->coeffs;
  uint16_t sizeIR = fir->size;
  float out[nSamples];
  uint16_t tailEnd = sizeIR;
  float *tail = fir->delays;

  // First iteration, use tail
  for (k = 0 ; k < sizeIR; k++)
  {

    tailEnd = sizeIR - k;
    acc = 0;
    for (n = 0; (n < tailEnd); n++)
    {
      acc += *coeff++ * tail[n+k];
    }
    for (n = 0; (n < k); n++)
    {
      acc += *coeff++ * buf[n];
    }
    coeff = &coeff[-sizeIR]; // Reset IR coeff pointer
      
    out[k] = acc;
  }
  
  // Save tail for next convolution
  memcpy(tail, &buf[nSamples - sizeIR], sizeof(float) * sizeIR);

  // Rest of the signal, dont use tail
  while (k < nSamples)
  {

      acc = 0;
      for (n = 4; (n < sizeIR); n += 4) // Unrolled for better efficency
      {
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
      }
      for (n -= 4; (n < sizeIR); n++)
      {
        acc += *coeff++ * *buf++;
      }
      
      coeff = &coeff[-sizeIR]; // Reset IR coeff pointer
      buf = &buf[-sizeIR];     // Reset buffer pointer
      buf++;                   // Move one position in advance

      out[k] = acc;
      k++;
  }
  buf -= nSamples - sizeIR;
  // Export result
  memcpy(buf, out, sizeof(float) * nSamples);
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_FIR_q15(int16_t *buf, uint16_t nSamples, tFIRq15 *fir)
// *****************************************************************************
// Description: Basic FIR filter function. Convolutes input signal with 
// stored impulse response. IR size must be greater than 4.
// Parameters: 
//   *buf: Pointer to buffer with samples to process
//   nSamples: Number of samples to process
//   *fir: Pointer to structure storing impulse repsonse of FIR filter
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf) || (NULL == fir)) return RES_ERROR_PARAM;
  if (4 > fir->size) return RES_ERROR_MATH;

  uint32_t acc = 0;
  uint16_t n, k;
  int16_t *coeff = fir->coeffs;
  uint16_t sizeIR = fir->size;
  int16_t out[nSamples];
  uint16_t tailEnd = sizeIR;
  int16_t *tail = fir->delays;

  // First iteration, use tail
  for (k = 0 ; k < sizeIR; k++)
  {

    tailEnd = sizeIR - k;
    acc = 0;
    for (n = 0; (n < tailEnd); n++)
    {
      acc += *coeff++ * tail[n+k];
    }
    for (n = 0; (n < k); n++)
    {
      acc += *coeff++ * buf[n];
    }
    coeff = &coeff[-sizeIR]; // Reset IR coeff pointer
      
    out[k] = acc >> 15;
  }
  
  // Save tail for next convolution
  memcpy(tail, &buf[nSamples - sizeIR], sizeof(uint16_t) * sizeIR);

  // Rest of the signal, dont use tail
  while (k < nSamples)
  {

      acc = 0;
      for (n = 4; (n < sizeIR); n += 4) // Unrolled for better efficency
      {
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
        acc += *coeff++ * *buf++;
      }
      for (n -= 4; (n < sizeIR); n++)
      {
        acc += *coeff++ * *buf++;
      }
      
      coeff = &coeff[-sizeIR]; // Reset IR coeff pointer
      buf = &buf[-sizeIR];     // Reset buffer pointer
      buf++;                   // Move one position in advance

      out[k] = acc >> 15;
      k++;
  }
  buf -= nSamples - sizeIR;
  // Export result
  memcpy(buf, out, sizeof(int16_t) * nSamples);
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_FIR_f32_arm(float *buf, uint16_t nSamples, 
                           arm_fir_instance_f32 *f)
// *****************************************************************************
// Description: Basic FIR filter function. Convolutes input signal with 
// stored impulse response. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to buffer with samples to process
//   nSamples: Number of samples to process
//   *inst: Pointer to instance storing impulse repsonse of FIR filter
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf) || (NULL == f)) return RES_ERROR_PARAM;

  float out[nSamples];
  
  arm_fir_f32(f, buf, out, nSamples);

  memcpy(buf, out, sizeof(float) * nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_FIR_q15_arm(int16_t *buf, uint16_t nSamples, 
                           arm_fir_instance_q15 *g)
// *****************************************************************************
// Description: Basic FIR filter function. Only for ARM MCUs.
// Parameters: 
//   *buf: Pointer to buffer with samples to process
//   nSamples: Number of samples to process
//   *g: Pointer to instance storing impulse repsonse of FIR filter
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == buf) || (NULL == g)) return RES_ERROR_PARAM;

  arm_fir_fast_q15(g, buf, buf, nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_IIR_f32(float *buf, uint16_t nSamples, tIIRf32 *iir)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *iir: pointer to structure storing coefficients and delayed samples
// Returns: output sample
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == iir)) return RES_ERROR_PARAM;
    float t;                          // Temporal variable   
    float *pDelays = &(iir->delays[0]); // Pointer to state buffer
    float *pCoef = &(iir->coeffs[0]);   // Pointer to filter coefficients

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
tErrorCode DSP_IIR_q15(int16_t *buf, uint16_t nSamples, tIIRq15 *iir)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form I. Fixed point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: Number of samples to process
//   *iir: Pointer to structure storing coefficients and delayed samples
// Returns: Error code
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == iir)) return RES_ERROR_PARAM;
    int32_t acc;                     // 32 bit Accumulator   
    int16_t *pDelays = iir->delays; // Pointer to state buffer

    // Reserve 32 bit variables to perform operations without overflow
    int32_t coef[5] = {iir->coeffs[0], iir->coeffs[2], iir->coeffs[3],
                       iir->coeffs[4], iir->coeffs[5]};

    for ( ;0 < nSamples--; buf++)
    {

      acc = (coef[0] * *buf);
      acc += (coef[1] * pDelays[0]);
      acc += (coef[2] * pDelays[1]);
      acc += (coef[3] * pDelays[2]);
      acc += (coef[4] * pDelays[3]);
      acc >>= (15 - iir->postShift);
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
tErrorCode DSP_IIR_q31(int32_t *buf, uint16_t nSamples, tIIRq31 *iir)
// *****************************************************************************
// Description: Basic second order biquad filter. Filter structure is direct 
// form II. Floating point arithmetic.
// Parameters: 
//   *buf: pointer to the buffer containing data to be processed
//   nSamples: number of samples to process
//   *iir: pointer to structure storing coefficients and delayed samples
// Returns: Error code
// *****************************************************************************
{
    if ((NULL == buf) || (NULL == iir)) return RES_ERROR_PARAM;
    int64_t acc;                     // 64 bit Accumulator   
    int32_t *pDelays = iir->delays; // Pointer to state buffer

    // Reserve 64 bit variables to perform operations without overflow
    int64_t coef[5] = {iir->coeffs[0], iir->coeffs[1], iir->coeffs[2],
                       iir->coeffs[3], iir->coeffs[4]};

    for ( ;0 < nSamples--; buf++)
    {

      acc = (coef[0] * *buf);
      acc += (coef[1] * pDelays[0]);
      acc += (coef[2] * pDelays[1]);
      acc += (coef[3] * pDelays[2]);
      acc += (coef[4] * pDelays[3]);
      acc >>= (31 - iir->postShift);
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
tErrorCode DSP_IIR_f32_arm(float *buf, uint16_t nSamples, 
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
tErrorCode DSP_IIR_q15_arm(int16_t *buf, uint16_t nSamples, 
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
tErrorCode DSP_IIR_q31_arm(int32_t *buf, uint16_t nSamples, 
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
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint16_t nSamples)
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
tErrorCode DSP_q31_to_f32_arm(int32_t *inBuf, float *outBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts passed int32_t buffer into float 32. Only for ARM MCUs.
// Parameters: 
//   *inBuf: Pointer to buffer containing q31 data
//   *outBuf: Pointer to buffer that will store f32 data
//    nSamples: Number of samples to convert
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == outBuf)) return RES_ERROR_PARAM;

  arm_q31_to_float(inBuf, outBuf, nSamples);

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint16_t nSamples)
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
tErrorCode DSP_q31_to_q15_arm(int32_t *inBuf, int16_t *outBuf, uint16_t nSamples)
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
tErrorCode DSP_Gain_f32_arm(float *buf, float gain, uint16_t nSamples)
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
                            uint16_t nSamples)
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
                            uint16_t nSamples)
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
tErrorCode DSP_UpdateConvolutionInstances(tConvq15 *conv)
// *****************************************************************************
// Description: Updates Convolution Instances from configuration file
// Parameters: 
//   *conv: Convolution instance.
// Returns: Error code.
// *****************************************************************************
{
  if (NULL == conv) return RES_ERROR_PARAM;

  CONV_Test(conv->coeffs, &(conv->size));
  conv->channel = CHANNEL_NONE;

  return RES_OK;
}

// *****************************************************************************
static tErrorCode DSP_PlotMagnitude(tIIRf32 *IIRf32, double *hLinear, 
                                    tChannel channel)
// *****************************************************************************
// Description: Plots the magnitude response of the combination of all IIR 
// filters used in one channel.
// Parameters: 
//   *IIRf32: Pointer to the IIR floating point filters.
//   *hLinear: Pointer to the magnitude response vector in linear scale.
//   channel: Channel which will be analyzed.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == IIRf32) || (NULL == hLinear)) return RES_ERROR_PARAM;
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
    frPoints[j] = FRANGE_MIN * pow((FRANGE_MAX/FRANGE_MIN), exp); // Get Hz
    dFreq = c2d(frPoints[j]); // Hz to digital frequency
    phi = pow(sin(dFreq*PI/2), 2);
    for (i = 0; i < MAX_FILTERS; i++)
    {
      if (IIRf32->channel == channel)
      {
        coeff = IIRf32->coeffs;
        magnitude2 = 
        (pow((coeff[0] + coeff[1] + coeff[2])/2, 2) - phi * (4 * coeff[0] *
        coeff[2] * (1 - phi) + coeff[1] * (coeff[0] + coeff[2])))
                                         / 
        (pow((1 - coeff[3] - coeff[4])/2, 2) - phi * (4 * (-coeff[4]) * 
        (1 - phi) - coeff[3] * (1 - coeff[4])));
        *hLinear *= sqrt(magnitude2);
      }
      IIRf32++;
    }
    IIRf32 -= MAX_FILTERS;
    hLinear++;
  }

  return RES_OK;
}


// *****************************************************************************
static tErrorCode DSP_GetFIRCoeffsf32(float fc, float *coeffs, 
                                      uint16_t *firSize)
// *****************************************************************************
// Description: Computes FIR lowpass coeffs based in the Windowed-Sinc Method. 
// Uses Hann window. Odd and symmetrical Impulse Responses.
// Parameters: 
//   fc: cutoff frequency.
//   *coeffs: Pointer to the array storing the coefficients.
//   *firSize: Pointer to the variable storing the number of filter taps.
// Returns: Error code.
// *****************************************************************************
{
  if (NULL == coeffs) return RES_ERROR_PARAM;

  uint16_t i;
  uint16_t halfSize;
  double w = (double)PI * c2d(fc);

  // Optimal size to cover the main and first side lobes of sinc function
  *firSize = (uint16_t)((2 * round((SAMPLE_RATE / fc))) - 1);
  halfSize = *firSize >> 1;

  // Get truncated sinc
  for (i = 0; i < halfSize; i++)
  {
    coeffs[i] = sin(w * (float)(i - halfSize)) / (PI * (float)(i - halfSize));
    coeffs[*firSize-1-i] = coeffs[i]; // Symmetrical copy
  }
  coeffs[halfSize] = w / PI; // Center tap

  // Apply Hann window
  for (i = 0; i < *firSize; i++)
  {
    *coeffs *= 0.54 - 0.46 * cos(2.0 * PI * ((float)i / (*firSize - 1))); 
    coeffs++;
  }

  return RES_OK;
}

// *****************************************************************************
static tErrorCode DSP_ComputeFixedFIRCoefs(float *coeff32, tFIRq15 *FIRq15)
// *****************************************************************************
// Description: Quantizes the fixed point FIR coefficients from a given
// set of floating point coefficients
// Parameters: 
//   *coeff32: Pointer to f32 coefficient array
//   *FIRq15: Pointer to q15 FIR filter structure
// Returns: 
// *****************************************************************************
{
  if ((NULL == coeff32) || (NULL == FIRq15))
  return RES_ERROR_PARAM;

  uint16_t i;
  uint16_t size = FIRq15->size;
  int16_t *coefq15 = FIRq15->coeffs;

  for (i = 0; i < size; i++)
  {
    *coefq15++ = *coeff32++ * SHRT_MAX;
  }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_UpdateFIRInstances(tFilterConfig *pCfg, tFIRf32 *FIRf32, 
                                  tFIRq15 *FIRq15, arm_fir_instance_f32 *f,
                                  arm_fir_instance_q15 *g)
// *****************************************************************************
// Description: Computes FIR coefficients with windowed-Sinc method and updates 
// the passed instances.
// Parameters: 
//   *pCfg: Pointer to the structure containing design parameters.
//   *FIRf32: pointer to the f32 structure containing array for coefficients.
//   *FIRq15: pointer to the q15 structure containing array for coefficients.
//   *f: Pointer to the f32 structure containing pointers for CMSIS-DSP filters.
//   *g: Pointer to the q15 structure containing pointers for CMSIS-DSP filters.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == FIRf32) || (NULL == f)) return RES_ERROR_PARAM;
  
  uint16_t i, n;
  uint16_t *firSize;
  float *firCoeffs;
  for (i = 0; i < MAX_FILTERS; i++)
  {
    if ((CHANNEL_NONE != pCfg->channel) && (IIRHIGHSHELF < pCfg->type))
    {
      firCoeffs = FIRf32->coeffs;
      firSize = &(FIRf32->size);
      if (RES_OK != COEF_GetFIRCoeffsf32(pCfg->freq, firCoeffs, firSize))
      {return RES_ERROR;}

      switch (pCfg->type)
      {
        /*Widowed-Sinc method*/
        case FIRLOWPASS: // Filter already done 
          break;
  
        case FIRHIGHPASS: // Transform to highpass
          for  (n = 0; n < *firSize; n++)
          {
            firCoeffs[n] = - firCoeffs[n];
          }
          firCoeffs[(*firSize >> 1)] += 1; 
          break;
        
        default:
          return RES_ERROR_PARAM;
          break;
      }


        /*Minimax method*/  
        /*Inverse FT from arbitrary complex FR*/
  
      FIRf32->channel = pCfg->channel;
      FIRq15->channel = pCfg->channel;
      FIRf32->size = *firSize;
      FIRq15->size = *firSize;

      if (RES_OK != DSP_ComputeFixedFIRCoefs(firCoeffs, FIRq15))
      {return RES_ERROR;}

      arm_fir_init_f32(f, FIRf32->size, FIRf32->coeffs, FIRf32->delays, 
      (uint32_t)(BUFFER_SIZE >> 1));

      arm_fir_init_q15(g, FIRq15->size + 1, FIRq15->coeffs, FIRq15->delays, 
      (uint32_t)(BUFFER_SIZE >> 1));
    }
    pCfg++;
    FIRf32++;
    FIRq15++;
    f++;
    g++;
  }

  return RES_OK;
}

// *****************************************************************************
static tErrorCode DSP_UpdateNormGain(tGain *normGain, double *fResponse, 
                              uint8_t nPoints,   tChannel channel)
// *****************************************************************************
// Description: Updates given normalization block with the needed gain 
// adjustement to obtain a final magnitude response that peaks at 0 dB for
// passed channel.
// Parameters: 
//   *normGain: Pointer to the normalization gain array.
//   *fResponse: Pointer to the channel's magnitude response.
//   nPoints: Number of frequency points to compute.
//   channel: Channel of the given magnitude response.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == normGain) || (NULL == fResponse)) return RES_ERROR_PARAM;

  double *maxValue;
  double inversef32 = 0;
  int16_t inverseq15 = 0;
  int32_t inverseq31 = 0;

  maxValue = (double *)UTY_GetMaxValue(fResponse, nPoints, sizeof(double), 
                                       TYPE_DOUBLE);

  inversef32 = 1.0 / *maxValue; // Get inverse gain
  
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
static tErrorCode DSP_UpdateFixedIIRs(float *coeff32, tIIRq15 *IIRq15, 
                                      tIIRq31 *IIRq31)
// *****************************************************************************
// Description: Quantizes the fixed point IIR coefficients from a given
// set of floating point coefficients
// Parameters: 
//   *coeff32: Pointer to f32 coefficient array.
//   *IIRq15: Pointer to q15 IIR filter instance.
//   *IIRq31: Pointer to q31 IIR filter instance.
// Returns: 
// *****************************************************************************
{
  if ((NULL == coeff32) || (NULL == IIRq15) || (NULL == IIRq31))
  return RES_ERROR_PARAM;

  uint8_t i;
  // Compute fixed-point coefficients
  if ((fabs(coeff32[0]) > 1.0) || (fabs(coeff32[1]) > 1.0) || 
     (fabs(coeff32[2]) > 1.0)  || (fabs(coeff32[3]) > 1.0) || 
     (fabs(coeff32[4]) > 1.0))
  {
    IIRq15->coeffs[0] = (int16_t)(coeff32[0] * SHRT_MAX/2);
    IIRq31->coeffs[0] = (int32_t)(coeff32[0] * LONG_MAX/2);

    // Must be done this way because q15 coefs are arranged differently than q31 
    for (i = 1; i < 5; i++)
    {
      IIRq15->coeffs[i+1] = (int16_t)(coeff32[i] * SHRT_MAX/2);
      IIRq31->coeffs[i] = (int32_t)(coeff32[i] * LONG_MAX/2);
    }

    IIRq15->postShift = 1;
    IIRq31->postShift = 1;
  }
  else 
  {
    IIRq15->coeffs[0] = (int16_t)(coeff32[0] * SHRT_MAX);
    IIRq31->coeffs[0] = (int32_t)(coeff32[0] * LONG_MAX);

    for (i = 1; i < 5; i++)
    {
      IIRq15->coeffs[i+1] = (int16_t)(coeff32[i] * SHRT_MAX);
      IIRq31->coeffs[i] = (int32_t)(coeff32[i] * LONG_MAX);
    }

    IIRq15->postShift = 0;
    IIRq31->postShift = 0;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_UpdateIIRs(tFilterConfig *pCfg, tIIRf32 *IIRf32, tIIRq15 *IIRq15,
                          tIIRq31 *IIRq31,
                          arm_biquad_cascade_df2T_instance_f32 *s,
                          arm_biquad_casd_df1_inst_q15 *q,
                          arm_biquad_casd_df1_inst_q31 *r,
                          tGain *normGain)
// *****************************************************************************
// Description: Updates the filter coefficients with the new parameter config
// received from master. Adapted to work with CMSIS-DSP filters from 
// Audio EQ Cookbook, by Robert Bristow-Johnson. Updates normalization gain.
// Parameters: 
//   *pCfg: Pointer to the structure containing design parameters.
//   *IIRf32: pointer to the f32 structure containing array for coefficients.
//   *IIRq15: Pointer to the q15 structure containing array for coefficients.
//   *IIRq31: Pointer to the q31 structure containing array for coefficients.
//   *s: Pointer to the f32 structure containing pointers for CMSIS-DSP filters.
//   *q: Pointer to the q15 structure containing pointers for CMSIS-DSP filters.
//   *r: Pointer to the q31 structure containing pointers for CMSIS-DSP filters.
//   *normGain: Pointer to the normalization gain array.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == pCfg) || (NULL == s) || (NULL == q) || (NULL == r) ||
      (NULL == IIRf32) || (NULL == IIRq15) || (NULL == IIRq31) || 
      (NULL == normGain)) 
  return RES_ERROR_PARAM;
  
  uint8_t i; 
  float w0 = 0;
  double a = 0;
  float A = 0;
  float *coeff; // Pointer to filter coefficient array

  for (i = 0; i < MAX_FILTERS; i++)
  {  
    if ((CHANNEL_NONE != pCfg->channel) && (FIRLOWPASS > pCfg->type))
    {
      w0 = 2 * PI * pCfg->freq / SAMPLE_RATE;
      a = sin(w0) / (2 * pCfg->q);
      A =  pow(10.0,(double)(pCfg->gain/40));
      coeff = IIRf32->coeffs; // Pointer to filter coefficient array

      // Get f32 coefficients for passed parameters
      switch (pCfg->type)
      {
        case IIRLOWPASS: 
          if (RES_OK != COEF_GetIIRLowPass(coeff, w0, a)) return RES_ERROR;
          break;
    
        case IIRHIGHPASS:
          if (RES_OK != COEF_GetIIRHighPass(coeff, w0, a)) return RES_ERROR;
          break;
    
        case IIRPEAK:
          if (RES_OK != COEF_GetIIRPeak(coeff, w0, a, A)) return RES_ERROR;
          break;
    
        case IIRLOWSHELF:
          if (RES_OK != COEF_GetIIRLowShelf(coeff, w0, a, A)) return RES_ERROR;
          break;
    
        case IIRHIGHSHELF:
          if (RES_OK != COEF_GetIIRHighShelf(coeff, w0, a, A)) return RES_ERROR;
          break;
    
        default:
          return RES_ERROR_PARAM;
      }

      // Quantize coefficients for q31 and q15 arithmetics
      if (RES_OK != DSP_UpdateFixedIIRs(coeff, IIRq15, IIRq31))
      {return RES_ERROR;}
  
      // Update CMSIS-DSP library instances with computed coefficients
      arm_biquad_cascade_df2T_init_f32(s, 1, IIRf32->coeffs, IIRf32->delays);
      arm_biquad_cascade_df1_init_q15(q, 1, IIRq15->coeffs, IIRq15->delays, 
                                      IIRq15->postShift);
      arm_biquad_cascade_df1_init_q31(r, 1, IIRq31->coeffs, IIRq31->delays, 
                                      IIRq31->postShift);
  
      // Assign channel to calculated filters
      IIRf32->channel = pCfg->channel;
      IIRq15->channel = pCfg->channel;
      IIRq31->channel = pCfg->channel;
    }

    // Increment pointers for next filter
    IIRf32++;
    IIRq31++;
    IIRq15++;
    q++;
    r++;
    s++;
    pCfg++;
  }

  // Reset f32 pointer to first filter
  IIRf32 -= MAX_FILTERS;
  
  // Normalize channel gain for magnitude response of calculated filters
  double fResponse[PLOT_RESOLUTION];
  for (i = CHANNEL_0; i < MAX_CHANNELS; i++)
  {
    if (RES_OK != DSP_PlotMagnitude(IIRf32, fResponse, i)) return RES_ERROR;
    if (RES_OK != DSP_UpdateNormGain(normGain, fResponse, PLOT_RESOLUTION, i)) 
    {return RES_ERROR;}
    normGain++;
  }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_SumChannel(void *dstBuf, void *srcBuf, uint32_t nSamples, 
                          tArithmetic bufType)
// *****************************************************************************
// Description: Dumps one channel's samples into another and normalizes gain. 
// Both buffers must be equal in length.
// Parameters: 
//   *dstBuf: Destination buffer where source samples will be dumped and summed
//   *srcBuf: Source buffer with samples to dump into destination
//   nSamples: Length of any of the buffer in samples.
//   bufType: Type of arithmetic to use.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == dstBuf) || (NULL == srcBuf) || (nSamples < 1)) 
  return RES_ERROR_PARAM;

  // Sum source and destination
  switch (bufType)
  {
    case Q15:
      arm_add_q15((q15_t *)dstBuf, (q15_t *)srcBuf, (q15_t *)dstBuf, nSamples);
      break;
    case Q31:
      arm_add_q31((q31_t *)dstBuf, (q31_t *)srcBuf, (q31_t *)dstBuf, nSamples);
      break;
    case F32:
      arm_add_f32((float32_t *)dstBuf, (float32_t *)srcBuf, 
                  (float32_t *)dstBuf, nSamples);
      break;

    default:
      return RES_ERROR_PARAM;
  }
  // Normalize in case the result exceeds -0dB

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Int24ToInt16(int32_t *inBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  nSamples >>= 2;

  while(nSamples--)
  {
    *inBuf >>= 8; 
    inBuf++;
    *inBuf >>= 8; 
    inBuf++;
    *inBuf >>= 8; 
    inBuf++;
    *inBuf >>= 8; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Int24ToInt32(int32_t *inBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  nSamples >>= 2;

  while(nSamples--)
  {
    *inBuf <<= 8; 
    inBuf++;
    *inBuf <<= 8; 
    inBuf++;
    *inBuf <<= 8; 
    inBuf++;
    *inBuf <<= 8; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Int16ToInt32(int32_t *inBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  nSamples >>= 2;

  while(nSamples--)
  {
    *inBuf <<= 16; 
    inBuf++;
    *inBuf <<= 16; 
    inBuf++;
    *inBuf <<= 16; 
    inBuf++;
    *inBuf <<= 16; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_Int32ToInt16(int32_t *inBuf, uint16_t nSamples)
// *****************************************************************************
// Description: Converts received samples from signed 24 bit to signed 16 bit
// Parameters:
//   inBuf: Pointer to the first input buffer position to be converted
//   nSamples: Number of samples to convert 
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf)) return RES_ERROR_PARAM;
  
  nSamples >>= 2;

  while(nSamples--)
  {
    *inBuf >>= 16; 
    inBuf++;
    *inBuf >>= 16; 
    inBuf++;
    *inBuf >>= 16; 
    inBuf++;
    *inBuf >>= 16; 
    inBuf++;
  }
  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_DecodePCM_Int16(int32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                         uint16_t nSamples)
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
  
  nSamples >>= 2;

  while(nSamples--)
  {
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
  }

  return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_DecodePCM_Int32(int32_t *inBuf, int32_t *bufL, int32_t *bufR, 
                         uint16_t nSamples)
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

  nSamples >>= 2;
    
  while(nSamples--)
  {
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
    *bufL++ = *inBuf++;
    *bufR++ = *inBuf++;
  }

    return RES_OK;
}

// *****************************************************************************
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                         uint16_t nSamples)
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

  nSamples >>= 2;
  
  while(nSamples--)
  {
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
    *outBuf++ = *bufL++;
    *outBuf++ = *bufR++;
  }

  return RES_OK;
}

// *****************************************************************************
void DSP_TestFilters(tFilterConfig *pCfg)
// *****************************************************************************
// Description: Set filter parameters for testing purposes
// Parameters: 
//   *pCfg: Pointer to structure containing filter parameters
// Returns: Nothing
// *****************************************************************************
{
  pCfg->freq =  80;
  pCfg->q =  0;
  pCfg->gain =  0;
  pCfg->type =  FIRLOWPASS;
  pCfg->channel =  CHANNEL_NONE;
  pCfg++;
  pCfg->freq =  3000;
  pCfg->q =  0;
  pCfg->gain =  0;
  pCfg->type =  FIRHIGHPASS;
  pCfg->channel =  CHANNEL_NONE;
  pCfg++;
}
