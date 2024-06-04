// *****************************************************************************
// File: coeffs.c
// Description: Source file of coefficient calculation functions for DSP 
// algorithms used in dsp.c
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include "dsp.h"

// *****************************************************************************
// Defines 

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

// *****************************************************************************
tErrorCode COEF_GetFIRCoeffsf32(float fc, float *coeffs, uint16_t *firSize)
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
tErrorCode COEF_GetIIRLowPass(float *coeff, float w0, double a)
// *****************************************************************************
// Description: Calculates f32 coefficients for a biquad IIR lowpass filter. 
// Coefficients are normalized so that a0 = 1.
// Parameters: 
//   *coeff: Pointer to array containing filter coefficients.
//   w0: Natural frequency of the filter.
//   a: Alpha parameter.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == coeff) || (PI < w0)) return RES_ERROR_PARAM;

  float a0 =                  1 + a;
  coeff[3] =     (2 * cos(w0)) / a0;  // a1
  coeff[4] =         - (1 - a) / a0;  // a2
  coeff[0] = ((1 - cos(w0))/2) / a0;  // b0
  coeff[1] =           2 * coeff[0];  // b1
  coeff[2] =               coeff[0];  // b2

  return RES_OK;
}

// *****************************************************************************
tErrorCode COEF_GetIIRHighPass(float *coeff, float w0, double a)
// *****************************************************************************
// Description: Calculates f32 coefficients for a biquad IIR highpass filter. 
// Coefficients are normalized so that a0 = 1.
// Parameters: 
//   *coeff: Pointer to array containing filter coefficients.
//   w0: Natural frequency of the filter.
//   a: Alpha parameter.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == coeff) || (PI < w0)) return RES_ERROR_PARAM;

  float a0 =                  1 + a;
  coeff[3] =  - (-2 * cos(w0)) / a0;  // a1
  coeff[4] =         - (1 - a) / a0;  // a2
  coeff[0] = ((1 + cos(w0))/2) / a0;  // b0
  coeff[1] =          -2 * coeff[0];  // b1
  coeff[2] =               coeff[0];  // b2

  return RES_OK;
}

// *****************************************************************************
tErrorCode COEF_GetIIRPeak(float *coeff, float w0, double a, float A)
// *****************************************************************************
// Description: Calculates f32 coefficients for a biquad IIR peaking filter. 
// Coefficients are normalized so that a0 = 1.
// Parameters: 
//   *coeff: Pointer to array containing filter coefficients.
//   w0: Natural frequency of the filter.
//   a: Alpha parameter.
//   A: Amplitude parameter.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == coeff) || (PI < w0)) return RES_ERROR_PARAM;

  float a0 =           1 + (a / A);
  coeff[3] = - (-2 * cos(w0)) / a0;  // a1
  coeff[4] =  - (1 - (a / A)) / a0;  // a2
  coeff[0] =    (1 + (a * A)) / a0;  // b0
  coeff[1] =            - coeff[3];  // b1
  coeff[2] =    (1 - (a * A)) / a0;  // b2

  return RES_OK;
}

// *****************************************************************************
tErrorCode COEF_GetIIRLowShelf(float *coeff, float w0, double a, float A)
// *****************************************************************************
// Description: Calculates f32 coefficients for a biquad IIR lowshelf filter. 
// Coefficients are normalized so that a0 = 1.
// Parameters: 
//   *coeff: Pointer to array containing filter coefficients.
//   w0: Natural frequency of the filter.
//   a: Alpha parameter.
//   A: Amplitude parameter.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == coeff) || (PI < w0)) return RES_ERROR_PARAM;

  float a0 =            (1 + A) + (A - 1) * cos(w0) + 2 * a * sqrt(A);
  coeff[3] =                   2 * ((A - 1) + (A + 1) * cos(w0)) / a0;
  coeff[4] =   - ((1 + A) + (A - 1) * cos(w0) - 2 * a * sqrt(A)) / a0;
  coeff[0] = A * ((A + 1) - (A - 1) * cos(w0) + 2 * sqrt(A) * a) / a0;
  coeff[1] =               2 * A * ((A - 1) - (A + 1) * cos(w0)) / a0;
  coeff[2] = A * ((A + 1) - (A - 1) * cos(w0) - 2 * sqrt(A) * a) / a0;

  return RES_OK;
}

// *****************************************************************************
tErrorCode COEF_GetIIRHighShelf(float *coeff, float w0, double a, float A)
// *****************************************************************************
// Description: Calculates f32 coefficients for a biquad IIR highshelf filter. 
// Coefficients are normalized so that a0 = 1.
// Parameters: 
//   *coeff: Pointer to array containing filter coefficients.
//   w0: Natural frequency of the filter.
//   a: Alpha parameter.
//   A: Amplitude parameter.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == coeff) || (PI < w0)) return RES_ERROR_PARAM;

  float a0 =           (1 + A) - (A - 1) * cos(w0) + 2 * a * sqrt(A);
  coeff[3] =                  -2 * ((A - 1) - (A + 1) * cos(w0)) / a0;
  coeff[4] =   - ((1 + A) - (A - 1) * cos(w0) - 2 * a * sqrt(A)) / a0;
  coeff[0] = A * ((A + 1) + (A - 1) * cos(w0) + 2 * sqrt(A) * a) / a0;
  coeff[1] =             - 2 * A * ((A - 1) + (A + 1) * cos(w0)) / a0;
  coeff[2] = A * ((A + 1) + (A - 1) * cos(w0) - 2 * sqrt(A) * a) / a0;

  return RES_OK;
}