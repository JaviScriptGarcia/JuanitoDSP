// *****************************************************************************
// File: 
// Description: 
// *****************************************************************************

// *****************************************************************************
// Includes

#ifndef GLOBAL_H
#include "global.h"
#endif

// *****************************************************************************
// Defines 

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

// FIR windowed-sinc coefficients
tErrorCode COEF_GetFIRCoeffsf32(float fc, float *coeffs, uint16_t *firSize);

// IIR biquad coefficients
tErrorCode COEF_GetIIRLowPass(float *coeff, float w0, double a);
tErrorCode COEF_GetIIRHighPass(float *coeff, float w0, double a);
tErrorCode COEF_GetIIRPeak(float *coeff, float w0, double a, float A);
tErrorCode COEF_GetIIRLowShelf(float *coeff, float w0, double a, float A);
tErrorCode COEF_GetIIRHighShelf(float *coeff, float w0, double a, float A);
