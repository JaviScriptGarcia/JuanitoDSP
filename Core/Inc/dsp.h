// *****************************************************************************
// File: dsp.h
// Description: header file containing exported functions from dsp.c
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
// #include <arm_math.h>
#include <math.h>
#include "global.h"


// *****************************************************************************
// Defines 
#ifndef __FPU_PRESENT
#define __FPU_PRESENT
#endif

#define MAX_FILTERS           8
#define SAMPLE_RATE           48000

// *****************************************************************************
//  Variables

typedef struct
{
  int32_t t1, t2;
} tInstanceIIR;

typedef struct 
{
  float b0, b1, b2, a1, a2;
} tCoeffsIIR;

typedef struct
{
  float freq, q, gain;
} tParamConfig;



// *****************************************************************************
// Functions 

tErrorCode DSP_SecondOrderIIR(int16_t *buf, uint16_t nSamples, 
                             tInstanceIIR *inst, tCoeffsIIR coeff);
void DSP_TestFilterInstances(tCoeffsIIR *coeffs);
void DSP_Init(tInstanceIIR *inst, tCoeffsIIR *coeffs);
