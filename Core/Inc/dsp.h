// *****************************************************************************
// File: dsp.h
// Description: header file containing exported functions from dsp.c
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
#include <arm_math.h>
#include <math.h>
#include "global.h"


// *****************************************************************************
// Defines 
#ifndef __FPU_PRESENT
#define __FPU_PRESENT
#endif

#define MAX_FILTERS           8
#define SAMPLE_RATE           48000
#define USE_LIBRARY  // Use CMSIS-DSP library for ARM microprocessors

// *****************************************************************************
//  Variables

typedef struct 
{
  float coeffs[5]; // order is b0, b1, b2, a1, a2,
  float delays[2]; // order is t1, t2
} tInstanceIIR;

typedef enum
{
  LOWPASS   = 0,
  HIGHPASS  = 1,
  PEAK      = 2,
  LOWSHELF  = 3,
  HIGHSHELF = 4
} tTypeIIR;

typedef struct
{
  float freq, q, gain;
  tTypeIIR type;
} tParamConfig;



// *****************************************************************************
// Functions 

// Filters
tErrorCode DSP_IIR_f32(int16_t *buf, uint32_t nSamples, tInstanceIIR *inst);
tErrorCode DSP_IIR_f32_arm(int16_t *buf, uint32_t nSamples, 
                            arm_biquad_cascade_df2T_instance_f32 *s);

// Encoding/Decoding
tErrorCode DSP_Uint24ToInt16(uint32_t *inBuf, uint16_t nSamples);
tErrorCode DSP_DecodePCM(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                               uint16_t nSamples);
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                               uint16_t nSamples);

// Coefficient computation
tErrorCode DSP_UpdateFilterInstances(tParamConfig *pCfg, tInstanceIIR *inst, 
                                     arm_biquad_cascade_df2T_instance_f32 *s);

// Others
void DSP_TestFilters(tParamConfig *pCfg);
void DSP_Init(tInstanceIIR *inst);

