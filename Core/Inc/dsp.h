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

typedef enum
{
  CHANNEL_0 = 0,
  CHANNEL_1 = 1,
  CHANNEL_2 = 2,
  CHANNEL_3 = 3,
  CHANNEL_4 = 4,
  CHANNEL_5 = 5,
  CHANNEL_6 = 6,
  CHANNEL_7 = 7,
  CHANNEL_NONE = 256
} tGeneralChannel;

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
  float coeffs[5]; // order is b0, b1, b2, a1, a2,
  float delays[2]; // order is t1, t2
  tGeneralChannel channel;
} tInstanceIIR;

typedef struct
{
  float freq, q, gain;
  tTypeIIR type;
  tGeneralChannel channel;
} tParamConfig;

// *****************************************************************************
// Functions 

// Filters
tErrorCode DSP_IIR_f32(float *buf, uint32_t nSamples, tInstanceIIR *inst);
tErrorCode DSP_IIR_f32_arm(float *buf, uint32_t nSamples, 
                            arm_biquad_cascade_df2T_instance_f32 *s);
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint32_t nSamples);
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint32_t nSamples);

// Encoding/Decoding
tErrorCode DSP_Int24ToInt16(uint32_t *inBuf, uint32_t nSamples);
tErrorCode DSP_DecodePCM(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                               uint32_t nSamples);
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                               uint32_t nSamples);

// Coefficient computation
tErrorCode DSP_UpdateFilterInstances(tParamConfig *pCfg, tInstanceIIR *inst, 
                                     arm_biquad_cascade_df2T_instance_f32 *s);

// Others
void DSP_TestFilters(tParamConfig *pCfg);
void DSP_Init(tParamConfig *pCfg, tInstanceIIR *inst,
              arm_biquad_cascade_df2T_instance_f32 *s);

