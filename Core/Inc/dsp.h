// *****************************************************************************
// File: dsp.h
// Description: header file containing exported functions from dsp.c
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
#ifndef _ARM_MATH_H
#include <arm_math.h>
#endif
#ifndef GLOBAL_H
#include "global.h"
#endif

// *****************************************************************************
// Defines 
#define DSP_H

// Global DSP system configuration
#define BUFFER_SIZE  2048 // Must be even and +2 times greater than largest IR
#define MAX_FILTERS  8
#define FIR_MAX_SIZE 256
#define SAMPLE_RATE  48000

// Hardware configuration
#define DAC_QUANTITY 1
#define ADC_QUANTITY 1
#define MAX_CHANNELS 4

// Configure processing algorithm method
#define USE_LIBRARY  // Use CMSIS-DSP library functions if defined

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
  CHANNEL_NONE = 255
} tGeneralChannel;

typedef enum
{
  IIRLOWPASS   = 0,
  IIRHIGHPASS  = 1,
  IIRPEAK      = 2,
  IIRLOWSHELF  = 3,
  IIRHIGHSHELF = 4,
  FIRLOWPASS = 5,
  FIRHIGHPASS = 6
} tTypeIIR;

typedef struct
{
  float coeffs[FIR_MAX_SIZE];
  float delays[FIR_MAX_SIZE + BUFFER_SIZE/2 -1];
  uint16_t size;
  tGeneralChannel channel;
} tInstanceFIRf32;

typedef struct // Direct form II
{
  float coeffs[5]; // order is b0, b1, b2, a1, a2,
  float delays[2]; // order is t1, t2
  tGeneralChannel channel;
} tInstanceIIRf32;

typedef struct // Direct form I
{
  int16_t coeffs[6]; // order is b0, 0, b1, b2, a1, a2,
  int16_t delays[4]; // order is x[n-1], x[n-2], y[n-1], y[n-2]
  tGeneralChannel channel;
  int8_t postShift;
} tInstanceIIRq15;

typedef struct // Direct form I
{
  int32_t coeffs[5]; // order is b0, b1, b2, a1, a2,
  int32_t delays[4]; // order is x[n-1], x[n-2], y[n-1], y[n-2]
  tGeneralChannel channel;
  int8_t postShift;
} tInstanceIIRq31;

typedef struct 
{
  float f32;
  int16_t q15;
  int32_t q31;
  int8_t postShift;
} tGain;

typedef struct
{
  float freq, q, gain;
  tTypeIIR type;
  tGeneralChannel channel;
} tParamConfig;

// *****************************************************************************
// Functions 

// FIR Filters
tErrorCode DSP_FIR_f32(float *buf, uint16_t nSamples, tInstanceFIRf32 *inst);
tErrorCode DSP_FIR_f32_arm(float *buf, uint16_t nSamples, 
                           arm_fir_instance_f32 *f);

// IIR Filters
tErrorCode DSP_IIR_f32(float *buf, uint32_t nSamples, tInstanceIIRf32 *inst);
tErrorCode DSP_IIR_q15(int16_t *buf, uint32_t nSamples, tInstanceIIRq15 *inst);
tErrorCode DSP_IIR_q31(int32_t *buf, uint32_t nSamples, tInstanceIIRq31 *inst);
tErrorCode DSP_IIR_f32_arm(float *buf, uint32_t nSamples, 
                            arm_biquad_cascade_df2T_instance_f32 *s);
tErrorCode DSP_IIR_q15_arm(int16_t *buf, uint32_t nSamples, 
                           arm_biquad_casd_df1_inst_q15 *q);
tErrorCode DSP_IIR_q31_arm(int32_t *buf, uint32_t nSamples, 
                           arm_biquad_casd_df1_inst_q31 *r);

// Gain
tErrorCode DSP_Gain_f32_arm(float *buf, float gain, int32_t nSamples);
tErrorCode DSP_Gain_q15_arm(int16_t *buf, int16_t gain, int8_t postShift, 
                            int32_t nSamples);
tErrorCode DSP_Gain_q31_arm(int32_t *buf, int32_t gain, int8_t postShift, 
                            int32_t nSamples);

// Arithmetic conversion
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint32_t nSamples);
tErrorCode DSP_q31_to_f32_arm(int32_t *inBuf, float *outBuf, uint32_t nSamples);
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint32_t nSamples);
tErrorCode DSP_q31_to_q15_arm(int32_t *inBuf, int16_t *outBuf, 
                              uint32_t nSamples);

// Encoding/Decoding
tErrorCode DSP_Int24ToInt16(uint32_t *inBuf, uint32_t nSamples);
tErrorCode DSP_Int24ToInt32(uint32_t *inBuf, uint32_t nSamples);
tErrorCode DSP_DecodePCM_Int16(uint32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                               uint32_t nSamples);
tErrorCode DSP_DecodePCM_Int32(uint32_t *inBuf, int32_t *bufL, int32_t *bufR, 
                               uint32_t nSamples);
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                               uint32_t nSamples);

// Coefficient computation
tErrorCode DSP_UpdateFIRInstances(tParamConfig *pCfg, tInstanceFIRf32 *instf32,
                                  arm_fir_instance_f32 *f, uint16_t nSamples);

tErrorCode DSP_UpdateIIRInstances(tParamConfig *pCfg, 
                                  tInstanceIIRf32 *instf32,
                                  tInstanceIIRq15 *instq15,
                                  tInstanceIIRq31 *instq31, 
                                  arm_biquad_cascade_df2T_instance_f32 *s,
                                  arm_biquad_casd_df1_inst_q15 *q,
                                  arm_biquad_casd_df1_inst_q31 *r,
                                  tGain *normGain);

// Others
tErrorCode DSP_CheckConfig(uint16_t bufSize, tInstanceFIRf32 *instf32, 
                           uint16_t nAdc, uint16_t nDac);
void DSP_TestFilters(tParamConfig *pCfg);
void DSP_Init(tParamConfig *pCfg, tInstanceFIRf32 *instFIRf32, 
              tInstanceIIRf32 *instf32, tInstanceIIRq15 *instq15, 
              arm_biquad_cascade_df2T_instance_f32 *s,
              arm_biquad_casd_df1_inst_q15 *q, tGain *staticGain);

