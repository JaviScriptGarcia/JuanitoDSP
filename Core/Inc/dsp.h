// *****************************************************************************
// File: dsp.h
// Description: header file containing struct definitions and exported functions
// from dsp.c
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
#define BUFFER_SIZE  256 // Must be even and +2 times greater than largest IR
#define MAX_CONV     2
#define MAX_FILTERS  4
#define FIR_MAX_SIZE BUFFER_SIZE/2
#define CONV_MAX_SIZE 8192
#define SAMPLE_RATE  48000

// Hardware configuration
#define OUTPUT_QUANTITY 1
#define INPUT_QUANTITY 2
#define MAX_CHANNELS 4

// Frequency response plot parameters
#define PLOT_RESOLUTION       100
#define FRANGE_MIN            20.0    //Hz
#define FRANGE_MAX            20000.0 //Hz

// Filter configuration limits
#define GAIN_MIN  -15.0
#define GAIN_MAX   15.0
#define Q_MIN      0.1
#define Q_MAX      10.0

// Configure processing algorithm method
#define USE_LIBRARY  // Use CMSIS-DSP library functions if defined

// Macros
#define c2d(a) (a/((double)SAMPLE_RATE/2)) // Convert to discrete frequency

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
} tChannel;

typedef enum
{
  IIRLOWPASS   = 0,
  IIRHIGHPASS  = 1,
  IIRPEAK      = 2,
  IIRLOWSHELF  = 3,
  IIRHIGHSHELF = 4,
  FIRLOWPASS   = 5,
  FIRHIGHPASS  = 6
} tFiltType;

typedef struct
{
  int16_t coeffs[CONV_MAX_SIZE];
  int16_t tail[CONV_MAX_SIZE];
  uint16_t size;
  tChannel channel;
} tConvq15;

typedef struct
{
  float coeffs[FIR_MAX_SIZE];
  float delays[FIR_MAX_SIZE + BUFFER_SIZE/2 - 1];
  uint16_t size;
  tChannel channel;
} tFIRf32;

typedef struct
{
  int16_t coeffs[FIR_MAX_SIZE];
  int16_t delays[FIR_MAX_SIZE + BUFFER_SIZE/2 - 1];
  uint16_t size;
  tChannel channel;
} tFIRq15;

typedef struct // Direct form II
{
  float coeffs[5]; // order is b0, b1, b2, a1, a2,
  float delays[2]; // order is t1, t2
  tChannel channel;
} tIIRf32;

typedef struct // Direct form I
{
  int16_t coeffs[6]; // order is b0, 0, b1, b2, a1, a2,
  int16_t delays[4]; // order is x[n-1], x[n-2], y[n-1], y[n-2]
  tChannel channel;
  int8_t postShift;
} tIIRq15;

typedef struct // Direct form I
{
  int32_t coeffs[5]; // order is b0, b1, b2, a1, a2,
  int32_t delays[4]; // order is x[n-1], x[n-2], y[n-1], y[n-2]
    tChannel channel;
  int8_t postShift;
} tIIRq31;

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
  tFiltType type;
  tChannel channel;
} tFilterConfig;

typedef enum
{
  F32,
  Q15,
  Q31
} tArithmetic;

// *****************************************************************************
// Functions 

// Convolution
tErrorCode DSP_Convolution_q15_arm(int16_t *buf, uint16_t nSamples, 
                                   tConvq15 *ir);

// FIR Filters
tErrorCode DSP_FIR_f32(float *buf, uint16_t nSamples, tFIRf32 *inst);
tErrorCode DSP_FIR_q15(int16_t *buf, uint16_t nSamples, tFIRq15 *fir);
tErrorCode DSP_FIR_f32_arm(float *buf, uint16_t nSamples, 
                           arm_fir_instance_f32 *f);
tErrorCode DSP_FIR_q15_arm(int16_t *buf, uint16_t nSamples, 
                           arm_fir_instance_q15 *g);

// IIR Filters
tErrorCode DSP_IIR_f32(float *buf, uint16_t nSamples, tIIRf32 *inst);
tErrorCode DSP_IIR_q15(int16_t *buf, uint16_t nSamples, tIIRq15 *inst);
tErrorCode DSP_IIR_q31(int32_t *buf, uint16_t nSamples, tIIRq31 *inst);
tErrorCode DSP_IIR_f32_arm(float *buf, uint16_t nSamples, 
                            arm_biquad_cascade_df2T_instance_f32 *s);
tErrorCode DSP_IIR_q15_arm(int16_t *buf, uint16_t nSamples, 
                           arm_biquad_casd_df1_inst_q15 *q);
tErrorCode DSP_IIR_q31_arm(int32_t *buf, uint16_t nSamples, 
                           arm_biquad_casd_df1_inst_q31 *r);

// Gain
tErrorCode DSP_Gain_f32_arm(float *buf, float gain, uint16_t nSamples);
tErrorCode DSP_Gain_q15_arm(int16_t *buf, int16_t gain, int8_t postShift, 
                            uint16_t nSamples);
tErrorCode DSP_Gain_q31_arm(int32_t *buf, int32_t gain, int8_t postShift, 
                            uint16_t nSamples);

// Arithmetic conversion
tErrorCode DSP_q15_to_f32_arm(int16_t *inBuf, float *outBuf, uint16_t nSamples);
tErrorCode DSP_q31_to_f32_arm(int32_t *inBuf, float *outBuf, uint16_t nSamples);
tErrorCode DSP_f32_to_q15_arm(float *inBuf, int16_t *outBuf, uint16_t nSamples);
tErrorCode DSP_q31_to_q15_arm(int32_t *inBuf, int16_t *outBuf, 
                              uint16_t nSamples);

// Coefficient calculation
tErrorCode DSP_UpdateConvolutionInstances(tConvq15 *conv);

tErrorCode DSP_UpdateFIRInstances(tFilterConfig *pCfg, tFIRf32 *FIRf32, 
                                  tFIRq15 *FIRq15, arm_fir_instance_f32 *f,
                                  arm_fir_instance_q15 *g);

tErrorCode DSP_UpdateIIRs(tFilterConfig *pCfg, 
                                  tIIRf32 *instf32,
                                  tIIRq15 *instq15,
                                  tIIRq31 *instq31, 
                                  arm_biquad_cascade_df2T_instance_f32 *s,
                                  arm_biquad_casd_df1_inst_q15 *q,
                                  arm_biquad_casd_df1_inst_q31 *r,
                                  tGain *normGain);

// Encoding/Decoding
tErrorCode DSP_Int24ToInt16(int32_t *inBuf, uint16_t nSamples);
tErrorCode DSP_Int24ToInt32(int32_t *inBuf, uint16_t nSamples);
tErrorCode DSP_Int16ToInt32(int32_t *inBuf, uint16_t nSamples);
tErrorCode DSP_Int32ToInt16(int32_t *inBuf, uint16_t nSamples);
tErrorCode DSP_DecodePCM_Int16(int32_t *inBuf, int16_t *bufL, int16_t *bufR, 
                               uint16_t nSamples);
tErrorCode DSP_DecodePCM_Int32(int32_t *inBuf, int32_t *bufL, int32_t *bufR, 
                               uint16_t nSamples);
tErrorCode DSP_EncodePCM(int16_t *outBuf, int16_t *bufL, int16_t *bufR, 
                               uint16_t nSamples);

// Others
void DSP_TestFilters(tFilterConfig *pCfg);
tErrorCode DSP_SumChannel(void *dstBuf, void *srcBuf, uint32_t nSamples, 
                          tArithmetic bufType);

