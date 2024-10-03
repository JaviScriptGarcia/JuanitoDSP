// *****************************************************************************
// File: uty.c
// Description: Utility functions for general purposes
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include "uty.h"

// *****************************************************************************
// Defines 

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

// *****************************************************************************
void *UTY_GetMaxValue(void *array, uint32_t length, size_t size, tDataType type)
// *****************************************************************************
// Description: Finds the highest value of the passed array
// Parameters: 
//   *array: Pointer to the array containing data
//   *length: Number of array positions to scan
// Returns: Pointer to the direction of the found array position
// *****************************************************************************
{
  if (array == NULL || length == 0) return NULL;

  void *maxValue = array;
  
  if (TYPE_FLOAT == type) 
  {
    while (0 < length--)
    {
      if (*(float *)array > *(float *)maxValue) maxValue = array;
      array += size;
    }
  }
  else if (TYPE_DOUBLE == type) 
  {
    while (0 < length--)
    {
      if (*(double *)array > *(double *)maxValue) maxValue = array;
      array += size;
    }
  }
  else 
  {
    while (0 < length--)
    {
      if (memcmp((char *)array, (char *)maxValue, size) > 0) maxValue = array;
      array += size;
    }
  }

  return maxValue;
}

// *****************************************************************************
float LinearInterpolation_Uint16ToFloat(uint16_t x, uint16_t xMin, 
                                        uint16_t xMax, float yMin, float yMax)
// *****************************************************************************
// Description: Obtains the proportional floating point equivalent of an integer
// whithin a restrained range for both variables. 
// Parameters: 
//   x: Uint16 value to convert
//   xMin: Minimum possible value of the integer input
//   xMax: Maximum possible value of the integer input
//   yMin: Minimum possible value of the floating point output
//   yMax: Maximum possible value of the floating point output
// Returns: Floating point equivalent
// *****************************************************************************
{
  float y;

  y = yMin + ((yMax - yMin) * (float)(x - xMin) / (float)(xMax - xMin));
  return y;
}

