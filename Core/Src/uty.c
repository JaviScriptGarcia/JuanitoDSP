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
