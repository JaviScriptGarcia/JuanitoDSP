// *****************************************************************************
// File: uty.h
// Description: Exported utility functions for general purposes
// *****************************************************************************

// *****************************************************************************
// Includes
#include <stdio.h>
#include <stdint.h>
#ifndef GLOBAL_H
#include "global.h"
#endif

// *****************************************************************************
// Defines 
#define UTY_H

// *****************************************************************************
// Variables 

// *****************************************************************************
// Functions 

void *UTY_GetMaxValue(void *array, uint32_t length, size_t size, 
                      tDataType type);