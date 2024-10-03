// *****************************************************************************
// File: bluetooth.c
// Description: hardware library for bluetooth module source file
// *****************************************************************************

// *****************************************************************************
// Includes
#include "bluetooth.h"
#include "uty.h"
#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

// *****************************************************************************
// Defines 

// **************** START OF FRAME *********************************************
#define SOF 2517981213

// **************** Frame sections length **************************************
#define PASS_BYTES 4
#define START_BYTES 8
#define FILTER_FRAME_BYTES 5
#define FILTER_FREQ_BYTES 2
#define FILTER_Q_BYTES 1
#define FILTER_GAIN_BYTES 1
#define FILTER_TYPE_BYTES 1
#define FILTER_CHAN_BYTES 1

// ***************** Other *****************************************************
#define USHRT_MIN 0
#define UCHAR_MIN 0

// *****************************************************************************
// Variables 
const enum
{
  TEST,
  ACKNOWLEDGE,
  RXFAILED,
  MAX_COMMANDS // Add new commands above this line
} cmdIdx;

const char *txCmd[MAX_COMMANDS] =
{
  "TEST",
  "ACK",
  "RXFAILED"
};

// *****************************************************************************
// Functions 

// *****************************************************************************
void BT_Init(void)
// *****************************************************************************
// Description: Initializes the bluetooth module
// Parameters: none
// Returns: nothing
// *****************************************************************************
{
  HAL_GPIO_WritePin(SYS_CTRL_GPIO, SYS_CTRL_PIN, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(SYS_CTRL_GPIO, SYS_CTRL_PIN, GPIO_PIN_SET);
}

// *****************************************************************************
tErrorCode BT_Send_Ack(char *buf)
// *****************************************************************************
// Description: Sends acknowledge string.
// Parameters: 
//   *buf: Pointer to the transmit buffer.
// Returns: Error code.
// *****************************************************************************
{
  if (NULL == buf) return RES_ERROR;

  buf[0] = strlen(txCmd[ACKNOWLEDGE]) + BYTES_FOR_LEN;
  memcpy(&buf[1], &txCmd[ACKNOWLEDGE], sizeof(txCmd[ACKNOWLEDGE]));
  
  return RES_OK;
}

// *****************************************************************************
tErrorCode BT_Send_RxFail(char *buf)
// *****************************************************************************
// Description: Sends error string.
// Parameters: 
//   *buf: Pointer to the transmit buffer.
// Returns: Error code.
// *****************************************************************************
{
  if (NULL == buf) return RES_ERROR;

  buf[0] = strlen(txCmd[RXFAILED]) + BYTES_FOR_LEN;
  memcpy(&buf[1], &txCmd[RXFAILED], sizeof(txCmd[RXFAILED]));
  
  return RES_OK;
}

// *****************************************************************************
tErrorCode BT_GetStart(char *buf, uint16_t *len, tRxCommand *command)
// *****************************************************************************
// Description: Finds the start code in passed frame and gets command and length
// Parameters: 
//   *buf: pointer to the buffer storing the received data
// Returns: Error code
// *****************************************************************************
{
  if (NULL == buf) return RES_ERROR;
  // Pass: 29-88-21-150 HEX: 1D 58 15 96
  if (*((uint32_t*)buf) != SOF) return RES_ERROR; 
  else if (buf[7] != 0) return RES_ERROR;
  else
  {
    *len  = *(uint16_t*)(&buf[4]);
    *command = (tRxCommand)buf[6];
  }

  return RES_OK;
}

// *****************************************************************************
tErrorCode BT_GetUartSt(UART_HandleTypeDef *huart, tUartSt *uartSt)
// *****************************************************************************
// Description: Gets the state of the UART preripheral by reading its registers.
// Parameters: 
//   *huart: Pointer to the uart handler.
//   *uartSt: Pointer to the uart state variable.
// Returns: Error code.
// *****************************************************************************
{
  uint16_t bitField = ((huart->RxState) | ((huart->gState) << 8));
  switch (bitField)
  {
    case 0b0010000000100000:
      *uartSt = IDLE;
      break;

    case 0b0010000000100010:
      *uartSt = RXBUSY;
      break;
    
    case 0b0010000100100000:
      *uartSt = TXBUSY;
      break;

    default: 
    *uartSt = UARTERROR;
    return RES_ERROR;
  }
  
  return RES_OK;
}

// *****************************************************************************
tErrorCode BT_ParseMsg(char *rxBuf, char *txBuf, uint16_t len, 
                      tRxCommand command, tFilterConfig *pCfg, 
                      uint8_t *filterConfigFlag)
// *****************************************************************************
// Description: Parses received message and does actions based on the command 
// received.
// Parameters: 
//   *buf: Pointer to buffer to be parsed.
//   len: Length of the received message.
//   command: Command to be executed.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == rxBuf) || (NULL == txBuf) || (START_BYTES > len)) return RES_ERROR;

  switch (command)
  {
    case NONE:
      break;
    
    case DATA:
      break;
    
    case APP_START:
      break;
    
    case SET_FILTERS:
      uint8_t i;
      // CHECK THE CRC FIRST
      for (i = START_BYTES; i < len;)
      {
        pCfg->freq = LinearInterpolation_Uint16ToFloat(*((uint16_t *)&rxBuf[i]),
                                  USHRT_MIN, USHRT_MAX, FRANGE_MIN, FRANGE_MAX);
        i += FILTER_FREQ_BYTES;
        pCfg->q = LinearInterpolation_Uint16ToFloat(rxBuf[i], UCHAR_MIN, 
                                               UCHAR_MAX, Q_MIN, Q_MAX);
        i += FILTER_Q_BYTES;
        pCfg->gain = LinearInterpolation_Uint16ToFloat(rxBuf[i], UCHAR_MIN, 
                                            UCHAR_MAX, GAIN_MIN, GAIN_MAX);
        i += FILTER_GAIN_BYTES;
        pCfg->type = (tFiltType)rxBuf[i];
        i += FILTER_TYPE_BYTES;
        pCfg->channel = (tChannel)rxBuf[i];
        i += FILTER_CHAN_BYTES;

        pCfg++;
      }

      *filterConfigFlag = 1;
      break;
    
    case SET_CONVOLUTION:
      break;
    
    case STORE_IR:
      break;
    
    case SET_BUFFER_SIZE:
      break;
    
    default:
    return RES_ERROR;
  }  

  return RES_OK;
}