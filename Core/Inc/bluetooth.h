// *****************************************************************************
// File: bluetooth.h
// Description: bluetooth module hardware library header file
// *****************************************************************************

// *****************************************************************************
// Includes
#include "global.h"
#include "main.h"
#include "dsp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// *****************************************************************************
// Defines 
#define SYS_CTRL_GPIO GPIOF
#define SYS_CTRL_PIN 10
#define MAX_CMD_LEN 10
#define BYTES_FOR_LEN 1
#define UART_CHUNK_SIZE 128
#define UART_START_SIZE 8

// *****************************************************************************
// Macros
#define B(x) S_to_binary_(#x)
static inline unsigned long long S_to_binary_(const char *s)
{
        unsigned long long i = 0;
        while (*s) {
                i <<= 1;
                i += *s++ - '0';
        }
        return i;
}

// *****************************************************************************
// Variables 
typedef enum
{
  NONE,
  DATA,
  APP_START,
  SET_FILTERS,
  SET_CONVOLUTION,
  STORE_IR,
  SET_BUFFER_SIZE
} tRxCommand;

typedef enum
{
  IDLE,
  RXBUSY,
  TXBUSY,
  UARTERROR
} tUartSt;

// *****************************************************************************
// Functions 
void BT_Init(void);
tErrorCode BT_Send_Ack(char *buf);
tErrorCode BT_Send_RxFail(char *buf);
tErrorCode BT_GetStart(char *buf, uint16_t *len, tRxCommand *command);
tErrorCode BT_ParseMsg(char *rxBuf, char *txBuf, uint16_t len, 
                      tRxCommand command, tFilterConfig *pCfg, 
                      uint8_t *filterConfigFlag);
tErrorCode BT_GetUartSt(UART_HandleTypeDef *huart, tUartSt *uartSt);