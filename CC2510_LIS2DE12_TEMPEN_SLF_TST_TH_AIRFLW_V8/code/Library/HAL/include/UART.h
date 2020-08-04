/*-----------------------------------------------------------------------------
|   File:      UART.h
|   Target:    cc1110, cc2510
|   Author:    Rahul Bhagwat
|   Revised:   2018-07-20
|   Revision:  1.0
+------------------------------------------------------------------------------
|Code written and desing at Vacustech pvt ltd
+------------------------------------------------------------------------------
| Purpose:    UART configuration and sending data
+------------------------------------------------------------------------------
| Decription: Macros for Baud rate and functions
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef UART_H
#define UART_H

/*==== INCLUDES ==============================================================*/

#include <ioCCxx10_bitdef.h>
#include "hal_defines.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "per_test_main.h"

/*==== CONSTS ================================================================*/
#define UART_BAUD_M  131                // for baud rate 9600
#define UART_BAUD_E  8

//#define UART_BAUD_M  34                // for baud rate 115200
//#define UART_BAUD_E  12

//#define BIT2              0x04
//#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT2              0x04
#define BIT3              0x08

#define _receiveBufferSize 12
extern char _receiveBuffer[_receiveBufferSize];

extern UINT8 _receiveBufferIndex;

extern UINT16 updateInterval;

extern UINT16 userData1,userData2,userData3,userData4;

/*==== EXPORTS ===============================================================*/
/******************************************************************************
* @fn  initUART
*
* @brief
*      This funtion configures the UART for baudrate, 8bit , no parity.
*
* @param  void
*
* @return void
*
******************************************************************************/
void initUART();

/******************************************************************************
* @fn  sendUART
*
* @brief
*      This function sends data on UART pins. Data is sent to this function 
*      through function parameters
*
* Parameters:
* 
* @para unsigned char data
*       Any unsingned char data can be sent as parameter which will be 
*       transitted
*
* @return void
*
******************************************************************************/
void sendUART(unsigned char data);

/******************************************************************************
* @fn  sendString
*
* @brief
*      This function sends data on UART pins. Data is sent to this function 
*      through function parameters
*
* Parameters:
* 
* @para char * str
*       String pointer to send data to string
*
* @return void
*
******************************************************************************/
void sendString(char * str);

void storeData(char _receiveData);
UINT8 decodeData();
UINT8 setParameter(UINT8 commandNumber, int commandValue);

#endif
/*==== END OF FILE ==========================================================*/

