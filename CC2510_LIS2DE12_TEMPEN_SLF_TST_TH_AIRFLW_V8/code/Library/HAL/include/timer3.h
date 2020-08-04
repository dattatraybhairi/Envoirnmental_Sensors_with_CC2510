/*-----------------------------------------------------------------------------
|   File:      UART.h
|   Target:    cc1110, cc2510
|   Author:    Rahul Bhagwat
|   Revised:   2018-07-20
|   Revision:  1.0
+------------------------------------------------------------------------------
|Code written and desing at Vacustech pvt ltd
+------------------------------------------------------------------------------
| Purpose:    configuring timer3 to count after every 1 milliseconds period
+------------------------------------------------------------------------------
| Decription: Macros, global variable and functions required for timer
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef TIMER3_H
#define TIMER3_H

/*==== INCLUDES ==============================================================*/

#include <ioCCxx10_bitdef.h>
#include "hal_defines.h"

/*==== CONSTS ================================================================*/
extern volatile UINT32 millis;

/******************************************************************************
* @fn  initTimer3
*
* @brief
*      This funtion configures timer3 for 1 msec interrupt 
*       26Mhz clock, tickspeed is clock/128 = 203125 hz
*       1/203125 = 4.9230usec 
*       4.9230usec * 205 = 1.0092 msec
*       T3CC0 = 205
*
* @param  void
*
* @return void
*
******************************************************************************/
void initTimer3();

/******************************************************************************
* @fn  startTimer3
*
* @brief
*      This function will start the timer and set the overflow interrupt
*
* @param  void
*
* @return void
*
******************************************************************************/
void startTimer3();

/******************************************************************************
* @fn  stopTimer3
*
* @brief
*      This function will stop the timer and disable the overflow interrupt
*
* @param  void
*
* @return void
*
******************************************************************************/
void stopTimer3();

#endif
/*==== END OF FILE ==========================================================*/
