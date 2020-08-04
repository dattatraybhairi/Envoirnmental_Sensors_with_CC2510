/*-----------------------------------------------------------------------------
|   File:      PM2sleep.h
|   Target:    cc1110, cc2510
|   Author:    Rahul Bhagwat
|   Revised:   2018-07-23
|   Revision:  1.0
+------------------------------------------------------------------------------
|Code written and desing at Vacustech pvt ltd
+------------------------------------------------------------------------------
| Purpose:    configuring PM2 mode for sleep
+------------------------------------------------------------------------------
| Decription: functions for puting device into the sleep PM2 mode
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/

#ifndef PM2SLEEP_H
#define PM2SLEEP_H

/*==== INCLUDES ==============================================================*/

//#include <ioCC2510.h>
#include <ioCC1110.h>
#include <ioCCxx10_bitdef.h>
#include "hal_defines.h"
#include "hal_main.h"


/*==== CONSTS ================================================================*/
/***********************************************************************************
* LOCAL VARIABLES
*/
extern UINT8 activityFlag;
extern UINT8 panicButtonPress;

/*==== EXPORTS ===============================================================*/
/***********************************************************************************
* @fn          setup_sleep_interrupt
*
* @brief       Function which sets up the Sleep Timer Interrupt
*              for Power Mode 2 usage.
*
* @param       void
*
* @return      void
************************************************************************************/
void setup_sleep_interrupt(void);

/***********************************************************************************
* @fn          PM2Sleep
*
* @brief       Function which put cc into sleep for given interval.
*
* @param       UINT16 currentSleepInterval
*               PM2 sleep interval time in the milliseconds
*
* @return      void
************************************************************************************/
void PM2Sleep(UINT8 currentSleepIntervalHigh,UINT8 currentSleepIntervalLow);

#endif
/*==== END OF FILE ==========================================================*/
