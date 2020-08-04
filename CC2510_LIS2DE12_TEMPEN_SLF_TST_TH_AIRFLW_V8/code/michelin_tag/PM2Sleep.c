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

/*==== INCLUDES ==============================================================*/
#include "PM2sleep.h"


// Initialization of source buffers and DMA descriptor for the DMA transfer
// (ref. CC111xFx/CC251xFx Errata Note)
static UINT8 __xdata PM2_BUF[7] = {0x06,0x06,0x06,0x06,0x06,0x06,0x04};
static UINT8 __xdata dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};

UINT8 activityFlag;
/*=======FUNCTIONS ===========================================================*/
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
void setup_sleep_interrupt(void)
{
   // Clear Sleep Timer CPU Interrupt flag (IRCON.STIF = 0)
    STIF = 0;

    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;

    // Enable Sleep Timer Module Interrupt (WORIRQ.EVENT0_MASK = 1)
    WORIRQ |= WORIRQ_EVENT0_MASK;

    // Enable Sleep Timer CPU Interrupt (IEN0.STIE = 1)
    STIE = 1;
    
    EA = 1;

}

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
void PM2Sleep(UINT8 currentSleepIntervalHigh,UINT8 currentSleepIntervalLow)
{
    UINT8 storedDescHigh = DMA0CFGH;
    UINT8 storedDescLow = DMA0CFGL;
    UINT8 temp;
    
      // Setup + enable the Sleep Timer Interrupt, which is
    // intended to wake-up the SoC from Power Mode 2.
    setup_sleep_interrupt();
    
  //Enter/exit Power Mode 2.
    SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_HFRC_S) );
    CLKCON = (CLKCON & ~CLKCON_CLKSPD) | CLKCON_OSC | CLKSPD_DIV_1;
    while ( !(CLKCON & CLKCON_OSC) ) ;
    SLEEP |= SLEEP_OSC_PD;
    
    // Set LS XOSC as the Sleep Timer clock source (CLKCON.OSC32 = 0)
    //CLKCON |= CLKCON_OSC32;  // internal 32khz osc
    CLKCON &= ~CLKCON_OSC32;  // external 32khz osc
    ///////////////////////////////////////////////////////////////////////
    ////////// CC111xFx/CC251xFx Errata Note Code section Begin ///////////
    ///////////////////////////////////////////////////////////////////////
    
    // Store current DMA channel 2 descriptor and abort any ongoing transfers,
    // if the channel is in use.
    storedDescHigh = DMA0CFGH;
    storedDescLow = DMA0CFGL;
    DMAARM |= (DMAARM_ABORT | DMAARM0);
    
    // Update descriptor with correct source.
    dmaDesc[0] = (UINT16)&PM2_BUF >> 8;
    dmaDesc[1] = (UINT16)&PM2_BUF;
    
    // Associate the descriptor with DMA channel 0 and arm the DMA channel
    DMA0CFGH = (UINT16)&dmaDesc >> 8;
    DMA0CFGL = (UINT16)&dmaDesc;
    DMAARM = DMAARM0;
    
    // NOTE! At this point, make sure all interrupts that will not be used to
    // wake from PM are disabled as described in the "Power Management Control"
    // chapter of the data sheet.
    
    // The following code is timing critical and should be done in the
    // order as shown here with no intervening code.
    
    // Align with positive 32 kHz clock edge as described in the
    // "Sleep Timer and Power Modes" chapter of the data sheet.
    if ((activityFlag == 0)||(panicButtonPress))
        WORCTRL |= 0x05; // Reset Sleep Timer
    else 
        WORCTRL |= 0x01; // do not reset sleep timer
    
    temp = WORTIME0;
    while(temp == WORTIME0);
     
    temp = WORTIME0;
    while(temp == WORTIME0);
    
    // Set Sleep Timer Interval
    //WOREVT1 = (currentSleepInterval >> 8) ;
    //WOREVT0 = currentSleepInterval;
    WOREVT1 = currentSleepIntervalHigh;
    WOREVT0 = currentSleepIntervalLow;
    
    // Make sure HS XOSC is powered down when entering PM{2 - 3} and that
    // the flash cache is disabled.
    MEMCTR |= MEMCTR_CACHD;
    SLEEP = 0x06;
    
    // Enter power mode as described in chapter "Power Management Control"
    // in the data sheet. Make sure DMA channel 0 is triggered just before
    // setting [PCON.IDLE].
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    if(SLEEP & 0x03)
    {  
      asm("MOV 0xD7,#0x01");      // DMAREQ = 0x01;
      asm("NOP");                 // Needed to perfectly align the DMA transfer.
      asm("ORL 0x87,#0x01");      // PCON |= 0x01 -- Now in PM2;
      asm("NOP");
    }
    // End of timing critical code
    
    // Enable Flash Cache.
    MEMCTR &= ~MEMCTR_CACHD;
    
    // Update DMA channel 0 with original descriptor and arm channel if it was
    // in use before PM was entered.
    DMA0CFGH = storedDescHigh;
    DMA0CFGL = storedDescLow;
    DMAARM = DMAARM0;
    
    ///////////////////////////////////////////////////////////////////////
    /////////// CC111xFx/CC251xFx Errata Note Code section End ////////////
    ///////////////////////////////////////////////////////////////////////
    
    // Wait until HS RCOSC is stable
    while( !(SLEEP & SLEEP_HFRC_S) );

  // Set LS XOSC as the clock oscillator for the Sleep Timer (CLKCON.OSC32 = 0)
  CLKCON &= ~CLKCON_OSC32; 
  
  halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  //CLKCON = (CLKCON & ~CLKCON_TICKSPD) | TICKSPD_DIV_128;
       
}

