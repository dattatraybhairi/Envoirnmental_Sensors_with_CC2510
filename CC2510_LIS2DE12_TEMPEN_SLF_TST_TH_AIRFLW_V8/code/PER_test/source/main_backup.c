/*-----------------------------------------------------------------------------
|   File:      per_test_main.c
|   Target:    cc1110, cc2510
|   Author:    ESY
|   Revised:   2007-09-06
|   Revision:  1.0
|   Project:   PER_test
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
| Purpose:    Packet Error Rate test for CC1110/CC2510 Development Kit
+------------------------------------------------------------------------------
| Decription: Main program file for Packet Error Rate test
|             for CC1110/CC2510 Development Kits
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_main.h"
#include <ioCCxx10_bitdef.h>

#include "timer3.h"
#include "PM2Sleep.h"
#include <stdlib.h>

/*==================== tag related constant ===================================*/
#define tagIDHigh 0x00                  // last two bytes of tag ID
#define tagIDLow 108

#define CCAThreshold 30                // threshold for the CCA count

#define syncPktLength 13
#define antennaPktLength 3

#define CCASleepTime 80                 // sleep 80ms if channel fount busy
#define CCAMaxCheck 12                   // Check CCA busy max 4 times                  
#define CCACheckTime 7                 // Check CCA flag for 10msec
#define updateInterval 10000            // update time 10 sec
#define panicInterval 5000
#define antennaPktInterval 9            // three packets for each antenna to be sent in 9 msec time

#define N_iteration 11

#define tagChannel 0                   // tag transmits data on channel 0
#define total_channel_number 3

#define instructionIDNormal 0xF5
#define instructionIDPanic 0xF0

#define panicButtonHoldTime 1000

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80
/*==================== DMA related function ===================================*/

UINT8 PACKET_LENGTH = 17;
volatile BOOL pktSentFlag = FALSE;            // Flag set whenever a packet is sent
volatile BOOL pktRcvdFlag = FALSE;            // Flag set whenever a packet is received
                           
UINT8 channelFreeFlag = 0;
UINT8 channleBusyCount;
UINT8 currentChnlBusyCount;

UINT8 currentIteration;  

static UINT8 currentTagID[6] = {0x5A,0xC2,0x15,0xA3,0x00,0xA1};
struct{
  UINT8 baseFreq;
  UINT8 _channel;
}centerFreq[total_channel_number];

UINT16 Ch_free;
UINT8 baseFreq,_channel;
//static UINT8 instructionID;
UINT8 panicState=0;
UINT8 panicButtonPress = 0;
volatile UINT32 panicInverval;

 
/*==== PUBLIC FUNCTIONS ======================================================*/

UINT16 getBatteryVoltage();
UINT8 getTemperature();
UINT8 CCACheck();
void sentPacket(UINT16 tagBatStatus, UINT8 tagTemp);
void rfConfiguration(UINT8 baseFreq, UINT8 _channel);
UINT16 CCACountFunction();
void panicCheck();

/******************************************************************************
* @fn  main
*
* @brief
*      Main function. 
*      1. read vbat and temp
*      2. check channel is free
*           1. if free intialize radios then send packets and sleep for update interval time. After wake up jump step 1
*           2. if channel is busy sleep for 80msec and jump to step 2
*           3. if channel is busy continuously for four iteration then sleep for sleep time - (90msec * 4)
               after waking jump to step 2
*               
*           
* 
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void main(void)
{
    UINT16 batVoltage;
    UINT8 tempVal;
    UINT16 sleepInterval;
    
    halPowerClkMgmtSetMainClkSrc(CRYSTAL);
    CLKCON = (CLKCON & ~CLKCON_TICKSPD);
    
    initTimer3();
   
    //CHANNR   = tagChannel;                                                            // change channel number accordingly.
    //radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);                     // configure radio for desired rate and frequency
   // radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_1_CC2510);
    
    //P0_1 for panic button 
    P0SEL &= ~(BIT1);                                           //Select P0_1 as GPIO.
    P0DIR |= ~(BIT1);                                           //Set P0_1 as Input.
    P0IFG = 0x00;                                               //Clear Port 0 interrupt status flag.
    P0INP &= ~0x40;                                             //Pin P0_2 Pull_up/Pull_down mode. all other pins of this port are in tristate. 
    PICTL |= 0x01;                                              //Trigger interrupt on falling edge.
    
    //Set individual interrupt enable bit to 1 in IENx.(In This case IEN1).
    HAL_INT_ENABLE(INUM_P0INT, INT_ON);                         // Enable Port 0 general interrupt.  
 
    HAL_INT_ENABLE(INUM_RF, INT_ON);            // Enable RF general interrupt
    RFIM |= IRQ_DONE;                           // Mask IRQ_DONE flag only
    //IP1 = 0x09;                                 // 1.rx tx 2. Timer 3 3 sleep
    //IP0 = 0x21;
    
     //Set highest priority to panic control.
    IP1 |= 0x28;                                                //set highest priority to panic and timer 3 interrupt.
    IP0 |= 0x09;                                                //set highest priority to panic and timer 3 interrupt.
      
    INT_GLOBAL_ENABLE(INT_ON);                  // Enable interrupts globally
     
    srand(tagIDLow);                            // seed for randomization
    channleBusyCount = 1;
    currentIteration = 0;
    centerFreq[0].baseFreq = 0,centerFreq[0]._channel = 1;
    centerFreq[1].baseFreq = 2,centerFreq[1]._channel = 0;
    centerFreq[2].baseFreq = 4,centerFreq[2]._channel = 0;
   /* for(UINT8 z = 0; z < 3 ;z++)
    {
      if(z != 1)
      {
        //for loop to change the channel
        for(UINT8 x = 0; x< 2 ; x++)
        {
         centerFreq[(z*2)+x].baseFreq = z;
         centerFreq[(z*2)+x]._channel = x*50;
        }
      }
      else
      {
        //for loop to change the channel
        for(UINT8 x = 0; x<1 ; x++)
        {
         centerFreq[(z*2)+x].baseFreq = z;
         centerFreq[(z*2)+x]._channel = x;
        }
      }
    }*/
    while(1)
    {
      if(panicButtonPress)
      {
        panicButtonPress = 0;
        if(panicState)                // if tag is in panic update after every 5 sec
            PM2Sleep((UINT8)(panicInterval >> 8),(UINT8)panicInterval);
        else                          // if tag is not in panic udate every 10 sec
            PM2Sleep((UINT8)(updateInterval >> 8),(UINT8)updateInterval);
      }
      
      else
      {
      // make channel busy count
      currentChnlBusyCount = 0;     
      // get battery voltage
      batVoltage = getBatteryVoltage();
      // get temperature
      tempVal = getTemperature();
     
      //radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);                     // configure radio for desired rate and frequency
      //rfConfiguration(2,20);
       
      while(CCACheck())
      {
        channleBusyCount++;
        currentChnlBusyCount++;
        
        if(currentChnlBusyCount < CCAMaxCheck)                  //max CCA check channels
        {
          sleepInterval = CCASleepTime;
          PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
          //radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110); 
        }
        else
        {
          sleepInterval = updateInterval-(CCAMaxCheck*(CCASleepTime+CCACheckTime));
          sleepInterval = (rand() % sleepInterval);
          currentIteration++;
          PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
          break;
        }
      }
     
      if(channelFreeFlag)
      {
        UINT8 baseFreq = centerFreq[0].baseFreq,_channel = centerFreq[0]._channel;
        rfConfiguration(baseFreq,_channel);
          
        // assemble packet and send data
        sentPacket(batVoltage,tempVal);
        PICTL |= 0x08; 
        if(currentIteration < N_iteration)
        {
          currentIteration++;
          if(panicState)                // if tag is in panic update after every 5 sec
            PM2Sleep((UINT8)(panicInterval >> 8),(UINT8)panicInterval);
          else                          // if tag is not in panic udate every 10 sec
            PM2Sleep((UINT8)(updateInterval >> 8),(UINT8)updateInterval);
        }
        
        else
        {
          currentIteration = 0;
          sleepInterval = (rand() % 5000);
          sleepInterval+=2500;
          PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
        }
        channelFreeFlag = 0;
        channleBusyCount = 1;
      }
      }
    } 
}

/******************************************************************************
* @fn  getBatteryVoltage
*
* @brief
*      This funtion return the current voltage ADC value
*
* @param  void
*
* @return UINT16 voltage adc value
*
******************************************************************************/
UINT16 getBatteryVoltage()
{
  UINT16 tempVbat;
   //ADC configuration for battery monitoring.
        /*----------------------------------------------------------------------------------------------*/
   
        //configuration registers.
        ADCCON2  = 0x2F;                                        //configure 10bit ADC,select Vdd/3 as input,1.25v reference voltage.
        ADCCON1 |= 0x73;                                        //Start of conversion.
        
        startTimer3();
        while(!(ADCCON1 & 0x80))                               //check wheather conversion completed or not.
        {
          if(millis > 10)
            break;
        }
        stopTimer3();
                                               //Result of conversion Low byte.
        tempVbat = ADCH<<8; 
                                       //Result of conversion High byte.
        tempVbat |= ADCL;
        
        return(tempVbat);
        //return(tempVbat>>6);
}

/******************************************************************************
* @fn  getTemperature
*
* @brief
*      This funtion read the ambiant temperature and return as UINT8 value
*
* @param  void
*
* @return UINT8 temp adc value
*
******************************************************************************/

UINT8 getTemperature()
{
  UINT8 T_deg;
  UINT16 adc_val_temp;
  float Output_voltage;
  
  /*--------------------------------------------------------------------------------------------*/
  //ADC configuration for Temperature monitoring.
  /*----------------------------------------------------------------------------------------------*/

  //configuration registers.
  ADCCON2  = 0x2E;                                        //configure 10bit ADC,select Teperature sensor as input,1.25v reference voltage.
  ADCCON1 |= 0x73;                                        //Start of conversion.
  
  startTimer3();
  while(!(ADCCON1 & 0x80))                               //check wheather conversion completed or not.
  {
   if(millis > 10)
      break;
  }
  stopTimer3();
  
  adc_val_temp = ADCL;                                    //Result of conversion Low byte.
  adc_val_temp |= ADCH << 8;            //Result of conversion High byte.
  
  //Total num of bits in adc_val_temp = 16. ADC configured for 10 bits.
  adc_val_temp >>= 6; 
  
  Output_voltage = adc_val_temp * (1250.00/511.00);       //Signifies 1.25V ref voltage and 2^9 ADC range as result is in 2's compliment form.
  
  T_deg = (UINT8)((Output_voltage - (750))/2.43);
  
  return(T_deg);
}

/******************************************************************************
* @fn  CCACheck
*
* @brief
*      This function check if Channel is clear, if clear then send packets 
*       otherwise sleep
*
* @param  void
*
* @return UINT8 channel free/busy
*
******************************************************************************/
UINT8 CCACheck()
{
   for(UINT8 k = 0 ; k < total_channel_number ; k++)
   {
     baseFreq = centerFreq[k].baseFreq,_channel = centerFreq[k]._channel;
     rfConfiguration(baseFreq,_channel);
   
     if(CCACountFunction() < CCAThreshold)  // if channel is free on perticular channel
     continue;
     else 
       return(255);                      // if channel is busy
   }
     channelFreeFlag = 1;
     return 0;
}
/******************************************************************************
* @fn  sentPacket
*
* @brief
*      This function used to send data to tag sync and per antenna packets
*
* @param  void
*
* @return void 
*
******************************************************************************/
 void sentPacket(UINT16 tagBatStatus, UINT8 tagTemp)
 {
   UINT8 i;
   //UINT8 send = 0;
    //fill the packet.
    radioPktBuffer[0] = syncPktLength;                  //Length byte
    /*radioPktBuffer[1] = 0x5A;                           //vendor identifier.
    radioPktBuffer[2] = 0xC2;                           //vendor identifier.
    radioPktBuffer[3] = 0x15;                           //vendor identifier.
    radioPktBuffer[4] = 0xA1;                           //signifies Tag or zone monitor.
    radioPktBuffer[5] = tagIDHigh;                      //Tag id.
    radioPktBuffer[6] = tagIDLow;                       //Tag id.
    */
    for(UINT8 x = 0 ; x < 6; x++)
        radioPktBuffer[x+1] = currentTagID [x];
    if(panicState)
      radioPktBuffer[7] = instructionIDPanic;                           //Tag is panic.
    else
      radioPktBuffer[7] = instructionIDNormal;                          //Not panic
   
    radioPktBuffer[8] = (UINT8)tagBatStatus;            //Battery monitoring byte low.
    radioPktBuffer[9] = (UINT8)(tagBatStatus >> 8);     //Battery monitoring byte high.
    radioPktBuffer[10] = channleBusyCount;              //Indicates channel got in first attempt or randomization applied. 0 = not applied.
    radioPktBuffer[11] = tagTemp;  
    
    radioPktBuffer[12] = 0xAA;          //Reserve data
    radioPktBuffer[13] = 0xAA;
    
    //radioPktBuffer[12] = (UINT8)Ch_free;          // CCA count low byte
    //radioPktBuffer[13] = (UINT8)(Ch_free >> 8);          // CCA count High
    
   DMAConfig(RADIO_MODE_TX, syncPktLength);
   
   //startTimer3();
   //sending sync data
   for(i = 0; i<3 ;i++)
   {
    DMACurrentMode(RADIO_MODE_TX);
    while(!pktSentFlag);
    /*{
      if (millis > 3)                           // fail safe if pktSentFlag never gets set, after 9 milliseconds it will come out of the loop
        break;
    }*/
    pktSentFlag = FALSE;
   }
   //stopTimer3();
   
   //sending packets for antenna
     
    radioPktBuffer[0]= antennaPktLength;                //Packet length.                   
     /*radioPktBuffer[1]= tagIDHigh;                       //Tag ID Low.
    radioPktBuffer[2]= tagIDLow;                        //Tag ID High.
    */
    // send last three address byte
   for(UINT8 x = 0 ; x < 3; x++)
        radioPktBuffer[x+1] = currentTagID[x+3];
   
   //for first freq f1 
   DMAConfig(RADIO_MODE_TX, antennaPktLength);
   
   for(UINT8 k = 0 ; k < total_channel_number ; k++)
   {
     for(i = 0; i < 17;i++)
     {
        DMACurrentMode(RADIO_MODE_TX);
        while(!pktSentFlag);
        pktSentFlag = FALSE;
      }
     if(k < total_channel_number-1)
     {
       UINT8 baseFreq = centerFreq[k+1].baseFreq,_channel = centerFreq[k+1]._channel;
       rfConfiguration(baseFreq,_channel);
     }
    }   
 }

/*==== INTERRUPT SERVICE ROUTINES ============================================*/

/******************************************************************************
* @fn  rf_IRQ
*
* @brief
*      The only interrupt flag which throws this interrupt is the IRQ_DONE interrupt.
*      So this is the code which runs after a packet has been received or
*      transmitted.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
#pragma vector=RF_VECTOR
__interrupt void rf_IRQ(void) {
    RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
    S1CON &= ~0x03;           // Clear the general RFIF interrupt registers

    if (mode == RADIO_MODE_RX) {
        pktRcvdFlag = TRUE;
    }
    else {
        pktSentFlag = TRUE;
    }
}
/******************************************************************************
* @fn  t3_IRQ
*
* @brief
*      When timer 3 overflows it generates the T3OVFIF flag and jump to this 
*       IRQ. After every one milliseconds the millis variable is keep incrementing
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/

#pragma vector = T3_VECTOR
__interrupt void t3_IRQ(void)
{
    /* Clears the module interrupt flag. */
    T3OVFIF = 0;
     
   // T3CTL &= ~T3CTL_START;
    millis++;
    /* Clears the CPU interrupt flag. */
    T3IF = 0;
}


/***********************************************************************************
* @fn          sleep_timer_isr
*
* @brief       Sleep Timer Interrupt Service Routine, which executes when
*              the Sleep Timer expires. Note that the [SLEEP.MODE] bits must
*              be cleared inside this ISR in order to prevent unintentional
*              Power Mode 2 entry.
*
* @param       void
*
* @return      void
*/
#pragma vector = ST_VECTOR
__interrupt void sleep_timer_isr(void)
{
    // Clear Sleep Timer CPU interrupt flag (IRCON.STIF = 0)
    STIF = 0;

    // Clear Sleep Timer Module Interrupt Flag (WORIRQ.EVENT0_FLAG = 0)
    WORIRQ &= ~WORIRQ_EVENT0_FLAG;

    // Clear the [SLEEP.MODE] bits, because an interrupt can also occur
    // before the SoC has actually entered Power Mode 2.
    SLEEP &= ~SLEEP_MODE;
     
}
/***********************************************************************************
* @fn          Panic_interrupt
*
* @brief       Fuction will call when ever panic button press event occurs
*
* @param       void
*
* @return      void
*/
#pragma vector = P0INT_VECTOR
__interrupt void Panic_interrupt(void)
{
 
  
    
  // Wait until HS RCOSC is stable
  //  while( !(SLEEP & SLEEP_HFRC_S) );

  // Set LS XOSC as the clock oscillator for the Sleep Timer (CLKCON.OSC32 = 0)
  //CLKCON &= ~CLKCON_OSC32; 
   INT_GLOBAL_ENABLE(INT_OFF);
  halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  INT_GLOBAL_ENABLE(INT_ON);
  
    //Clear modular interrupt first.
  P0IFG = 0x00;
  
  //Clear CPU interrupt.
  IRCON &= ~0x20;
  
  SLEEP &= 0xFC;
  
  panicCheck();
}

/***********************************************************************************
* @fn          rfConfiguration
*
* @brief       Function to configure radio according to given values
*
* @param       UINT8 baseFreq: 0-4, UINT8 _channel:0-100
*
* @return      void
*/
void rfConfiguration(UINT8 baseFreq, UINT8 _channel)
{
  CHANNR   = _channel;            // Channel number.
  switch(baseFreq)
  {
    //freq 2400
    case 0:
    default:
    radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_6_CC2510);
    break;
    
    //freq 2420
  case 1:
    radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_4_CC2510);
    break;
    
    //freq 2440
  case 2:
    radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_3_CC2510);
    break;
    
    //freq 2460
  case 3:
    radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_2_CC2510);
    break;
    
  case 4:
    if(_channel == 50)
    {
      CHANNR = 0;
      //freq 2400
      radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_6_CC2510);
    }
    else
      //freq 2480
      radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_1_CC2510);
    break;
  }
}
/***********************************************************************************
* @fn          CCACountFunction
*
* @brief       Function to check channel is free on given different channel
*
* @param      none
*
* @return      UINT16 chfree count
*/
UINT16 CCACountFunction()
{
  //UINT16 Ch_free = 0;
  Ch_free = 0;
   
   RFST = STROBE_SIDLE; 
   RFST = STROBE_RX;
    
    // Wait for radio to enter the RX mode
    while(MARCSTATE != 0x0D);
    
     startTimer3();
     while(millis < CCACheckTime)                        //Check CCA for 10msec
      {
        if(!(PKTSTATUS & 0x10))                             //Check for clear channel.
        { 
          Ch_free++;
        }
      }
     stopTimer3();
     return(Ch_free);
}

void panicCheck()
{
  panicInverval = 0;
  
  startTimer3();
  while((P0_1 == LOW) && (millis  < (panicButtonHoldTime+2)));
  stopTimer3();
  panicInverval = millis;
  //printf("Interval is %d\n",interval);
  
  if(panicInverval > panicButtonHoldTime &&  panicState == TRUE)
  {
    panicState = FALSE; 
    PICTL &= ~0x08;   
  }
  else if(panicInverval > panicButtonHoldTime &&  panicState == FALSE)
  {
    panicState = TRUE;
    PICTL &= ~0x08;   
  }
  else
    panicButtonPress = 1;
  
  /*else if (!panicState)
  {
   PM2Sleep((UINT8)(updateInterval >> 8),(UINT8)updateInterval);  // sleep for update interval
  }
  else if(panicState)
  {
    /*interval = panic_sleep_time;
    EVENT0_HIGH = (UINT8)(interval >> 8);                                    //Set sleep timer for 2 sec.
    EVENT0_LOW = (UINT8)(interval);
    PM2_sec(EVENT0_HIGH,EVENT0_LOW,1);        //Enter in Power mode 2. Param1 shows WOREVT0 and param2 shows WOREVT1.*/
    /*PM2Sleep((UINT8)(panicInterval >> 8),(UINT8)panicInterval);  // sleep for panic interval
  }*/
}
/*==== END OF FILE ==========================================================*/


