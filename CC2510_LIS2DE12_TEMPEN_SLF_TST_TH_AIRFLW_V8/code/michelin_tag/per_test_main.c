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
#include "UART.h"
#include <stdlib.h>
#include <string.h>
#include "cc2510_spi.h"
#include "AXL.h"
#include "bme280.h"
#include <math.h>

/*==================== tag related constant ===================================*/
#define tagIDHigh 0x00                  // last two bytes of tag ID
#define tagIDLow 12

#define CCAThreshold 30                // threshold for the CCA count

#define syncPktLength 14
#define antennaPktLength 3
#define dummyTxPktLength 8

#define CCASleepTime 80                 // sleep 80ms if channel fount busy
#define CCAMaxCheck 12                   // Check CCA busy max 4 times                  
#define CCACheckTime 7                 // Check CCA flag for 10msec 
#define updateInterval 10000            // update time 10 sec
#define panicInterval 5000
#define antennaPktInterval 9            // three packets for each antenna to be sent in 9 msec time

#define N_iteration 11

#define tagChannel 0                   // tag transmits data on channel 0
#define total_channel_number 1
#define tatal_count_freq 1
#define final_data_send_pkt_len 13
#define tag1_channel 50

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

#define firstTagId 95
#define countSpacing 250
#define tagTransmitCount ((tagIDLow - firstTagId) * countSpacing)
#define nwMangerTotalCount 5000
#define window 15

//#define ticktime 3.62
#define ticktime 2
#define correction 15.00
#define allowedMissednwCount 5
/*==================== DMA related function ===================================*/

UINT8 PACKET_LENGTH = 17;
volatile BOOL pktSentFlag = FALSE;            // Flag set whenever a packet is sent
volatile BOOL pktRcvdFlag = FALSE;            // Flag set whenever a packet is received

UINT8 channelFreeFlag = 0;
UINT8 countReceived = 0;
UINT8 channleBusyCount;
UINT8 currentChnlBusyCount;
UINT8 cnt = 0;
UINT32 currmillis = 0;
UINT16 activity_cnt = 65535;
UINT8 currentIteration;


/* Airflow parameters setting */
float V_0 = 3.3; // supply voltage to the pressure sensor
float rho = 1.204; // density of air 
float adc_avg = 0; 
float veloc = 0.0;

// parameters for averaging and offset
INT16 offset = 0;
INT8 offset_size = 10;
INT8 veloc_mean_size = 20;
INT8 zero_span = 2;

UINT16 adc_val = 0;
UINT16 Readval = 0;


//static UINT8 currentTagID[6] = {0x5A,0xC2,0x15,0xA3,tagIDHigh,tagIDLow};
struct{
  UINT8 baseFreq;
  UINT8 _channel;
}centerFreq[total_channel_number];

//struct{
//  UINT8 baseFreq;
//  UINT8 _channel;
//}nwCountFreq[tatal_count_freq];

UINT16 Ch_free;
UINT8 baseFreq,_channel;

//static UINT8 instructionID;
UINT8 panicState=0;
UINT8 panicButtonPress = 0;
UINT8 falseTrigger = 0;
volatile UINT32 panicInverval;

const UINT8 dummyTagZMId[6] = {90,194,21,241,0,2};

UINT16 old_count = 0;
UINT16 current_count=0;

UINT8 nwCount = 0;                      //No of current count

//UINT16 diff_count= 0;

UINT16 sleepInterval;
UINT16 tick;
UINT8 cur_freq = 0;

UINT8 debug=5;

UINT16 batVoltage = 0;
double currentbatVoltage = 0;
UINT8 batStatus = 0;
/*==== PUBLIC FUNCTIONS ======================================================*/

UINT16 getBatteryVoltage();
UINT8 getTemperature();
UINT8 CCACheck();
void sentPacket();
void rfConfiguration(UINT8 baseFreq, UINT8 _channel);
UINT16 CCACountFunction();
void panicCheck();
UINT8 matchMACId();
UINT8 countReceive();
void checkCurrentCount();
void nwCountTransmit();
UINT16 analogRead();
float Airflow();
void initAirflow();
void WDT_set();
void WDT_clear();
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
  halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  CLKCON = (CLKCON & ~CLKCON_TICKSPD);
  
  radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_3_CC2510);     //Radio configuration.
  
  //P0_1 for panic button 
  P0SEL &= ~(BIT1);                                           //Select P0_1 as GPIO.
  P0DIR &= ~(BIT1);                                           //Set P0_1 as Input.
  P0IFG = 0x00;                                               //Clear Port 0 interrupt status flag.
  P0INP &= ~0x02;                                             //Pin P0_2 Pull_up/Pull_down mode. all other pins of this port are in tristate. 
  PICTL |= 0x01;                                              //Trigger interrupt on falling edge.
  
  //P0_5 GPIO init.
  P0SEL &= ~(BIT5);                                           //Select P0_1 as GPIO.
  P0DIR |= (BIT5);                                            //Set P0_1 as Input.
  P0_5 = 1;
  
  //Set individual interrupt enable bit to 1 in IENx.(In This case IEN1).
  HAL_INT_ENABLE(INUM_P0INT, INT_ON);                         //Enable Port 0 general interrupt.
  HAL_INT_ENABLE(INUM_P1INT, INT_OFF);                        //Disable MEMS Sensor. 
  HAL_INT_ENABLE(INUM_RF, INT_ON);                            //Enable RF general interrupt.
  RFIM |= IRQ_DONE;                                           //Mask IRQ_DONE flag only.
  PICTL |= 0x08;
  
  //Set highest priority to panic control.
  IP1 |= 0x21;                                                //set highest st,rx,p0.
  IP0 |= 0x30;
  
  //LED setting for sensor tag.
  P1SEL &= ~(0x01); 
  P1DIR |= (0x01);                                            //Set P1_0 as Output.
  P1_0 = 0;
  
  initTimer3();                                               //Initialize timer3.
  //initUART();                                               //Initialize UART.
  AXL();                                                      //Initialize MEMS sensor.
  BME280();                                                   //Initialize BME280 sensor.
  initAirflow();                                              //Initialize Airflow sensor.
  
  INT_GLOBAL_ENABLE(INT_ON);                                  //Enable interrupts globally.
  //printf("Reset\n");                                        //Reset indicator.
  WDT_set();                                                  //Set watchdog timer.
  
  while(1)
  {
    WDT_clear();                                              //Clear watchdog timer.
    P0_5 = 1;                                                 //Enable Airflow sensor.                                        
    
    BME280_measurement();                                     //Trigger BME280 in forced measurement mode.
    radioPktBuffer[9] = Temp_bme280;                          //Temperature value. 
    radioPktBuffer[10] = Hum_bme280;                          //Humidity value.
    //radioPktBuffer[11] = 0;                                 //Airflow Value.
    radioPktBuffer[11] = Airflow();                           //Airflow Value.
    
    P0_5 = 0;                                                 //Disable Airflow sensor to save power. 
    
    sentPacket();                                             //Send Packet to gateway.
    
    WDT_clear();                                              //Clear watchdog timer before sleep.
    PM2Sleep((UINT8)(updateInterval >> 8),(UINT8)updateInterval); //Sleep. 
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
  
  //return(tempVbat);
  return(tempVbat>>6);
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
  adc_val_temp |= ADCH << 8;                              //Result of conversion High byte.
  
  //Total num of bits in adc_val_temp = 16. ADC configured for 10 bits.
  adc_val_temp >>= 6; 
  
  Output_voltage = adc_val_temp * (1250.00/511.00);       //Signifies 1.25V ref voltage and 2^9 ADC range as result is in 2's compliment form.
  
  T_deg = (UINT8)((Output_voltage - (750))/2.43);
  
  return(T_deg);
}

/******************************************************************************
* @fn  analogRead
*
* @brief
*      This funtion return the current voltage ADC value
*
* @param  void
*
* @return UINT16 voltage adc value
*
******************************************************************************/
UINT16 analogRead()
{
  /* Set system clock source to HS XOSC, with no pre-scaling.
  * Ref. [clk]=>[clk_xosc.c]
  */
  SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;
  
  // Setup ADC if changed while using other functions. 
  ADCCFG |= ADCCFG_4;
  ADCCON1 = (ADCCON1 & ~ADCCON1_STSEL) | STSEL_ST | BIT1 | BIT0;
  ADCCON2 = ADCCON2_SREF_AVDD | ADCCON2_SDIV_256 | ADCCON2_SCH_AIN4;
  
  /* Set [ADCCON1.ST] and await completion (ADCCON1.EOC = 1) */
  ADCCON1 |= ADCCON1_ST | BIT1 | BIT0;
  while( !(ADCCON1 & ADCCON1_EOC));
  
  adc_val = ADCL & 0xF0;
  adc_val |= (ADCH << 8);
  
  //Total num of bits in adc_val = 16. ADC configured for 10 bits.
  adc_val >>= 6;
  //printf("%d\n",adc_val);
  
  halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  
  return adc_val;
}

/******************************************************************************
* @fn  initAirflow
*
* @brief
*      This funtion initialize Airflow sensor.
*
* @param  void
*
* @return null.
*
******************************************************************************/
void initAirflow()
{
  ADCCFG |= ADCCFG_4;                                                //Set [ADCCFG.ADCCFG7 = 1].   
  ADCCON1 = (ADCCON1 & ~ADCCON1_STSEL) | STSEL_ST | BIT1 | BIT0;     //Set [ADCCON1.STSEL] according to ADC configuration.
  ADCCON2 = ADCCON2_SREF_AVDD | ADCCON2_SDIV_256 | ADCCON2_SCH_AIN4; //Set [ADCCON2.SREF/SDIV/SCH] according to ADC configuration.
  
  for(int i=0; i<offset_size; i++)                                   //Offset calibration.
  {
    Readval = analogRead();
    offset = offset + (Readval-(511/2));
  }
  offset /= (offset_size);
}

/******************************************************************************
* @fn  Airflow
*
* @brief
*      This funtion return the velocity value in m/s.
*
* @param  float
*
* @return float air velocity.
*
******************************************************************************/
float Airflow()
{
  adc_avg = 0;
  veloc = 0;
  
  // average a few ADC readings for stability
  for (int i=0;i<veloc_mean_size;i++)
  {
    Readval = analogRead();
    adc_avg+= Readval - offset;
  }
  adc_avg/=veloc_mean_size;
  
  // make sure if the ADC reads below 256, then we equate it to a negative velocity
  if (adc_avg>256-zero_span && adc_avg<256+zero_span)
  {
  } 
  else
  {
    if (adc_avg<256)
    {
      veloc = -sqrt((-10000.0*((adc_avg/511.0)-0.5))/rho);
    } 
    else
    {
      veloc = sqrt((10000.0*((adc_avg/511.0)-0.5))/rho);
    }
  } 
  //printf("%f\n",veloc); 
  return veloc;
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
  if(Ch_free < CCAThreshold)
  {
    channelFreeFlag = 1;
    return 0;
  }
  else return(255);
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
void sentPacket()
{
  WDT_clear();                                        //Clear watchdog timer.

  //Get battery voltage.
  batVoltage = getBatteryVoltage();                   //Check Battery status.
  currentbatVoltage = ((batVoltage) * (3.75))/(512);
  batStatus = (UINT8)((currentbatVoltage * 100)/(3.3));
  
  //fill the packet.
  radioPktBuffer[0] = syncPktLength;                  //Length byte.
  radioPktBuffer[1] = 0x5A;                           //vendor identifier.
  radioPktBuffer[2] = 0xC2;                           //vendor identifier.
  radioPktBuffer[3] = 0x15;                           //vendor identifier.
  radioPktBuffer[4] = 0xC1;                           //signifies Sensor tag.
  radioPktBuffer[5] = tagIDHigh;                      //Tag id.
  radioPktBuffer[6] = tagIDLow;                       //Tag id.
  radioPktBuffer[7] = 0xC5;                           //Fill instruction id to indicate sensor.
  radioPktBuffer[8] = batStatus;                      //Battery status byte.
  radioPktBuffer[12] = 0;                             //Reserve.
  radioPktBuffer[13] = 0;                             //Reserve.
  radioPktBuffer[14] = 0;                             //Reserve.
  
  CHANNR =  tag1_channel;                             //Setup Tag TX channel.
  DMAConfig(RADIO_MODE_TX, syncPktLength);
  DMACurrentMode(RADIO_MODE_TX);
  while(!pktSentFlag);
  pktSentFlag = FALSE;
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

/******************************************************************************
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
******************************************************************************/
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

/******************************************************************************
* @fn          Panic_interrupt
*
* @brief       Fuction will call when ever panic button press event occurs
*
* @param       void
*
* @return      void
******************************************************************************/
#pragma vector = P0INT_VECTOR
__interrupt void Panic_interrupt(void)
{
  // Wait until HS RCOSC is stable
  //  while( !(SLEEP & SLEEP_HFRC_S) );
  
  // Set LS XOSC as the clock oscillator for the Sleep Timer (CLKCON.OSC32 = 0)
  //CLKCON &= ~CLKCON_OSC32; 
  //  INT_GLOBAL_ENABLE(INT_OFF);
  //  halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  //  INT_GLOBAL_ENABLE(INT_ON);
  //  
  //Clear modular interrupt first.
  P0IFG = 0x00;
  
  //Clear CPU interrupt.
  IRCON &= ~0x20;
  
  SLEEP &= 0xFC;
  panicButtonPress = 1;
  debug = 0;
  
  //panicCheck();
}

/******************************************************************************
* @fn          externalPinInterrupt
*
* @brief       Fuction will call when MEMS interrupt occurs.
*
* @param       void
*
* @return      void
******************************************************************************/
#pragma vector = P1INT_VECTOR
__interrupt void externalPinInterrupt(void)
{
  P1IFG &= ~(0x02);       //Clear GDO0 pin flag
  
  IRCON2 &= ~ (0x08);     //Clear IRCON flag
  
  SLEEP &= 0xFC;
  
  temp = read_reg(LIS2DE12_REG_INT2SRC);        //previous code reads INT1SRC. 
  if((temp & 0x40) == 0x40)
  {
    activityFlag = 1;
    //HAL_INT_ENABLE(INUM_P1INT, INT_OFF);
    //sleepInterval = 0;
    //PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
  }
}

/******************************************************************************
* @fn          rfConfiguration
*
* @brief       Function to configure radio according to given values
*
* @param       UINT8 baseFreq: 0-4, UINT8 _channel:0-100
*
* @return      void
******************************************************************************/
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
/******************************************************************************
* @fn          CCACountFunction
*
* @brief       Function to check channel is free on given different channel
*
* @param      none
*
* @return      UINT16 chfree count
******************************************************************************/
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

/******************************************************************************
* @fn          panicCheck
*
* @brief       Function to check Valid Panic press event.
*
* @param      none
*
* @return     none
******************************************************************************/
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
    //PICTL &= ~0x08;   
  }
  else if(panicInverval > panicButtonHoldTime &&  panicState == FALSE)
  {
    panicState = TRUE;
    //PICTL &= ~0x08;   
  }
  //else
  
  else if (!panicState)
  {
    falseTrigger = 1;
    HAL_INT_ENABLE(INUM_P0INT, INT_ON);
    PM2Sleep((UINT8)(updateInterval >> 8),(UINT8)updateInterval);  // sleep for update interval
  }
  else if(panicState)
  {
    falseTrigger = 1;
    HAL_INT_ENABLE(INUM_P0INT, INT_ON);
    /*interval = panic_sleep_time;
    EVENT0_HIGH = (UINT8)(interval >> 8);                                    //Set sleep timer for 2 sec.
    EVENT0_LOW = (UINT8)(interval);
    PM2_sec(EVENT0_HIGH,EVENT0_LOW,1);        //Enter in Power mode 2. Param1 shows WOREVT0 and param2 shows WOREVT1.*/
    PM2Sleep((UINT8)(panicInterval >> 8),(UINT8)panicInterval);  // sleep for panic interval
  }
}

/******************************************************************************
* @fn          matchMACId
*
* @brief       Function to check Valid MacID received.
*
* @param      none
*
* @return     UINT8.
******************************************************************************/
UINT8 matchMACId(){
  for(UINT8 k = 0; k < 6; k++)
  {
    if(radioPktBuffer[k+1] == dummyTagZMId[k])
      continue;
    else 
      return(0);
  }
  return(1);
}

/******************************************************************************
* @fn          countReceive
*
* @brief       Function to check Valid packet received.
*
* @param      none
*
* @return     UINT8.
******************************************************************************/
UINT8 countReceive()
{
  DMAConfig(RADIO_MODE_RX, dummyTxPktLength);
  countReceived = 0;
  for(UINT8 k = 0; k < tatal_count_freq ; k++)
  {
    //UINT8 baseFreq = nwCountFreq[k].baseFreq,_channel = nwCountFreq[k]._channel;                  //2400 channel 5 for network manager
    UINT8 baseFreq = 0,_channel = 5*(k+2);                  //2400 channel 5 for network manager
    rfConfiguration(baseFreq,_channel);
    
    DMAConfig(RADIO_MODE_RX, dummyTxPktLength);
    
    DMACurrentMode(RADIO_MODE_RX);
    startTimer3();
    while(millis < 5)
    {
      if(pktRcvdFlag)
      {
        pktRcvdFlag = 0;
        if(radioPktBuffer[radioPktBuffer[0]+2] &0x80)
        {
          if(matchMACId())
          {
            memcpy((UINT8*)&current_count,(UINT8*)&radioPktBuffer[7] ,2);
            //old_count = current_count;
            //sendUART((UINT8)current_count);
            //sendUART((UINT8)(current_count>>8));
            //sendUART(k);
            //sendUART(activityFlag);
            countReceived = 1;
            channleBusyCount++;
            
            millis = 5;
            break;
          }
          else
            DMACurrentMode(RADIO_MODE_RX);
        }
        else
          DMACurrentMode(RADIO_MODE_RX);
      }
    }
    stopTimer3();
    
    if(countReceived)
      return(1);
  }
  return(0);
}

/******************************************************************************
* @fn         checkCurrentCount
*
* @brief      
*
* @param      none
*
* @return     none
******************************************************************************/
void checkCurrentCount()
{
  cur_freq = 0;
  HAL_INT_ENABLE(INUM_P1INT, INT_OFF);
  cur_freq = countReceive();
  
  if(cur_freq)
  {
    nwCount = allowedMissednwCount;
    //old_count = current_count;
    nwCountTransmit();
  }
  else
  {
    if(nwCount > 0)             // previous count is available
    {
      nwCount--;
      if((abs(tagTransmitCount-old_count)<= window) || (old_count > tagTransmitCount))
      {
        current_count = tagTransmitCount;
        old_count = tagTransmitCount;
      }
      else                        
      {
        tick = tagTransmitCount - old_count;
        sleepInterval =(UINT16)(((float)(tick)*(float)(ticktime)) + ((float)(((float)tick/100)*13)));
        PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);    
        current_count = tagTransmitCount;
        old_count = tagTransmitCount;
      }
      nwCountTransmit();
    }
    else// no previous count available go with CCA
    {
      if(CCACheck() == 0)
      {
        sentPacket();
        if(activityFlag)
        {
          activityFlag--;
        }
        tick = nwMangerTotalCount;
        sleepInterval =(UINT16)(((float)(tick)*(float)(ticktime)) + ((float)(((float)tick/100)*13)));
        //sleepInterval = updateInterval;
        
        HAL_INT_ENABLE(INUM_P1INT, INT_ON);
        
        PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);    
      }
      else
      {
        channleBusyCount++;
        //sleep for total count time
        tick = nwMangerTotalCount;
        sleepInterval =(UINT16)(((float)(tick)*(float)(ticktime)) + ((float)(((float)tick/100)*13)));
        //sleepInterval = updateInterval;
        HAL_INT_ENABLE(INUM_P1INT, INT_ON);
        
        PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
        
        // sleepInterval = (rand() % updateInterval);
        
        tick = (rand() % nwMangerTotalCount);
        //sleep for random time
        sleepInterval =(UINT16)(((float)(tick)*(float)(ticktime)) + ((float)(((float)tick/100)*13)));
        PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);
      }
    }
  }
}

/******************************************************************************
* @fn         nwCountTransmit
*
* @brief      
*
* @param      none
*
* @return     none
******************************************************************************/
void nwCountTransmit()
{
  if(abs(tagTransmitCount-current_count)<= window)
  {
    sentPacket();
    if(activityFlag)
    {
      activityFlag--;
    }
    tick = nwMangerTotalCount - current_count + tagTransmitCount - 20;// -((cur_freq-1)*2);
  }
  else
  {
    if(current_count < tagTransmitCount)
    {
      tick = tagTransmitCount - current_count;
    }
    else
    {
      tick = nwMangerTotalCount - current_count + tagTransmitCount;
    }
  }
  sleepInterval =(UINT16)(((float)(tick)*(float)(ticktime)) + ((float)(((float)tick/100)*13)));
  HAL_INT_ENABLE(INUM_P1INT, INT_ON);
  PM2Sleep((UINT8)(sleepInterval >> 8),(UINT8)sleepInterval);    
  /*startTimer3();
  while(millis < sleepInterval);
  stopTimer3();*/
}

/******************************************************************************
* @fn          WDT_set()
*
* @brief       Function to set watchdog timer.
*
* @param      none
*
* @return     void
******************************************************************************/
void WDT_set()
{
  WDCTL = 0x08;
}

/******************************************************************************
* @fn          WDT_clear()
*
* @brief       Function to clear watchdog timer.
*
* @param      none
*
* @return     void
******************************************************************************/
void WDT_clear()
{
  WDCTL = (0xA8);
  WDCTL = (0x58);
}
/*==== END OF FILE ==========================================================*/