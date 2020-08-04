/*-----------------------------------------------------------------------------
|   File:      per_test_main.c
|   Target:    cc1110, cc2510
|   Author:    ESY
|   Revised:   20-09-06
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

#include "cc2510_spi.h"
#include "AXL.h"
#include "timer3.h"
#include <ioCCxx10_bitdef.h>
#include <stdio.h>
#include <stdlib.h>

/*================ CONSTANTS =================================================*/
#define LIS2DE12_DEFAULT_ADDRESS  0x18
//#define LIS2DE12_REG_WHOAMI       0x0F
#define LIS2DE12_REG_CTRL1        0x20
#define LIS2DE12_REG_CTRL2        0x21
#define LIS2DE12_REG_CTRL3        0x22
#define LIS2DE12_REG_CTRL4        0x23
#define LIS2DE12_REG_CTRL5        0x24
#define LIS2DE12_REG_CTRL6        0x25
#define LIS2DE12_REG_REFERENCE    0x26
#define LIS2DE12_REG_STATUS       0x27

#define LIS2DE12_REG_INT1CFG      0x30
#define LIS2DE12_REG_INT1SRC      0x31
#define LIS2DE12_REG_INT1THS      0x32
#define LIS2DE12_REG_INT1DUR      0x33

#define LIS2DE12_REG_INT2CFG      0x34
#define LIS2DE12_REG_INT2SRC      0x35
#define LIS2DE12_REG_INT2THS      0x36
#define LIS2DE12_REG_INT2DUR      0x37

#define LIS2DE12_DATARATE_100_HZ  0x05
#define LIS2DE12_REG_TEMPCFG      0x1F
#define OUT_TEMP_L                0x0C
#define OUT_TEMP_H                0x0D
#define STATUS_REG_AUX            0x07
#define LIS2DE12_RANGE_4_G        0x01
#define LIS2DE12_REG_OUT_X        0x29
#define LIS2DE12_REG_OUT_Y        0x2B
#define LIS2DE12_REG_OUT_Z        0x2D
#define SENSORS_GRAVITY_STANDARD  9.80665
#define SENSITIVITY_DIV_16G       8.0
#define SENSITIVITY_DIV_8G        16.0
#define SENSITIVITY_DIV_4G        32.0
#define SENSITIVITY_DIV_2G        64.0

/*================ VARIABLES =================================================*/
float X,Y,Z = 0;
uint8 status = 0;
uint8 TL,TH = 0;
uint8 DA = 0;
uint8 i = 0;
INT16 Temperature = 0;
float Final_temp = 0;
BYTE temp = 0x00;
UINT32 currentmillis = 0;
INT8 X_data,Y_data,Z_data = 0;
INT16 OUTX_NOST,OUTY_NOST,OUTZ_NOST = 0;
INT16 OUTX_ST,OUTY_ST,OUTZ_ST = 0;

/*==== PRIVATE FUNCTIONS ======================================================*/
BOOL lis2de12_Begin();
void setDataRate(uint8 dataRate);
void setRange(uint8 range);
static INT8 getX();
static INT8 getY();
static INT8 getZ();
static INT8 readData8(uint8 regId);
static float readTemperature();
void xyzAvg(INT16 *XOUT, INT16 *YOUT, INT16 *ZOUT);
BOOL selfTest();
/******************************************************************************
* @fn  main
*
* @brief
*      Main function. Triggers setup menus and main loops for both receiver
*      and transmitter. This function supports both CC1110 and CC2510.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void AXL()
{
  //Initialization.
  //========================================================================//
  //CLKCON = 0x00;
  //halPowerClkMgmtSetMainClkSrc(CRYSTAL);
  
  //spi_init();
  //while(!(lis2de12_Begin()));
  
  //setRange(LIS2DE12_RANGE_4_G);
  
  //Clear CPU interrupt flag(Port 1).
  IRCON2 &= ~0x80;
  //Enable individual interrupt enable bits in SFR's.
  P1SEL &= ~(0x06);                                           //Select P1_1 and P1_2 as GPIO.
  
  P1DIR &= ~(0x02);                                           //Set P1_1 as Input.INT2 configuration.
  P1DIR |= (0x04);                                            //Set P1_2 as Output.
  
  //P1 &= ~(0x02);
  P1_2 = 0;
  
  P1IFG = 0x00;                                               //Clear Port 1 interrupt status flag.
  P1INP &= ~0x06;                                             //Pin P1_1 Pull_up/Pull_down mode. all other pins of this port are in tristate. 
  P1IEN = 0x02;                                               //Enable interrupt on P1_1. 
  PICTL = 0x00;                                               //Trigger interrupt on rising edge.
  
  
  
  spi_init();                                                 //Defined after pin configuration.
  while(!(lis2de12_Begin()));
  //setRange(LIS2DE12_RANGE_4_G);
  
  
  //Set individual interrupt enable bit to 1 in IENx.(In This case IEN1).
  //HAL_INT_ENABLE(INUM_P1INT, INT_ON);                         // Enable Port 1 general interrupt.  
  
  //Enable global interrupt.
  INT_GLOBAL_ENABLE(INT_ON);                                  // Enable interrupts globally.
  //========================================================================//
  
  //Interrupt configuration for sensor.
  //========================================================================//
  uint8 temp1 = read_reg(LIS2DE12_REG_REFERENCE);
  printf("%x\n",temp1);
  
  temp1 = read_reg(LIS2DE12_REG_STATUS);
  printf("%x\n",temp1);
  
  //Threshold for Interrupt2 is +-2g.
  write_reg(LIS2DE12_REG_INT2THS, 0x75);
  
  //Set Interrupt duration to 0ms.
  write_reg(LIS2DE12_REG_INT2DUR, 0x00);
  
  //Configure Interrupt2 on positive limits of Z axis.
  write_reg(LIS2DE12_REG_INT2CFG, 0x6F);                      //previous is 0x6F with LIS2DE12_REG_INT1CFG.
  
  
  //while(1)
  {
    //float a = readTemperature();
    //printf("%f\n",a);
    
     //BOOL R = selfTest();
     //printf("%d\n",R);
    
  }
  
  //write_reg(LIS2DE12_REG_INT1CFG, 0x20);
  
  //Read Interrupt source interrupt.
  //temp = read_reg(LIS2DE12_REG_INT1SRC);
  //printf("%x\n",temp);
  
  //temp = read_reg(LIS2DE12_REG_INT1SRC);
  //delay(10);
  
  //========================================================================//   
}


/*==== PRIVATE FUNCTIONS =====================================================*/
BOOL lis2de12_Begin()
{
  //Check connections.
  uint8 deviceId = read_reg(LIS2DE12_REG_WHOAMI);
  printf("%x\n",deviceId);
  if(deviceId != 0x33)
  {
    return 0;
  }
  
  //Enable all axes.
  write_reg(LIS2DE12_REG_CTRL1, 0x5F);
  
  //Set Sampling rate(100 HZ).
  setDataRate(LIS2DE12_DATARATE_100_HZ);
  
  //CTRL2 register cleared.
  write_reg(LIS2DE12_REG_CTRL2, 0x00);
  
  //Enable IA1 on INT1.
  write_reg(LIS2DE12_REG_CTRL3, 0x00);  
  
  //CTRL4 register cleared.
  write_reg(LIS2DE12_REG_CTRL4, 0x00);         //BDU disabled.
  
  //Get rising edge on INT1 by setting to 0x00.
  write_reg(LIS2DE12_REG_CTRL5, 0x00);//08
  
  //CTRL6 INT2 Enabled
  write_reg(LIS2DE12_REG_CTRL6, 0x20);         //0x40 in previous code.
  
  //Enable Temperature.
  write_reg(LIS2DE12_REG_TEMPCFG, 0xC0);       
  
  return 1;
}

BOOL selfTest()
{
  //Normal mode data collection.
  write_reg(LIS2DE12_REG_CTRL4, 0x00);
  
  OUTX_NOST = OUTY_NOST = OUTZ_NOST = 0;
  xyzAvg(&OUTX_NOST,&OUTY_NOST,&OUTZ_NOST);
  
  //Enable Self test in CTRL4.
  write_reg(LIS2DE12_REG_CTRL4, 0x02);      //Self test 0 configuration.
  
  //Self Test Data collection.
  OUTX_ST = OUTY_ST = OUTZ_ST = 0;
  xyzAvg(&OUTX_ST,&OUTY_ST,&OUTZ_ST);
  
  //Off self test mode.
  write_reg(LIS2DE12_REG_CTRL4, 0x00);
  
  //Checking outputs within limits.
  if(4 <= abs(OUTX_ST - OUTX_NOST) <= 90)
  {
    if(4 <= abs(OUTY_ST - OUTY_NOST) <= 90)
    {
      if(4 <= abs(OUTZ_ST - OUTZ_NOST) <= 90)
      {
        return 1;
      }
    }
  }
  else
  {
    return 0;
  }
}

static float readTemperature()
{
  Temperature = 0;Final_temp = 0;
  TH = 0;TL = 0;
  
  write_reg(LIS2DE12_REG_CTRL4, 0x80);         //BDU Enabled as we have to take current frame of temperature from L and H registers.
  status = read_reg(STATUS_REG_AUX);
  //printf("%x\n",status);
  
  if((status & 0x44) == 0x44)               //Check for DA and DOVRN bits. 
  {
    TH = read_reg(OUT_TEMP_H);
    //printf("%x\n",TH);
    TL = read_reg(OUT_TEMP_L);
    //printf("%x\n",TL);
    
    Temperature |= TH;
    Temperature = (Temperature << 8);
    Temperature |= TL;
    //printf("%d\n",Temperature);
    
    if(Temperature & 0x8000 == 0x8000)
    {
      Temperature = ~(Temperature);
      Temperature = Temperature + 0x01;
      Temperature = ((-1) * Temperature);
    }
    Final_temp = (float)(Temperature/127.0);
    //printf("%f\n",Final_temp);
  }
  return Final_temp;
}

void xyzAvg(INT16 *XOUT, INT16 *YOUT, INT16 *ZOUT)
{
  X_data = Y_data = Z_data = 0;DA = 0;
  
  startTimer3();
  currentmillis = millis;
  while(millis - currentmillis < 90);
  stopTimer3();
  
  DA = read_reg(LIS2DE12_REG_STATUS);
  if(DA & 0x08 == 0x08)
  {
    //Discarding 1st data by reading X Y Z registers.
    X_data = getX();
    Y_data = getY();
    Z_data = getZ();
    
    X_data = Y_data = Z_data = 0;
    
    //Average 5 sample XYZ data.
    for(i=0; i<5; i++)
    {
      startTimer3();
      currentmillis = millis;
      while(millis - currentmillis < 90);
      stopTimer3();
      
      DA = read_reg(LIS2DE12_REG_STATUS);
      if(DA & 0x08 == 0x08)
      {
        X_data = getX();
        Y_data = getY();
        Z_data = getZ();
      }
      *(XOUT) = *(XOUT) + X_data;
      *(YOUT) = *(YOUT) + Y_data;
      *(ZOUT) = *(ZOUT) + Z_data;
      
      X_data = Y_data = Z_data = 0;
      DA = 0;
    }   
    *(XOUT) = *(XOUT)/5;
    *(YOUT) = *(YOUT)/5;
    *(ZOUT) = *(ZOUT)/5;
  }
}

void setDataRate(uint8 dataRate)
{
  uint8 ctl1 = read_reg(LIS2DE12_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  write_reg(LIS2DE12_REG_CTRL1, ctl1);
}

void setRange(uint8 range)
{
  uint8 r = read_reg(LIS2DE12_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  write_reg(LIS2DE12_REG_CTRL4, r);
}

static INT8 getX()
{
  return readData8(LIS2DE12_REG_OUT_X);
}

static INT8 getY()
{
  return readData8(LIS2DE12_REG_OUT_Y);
}

static INT8 getZ()
{
  return readData8(LIS2DE12_REG_OUT_Z);
}

static INT8 readData8(uint8 regId)
{
  uint8 regValue,Result = 0;
  
  regValue = read_reg(regId);
  
  Result = regValue;
  
  if(regValue & 0x80 == 0x80)
  {
    Result = ~(regValue);
    Result = Result + 0x01;
    return ((-1) * Result);
  }
  else
  {
    return Result;
  }
}
/*==== INTERRUPT SERVICE ROUTINES ============================================*/
/******************************************************************************
* Interrupt on pin P1_0.
*
*
******************************************************************************/
/*
#pragma vector = P1INT_VECTOR
__interrupt void Forcefulremoval_interrupt(void)
{
//Clear modular interrupt first.
P1IFG = 0x00;

//Clear CPU interrupt.
IRCON2 &= ~0x80;

temp = read_reg(LIS2DE12_REG_INT1SRC);
halWait(10); 

if((temp & 0x40) == 0x40)
{
printf("Hi\n");
temp = 0x00;
LED1 = LED_ON;
halWait(10); 
LED1 = LED_OFF;
  }
}*/

/*==== END OF FILE ==========================================================*/
