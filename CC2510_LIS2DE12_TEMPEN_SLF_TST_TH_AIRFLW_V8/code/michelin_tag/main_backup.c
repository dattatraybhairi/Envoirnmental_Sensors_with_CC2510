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
//#include <stdio.h>
#include <math.h>
#include "UART.h"

//#define PI 3.14159265358979323846
#define log_val 25.76331119
/*================ channels used for communication ============*/
#define sync_channel 10
#define data_channel 30
#define tag1_channel 20              // ZM1 skywork 8 antenna tag channel
//#define tag3_channel 50               // ZM2 analog device 8 antenna tag channel
//#define tag5_channel 70               // ZM3 analog device 8 antenna tag channel
//#define tag7_channel 90               // ZM4 analog device 8 antenna tag channel

/*================ packet size for different communication============*/

#define sync_receive_pkt_len 1
#define dat_receive_pkt_len 3
#define dat_send_cmd_pkt_len 1
#define final_data_send_pkt_len 63


/*================ packet related values============*/
#define cm_id 1
#define Data1 150

#define count_iteration  5
#define Antenna_num  9
#define RX_paktlen   3

#define time_per_packet 70
#define No_of_times_packet_repeat 3

#define total_tag_no 1

#define total_number_antennas 8
#define values_per_second 60

UINT8 PACKET_LENGTH = 17;
volatile UINT32 millis;
UINT32 curmillis;

volatile BOOL received = FALSE;

volatile BOOL pktSentFlag = FALSE;            // Flag set whenever a packet is sent
volatile BOOL pktRcvdFlag = FALSE;            // Flag set whenever a packet is received
volatile BOOL time_up = FALSE;                 //flag set when timer over flows
volatile BYTE mode;                           // Radio operating mode, either RX or TX

/*===================== Iimer3 Relatated =======================================*/

UINT8 total_time = 0;
volatile UINT8 timer3_count = 0;

void init_timer3();
void start_timer3(UINT8 T3CC0_count, UINT8 total_count);

/*==================== DMA related function ===================================*/

void DMA_config(BYTE current_mode,BYTE length, BYTE channel);
void current_mode(BYTE current_mode_val);
void switch_antenna(BYTE pin_val);
void init_pin_for_antenna();
void transmit_final_data(UINT8 index_val);
void ReadData(UINT8 iteration);
void filter_val();

INT8 repeat_count = 3;
//INT8 All_packet_array[count_iteration*12*3];
INT8 All_packet_array[180];
INT8 All_counter_array[12];
//INT8 Filter_maiden_array[27];

UINT8 current_count = 0;

UINT8 med_rssi(UINT8 index);
UINT8 filter_function(UINT8 *filter_array);  
void process_Multipath(UINT8 *array, UINT8 len);
void standard_deviation(UINT8 *array_for_sd);

double RSSI_to_E_field(INT8 RSSI_val);
UINT8 E_field_to_RSSI(double E_field_strength);

float Compute_AoA_225(UINT8 *temp_array);
float AoA_resolution_225(float Computed_AoA);

void all_AoA();
//UINT8 final_rssi_array[45]={51,52,53,52,54,47,45,47,48,45,42,43,41,42,41,35,36,35,34,37,55,58,59,58,56,60,61,62,62,63,68,69,69,67,66,70,75,74,72,71,33,32,31,32,33};

UINT8 Index_Array_1X3[3] = {1,2,3};


BYTE Headers1[3] = {0xDA,0xA1,0x19};             // for ZM1 8 antenna skywork structure
BYTE Headers2[3] = {0xDB,0xA2,0x1A};             // for ZM2 8 antenna analog devices structure
BYTE Headers3[3] = {0xDC,0xA3,0x1B};             // for ZM3 8 antenna analog devices structure
BYTE Headers4[3] = {0xDD,0xA4,0x1C};             // for ZM4 8 antenna analog devices structure
/*==== 9th antenna ======================================================*/
BYTE nineth_antenna_len = 0;             // total data to be received from 9th antenna       
BYTE nineth_antenna_index = 0;           // index for data receiving from 9th antenna from UART
BYTE current_iteration;                 // current no of receiving data from tag

/*==== PUBLIC FUNCTIONS ======================================================*/

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
void main(void)
{
  
    halPowerClkMgmtSetMainClkSrc(CRYSTAL);
    
    CLKCON = (CLKCON & ~CLKCON_TICKSPD) | TICKSPD_DIV_128;
    init_timer3();
    UART0_init();
    init_pin_for_antenna();
    
    PACKET_LENGTH = 64; 
    radioConfigure(DATA_RATE_3_CC2510, FREQUENCY_4_CC2510);

    HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
    RFIM |= IRQ_DONE;                    // Mask IRQ_DONE flag only
    IP1 = 0x05;         //keep rx tx interrupt on highest priority then uart 0 and last timer 4
    IP0 = 0x08;
    INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
    
    while(1)
    {
      DMA_config(RADIO_MODE_RX, sync_receive_pkt_len , sync_channel);
      current_mode(RADIO_MODE_RX);
      
      while(!pktRcvdFlag);
      pktRcvdFlag = FALSE;
      
      if(radioPktBuffer[1] == 100)
      {
       
       DMA_config(RADIO_MODE_RX, dat_receive_pkt_len , data_channel);
       for(UINT8 k = 0 ; k < 3; k++)
       {
        current_iteration = k;
        for( UINT8 z=0 ; z < total_number_antennas ; z++)
        {
          switch_antenna(z);
          current_mode(RADIO_MODE_RX);
          
          curmillis = millis;
          
          while(millis - curmillis < 120)
          {         
              if(pktRcvdFlag)
              {
                pktRcvdFlag = FALSE;
                //Adding condition for the correct antenna switch
                if((z+1) == radioPktBuffer[2])
                ReadData(k);
                
                current_mode(RADIO_MODE_RX);
              }
          }
          
         
          
        }
        
       // wait for 120 ms more
        curmillis = millis;
        while(millis - curmillis < 150);
        
        P0 &= ~(0x1C);
        P0 |= 0x05;
        
        
        for(UINT8 i = 0; i<12; i++)
        {
        All_counter_array[i] = 0;
        }
        
        //curmillis = millis;
        //while(millis - curmillis < 20);
        //all_AoA();
        }
        
        
        
        DMA_config(RADIO_MODE_RX, dat_send_cmd_pkt_len , tag1_channel);       // ZM1 skywork 8 antenna
        //DMA_config(RADIO_MODE_RX, dat_send_cmd_pkt_len , tag3_channel);         // ZM2 analog devices 8 antenna
        //DMA_config(RADIO_MODE_RX, dat_send_cmd_pkt_len , tag5_channel);         // ZM3 analog devices 8 antenna
        //DMA_config(RADIO_MODE_RX, dat_send_cmd_pkt_len , tag7_channel);         // ZM4 analog devices 8 antenna
        
        current_mode(RADIO_MODE_RX);
        
        while(!received)
        {
          if(pktRcvdFlag)
          {
            pktRcvdFlag = FALSE;
            if(radioPktBuffer[1] == 100)
            {
               curmillis = millis;
               while(millis - curmillis < 50);
                  
              transmit_final_data(1);
               
              transmit_final_data(2);
              
               
              transmit_final_data(3);
              
               
              transmit_final_data(4);
              
              received = TRUE;
            }  
            else
              current_mode(RADIO_MODE_RX);
          }
        }
        
        received = FALSE;
        
       
    }
    for(UINT8 i = 0; i<12*count_iteration; i++)
      {
        All_packet_array[i] = 0;
      }
      for(UINT8 i = 0; i<12; i++)
      {
        All_counter_array[i] = 0;
      }
    }
  
}


/*==== PRIVATE FUNCTIONS =====================================================*/
void init_timer3()
{
  T3CTL = T3CTL_DIV_1 | T3CTL_OVFIM |
        T3CTL_CLR | T3CTL_MODE_MODULO;
  T3IE = 1;
  T3CNT = 0;
  T3CC0 = 204;
  millis = 0;
  T3CTL |= (T3CTL_START | T3CTL_CLR);  
}

/*============ DMA Related Fuctions ======================================*/
void DMA_config(BYTE current_mode, BYTE length, BYTE channel)
{
   PACKET_LENGTH = length;
   dmaRadioSetup(current_mode);
   mode = current_mode;
   CHANNR = channel;
}

void current_mode(BYTE current_mode_val)
{
  if(current_mode_val == RADIO_MODE_RX)
  {
    DMAARM = 0x83;
    DMAARM = DMAARM_CHANNEL1;           // Arm DMA channel 0
    RFST   = STROBE_SIDLE;
    RFST   = STROBE_RX;  
  }
  
  else 
  {   
    DMAARM = 0x83;
    DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
    RFST   = STROBE_SIDLE;
    RFST   = STROBE_TX;  
  } 
}

void switch_antenna(BYTE pin_val)
{
    P0 &= ~(0x1C);
    P0 |= ((pin_val) << 2); 
}
void init_pin_for_antenna()
{
  //Pin P0_2 - P0_4 are for the one RF switch and P1_4,P1_5 are for second swich
  P0SEL &= ~(0x1C);
  P0DIR |= 0x1C;
  P0 &= ~(0x1C);
  P0 |= 0x05;
  
  /*P1SEL &= ~(0x50);
  P1DIR |= 0x50;
  P1 &= ~(0x50);*/ 
}

void ReadData(UINT8 iteration)
{
 
     All_packet_array[(radioPktBuffer[2] - 1) * 5 + All_counter_array[radioPktBuffer[2] - 1]+(60*iteration)] = (-1)*convertRssiByte(radioPktBuffer[RX_paktlen + 1]);
     if (radioPktBuffer[2] == 1 && All_counter_array[0] < 5)
     {
        All_counter_array[0]++;
     }
     else if (radioPktBuffer[2] == 2 && All_counter_array[1] < 5)
     {
        All_counter_array[1]++;
     }
     else if (radioPktBuffer[2] == 3 && All_counter_array[2] < 5)
     {
        All_counter_array[2]++;
     }
     else if (radioPktBuffer[2] == 4 && All_counter_array[3] < 5)
     {
        All_counter_array[3]++;
     }
     else if (radioPktBuffer[2]== 5 && All_counter_array[4] < 5)
     {
        All_counter_array[4]++;
     }
     else if (radioPktBuffer[2] == 6 && All_counter_array[5] < 5)
     {
        All_counter_array[5]++;
     }
     else if (radioPktBuffer[2] == 7 && All_counter_array[6] < 5)
     {
        All_counter_array[6]++;
     }
     else if (radioPktBuffer[2] == 8 && All_counter_array[7] < 5)
     {
        All_counter_array[7]++;
     }
     else if (radioPktBuffer[2] == 9 && All_counter_array[8] < 5)
     {     
        All_counter_array[8]++;
     }
     else if (radioPktBuffer[2] == 10 && All_counter_array[9] < 5)
     {
        All_counter_array[9]++;
     }
     else if (radioPktBuffer[2] == 11 && All_counter_array[10] < 5)
     {
        All_counter_array[10]++;
     }
     else if (radioPktBuffer[2] == 12 && All_counter_array[11] < 5)
     {
        All_counter_array[11]++;
     }
    
     //DMAARM |= DMAARM_CHANNEL1;  // Arm DMA channel 1
     //RFST = STROBE_RX;
    //printf("%d\n",radioPktBuffer[0]);
    //printf("%d\n",radioPktBuffer[1]);
    //printf("%d\n",radioPktBuffer[2]);
    //printf("%d\n",radioPktBuffer[3]);
    //printf("%d\n",(-1)*convertRssiByte(radioPktBuffer[RX_paktlen + 1]));
  
}

void transmit_final_data(UINT8 index_val)
{     
        DMA_config(RADIO_MODE_TX, final_data_send_pkt_len , tag1_channel);    // for ZM1 skywork 8 antennas
        //DMA_config(RADIO_MODE_TX, final_data_send_pkt_len , tag3_channel);    // for ZM2 skywork 8 antennas
        //DMA_config(RADIO_MODE_TX, final_data_send_pkt_len , tag5_channel);    // for ZM3 skywork 8 antennas
        // DMA_config(RADIO_MODE_TX, final_data_send_pkt_len , tag7_channel);    // for ZM4 skywork 8 antennas
        
        if(index_val == 1)
        {
          radioPktBuffer[0] = PACKET_LENGTH;
          radioPktBuffer[1] = Headers1[0];
          radioPktBuffer[2] = Headers1[1];
          radioPktBuffer[3] = Headers1[2];
        }
         
        else if(index_val == 2)
        {
          radioPktBuffer[0] = PACKET_LENGTH;
          radioPktBuffer[1] = Headers2[0];
          radioPktBuffer[2] = Headers2[1];
          radioPktBuffer[3] = Headers2[2];
        }
           
        else if(index_val == 3)
        {
          radioPktBuffer[0] = PACKET_LENGTH;
          radioPktBuffer[1] = Headers3[0];
          radioPktBuffer[2] = Headers3[1];
          radioPktBuffer[3] = Headers3[2];
        }
            
        else if(index_val == 4)
         {
          radioPktBuffer[0] = 48;
          radioPktBuffer[1] = Headers4[0];
          radioPktBuffer[2] = Headers4[1];
          radioPktBuffer[3] = Headers4[2];
          
          filter_val();
        }
            
         if(index_val != 4)
         {
         for (UINT8 i = 4; i < PACKET_LENGTH; i++) 
          {
              radioPktBuffer[i] = All_packet_array[(i-4)+(60*(index_val-1))];
          }
         }
          
        current_mode(RADIO_MODE_TX);
        while(!pktSentFlag);
        pktSentFlag = FALSE;
                  
}

void filter_val()
{
  INT8 Filter_maiden_array[27];
  UINT8 Final_filter_array[9];
  
  for (UINT8 i = 0; i < 9; i++) 
    Filter_maiden_array[i] = med_rssi(i);
  
   for (UINT8 i = 12; i < 21; i++)
     Filter_maiden_array[i-3] = med_rssi(i);
   
    for (UINT8 i = 24; i < 33; i++)
     Filter_maiden_array[i-6] = med_rssi(i);
    
   for (UINT8 i = 0; i < 27; i++) 
  {
    radioPktBuffer[i+4] = Filter_maiden_array[i];
  }
  
  
  UINT8 test_array[3];
  for (UINT8 i = 0; i < 9; i++) 
  {
    test_array[0] = Filter_maiden_array[i];
    test_array[1] = Filter_maiden_array[i+9];
    test_array[2] = Filter_maiden_array[i+18];
    
    // here need to implement Stad deviation
    standard_deviation(test_array);
    
    Final_filter_array[i] = filter_function(test_array);  
  }
   for (UINT8 i = 0; i < 9; i++) 
  {
    radioPktBuffer[31+i] = Final_filter_array[i];
  }
  
  UINT8 RP_XF1[3], RP_XF2[3],RP_XF3[3]; 
   
    for(UINT8 i = 0; i<3 ;i++)
    {
      RP_XF1[i] = Final_filter_array[i];
      RP_XF2[i] = Final_filter_array[i+3];
      RP_XF3[i] = Final_filter_array[i+6];
    }
    process_Multipath(RP_XF1,3);
    process_Multipath(RP_XF2,3);
    process_Multipath(RP_XF3,3);
    
    for(UINT8 i = 0; i<3 ;i++)
    radioPktBuffer[40+i] = RP_XF1[i];
    
    for(UINT8 i = 0; i<3 ;i++)
    radioPktBuffer[43+i] = RP_XF2[i];
    
    for(UINT8 i = 0; i<3 ;i++)
    radioPktBuffer[46+i] = RP_XF3[i];
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


#pragma vector = T3_VECTOR
__interrupt void t3_isr(void)
{
    /* Clears the module interrupt flag. */
    T3OVFIF = 0;
     
   // T3CTL &= ~T3CTL_START;
    millis++;
    /* Clears the CPU interrupt flag. */
    T3IF = 0;
}

//Receive interrupt for the UART0
#pragma vector = URX0_VECTOR
__interrupt void UART0_rx_isr(void)
{
  UINT8 buffer;
  URX0IF = 0; 
  buffer = U0DBUF;
 // RX_BYTE = U0DBUF;
  if(!nineth_antenna_len)
  {
    if(buffer<6)
    nineth_antenna_len = buffer;
    nineth_antenna_index =0;
  }
  
  else
  {
    All_packet_array[(60*current_iteration)+40+nineth_antenna_index] = buffer;
    nineth_antenna_index++;
    if(nineth_antenna_index == nineth_antenna_len)
    {
     nineth_antenna_len = 0;
    }
  }
}


/*==== END OF FILE ==========================================================*/

UINT8 med_rssi(UINT8 index)
{
  UINT8 RSSI_array[5];// = {51,52,53,52,54};
  UINT8 _index = index * 5;
  for(UINT8 i = 0; i<5 ; i++)
   {
     RSSI_array[i] = All_packet_array[i+(_index)];
   }
   
   UINT8 med[5];
   UINT8 temp;
   UINT8 med_index;
   
   for(UINT8 i=0; i<5-1; i++) {
        for(UINT8 j=i+1; j<5; j++) {
            if(RSSI_array[j] <RSSI_array[i]) {
                // swap elements
                temp = RSSI_array[i];
                RSSI_array[i] = RSSI_array[j];
                RSSI_array[j] = temp;
            }
        }
    }
   // 51 52 52 53 54
     
    for(UINT8 i = 0; i<5 ; i++)
   {
     UINT8 low,high;
     med[i] = 0;
     
     if(RSSI_array[i] != 0)
     {  
     low = RSSI_array[i] - 1;
     high = RSSI_array[i] + 1;
     
     
      for(UINT8 j = 0; j<5 ; j++)
      {
        if(RSSI_array[j] >= low && RSSI_array[j] <= high)
          med[i]++;        
      }
     }
   }
   
    /*//printf("\n median is\n ");
    for(UINT8 i = 0; i<5 ; i++)
   {
     printf("\t%d", med[i]);
   }
   */
    temp = 0;
    for(UINT8 i = 0; i<5 ; i++)
   {
     
     if(temp < med[i])
     {
       temp = med[i];
       med_index = i;
     }
   }
   
   //printf("\n final median = %d", RSSI_array[med_index]);
   return (RSSI_array[med_index]);
}

/*============================ Process Multipath =============================*/
void process_Multipath(UINT8 *array, UINT8 len)
{
  UINT8 local_array[3];
  UINT8 *str_addr;
  str_addr = array;
  
  double E_field_Array[3];
  
  for(UINT8 i = 0; i < len ; i++)
    local_array[i] = *array++;
  
  /*printf("\n local array = \n");
    for(UINT8 i = 0; i<3 ; i++)
   {
     printf("\t%d", local_array[i]);
   }*/
  
  for(UINT8 i = 0; i < len ; i++)
  E_field_Array[i] = RSSI_to_E_field(-1*(local_array[i]));
  
  //value = ((((E_field_Array[1] + E_field_Array[2])/2) + E_field_Array[3])/2);
  *str_addr++ = E_field_to_RSSI(((((E_field_Array[0] + E_field_Array[1])/2) + E_field_Array[2])/2));
 
  *str_addr++ = E_field_to_RSSI((((E_field_Array[0] + E_field_Array[2])/2) + E_field_Array[1])/2);
  
  *str_addr = E_field_to_RSSI(((((E_field_Array[1] + E_field_Array[2])/2) + E_field_Array[0])/2)); 
}

/*============================ RSSI_to_E_field =============================*/
double RSSI_to_E_field(INT8 RSSI_val)
{
   if(RSSI_val != 0) return (pow(10,(RSSI_val - 30 +log_val)/20));
   else return(0);
}

/*============================ E_field_to_RSSI =============================*/

UINT8 E_field_to_RSSI(double E_field_strength)
{
   return((UINT8) (-1*(20*log10(E_field_strength) - log_val + 30-0.5)));
}

float Compute_AoA_225(UINT8 *temp_array)
{
  UINT8 Value_Array_1X3[3];
  UINT8 temp = 255;
  UINT8 min_val_index;
  UINT8 threshold = 3;
  
  float AoA_Low,AoA_High;
  
  for(UINT8 i = 0; i < 3 ; i++)
    Value_Array_1X3[i] = *temp_array++;
  
 // printf("\n value array 1 = \n");
  for(UINT8 i = 0; i<3 ; i++)
   {
    //printf(" %f",Value_Array_1X3[i]);
     if(temp > Value_Array_1X3[i])
     {
       temp = Value_Array_1X3[i];
       min_val_index = i;
     }
   }
  min_val_index++;
  //printf("\n min value index is = %d", min_val_index);
  if (min_val_index == 1)
  {
    //% Position around Antenna 1
    if ((Value_Array_1X3[1] - Value_Array_1X3[0]) > threshold && (Value_Array_1X3[2] - Value_Array_1X3[0]) > threshold)
    {
        AoA_Low = (float)(Index_Array_1X3[0]-1)*45;
        AoA_High = AoA_Low + 22.5;
    }
    //% Position around Antenna 1 & Antenna 2
    else if ((Value_Array_1X3[2] - Value_Array_1X3[0]) > threshold)
    {
        AoA_Low = (float)(Index_Array_1X3[0]-1)*45;
        AoA_High = AoA_Low + 45;
    }
    //% Unable to take decision
    else
    {
        /*decision_status = 0;
        AoA_Low = (Index_Array_1X3(2)-1)*45;
        AoA_High = AoA_Low;*/
        return(0);
    }
  }
   
//%--------------------------------------------------------------------------    
//% Antenna 2 with maximum received power
  
else if (min_val_index == 2)
{
  //  % -
  // % Position around Antenna 2
    if ((Value_Array_1X3[0] - Value_Array_1X3[1]) > threshold && (Value_Array_1X3[2] - Value_Array_1X3[1]) > threshold)
    {
      // % Position towards Antenna 1    
        if ((Value_Array_1X3[2] - Value_Array_1X3[0]) > 2)
        {
            AoA_Low = (Index_Array_1X3[0]-1)*45 + 22.5;
            AoA_High = AoA_Low + 22.5;
        }
    //% Position towards Antenna 3
        else if ((Value_Array_1X3[0] - Value_Array_1X3[2]) > 2)
        {
            AoA_Low = (Index_Array_1X3[1]-1)*45;
            AoA_High = AoA_Low + 22.5;
        }
    //% Position at Antenna 2
        else
        {
            AoA_Low = (Index_Array_1X3[1]-1)*45;
            AoA_High = AoA_Low;
        }
    }
    //% -    
    //% Position can be around Antenna 1 & Antenna 2
    else if ((Value_Array_1X3[2] - Value_Array_1X3[1]) > threshold)
    {
          //% Position NOT around Antenna 3
        if ((Value_Array_1X3[2] - Value_Array_1X3[0]) > 2)
        {
            AoA_Low = (Index_Array_1X3[0]-1)*45;
            AoA_High = AoA_Low + 45;
        }
        //% UNABLE to locate
        else
        {
          return(0);
            /*decision_status = 0;
            AoA_Low = (Index_Array_1X3(2)-1)*45 - 9;
            AoA_High = AoA_Low + 9;
          */
        }
     }
   // % -    
   // % Position can be around Antenna 2 & Antenna 3
    else if ((Value_Array_1X3[0] - Value_Array_1X3[1]) > threshold)
    {
        //% Position NOT around Antenna 1
        if ((Value_Array_1X3[0] - Value_Array_1X3[2]) > 2)
        {
            AoA_Low = (Index_Array_1X3[1]-1)*45;
            AoA_High = AoA_Low + 45;
        }
       // % Unable to locate
        else
        {
          return(0);
           /* decision_status = 0;
            AoA_Low = (Index_Array_1X3(2)-1)*45 - 9;
            AoA_High = AoA_Low + 9;
          */
        }
    }
    else
    {
      return(0);
        //% Unable to locate
       /* decision_status = 0;
        AoA_Low = (Index_Array_1X3(2)-1)*45 - 9;
        AoA_High = AoA_Low + 9;*/
    }
}
//%--------------------------------------------------------------------------
//% Antenna 3 with maximum received power
else if(min_val_index == 3)
{
    //% Position around Antenna 3
    if (((Value_Array_1X3[0] - Value_Array_1X3[2]) > threshold) && ((Value_Array_1X3[1] - Value_Array_1X3[2]) > threshold))
    {
        AoA_Low = (Index_Array_1X3[1]-1)*45 + 22.5;
        AoA_High = AoA_Low + 22.5;
    }
    //% Position around Antenna 2 & Antenna 3 
    else if ((Value_Array_1X3[0] - Value_Array_1X3[2]) > threshold)
    {
        AoA_Low = (Index_Array_1X3[1]-1)*45;
        AoA_High = AoA_Low + 45;
    }
    else
    {
      return(0);
       // % Unable to locate
        /*decision_status = 0;
        AoA_Low = (Index_Array_1X3(2)-1)*45 - 9;
        AoA_High = AoA_Low + 9;*/
    }
}
   return((AoA_Low + AoA_High)/2);
}

float AoA_resolution_225(float Computed_AoA)
{
  //%[0 20 25 42.5 47.5 65  70 90]
  float AoA;
  float ANG_Array[8] = {((Index_Array_1X3[0]-1)*45) ,((Index_Array_1X3[0]-1)*45+20), ((Index_Array_1X3[0]-1)*45+25), ((Index_Array_1X3[0]-1)*45+42.5), ((Index_Array_1X3[0]-1)*45+47.5),((Index_Array_1X3[0]-1)*45+65),((Index_Array_1X3[0]-1)*45+70),((Index_Array_1X3[0]-1)*45+90)}; 

if (Computed_AoA > ANG_Array[0] && Computed_AoA <= ANG_Array[1])
    AoA = (ANG_Array[0] + ANG_Array[1])/2;
else if (Computed_AoA > ANG_Array[1] && Computed_AoA < ANG_Array[2])
    AoA = Computed_AoA;
else if (Computed_AoA >= ANG_Array[2] && Computed_AoA <= ANG_Array[3])
    AoA = (ANG_Array[2] + ANG_Array[3])/2;
else if (Computed_AoA > ANG_Array[3] && Computed_AoA < ANG_Array[4])
    AoA = Computed_AoA;
else if (Computed_AoA >= ANG_Array[4] && Computed_AoA <= ANG_Array[5])
    AoA = (ANG_Array[4] + ANG_Array[5])/2;
else if (Computed_AoA > ANG_Array[5] && Computed_AoA < ANG_Array[6])
    AoA = Computed_AoA;
else if (Computed_AoA >= ANG_Array[6] && Computed_AoA < ANG_Array[7])
    AoA = (ANG_Array[6] + ANG_Array[7])/2;
else
   AoA = -1;

return(AoA);
}

void all_AoA()
{
  UINT8 sort_array[9];
  //float Computed_AoA;
  //UINT8 temp_count;
   for(UINT8 i = 0; i<9 ;i++)
   {
     sort_array[i] = med_rssi(i);
   // printf(" %d", sort_array[i]);
   }
   
   UINT8 RP_XF1[3], RP_XF2[3],RP_XF3[3]; 
   
    for(UINT8 i = 0; i<3 ;i++)
    {
      RP_XF1[i] = sort_array[i];
      RP_XF2[i] = sort_array[i+3];
      RP_XF3[i] = sort_array[i+6];
    }
    
    /*printf("\n before multipath process\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF1[i]);
    }
    
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF2[i]);
    }
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF3[i]);
    }
    */
    
    process_Multipath(RP_XF1,3);
    process_Multipath(RP_XF2,3);
    process_Multipath(RP_XF3,3);
    
    /*
    printf("\n after process multipath\n");
    for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF1[i]);
    }
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF2[i]);
    }
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",RP_XF3[i]);
    }
    */
    
     UINT8 ARRAY1[3],ARRAY2[3],ARRAY3[3];
     
     ARRAY1[0] = RP_XF1[0];ARRAY1[1]=RP_XF2[0];ARRAY1[2]=RP_XF3[0];
     ARRAY2[0] = RP_XF1[1];ARRAY2[1]=RP_XF2[1];ARRAY2[2]=RP_XF3[1];
     ARRAY3[0] = RP_XF1[2];ARRAY3[1]=RP_XF2[2];ARRAY3[2]=RP_XF3[2];
     
     /*printf("\n after sorting\n");
    for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",ARRAY1[i]);
    }
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",ARRAY2[i]);
    }
    
     printf("\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      printf(" %d",ARRAY3[i]);
    }*/
     
     float AoA_data[3];
     
     AoA_data[0] = Compute_AoA_225(ARRAY1);
     AoA_data[1] = Compute_AoA_225(ARRAY2);
     AoA_data[2] = Compute_AoA_225(ARRAY3);
     
     float Computed_AoA = 0;
     UINT8 temp_count;
     
     //printf("\n Aoa float data=\n");
     for(UINT8 i = 0; i<3 ;i++)
    {
      //printf("\t%f",AoA_data[i]);
      if(AoA_data[i])
      {
        Computed_AoA += AoA_data[i];
        temp_count++;
      }
    }
    
    Computed_AoA /= temp_count;
    //printf("\n mean AoA = %f",Computed_AoA);
    
    Computed_AoA = AoA_resolution_225(Computed_AoA);
    //printf("\n final computed AoA = %f",Computed_AoA);
   
}

UINT8 filter_function(UINT8 *filter_array)  
{
  double E_field_value=0;
  UINT8 temp_count = 0;
  
  for (UINT8 i = 0; i < 3; i++) 
  {
   UINT8 temp_value = *filter_array++;
   
   if(temp_value > 0)
   {
        E_field_value += RSSI_to_E_field((-1)*temp_value);
        temp_count++;
   }
  }
   
   if(E_field_value)
      return(E_field_to_RSSI(E_field_value / temp_count));
    
   return(0);
}
void standard_deviation(UINT8 *array_for_sd)
{
  UINT8 local_array[3];
  UINT8 *str_addr;
  str_addr = array_for_sd;
  UINT8 sd_val[3];
  
  for(UINT8 i = 0; i < 3 ; i++)
    local_array[i] = *array_for_sd++;
  
  for(UINT8 i = 0; i<3 ; i++)
   {
     UINT8 low,high;
     sd_val[i] = 0;
     
      if(local_array[i] > 30 && local_array[i] < 90)
     {  
     low = local_array[i] - 3;
     high = local_array[i] + 3;
     
     
      for(UINT8 j = 0; j<3 ; j++)
      {
        if(local_array[j] >= low && local_array[j] <= high)
          sd_val[i]++;        
      }
     }   
   }
  for(UINT8 i = 0; i<3 ; i++)
   {
      if(sd_val[i] > 1)
        *str_addr++ = local_array[i];
      else
        *str_addr++ = 0;
   }
  
}