/*-----------------------------------------------------------------------------
|   File:      UART.c
|   Target:    cc1110, cc2510
|   Author:    Rahul Bhagwat
|   Revised:   2018-07-20
|   Revision:  1.0
|   Project:   UART_Test
+------------------------------------------------------------------------------
|Code written and desing at Vacustech pvt ltd
|
+------------------------------------------------------------------------------
| Purpose:    Start developing initial stage for UART 
+------------------------------------------------------------------------------
| Decription: All UART related functions
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "UART.h"
#include <ioCC2510.h>

/*==== CONSTS ================================================================*/

const char *freq = "freq";
const char *updata ="updt";
const char *channel ="chnl";
const char *user1 ="udt1";
const char *user2 ="udt2";
const char *user3 ="udt3";
const char *user4 ="udt4";

char  _receiveBuffer[_receiveBufferSize];
UINT8 _receiveBufferIndex;

UINT16 userData1 = 0,userData2 = 0,userData3 = 0,userData4 = 0;

/*==== PUBLIC FUNCTIONS ======================================================*/

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
  void initUART()
{
  // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
  // To avoid potential I/O conflict with USART1:
  // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
  PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

  // Configure relevant Port P0 pins for peripheral function:
  // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
  P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;

// Initialise bitrate = 11.52 kbps (U0BAUD.BAUD_M = 34, U0GCR.BAUD_E = 11)
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;


  // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

  // USART mode = UART (U0CSR.MODE = 1)
  U0CSR |= U0CSR_MODE | U0CSR_RE;

  // Start bit level = low => Idle level = high  (U0UCR.START = 0)
  U0UCR &= ~U0UCR_START;

  // Stop bit level = high (U0UCR.STOP = 1)
  U0UCR |= U0UCR_STOP;

  // Number of stop bits = 1 (U0UCR.SPB = 0)
  U0UCR &= ~U0UCR_SPB;

  // Parity = disabled (U0UCR.PARITY = 0)
  U0UCR &= ~U0UCR_PARITY;

  // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
  U0UCR &= ~U0UCR_BIT9;

  // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
  // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
  // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
  // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
  U0UCR &= ~U0UCR_D9;

  // Flow control = disabled (U0UCR.FLOW = 0)
  U0UCR &= ~U0UCR_FLOW;

  // Bit order = LSB first (U0GCR.ORDER = 0)
  U0GCR &= ~U0GCR_ORDER;
  
  // Interrupt settings for RX data only
  
  IEN0 |= 0x04;               //enable interrupt for rx
}

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
void sendUART(unsigned char data)
{
  // Clear any pending TX interrupt request (set U0CSR.TX_BYTE = 0)
  U0CSR &= ~U0CSR_TX_BYTE;
  
    U0DBUF = data;
    //U0DBUF = uartTxBuf[uartTxIndex];
    while(! (U0CSR&U0CSR_TX_BYTE) );
    U0CSR &= ~U0CSR_TX_BYTE;
}

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
void sendString(char *str)
{
  while(*str)
  {
    sendUART(*str);
    str++;
  }
}

/******************************************************************************
* @fn  storeData
*
* @brief
*      This function used to stored the receive string from the UART
*
* Parameters:
* 
* @para UINT8 _receiveData
*       single character received on UART0 is sent from interrupt and concanate
*       to form the string
*       
* @return void
*
******************************************************************************/
void storeData(char _receiveData)
{
  if(_receiveData == '$')
  _receiveBufferIndex = 1;
  
  else 
  {
    if((_receiveBufferIndex > 0) && (_receiveBufferIndex <= _receiveBufferSize))
    {
    if(_receiveData == '\n')
    {
      _receiveBuffer[_receiveBufferIndex] = '\0';
       
       if(decodeData())
       {
         //printf("\ncmdOK");
         sendString("OK\n");
       }
       
       else
       {
         //printf("\nError");
         sendString("ERROR\n");
       }
       
      _receiveBufferIndex = 0;
    //printf("\n%s",_receiveBuffer);
    }
  
    else
    {
      _receiveBuffer[_receiveBufferIndex] = _receiveData;
      _receiveBufferIndex++;
    }
    }
  }
}
/******************************************************************************
* @fn  decodeData
*
* @brief
*      This function used to decode incoming data on UART from host MCU 
*
* @para void
*       
* @return void
*
******************************************************************************/
UINT8 decodeData()
{
  char stringName[10], stringValue[5];
  UINT8 tempValue,tempValue2;
  UINT8 nameLength;
  UINT8 cmdIndex = 255;
  
  int value;
  
  memset((char *)stringName,0,10);
  memset((char *)stringValue,0,5);
  
  
  //printf("\n%s", _receiveBuffer);
  
  for(tempValue = 0; _receiveBuffer[tempValue+1]!='=' && tempValue<=_receiveBufferIndex; tempValue++)
    stringName[tempValue] = _receiveBuffer[tempValue+1];
  
  if(tempValue <= 4)  nameLength = 4; else nameLength = tempValue;
  
  stringName[tempValue++] = '\0';
  //printf("\n%s", stringName);
  
  //tempValue++;
  
  for(tempValue2=0 ;tempValue<= _receiveBufferIndex; tempValue++,tempValue2++)
    stringValue[tempValue2] = _receiveBuffer[tempValue+1];
  
  
  stringValue[tempValue2] = '\0';
  
  //printf("\n%s", stringValue);
  
  value = atoi(stringValue);
  //printf("\n%d",value);
  
  if(strncmp(stringName, freq, nameLength)==0)
    cmdIndex = 0;
    //printf("\nf %d", value);
 
  else if(strncmp(stringName, updata, nameLength)==0)
    cmdIndex = 1;
    //printf("\nup %d", value);
  
  else if(strncmp(stringName, channel, nameLength)==0)
    cmdIndex = 2;
    //printf("\nch %d", value);
  
  else if(strncmp(stringName, user1, nameLength)==0)
    cmdIndex = 3;
    //printf("\nu1 %d", value);
  
  else if(strncmp(stringName, user2, nameLength)==0)
    cmdIndex = 4;
    //printf("\nu2 %d", value);
  
  else if(strncmp(stringName, user3, nameLength)==0)
   cmdIndex = 5;
    //printf("\nu3 %d", value);
  
  else if(strncmp(stringName, user4, nameLength)==0)
   cmdIndex = 6;
    //printf("\nu4 %d", value);
  
  else
    //printf("\ner");
   memset((char *)_receiveBuffer,0,_receiveBufferSize);
   if(cmdIndex != 255) 
   return (setParameter(cmdIndex,value));
   else
    return(0);
   
}
/******************************************************************************
* @fn  setParameter
*
* @brief
*      This function used set the parameter received from host MCU  
*
* Parameters
*
*@para UINT8 commandNumber
*       Corresponding to command received from the Host mcu in range from 0 onwards
*       0-frequcy
*       1-update rate
*       2-channel
*       3-user parameter1
*       4-user parameter2
*       5-user parameter3
*       6-user parameter4
*       
*       int commandValue
*       Corresponding to respective value of the parameter in range from –32,768 to 32,767
* @return void
*
******************************************************************************/
UINT8 setParameter(UINT8 commandNumber, int commandValue)
{
  UINT8 sentFlag = 0;
  switch(commandNumber)
  {
    case 0:     //Base frequency for RF      
      switch(commandValue)
      {
        case 1:   
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_1_CC2510);
           //printf("\n2480");
           sentFlag = 1;
        break;
        
        case 2:
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_2_CC2510);
           sentFlag = 1;
            //printf("\n2460");
        break;
        
        case 3:
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_3_CC2510);
           sentFlag = 1;
            //printf("\n2440");
        break;
        
        case 4:
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_4_CC2510);
           sentFlag = 1;
            //printf("\n2420");
        break;
        
        case 5:
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_5_CC2510);
           sentFlag = 1;
           // printf("\n2433");
        break;
       
        case 6:
           RFST   = STROBE_SIDLE;
           radioConfigure(DATA_RATE_2_CC2510, FREQUENCY_6_CC2510);
           sentFlag = 1;
           // printf("\n2400");
        break;
        
      default:
          sentFlag = 0;
          //printf("\nerror");
          break;
      }
    break;
    
    case 1:     // update rate for the sending data 
      if(commandValue >= 1 && commandValue <=60)
      {
        updateInterval = (commandValue * 1000);
        sentFlag = 1;
         //printf("\nupok");
      }
      else 
        sentFlag = 0;
        
    break;
    
    case 2:     // channel for RF
      if(commandValue >= 0 && commandValue <=255)
      {
       RFST   = STROBE_SIDLE;
       CHANNR = commandValue;
       sentFlag = 1;
        //printf("\nchOK");
      }
      else 
        sentFlag = 0;
    break;
  
    case 3:     // user paramter 1
        userData1 = commandValue;
       //printf("\nu1");
       sentFlag = 1;
    break;
  
    case 4:      // user paramter 2
      userData2 = commandValue;
      //printf("\nu2");
      sentFlag = 1;
    break;
  
    case 5:     // user paramter 3
      userData3 = commandValue;
      //printf("\nu3");
      sentFlag = 1;
    break;
  
    case 6:      // user paramter 4
      userData4 = commandValue;
      //printf("\nu4");
      sentFlag = 1;
    break;
  
  default:
    break;
  
  }
  
  return(sentFlag);
}

/*==== END OF FILE ==========================================================*/
