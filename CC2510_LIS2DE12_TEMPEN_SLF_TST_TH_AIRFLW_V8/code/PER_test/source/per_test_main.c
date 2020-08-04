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

    UINT8 i;
    UINT32 burstSize;                   // Number of packets to burst
    UINT32 seqNum;                      // Sequence number for TX packet


    // Initialize LED1
    INIT_LED1();

    // Choose the crystal oscillator as the system clock
    halPowerClkMgmtSetMainClkSrc(CRYSTAL);

    // Initialize the LCD
    halBuiInitLcd();

    // Show Chipcon logo and chip name + revision until S1 is pushed
    showLogo();
    while(!halBuiButtonPushed());

    // Check that chip version is not too old or too new to be supported by
    // the register settings used in this software.
    checkChipVersion();

    // Select frequency and data rate from LCD menu, then configure the radio
    radioConfigure(selectDataRate(), selectRadioFrequency());

    // Select from LCD menu either transmitter or receiver mode
    mode = selectMode();

    if (mode == RADIO_MODE_TX) {

        // Set up the DMA to move packet data from buffer to radio
        dmaRadioSetup(RADIO_MODE_TX);

        // Configure interrupt for every time a packet is sent
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

        // Construct the packet to be transmitted in buffer
        radioPktBuffer[0] = PACKET_LENGTH;                  // Length byte
        radioPktBuffer[1] = (BYTE) (NETWORK_ID_KEY>>8);     // Network identifier
        radioPktBuffer[2] = (BYTE) NETWORK_ID_KEY;
        // radioPktBuffer[3:6] = 4 byte packet sequence number, written later
        // Fill rest of payload with dummy data. Radio is using data whitening.
        for (i = 7; i <= PACKET_LENGTH; i++) {
            radioPktBuffer[i] = 0xCC;
        }

        // Select from LCD menu the packet burst size
        burstSize = selectBurstSize();

        while (TRUE) {

            // Wait for S1 button press before starting
            halBuiLcdUpdate("Press S1 to", "start burst ...");
            while (!halBuiButtonPushed());

            halBuiLcdUpdate("Pkts transmit'd:", "");

            LED1 = LED_ON;

            // Transmit the packet burst
            for (seqNum = 1; seqNum <= burstSize; seqNum++) {

                // Set correct sequence number to packet
                pktSetSeqNum(seqNum);

                // Send the packet
                DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
                RFST = STROBE_TX;           // Switch radio to TX

                // Wait until the radio transfer is completed,
                // and then reset pktSentFlag
                while(!pktSentFlag);
                pktSentFlag = FALSE;

                // Update LCD with the packet counter
                lcdWriteSeqNum(seqNum);

                // Wait approx. 3 ms to let the receiver perform its
                // tasks between each packet
                for(int x = 0; x < 5 ;x++)
                halWait(100);

                // Abort burst if button S1 is pushed.
                if (halBuiButtonPushed() == TRUE) {
                    break;
                }
            }

            LED1 = LED_OFF;

        }
    }
    else if (mode == RADIO_MODE_RX) {

        // Set up the DMA to move packet data from radio to buffer
        dmaRadioSetup(RADIO_MODE_RX);

        // Configure interrupt for every received packet
        HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
        RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
        INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally

        halBuiLcdUpdate("Ready to", "receive");

        // Start receiving
        DMAARM = DMAARM_CHANNEL0;           // Arm DMA channel 0
        RFST   = STROBE_RX;                 // Switch radio to RX

        // Do not update the LCD until the first packet is received
        while (!pktRcvdFlag);
        halBuiLcdUpdate("PER:   0.0 %", "RSSI:      dBm");

        while (TRUE) {
            // Poll for incoming packet delivered by radio + dma
            if (pktRcvdFlag) {
                pktRcvdFlag = FALSE;

                // Check if the received packet is a valid PER test packet
                if (pktCheckValidity()) {

                    // A PER test packet has been received, hence the statistics
                    // on the LCD must be updated.
                    updateLcd = TRUE;
                    LED1 = LED_ON;      // Turn on LED to indicate PER test link

                    // Subtract old RSSI value from sum
                    perRssiSum -= perRssiBuf[perRssiBufCounter];
                    // Store new RSSI value in ring buffer, will add it to sum later
                    perRssiBuf[perRssiBufCounter] = convertRssiByte(radioPktBuffer[PACKET_LENGTH+1]);
                }

                // We don't need our packet buffer anymore, prepare for the next packet
                DMAARM = DMAARM_CHANNEL0;
                RFST = STROBE_RX;

                // Update the LCD only if necessary
                if (updateLcd) {

                    // Calculate PER of received packets in unit per 1000
                    // and print to LCD
                    lcdWritePer();

                    // Add the new RSSI value to sum. Calculate and print
                    // average RSSI to LCD
                    perRssiSum += perRssiBuf[perRssiBufCounter];
                    lcdWriteRssi(perRssiSum);
                    if (++perRssiBufCounter == RSSI_AVG_WINDOW_SIZE) {
                        perRssiBufCounter = 0;      // Wrap ring buffer counter
                    }

                    // Update blinking cursor to indicate there is activity
                    // (LED toggles according to bit5 of counter)
                    halBuiLcdUpdateChar(LINE1, 15,
                                        blinkCursor[(++blinkCursorIdx & 0x20)
                                            >> 5]);

                    updateLcd = FALSE;
                    LED1 = LED_OFF;

                }
            }
        }
    }
}


/*==== PRIVATE FUNCTIONS =====================================================*/



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

/*==== END OF FILE ==========================================================*/
