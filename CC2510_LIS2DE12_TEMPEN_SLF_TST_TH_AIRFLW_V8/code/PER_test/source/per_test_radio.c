/*-----------------------------------------------------------------------------
|   File:      per_test_radio.c
|   Target:    cc1110, cc2510
|   Author:    ESY
|   Revised:   2007-09-06
|   Revision:  1.0
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
| Purpose:    Functions for radio and packet handling for PER test
+------------------------------------------------------------------------------
| Decription: All functions related to radio configuration and packet
|             handling for the packet error rate test application.
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ===================================================*/
/*==== INCLUDES ==============================================================*/

#include "hal_main.h"
#include "per_test_main.h"


/*==== GLOBAL VARS ===========================================================*/

UINT32 perRcvdSeqNum = 0;
UINT32 perExpectedSeqNum = 1;
UINT32 perBadPkts = 0;
UINT32 perRcvdPkts = 0;

/*==== PUBLIC FUNCTIONS ======================================================*/

/******************************************************************************
* @fn  radioConfigure
*
* @brief
*        Configures the radio, either CC1110 or CC2510, supporting a set
*        of preset data rates and frequency options. Also, the transmitter's
*        transmit power level and receiver's RSSI offset is set.
*
* Parameters:
*
* @param  UINT32 dataRate
*           Data rate in bits per second (bps)
* @param  UINT32 frequency
*           RX/TX radio frequency to use (kHz)
*
* @return void
*
******************************************************************************/
void radioConfigure(UINT32 dataRate, UINT32 frequency) {

    /* NOTE: The register settings are hard-coded for the predefined set of data
     * rates and frequencies. To enable other data rates or frequencies these
     * register settings should be adjusted accordingly (use SmartRF(R) Studio).
     */

    // Distinguish between chip models (only one revision of each is supported)
    if (PARTNUM == PARTNUM_CC1110) {

        // Set transmit power: +10 dBm (register setting is freq dependent)
        // and configure the radio frequency to use
        switch (frequency) {
        case FREQUENCY_1_CC1110:        // 915 MHz
        default:
            PA_TABLE0 = 0xC0;
            FREQ2 = 0x23;
            FREQ1 = 0x31;
            FREQ0 = 0x3B;
            break;
        case FREQUENCY_2_CC1110:        // 903 MHz
            PA_TABLE0 = 0xC0;
            FREQ2 = 0x22;
            FREQ1 = 0xBB;
            FREQ0 = 0x13;
            break;
        case FREQUENCY_3_CC1110:        // 868 MHz
            PA_TABLE0 = 0xC2;
            FREQ2 = 0x21;
            FREQ1 = 0x62;
            FREQ0 = 0x76;
            break;
        case FREQUENCY_4_CC1110:        // 433 MHz
            PA_TABLE0 = 0xC0;
            FREQ2 = 0x10;               // Actually 433.500 MHz to fit band
            FREQ1 = 0xAC;
            FREQ0 = 0x4E;
            break;
        }

        // Set modulation and other radio parameters
        switch (dataRate) {
        case DATA_RATE_1_CC1110:
            // Settings from SmartRFStudio for CC1110, VERSION == 0x03
            // 250 kBaud, GFSK modulation, 540 kHz RX filter bandwidth.

            // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
            FSCTRL1 = (frequency == FREQUENCY_4_CC1110) ? 0x0C : 0x12;
            FSCTRL1  = 0x12;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0x2D;   // Modem configuration.
            MDMCFG3  = 0x3B;   // Modem configuration.
            MDMCFG2  = 0x13;   // Modem configuration.
            MDMCFG1  = 0x22;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x62;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0xB6;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x1D;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x1C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0xC7;   // AGC control.
            AGCCTRL1 = 0x00;   // AGC control.
            AGCCTRL0 = 0xB0;   // AGC control.
            FSCAL3   = 0xEA;   // Frequency synthesizer calibration.
            // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
            FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
            FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
            TEST2    = 0x88;   // Various test settings.
            TEST1    = 0x31;   // Various test settings.
            TEST0    = 0x09;   // Various test settings.

            // Determine proper RSSI offset for receiver (freq and rate dependent)
            // and configure the radio frequency to use
            switch (frequency) {
            case FREQUENCY_1_CC1110:    // 915 MHz
                perRssiOffset = 77;
                break;
            case FREQUENCY_2_CC1110:    // 903 MHz
                perRssiOffset = 77;
                break;
            case FREQUENCY_3_CC1110:    // 868 MHz
                perRssiOffset = 77;
                break;
            case FREQUENCY_4_CC1110:    // 433 MHz
            default:
                perRssiOffset = 73;
                break;
            }
            break;

        case DATA_RATE_2_CC1110:
            // Settings from SmartRFStudio for CC1110, VERSION == 0x03
            // 38.4 kBaud, GFSK modulation, 100 kHz RX filter bandwidth.
            FSCTRL1  = 0x06;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0xCA;   // Modem configuration.
            MDMCFG3  = 0x83;   // Modem configuration.
            MDMCFG2  = 0x13;   // Modem configuration.
            MDMCFG1  = 0x22;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x34;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0x56;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x16;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x6C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0x43;   // AGC control.
            AGCCTRL1 = 0x40;   // AGC control.
            AGCCTRL0 = 0x91;   // AGC control.
            FSCAL3   = 0xE9;   // Frequency synthesizer calibration.
            // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
            FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
            FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
            TEST2    = 0x81;   // Various test settings.
            TEST1    = 0x35;   // Various test settings.
            TEST0    = 0x09;   // Various test settings.

            // Determine proper RSSI offset for receiver (freq and rate dependent)
            switch (frequency) {
            case FREQUENCY_1_CC1110:    // 915 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_2_CC1110:    // 903 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_3_CC1110:    // 868 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_4_CC1110:    // 433 MHz
            default:
                perRssiOffset = 74;
                break;
            }
            break;

        case DATA_RATE_3_CC1110:
        default:
            // Settings from SmartRFStudio for CC1110, VERSION == 0x03
            // 1.2 kBaud, GFSK modulation, 58 kHz RX filter bandwidth.
            FSCTRL1  = 0x06;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0xF5;   // Modem configuration.
            MDMCFG3  = 0x83;   // Modem configuration.
            MDMCFG2  = 0x13;   // Modem configuration.
            MDMCFG1  = 0x22;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x15;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0x56;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x18;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x16;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x6C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0x03;   // AGC control.
            AGCCTRL1 = 0x40;   // AGC control.
            AGCCTRL0 = 0x91;   // AGC control.
            FSCAL3   = 0xE9;   // Frequency synthesizer calibration.
            // Freq. dependent value: 433 MHz band vs 868/915 MHz band:
            FSCAL2 = (frequency == FREQUENCY_4_CC1110) ? 0x0A : 0x2A;
            FSCAL0   = 0x1F;   // Frequency synthesizer calibration.
            TEST2    = 0x81;   // Various test settings.
            TEST1    = 0x35;   // Various test settings.
            TEST0    = 0x09;   // Various test settings.

            // Determine proper RSSI offset for receiver (freq and rate dependent)
            switch (frequency) {
            case FREQUENCY_1_CC1110:    // 915 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_2_CC1110:    // 903 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_3_CC1110:    // 868 MHz
                perRssiOffset = 73;
                break;
            case FREQUENCY_4_CC1110:    // 433 MHz
            default:
                perRssiOffset = 75;
                break;
            }
            break;
        }
    }
    else if (PARTNUM == PARTNUM_CC2510) {

        // Set transmit power: 0 dBm
        PA_TABLE0 = 0xFE;

        // Configure the radio frequency to use
        switch (frequency) {
        case FREQUENCY_1_CC2510:        // 2480 MHz
        default:
            FREQ2 = 0x5F;
            FREQ1 = 0x62;
            FREQ0 = 0x76;
            break;
        case FREQUENCY_2_CC2510:        // 2460 MHz
            FREQ2 = 0x5E;
            FREQ1 = 0x9D;
            FREQ0 = 0x89;
            break;
        case FREQUENCY_3_CC2510:        // 2440 MHz
            FREQ2 = 0x5D;
            FREQ1 = 0xD8;
            FREQ0 = 0x9D;
            break;
        case FREQUENCY_4_CC2510:        // 2420 MHz
            FREQ2 = 0x5D;
            FREQ1 = 0x13;
            FREQ0 = 0xB1;
            break;
        }


        // Set modulation and other radio parameters
        switch (dataRate) {
        case DATA_RATE_1_CC2510:
            // Settings from SmartRFStudio for CC2510, VERSION == 0x04
            // 500 kBaud, MSK modulation, 812 kHz RX filter bandwidth.
            FSCTRL1  = 0x10;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0x0E;   // Modem configuration.
            MDMCFG3  = 0x3B;   // Modem configuration.
            MDMCFG2  = 0x73;   // Modem configuration.
            MDMCFG1  = 0x42;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x00;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0xB6;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x14;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x1D;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x1C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0xC7;   // AGC control.
            AGCCTRL1 = 0x40;   // AGC control.
            AGCCTRL0 = 0xB2;   // AGC control.
            FSCAL3   = 0xEA;   // Frequency synthesizer calibration.
            FSCAL2   = 0x0A;   // Frequency synthesizer calibration.
            FSCAL0   = 0x11;   // Frequency synthesizer calibration.
            TEST2    = 0x88;   // Various test settings.
            TEST1    = 0x31;   // Various test settings.
            TEST0    = 0x0B;   // Various test settings.

            perRssiOffset = 72;// Set proper RSSI offset for receiver
            break;

        case DATA_RATE_2_CC2510:
            // Settings from SmartRFStudio for CC2510, VERSION == 0x04
            // 250 kBaud, MSK modulation, 540 kHz RX filter bandwidth.
            FSCTRL1  = 0x0A;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0x2D;   // Modem configuration.
            MDMCFG3  = 0x3B;   // Modem configuration.
            MDMCFG2  = 0x73;   // Modem configuration.
            MDMCFG1  = 0x22;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x00;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0xB6;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x14;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x1D;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x1C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0xC7;   // AGC control.
            AGCCTRL1 = 0x00;   // AGC control.
            AGCCTRL0 = 0xB2;   // AGC control.
            FSCAL3   = 0xEA;   // Frequency synthesizer calibration.
            FSCAL2   = 0x0A;   // Frequency synthesizer calibration.
            FSCAL0   = 0x11;   // Frequency synthesizer calibration.
            TEST2    = 0x88;   // Various test settings.
            TEST1    = 0x31;   // Various test settings.
            TEST0    = 0x0B;   // Various test settings.

            perRssiOffset = 71;// Set proper RSSI offset for receiver
            break;

        case DATA_RATE_3_CC2510:
        default:
            // Settings from SmartRFStudio for CC2510, VERSION == 0x04
            // 10 kBaud, 2-FSK modulation, 232 kHz RX filter bandwidth.
            FSCTRL1  = 0x08;   // Frequency synthesizer control.
            FSCTRL0  = 0x00;   // Frequency synthesizer control.
            MDMCFG4  = 0x78;   // Modem configuration.
            MDMCFG3  = 0x93;   // Modem configuration.
            MDMCFG2  = 0x03;   // Modem configuration.
            MDMCFG1  = 0x22;   // Modem configuration.
            MDMCFG0  = 0xF8;   // Modem configuration.
            DEVIATN  = 0x44;   // Modem deviation setting (when FSK modulation is enabled).
            FREND1   = 0x56;   // Front end RX configuration.
            FREND0   = 0x10;   // Front end RX configuration.
            MCSM0    = 0x14;   // Main Radio Control State Machine configuration.
            FOCCFG   = 0x16;   // Frequency Offset Compensation Configuration.
            BSCFG    = 0x6C;   // Bit synchronization Configuration.
            AGCCTRL2 = 0x43;   // AGC control.
            AGCCTRL1 = 0x40;   // AGC control.
            AGCCTRL0 = 0x91;   // AGC control.
            FSCAL3   = 0xA9;   // Frequency synthesizer calibration.
            FSCAL2   = 0x0A;   // Frequency synthesizer calibration.
            FSCAL0   = 0x11;   // Frequency synthesizer calibration.
            TEST2    = 0x88;   // Various test settings.
            TEST1    = 0x31;   // Various test settings.
            TEST0    = 0x0B;   // Various test settings.

            perRssiOffset = 74;// Set proper RSSI offset for receiver
            break;
        }
    }

    // Common radio settings for CCxx10, any frequency and data rate
    CHANNR   = 0x00;            // Channel number.
    MCSM1 = 0x30;               // Main Radio Control State Machine configuration.
    IOCFG2 = 0x0B;              // GDO2 output pin configuration.
    IOCFG0 = 0x06;              // GDO0 output pin configuration. Sync word.
    PKTCTRL1 = 0x04;            // Packet automation control.
    PKTCTRL0 = 0x45;            // Packet automation control. Data whitening on.
    ADDR = 0x00;                // Device address. Not used.
    PKTLEN = PACKET_LENGTH;     // Packet length.

    return;
}


/******************************************************************************
* @fn  pktCheckValidity
*
* @brief
*      Checks the received packet length and network ID to decide if it is a
*      valid PER test packet, hence affecting PER statistics.
*      The packet's CRC and sequence number will be analyzed in the
*      process of updating the appropriate variables needed to keep PER
*      statistics
*
* Parameters:
*
* @param  void
*
* @return BOOL
*         TRUE: Packet was a PER test packet
*         FALSE: Packet was not recognized as a PER test packet
*
******************************************************************************/
BOOL pktCheckValidity(void)
{
    // Check if the packet length is correct (byte 0 in packet)
    if (radioPktBuffer[0] == PACKET_LENGTH) {

        // Check if the network identifier bytes matches our network ID (byte 1 + 2)
        if (pktCheckId()) {
            // We have a match, this packet is probably meant for us

            // Check if received packet has correct CRC
            if (pktCheckCrc()) {

                // Extract and check if the packet's sequence number is as estimated
                perRcvdSeqNum = pktGetSeqNum();

                // Check if received packet is the expected packet
                if (perRcvdSeqNum == perExpectedSeqNum) {
                    // Yes, this is the expected packet. Counters are incremented.
                    perExpectedSeqNum++;
                    perRcvdPkts++;
                    return TRUE;
                }

                // Check if the sequence number is lower than the previous one,
                // if so we will assume a new data burst has started and we
                // will reset our statistics variables.
                else if (perRcvdSeqNum < perExpectedSeqNum) {
                    // Update our expectations assuming this is a new burst
                    perExpectedSeqNum = perRcvdSeqNum + 1;
                    perBadPkts = 0;
                    perRcvdPkts = 1;
                    return TRUE;
                }

                // For the case of a correct packet, but we have a jump in the
                // sequence numbering meaning some packets in between has been lost.
                // This packet loss will be handled when calculating PER.
                else {
                    perExpectedSeqNum = perRcvdSeqNum + 1;
                    perRcvdPkts++;
                    return TRUE;
                }
            }
            // For packet with correct ID, but wrong CRC
            else {
                perExpectedSeqNum++;    // Assume the seq num would have been correct
                perBadPkts++;
                perRcvdPkts++;
                return TRUE;
            }
        }
        // For packets with incorrect network ID, ignore them. If this was caused by
        // e.g. a bit error in one of the network identification bytes this will
        // later be accounted for as a packet error if a following received
        // packet has an unexpectedly high sequence number.
        else {
            return FALSE;
        }
    }
    // Packets with incorrect length are also ignored.
    else {
        return FALSE;
    }
}



/******************************************************************************
* @fn  pktSetSeqNum
*
* @brief
*      Sets a 4 byte sequence number into the statically defined packet buffer
*      radioPktBuffer
*
* Parameters:
*
* @param  UINT32 seqNum
*         The sequence number to write into the buffer
*
* @return void
*
******************************************************************************/
void pktSetSeqNum(UINT32 seqNum)
{
    // Insert the 4 byte sequence number into a static packet buffer
    radioPktBuffer[3] = (BYTE) (seqNum>>24);
    radioPktBuffer[4] = (BYTE) (seqNum>>16);
    radioPktBuffer[5] = (BYTE) (seqNum>>8);
    radioPktBuffer[6] = (BYTE) seqNum;
    return;
}

/******************************************************************************
* @fn  pktGetSeqNum
*
* @brief
*      Extracts a 4 byte sequence number from the statically defined
*      packet buffer radioPktBuffer
*
* Parameters:
*
* @param  void
*
* @return UINT32
*         The received packet's sequence number as a UINT32
*
******************************************************************************/
UINT32 pktGetSeqNum(void)
{
    UINT32 seqId = 0;

    // Grab the 4 byte sequence number from static packet buffer
    seqId = ((UINT32)radioPktBuffer[3]) << 24;
    seqId |= ((UINT32)radioPktBuffer[4]) << 16;
    seqId |= ((UINT32)radioPktBuffer[5]) << 8;
    seqId |= ((UINT32)radioPktBuffer[6]);

    return (seqId);
}




/******************************************************************************
* @fn  convertRssiByte
*
* @brief
*      Converts the RSSI (received signal strength indicator) value,
*      given as a 2's complement number, to dBm value. This requires that a
*      proper RSSI offset value is specified in global variable perRssiOffset
*      before use.
*
* Parameters:
*
* @param  BYTE rssiComp
*                   The RSSI value received from the radio as a 2's complement
*                   number
*
* @return INT16
*           The RSSI value in dBm
*
******************************************************************************/
INT16 convertRssiByte(BYTE rssiComp)
{
    // Convert RSSI value from 2's complement to decimal value.
    INT16 rssiDec = (INT16) rssiComp;

    // Convert to absolute value (RSSI value from radio has resolution of
    // 0.5 dBm) and subtract the radio's appropriate RSSI offset.
    if(rssiDec < 128){
        return (rssiDec/2) - perRssiOffset;
    }
    else{
        return ((rssiDec - 256)/2) - perRssiOffset;
    }
}




/*==== PRIVATE FUNCTIONS =====================================================*/

/******************************************************************************
* @fn  pktCheckId
*
* @brief
*      Check the NETWORK_ID_KEY of the received packet to ensure we are the
*      intended recipient.
*
* Parameters:
*
* @param  void
*
* @return BOOL
*         TRUE: The NETWORK_ID_KEY was correct
*         FALSE: The NETWORK_ID_KEY was wrong
*
******************************************************************************/
static BOOL pktCheckId(void)
{
    // The NETWORK_ID_KEY is sent as the second and third byte in the packet
    if ((radioPktBuffer[1]==(BYTE)(NETWORK_ID_KEY>>8)) &&
        (radioPktBuffer[2]==(BYTE)NETWORK_ID_KEY)) {

        // Reset the NETWORK_ID_KEY from packet buffer to ensure that the next packet will
        // have to update the buffer with it again (to rule out false positives).
        radioPktBuffer[1] = 0x00;
        radioPktBuffer[2] = 0x00;
        return TRUE;
    }
    return FALSE;
}

/******************************************************************************
* @fn  pktCheckCrc
*
* @brief
*      Check the if the CRC is correct for the received packet
*
* Parameters:
*
* @param  void
*
* @return BOOL
*          TRUE: The CRC was correct
*          FALSE: The CRC was wrong
*
******************************************************************************/
static BOOL pktCheckCrc(void)
{
    // Check if CRC_OK bit (bit 7) in the second status byte received is set
    if(radioPktBuffer[PACKET_LENGTH + 2] & 0x80){
        radioPktBuffer[PACKET_LENGTH + 2] = 0x00;   // Clear status byte in buffer
        return TRUE;
    }
    else {
        return FALSE;
    }
}


/*==== END OF FILE ==========================================================*/
