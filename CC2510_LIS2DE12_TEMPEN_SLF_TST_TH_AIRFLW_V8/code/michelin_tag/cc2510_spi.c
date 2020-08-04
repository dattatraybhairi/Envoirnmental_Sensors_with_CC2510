#include "cc2510_spi.h"
#include <ioCC2510.h>
#include <stdio.h>
//#include "hal_main.h"
//#include "per_test_main.h"
//uint8 radioPktBuffer_1[radioPktBuffer_1_len];
//uint8 cc2500_rx = 0;

void spi_init()
{
    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by
     * USART0 SPI. Note however, when SPI is in master mode, only MOSI, MISO,
     * and SCK should be configured as peripheral I/O's. If the external
     * slave device requires a slave select signal (SSN), then the master
     * can control the external SSN by using one of its GPIO pin as output.
     */

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
    // To avoid potential I/O conflict with USART1:
    // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

    // Give priority to USART 0 over USART 1 for port 0 pins
    P2DIR = (P2DIR & ~P2DIR_PRIP0) | P2DIR_PRIP0_0;

    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 and pin 3 as GPIO output
    //P0SEL = (P0SEL & ~BIT4) | BIT5 | BIT3 | BIT2;
     P1SEL |= BIT7 | BIT6 | BIT5;    
     P1SEL &= (~(BIT4) | ~(BIT3));
     P1DIR |= (BIT4 | BIT3);
    
    SS = 1;
    SS_BME = 1;
    
    /***************************************************************************
     * Configure SPI
     */

    // Set system clock source to 26 Mhz XSOSC to support maximum transfer speed,
    // ref. [clk]=>[clk_xosc.c]
    /*SLEEP &= ~SLEEP_OSC_PD;
    while( !(SLEEP & SLEEP_XOSC_S) );
    CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
    while (CLKCON & CLKCON_OSC);
    SLEEP |= SLEEP_OSC_PD;
    */

    // Set USART to SPI mode and Master mode
    U1CSR &= ~(U1CSR_MODE | U1CSR_SLAVE);

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U1BAUD = SPI_BAUD_M;
    U1GCR = (U1GCR & ~(U1GCR_BAUD_E | U1GCR_CPHA  | U1GCR_CPOL) | (SPI_BAUD_E| U1GCR_ORDER));
    //U1GCR = 0x2F;
    
}
void write_reg(uint8 write_addr, uint8 value)
{
  uint8 temp_data;

  SS = 0;
  //while(MISO);
  temp_data = spi_write(write_addr);
  temp_data = spi_write(value);
  SS = 1;
}

uint8 read_reg(uint8 read_addr)
{
  uint8 temp_data;
  read_addr = read_addr | 0x80;
  SS = 0;
  //while(MISO);
  temp_data = spi_write(read_addr);
  temp_data = spi_write(0x00);
  SS = 1;
  return(temp_data);
}

uint8 spi_write(uint8 data)
{
  U1DBUF = data;
  while(!(U1CSR & U1CSR_TX_BYTE));//{asm("NOP");}
  U1CSR &= ~U1CSR_TX_BYTE;
  return(U1DBUF);
}

/*uint8 send_strobe(uint8 stobe_addr)
{
  uint8 temp_data;
  SS = 0;
  while(MISO);
  temp_data = spi_write(stobe_addr);
  SS = 1;
  return(temp_data);
}

 void write_all_reg()
 {
  write_reg(REG_IOCFG2,VAL_IOCFG2);
  write_reg(REG_IOCFG0,VAL_IOCFG0);
  write_reg(REG_IOCFG1,VAL_IOCFG1);

  write_reg(REG_FIFOTHR, VAL_FIFOTHR);
  write_reg(REG_SYNC1,VAL_SYNC1);
  write_reg(REG_SYNC0,VAL_SYNC0);
  write_reg(REG_PKTLEN,VAL_PKTLEN);
  write_reg(REG_PKTCTRL1,VAL_PKTCTRL1);
  write_reg(REG_PKTCTRL0, VAL_PKTCTRL0);
  write_reg(REG_ADDR,VAL_ADDR);
  write_reg(REG_CHANNR,VAL_CHANNR);
  write_reg(REG_FSCTRL1,VAL_FSCTRL1);
  write_reg(REG_FSCTRL0,VAL_FSCTRL0);
  write_reg(REG_FREQ2,VAL_FREQ2);
  write_reg(REG_FREQ1,VAL_FREQ1);
  write_reg(REG_FREQ0,VAL_FREQ0);
  write_reg(REG_MDMCFG4,VAL_MDMCFG4);
  write_reg(REG_MDMCFG3,VAL_MDMCFG3);
  write_reg(REG_MDMCFG2,VAL_MDMCFG2);
  write_reg(REG_MDMCFG1,VAL_MDMCFG1);
  write_reg(REG_MDMCFG0,VAL_MDMCFG0);
  write_reg(REG_DEVIATN,VAL_DEVIATN);
  write_reg(REG_MCSM2,VAL_MCSM2);
  write_reg(REG_MCSM1,VAL_MCSM1);
  write_reg(REG_MCSM0,VAL_MCSM0);
  write_reg(REG_FOCCFG,VAL_FOCCFG);

  write_reg(REG_BSCFG,VAL_BSCFG);
  write_reg(REG_AGCCTRL2,VAL_AGCCTRL2);
  write_reg(REG_AGCCTRL1,VAL_AGCCTRL1);
  write_reg(REG_AGCCTRL0,VAL_AGCCTRL0);
  write_reg(REG_WOREVT1,VAL_WOREVT1);
  write_reg(REG_WOREVT0,VAL_WOREVT0);
  write_reg(REG_WORCTRL,VAL_WORCTRL);
  write_reg(REG_FREND1,VAL_FREND1);
  write_reg(REG_FREND0,VAL_FREND0);
  write_reg(REG_FSCAL3,VAL_FSCAL3);
  write_reg(REG_FSCAL2,VAL_FSCAL2);
  write_reg(REG_FSCAL1,VAL_FSCAL1);
  write_reg(REG_FSCAL0,VAL_FSCAL0);
  write_reg(REG_RCCTRL1,VAL_RCCTRL1);
  write_reg(REG_RCCTRL0,VAL_RCCTRL0);
  write_reg(REG_FSTEST,VAL_FSTEST);
  write_reg(REG_PTEST,VAL_PTEST);
  write_reg(REG_AGCTEST,VAL_AGCTEST);
  write_reg(REG_TEST2,VAL_TEST2);
  write_reg(REG_TEST1,VAL_TEST1);
  write_reg(REG_TEST0,VAL_TEST0);
  write_reg(CC2500_PATABLE,VAL_PATABLE);
 }

 void read_all_reg()
 {
   printf("\n%x",read_reg(REG_IOCFG2));
   delay(10);
  /*printf("\n%x",read_reg(REG_IOCFG1));
   delay(10);
  printf("\n%x",read_reg(REG_IOCFG0));
   delay(10);
  printf("\n%x",read_reg(REG_FIFOTHR));
   delay(10);
  printf("\n%x",read_reg(REG_SYNC1));
   delay(10);
  printf("\n%x",read_reg(REG_SYNC0));
   delay(10);
  printf("\n%x",read_reg(REG_PKTLEN));
   delay(10);
  printf("\n%x",read_reg(REG_PKTCTRL1));
   delay(10);
  printf("\n%x",read_reg(REG_PKTCTRL0));
   delay(10);
  printf("\n%x",read_reg(REG_ADDR));
   delay(10);
  printf("\n%x",read_reg(REG_CHANNR));
   delay(10);
  printf("\n%x",read_reg(REG_FSCTRL1));
   delay(10);
  printf("\n%x",read_reg(REG_FSCTRL0));
   delay(10);
  printf("\n%x",read_reg(REG_FREQ2));
   delay(10);
  printf("\n%x",read_reg(REG_FREQ1));
   delay(10);
  printf("\n%x",read_reg(REG_FREQ0));
   delay(10);
  printf("\n%x",read_reg(REG_MDMCFG4));
   delay(10);
  printf("\n%x",read_reg(REG_MDMCFG3));
   delay(10);
  printf("\n%x",read_reg(REG_MDMCFG2));
   delay(10);
  printf("\n%x",read_reg(REG_MDMCFG1));
   delay(10);
  printf("\n%x",read_reg(REG_MDMCFG0));
   delay(10);
  printf("\n%x",read_reg(REG_DEVIATN));
   delay(10);
  printf("\n%x",read_reg(REG_MCSM2));
   delay(10);
  printf("\n%x",read_reg(REG_MCSM1));
   delay(10);
  printf("\n%x",read_reg(REG_MCSM0));
   delay(10);
  printf("\n%x",read_reg(REG_FOCCFG));
   delay(10);

  printf("\n%x",read_reg(REG_BSCFG));
   delay(10);
  printf("\n%x",read_reg(REG_AGCCTRL2));
   delay(10);
  printf("\n%x",read_reg(REG_AGCCTRL1));
   delay(10);
  printf("\n%x",read_reg(REG_AGCCTRL0));
   delay(10);
  printf("\n%x",read_reg(REG_WOREVT1));
   delay(10);
  printf("\n%x",read_reg(REG_WOREVT0));
   delay(10);
  printf("\n%x",read_reg(REG_WORCTRL));
   delay(10);
  printf("\n%x",read_reg(REG_FREND1));
   delay(10);
  printf("\n%x",read_reg(REG_FREND0));
   delay(10);
  printf("\n%x",read_reg(REG_FSCAL3));
   delay(10);
  printf("\n%x",read_reg(REG_FSCAL2));
   delay(10);
  printf("\n%x",read_reg(REG_FSCAL1));
   delay(10);
  printf("\n%x",read_reg(REG_FSCAL0));
   delay(10);
  printf("\n%x",read_reg(REG_RCCTRL1));
   delay(10);
  printf("\n%x",read_reg(REG_RCCTRL0));
   delay(10);
  printf("\n%x",read_reg(REG_FSTEST));
   delay(10);
  printf("\n%x",read_reg(REG_PTEST));
   delay(10);
  printf("\n%x",read_reg(REG_AGCTEST));
   delay(10);
  printf("\n%x",read_reg(REG_TEST2));
   delay(10);
  printf("\n%x",read_reg(REG_TEST1));
   delay(10);
  printf("\n%x",read_reg(REG_TEST0));
   delay(10);
 
  printf("\n%x",read_reg(REG_PARTNUM));
   delay(1000);
  printf("\n%x",read_reg(REG_VERSION));
   delay(1000);
  printf("\n%x",read_reg(REG_FREQEST));
   delay(1000);
  printf("\n%x",read_reg(REG_LQI));
   delay(1000);
  printf("\n%x",read_reg(REG_RSSI));
   delay(1000);
  printf("\n%x",read_reg(REG_MARCSTATE));
   delay(1000);
  printf("\n%x",read_reg(REG_WORTIME1));
   delay(1000);
  printf("\n%x",read_reg(REG_WORTIME0));
   delay(1000);
  printf("\n%x",read_reg(REG_PKTSTATUS));
   delay(1000);
  printf("\n%x",read_reg(REG_VCO_VC_DAC));
   delay(1000);
  printf("\n%x",read_reg(REG_TXBYTES));
   delay(1000);
  printf("\n%x",read_reg(REG_RXBYTES));
   delay(1000);
  printf("\n%x",read_reg(REG_RCCTRL1_STATUS));
   delay(1000);
  printf("\n%x",read_reg(REG_RCCTRL0_STATUS));
   delay(1000);
   */
   
//}

/*void delay(uint16 delay_us)
{
  for(uint16 count = 0 ; count < delay_us ; count ++)
  {
    for(uint16 ms_count = 0 ; ms_count < 1000 ; ms_count ++)
    {
       for(uint16 us_count = 0 ; us_count < 26 ; us_count ++)
       {  
         asm("NOP");
       }
    } 
  }
}*/

/*void send_rx_strobe()
{
  for(uint8 m = 0; m < radioPktBuffer_1_len ; m++)
  radioPktBuffer_1[m] = 0 ;
  
  send_strobe(CC2500_SIDLE);
  send_strobe(CC2500_SFRX);
  send_strobe(CC2500_SRX);
}
*/