#include <ioCCxx10_bitdef.h>
//#include "hal_main.h"
//#include "per_test_main.h"
//#include "cc2500_reg.h"
//#include "cc2500_val.h"

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define SS P1_4
#define SS_BME P1_3
//#define GDO0 P0_7
#define MISO P1_7


// These values will give a baud rate of approx. 1.002930 Mbps for 26 MHz clock
#define SPI_BAUD_M  60          //60
#define SPI_BAUD_E  15          //15

#define radioPktBuffer_1_len 17


typedef unsigned char uint8;
typedef unsigned int uint16;

//extern uint8 radioPktBuffer_1[radioPktBuffer_1_len];
//extern uint8 cc2500_rx;

extern void spi_init();
extern void AXL();
extern void write_reg(uint8 write_addr, uint8 value);
extern uint8 read_reg(uint8 read_addr);
extern uint8 spi_write(uint8 data);
//extern uint8 send_strobe(uint8 stobe_addr);
//extern void write_all_reg();
//extern void read_all_reg();
extern void delay(uint16 delay_us);
//extern void send_rx_strobe();


