#include "hal_main.h"
#include "per_test_main.h"

#include "cc2510_spi.h"
#include <ioCCxx10_bitdef.h>
#include <stdio.h>

#define LIS2DE12_REG_INT1SRC      0x31
#define LIS2DE12_REG_INT2SRC      0x35
#define LIS2DE12_REG_WHOAMI       0x0F

extern BYTE temp;
extern void AXL();