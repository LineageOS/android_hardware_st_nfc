#include <sys/ioctl.h>

#define ST21NFC_MAGIC 0xEA
#define ST21NFC_CLK_STATE _IOR(ST21NFC_MAGIC, 0x13, unsigned int)

extern int fidI2c;