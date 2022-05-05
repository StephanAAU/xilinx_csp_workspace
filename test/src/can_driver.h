/* CSP includes*/
#include <csp/csp.h>
#include <csp/include/csp/csp_debug.h>
#include <csp/include/csp/interfaces/csp_if_can.h>

/* This function refferences the XIlinx boards setup for CAN0*/
int xil_setup_can(void);

/* Function to send a single frame of four bytes with a long frame.*/
void xilSendLongCFPFrame(uint32_t Destination, unsigned int  DPort, unsigned int SPort, unsigned int Flags, unsigned int *CSPmsg);

/* Function that intializes the csp interface part of the CAN driver.
 * */
void csp_iface_can_init(int addr, int netmask, uint32_t bitrate);

int cspSender(uint8_t* dataToSend, uint8_t dataLength, uint16_t destination, uint8_t sourceP, uint8_t destP, uint8_t cspFlag);
