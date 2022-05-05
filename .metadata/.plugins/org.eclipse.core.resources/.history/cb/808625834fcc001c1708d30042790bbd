/* CSP includes*/
#include <csp/csp.h>
#include <csp/include/csp/csp_debug.h>
#include <csp/include/csp/interfaces/csp_if_can.h>

/* This function refferences the XIlinx boards setup for CAN0*/
int xil_setup_can(void);

/* The handler during recieve handling ISR based*/
static void RecvHandler(void *CallBackRef);

/* Handler after sending ISR based, this counts up the succesfull tx frames in the interface.*/
static void SendHandler(void *CallBackRef);

/* Function to send a single frame of four bytes with a long frame.*/
void xilSendLongCFPFrame(uint32_t Destination, unsigned int  DPort, unsigned int SPort, unsigned int Flags, unsigned int *CSPmsg);

/* Function that intializes the csp interface part of the CAN driver.
 * */
csp_driver_can_init(int addr, int netmask, uint32_t bitrate);
