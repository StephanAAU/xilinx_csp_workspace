/*
	Test implementation of CAN and CSP.
*/

#include <stdio.h>

// FreeRTOS includes.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

// CSP includes.
#include <csp/src/csp_qfifo.h>

// Xilinx includes.
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"

// Static memory allocation functions
#include "staticrtos.h"

// CAN includes and defines
#include "can_driver.h"

// Application stuff starts here
#define DELAY_1_SECOND		1000UL

/*-----------------------------------------------------------*/
static void vIdleTask( void *pvParameters );
/*-----------------------------------------------------------*/


/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xIdleTask;
TaskHandle_t xIdleTaskCreation;

int main( void )
{
	int Status;
	xil_printf( "Booting..\r\n" );

	Status = xil_setup_can();

	csp_buffer_init();

	csp_qfifo_init();

	csp_conn_init();

	csp_iface_can_init(0x1c3f, 8, 500000);

	//setUpIntialRoutes();

	//xilSendLongCFPFrame(0x1C3B, 0x0F, 0x0F, 0x1D, &Test);

	//uint8_t stuffToSend[12] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88, 0x99, 0xAA, 0xBB, 0xCC};

	//cspSender(stuffToSend, 12, 0x1c1f, 0xf, 0xf, 0x1d);

	/*
	if (Status != XST_SUCCESS) {
		xil_printf("CAN Interrupt Example Test Failed..\r\n");
		return XST_FAILURE;
	}

	/* Intialize the interface of the can device*/
	/*
	csp_driver_can_init(1, 5, 500000);
	*/
	xTaskCreate(vIdleTask, "Idle task", 2000, NULL, 5, &xIdleTask);


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	//for( ;; );

}


/*-----------------------------------------------------------*/
static void vIdleTask( void *pvParameters )
{
	csp_qfifo_t inputQueue;

	const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );
	int cntr = 0;
	for( ;; )
	{
		/* Delay for 1 second. */
		vTaskDelay( x1second );
		xil_printf("%d..\r\n", cntr);
		cntr++;
		uint8_t dataToSend[2] = {0,0};

		uint8_t length;
		uint8_t port;
		xil_printf("Send command to CAMSAT (1) or CSP request to CAMSAT (2)\n \r");

		char choice;

		scanf("%c", &choice);

		if (choice == '1'){
			scanf(" %x %x %x %x", &dataToSend[0], &dataToSend[1], &dataToSend[2], &dataToSend[3]);
			cspSender(dataToSend, 4, 0x1c1f, 0x0f, 0x0f, 0x1d);
		} else if(choice == '2'){
			//xil_printf("Specify port \n \r");

			//scanf("%x", &port);
			//xil_printf("Specify length \n \r");

			//scanf("%d", &length);
			//xil_printf("Write the data to be sent\n \r");
			//scanf(" %x %x %x %x", &dataToSend[0], &dataToSend[1], &dataToSend[2], &dataToSend[3]);
			dataToSend[0] = 0x00;
			dataToSend[1] = 0x03;
			cspSender(dataToSend, 2, 0x1c1f, 0x00, 0x00, 0x00);
		}




	}
}



