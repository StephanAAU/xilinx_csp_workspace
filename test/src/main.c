/*
	Test implementation of CAN and CSP.
*/

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
static void vIdleTask( TimerHandle_t pxTimer );
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

	csp_qfifo_init(); // Seems to make it work ???

	csp_iface_can_init(0x1c3f, 0, 500000);


	//xilSendLongCFPFrame(0x1C3B, 0x0F, 0x0F, 0x1D, &Test);

	uint8_t stuffToSend[12] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88, 0x99, 0xAA, 0xBB, 0xCC};

	cspSender(stuffToSend, 12, 0x1c1f, 0xf, 0xf, 0x1d);

	/*
	if (Status != XST_SUCCESS) {
		xil_printf("CAN Interrupt Example Test Failed..\r\n");
		return XST_FAILURE;
	}

	/* Intialize the interface of the can device*/
	/*
	csp_driver_can_init(1, 5, 500000);
	*/
	xIdleTaskCreation = xTaskCreate(vIdleTask, "Idle task", 200, NULL, 5, &xIdleTask);


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
static void vIdleTask( TimerHandle_t pxTimer )
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
		if (csp_qfifo_read(&inputQueue) == 0){
			xil_printf("There was something to read");

		} else{
			xil_printf("qfifo nothing to read");
		}

	}
}

