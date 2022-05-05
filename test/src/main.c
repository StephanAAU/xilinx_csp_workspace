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
static void vIdleTask( void *pvParameters );
static void vCameraTask( void *pvParameters );
/*-----------------------------------------------------------*/
SemaphoreHandle_t xCam_TakePicture;

/*-----------------------------------------------------------*/


/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xIdleTask;
static TaskHandle_t xCameraTask;

int main( void )
{
	int Status;
	xil_printf( "Booting..\r\n" );

	Status = xil_setup_can();

	csp_buffer_init();

	csp_qfifo_init(); // Seems to make it work ???

	csp_iface_can_init(0x1C1F, 0, 500000);


	//xilSendLongCFPFrame(0x1C3B, 0x0F, 0x0F, 0x1D, &Test);

	//uint8_t stuffToSend[12] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88, 0x99, 0xAA, 0xBB, 0xCC};

	//cspSender(stuffToSend, 12, 0x1c1f, 0xf, 0xf, 0x1d);

	/*
	if (Status != XST_SUCCESS) {
		xil_printf("CAN Interrupt Example Test Failed..\r\n");
		return XST_FAILURE;
	}
	*/

	// Intialize the interface of the can device
	/*
	csp_driver_can_init(1, 5, 500000);
	*/
	xTaskCreate(vIdleTask, "Idle task", 200, NULL, tskIDLE_PRIORITY, &xIdleTask);

	// Create task for camera handling
	xTaskCreate(vCameraTask, "Camera task", 512, NULL, tskIDLE_PRIORITY+1, &xCameraTask);

	// Create semaphore for taking an image.
	xCam_TakePicture = xSemaphoreCreateBinary();

	//xSemaphoreGive( xCam_TakePicture );

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
		if (csp_qfifo_read(&inputQueue) == 0){
			xil_printf("There was something to read\r\n");

			if (inputQueue.packet->data[0] == 0x1) {
				if (inputQueue.packet->data[1] == 0x1) {
					xSemaphoreGive( xCam_TakePicture );
				}
			}

		} else{
			xil_printf("qfifo nothing to read\r\n");
		}
/*// Take picture semaphore starts process.
		if ((cntr % 5) == 0) {
			xSemaphoreGive( xCam_TakePicture );
		}
*/
	}
}

static void vCameraTask ( void *pvParameters ) {

	for (;;) {

		// Take picture routine...
	    if( xCam_TakePicture != NULL )
	    {
	        /* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 1 ticks to see if it becomes free. */
	        if( xSemaphoreTake( xCam_TakePicture, ( TickType_t ) 1 ) == pdTRUE )
	        {
	            /* We were able to obtain the semaphore and can now access the
	            shared resource. */

	            xil_printf("\r\n ..Starting image process.. \r\n");

	            vTaskDelay (pdMS_TO_TICKS(4*1000));

	            uint8_t stuffToSend[2] = {0x1, 0xF};

	            cspSender(stuffToSend, 2, 0x1C3F, 0xF, 0xF, 0x1D);

	            xil_printf("\r\n ..Image process done.. \r\n");

	            /* We have finished accessing the shared resource.  Release the
	            semaphore. */
	            //xSemaphoreGive( xCam_TakePicture );
	        }
	        else
	        {
	            /* We could not obtain the semaphore and can therefore not access
	            the shared resource safely. */
	        	xil_printf("\r\n ..Camera task idle... \r\n");
	        }
	    }
		vTaskDelay( pdMS_TO_TICKS(1000) );
	}

}
