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

// Camera includes
#include "te_vdma.h"
#include "xil_cache.h"
#include "vdma.h"
#include "te_fsbl_hooks.h"
#include "gpio_ctrl.h"
#include <sleep.h>

// Camera defines
extern unsigned char VideoBuffer[1080*1920*4];
unsigned char CompressionBuffer[1080*1920*4];

// Static memory allocation functions
#include "staticrtos.h"

// CAN includes and defines
#include "can_driver.h"

// Image compression defines
#include "SatCamImage.h"
extern size_t bitPosInOutString;

// Application stuff starts here
#define DELAY_1_SECOND		1000UL

/*-----------------------------------------------------------*/
static void vIdleTask( void *pvParameters );
static void vCameraTask( void *pvParameters );
static void vCameraSetup( void *pvParameters );
/*-----------------------------------------------------------*/
SemaphoreHandle_t xCam_TakePicture;
SemaphoreHandle_t xCam_Configured;

/*-----------------------------------------------------------*/


/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xIdleTask;
static TaskHandle_t xCameraTask;
static TaskHandle_t xCameraSetup;

int main( void )
{

	xil_printf( "Booting..\r\n" );

	xil_setup_can();

	csp_buffer_init();

	csp_qfifo_init(); // Seems to make it work ???

	csp_iface_can_init(0x1C1F, 8, 500000);


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

	// Create task to configure the camera
	xTaskCreate(vCameraSetup, "Camera setup", 512, NULL, tskIDLE_PRIORITY+5, &xCameraSetup);

	xTaskCreate(vIdleTask, "Idle task", 200, NULL, tskIDLE_PRIORITY, &xIdleTask);

	// Create task for camera handling
	xTaskCreate(vCameraTask, "Camera task", 512, NULL, tskIDLE_PRIORITY+1, &xCameraTask);

	// Create semaphore for taking an image.
	xCam_TakePicture = xSemaphoreCreateBinary();
	xCam_Configured = xSemaphoreCreateBinary();

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
		cntr++;
		xil_printf("%d..\r\n", cntr);

		if (csp_qfifo_read(&inputQueue) == 0) {

			if (inputQueue.packet->id.dport >= 0 && inputQueue.packet->id.dport <= 6) {
				serviceToDo(inputQueue.packet);
			} else {

				if (inputQueue.packet->data[0] == 0x1) {

					if (inputQueue.packet->data[1] == 0x1) {
						xSemaphoreGive( xCam_TakePicture );
					}

				}

			}

		}
	}
}


// Camera handler task
static void vCameraTask ( void *pvParameters ) {

	for (;;) {

		if ( xSemaphoreTake( xCam_Configured, ( TickType_t ) 1 ) == pdTRUE )
		{
			// Take picture routine...
			if( xCam_TakePicture != NULL )
			{
				/* See if we can obtain the semaphore.  If the semaphore is not
				available wait 1 ticks to see if it becomes free. */
				if( xSemaphoreTake( xCam_TakePicture, ( TickType_t ) 1 ) == pdTRUE )
				{
					/* We were able to obtain the semaphore and can now access the
					shared resource. */
					TickType_t starttime = 0, stoptime = 0, tickstotal = 0;

					starttime = xTaskGetTickCount();
					xil_printf("\r\n ..Starting image process.. \r\n");

					// Tell on CSP we are done.
						uint8_t stuffToSend[2] = {0x1, 0xF};
					cspSender(stuffToSend, 2, 0x1C3F, 0xF, 0xF, 0x1D);


					xil_printf("Copying to compression buffer.. \r\n"); // Serial debug message.
					gpio_toggle(71); 	// Status pin used to signal SW running state.
					Xil_DCacheFlush();	// Flush cache to ensure cached is written to memory.
					memcpy(&CompressionBuffer, VideoBuffer, 1920*1080*4); // Width * Height * (RGBA).


					xil_printf("Starting compression.. \r\n"); // Serial debug message.
					ReadDataToBuffer((char *)CompressionBuffer, MID);
					xil_printf("Calculating DCT.. \r\n"); // Serial debug message.
					DCTToBuffers(MID);
					xil_printf("Calculating quantization.. \r\n"); // Serial debug message.
					QuantBuffers(MID);
					xil_printf("Calculating diff.. \r\n");
					DiffDCBuffers(MID);
					xil_printf("Calculating zigzag.. \r\n");
					ZigzagBuffers(MID);
					xil_printf("Calculating huffman.. \r\n");
					HuffmanEncode(MID);
					xil_printf("Creating output string.. \r\n");

					while(!(bitPosInOutString%8 == 0)) {
						AddToBitString(1, 1, 0);
					}

					// Tell on CSP we are done.
					stuffToSend[0] = 0x1;
					stuffToSend[1] = 0xA;
					cspSender(stuffToSend, 2, 0x1C3F, 0xF, 0xF, 0x1D);

					stoptime = xTaskGetTickCount();

					gpio_toggle(71); // Status pin used to signal SW running state.
					xil_printf("\r\n ..Image process done.. \r\n");
					tickstotal = stoptime - starttime;
					xil_printf("Took %d ticks to execute (%d ms).. \r\n", tickstotal, tickstotal*portTICK_PERIOD_MS);
				}
				else
				{
					/* We could not obtain the semaphore and can therefore not access
					the shared resource safely. */
					xil_printf("\t\tCam idle... \r\n");
				}
			}
			// It's still configured.
			xSemaphoreGive( xCam_Configured );
		} else {
			xil_printf("\r\nCamera is not configured.. \r\n");
		}

		vTaskDelay( pdMS_TO_TICKS(950) );
	}

}

static void vCameraSetup( void *pvParameters )
{
	u32 vdmaRdy = 1;
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_init();

	usleep(100 *1000);
	te_read_IDCODE();

	xil_printf( "\r\n--------------------------------------------------------------------------------\r\n" );
	xil_printf( "FreeRTOS starting...\r\n" );

	xil_printf( "Initializing VDMA...\r\n" );

	vTaskDelay(pdMS_TO_TICKS(500));
	vdmaRdy = EnableVideoTimingController();
	xil_printf( "VTC init: %i \r\n", vdmaRdy);

	initIIC();
	vTaskDelay(pdMS_TO_TICKS(500));

	configRPI();
	vTaskDelay(pdMS_TO_TICKS(500));

	if (vdmaRdy == 0) {
		xil_printf("VDMA initialized.... \r\n");
	}

	vTaskDelay(pdMS_TO_TICKS(2000));

	Xil_Out32(0x43c10040, 0x1);
	xil_printf("Enabled CSI.. \r\n");

	// Give semaphore...
	xSemaphoreGive( xCam_Configured );

	while (1) {
		// Go to sleep...
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
