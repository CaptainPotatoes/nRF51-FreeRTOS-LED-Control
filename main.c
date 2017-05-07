/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

//#include "nrf_delay.h"

/** LED PIN DEFINITIONS: **/
#define CUSTOM_LED_1 12
#define CUSTOM_LED_2 14
#define CUSTOM_LED_3 15
#define CUSTOM_LED_4 16
//LED0
#define TASK_DELAY        50    /**< Task delay - 10 Hz */
//LED1
#define TASK_DELAY_1			40		//1/2*40ms = 12.5Hz
//LED2
#define TASK_DELAY_2			33 		//1/66ms = 15.15Hz
//LED3
#define TASK_DELAY_3			30		//1/60ms = 16.66Hz
/**< Timer period. LED1 timer will expire after 1000 ms */

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void vLed0Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(CUSTOM_LED_1);
        vTaskDelay(TASK_DELAY); // Delay a task for a given number of ticks

        // Tasks must be implemented to never return...
    }
}

static void vLed1Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(CUSTOM_LED_2);
        vTaskDelay(TASK_DELAY_1); // Delay a task for a given number of ticks
        // Tasks must be implemented to never return...
    }
}/**/

static void vLed2Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(CUSTOM_LED_3);
        vTaskDelay(TASK_DELAY_2); // Delay a task for a given number of ticks
        // Tasks must be implemented to never return...
    }
}

static void vLed3Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(CUSTOM_LED_4);
        vTaskDelay(TASK_DELAY_3); // Delay a task for a given number of ticks
        // Tasks must be implemented to never return...
    }
}

/*static void vLed4Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(CUSTOM_LED_2);
        vTaskDelay(TASK_DELAY_4); // Delay a task for a given number of ticks
        // Tasks must be implemented to never return...
    }
}*/


/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.

static void vLed1Callback (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    nrf_gpio_pin_toggle(CUSTOM_LED_2);
} */


int main(void)
{
    TaskHandle_t  xLed0Handle;       /**< Reference to LED0 toggling FreeRTOS task. */
		TaskHandle_t  xLed1Handle;	/**< Reference to LED1 toggling FreeRTOS timer. */
		TaskHandle_t  xLed2Handle;
		TaskHandle_t  xLed3Handle;  	
	
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // Configure LED-pins as outputs
    nrf_gpio_cfg_output(CUSTOM_LED_1);
    nrf_gpio_cfg_output(CUSTOM_LED_2);
    nrf_gpio_cfg_output(CUSTOM_LED_3);
    nrf_gpio_cfg_output(CUSTOM_LED_4);
    nrf_gpio_pin_set(CUSTOM_LED_1);
    nrf_gpio_pin_set(CUSTOM_LED_2);
    nrf_gpio_pin_set(CUSTOM_LED_3);
    nrf_gpio_pin_set(CUSTOM_LED_4);

    UNUSED_VARIABLE(xTaskCreate( vLed0Function, "L0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed0Handle ));    // LED0 task creation
	  UNUSED_VARIABLE(xTaskCreate( vLed1Function, "L1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed1Handle ));		
		UNUSED_VARIABLE(xTaskCreate( vLed2Function, "L2", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed2Handle ));
		UNUSED_VARIABLE(xTaskCreate( vLed3Function, "L3", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed3Handle ));
		
		
		//xLed1Handle = xTimerCreate( "L1", TIMER_PERIOD, pdTRUE, NULL, vLed1Callback );                                 // LED1 timer creation
    //UNUSED_VARIABLE(xTimerStart( xLed1Handle, 0 ));                                                                // LED1 timer start
		
		//xLed1Handle = xTimerCreate( "L2", TASK_DELAY_2, pdTRUE, NULL, vLed2Callback );                                 // LED1 timer creation
    //UNUSED_VARIABLE(xTimerStart( xLed2Handle, 0 ));                                                                // LED1 timer start
		
		
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        // FreeRTOS should not be here...
    }
}

/* Used in debug mode for assertions */
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
  while(1)
  {
    /* Loop forever */
  }
}

/**
 *@}
 **/
