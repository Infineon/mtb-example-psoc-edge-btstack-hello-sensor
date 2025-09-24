/*******************************************************************************
* File Name: app_hw_device.c
*
* Description: This file contains functions for initialization and usage of
*              device peripheral GPIO for LED and button. It also
*              contains FreeRTOS timer implementations.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "inttypes.h"
#include <FreeRTOS.h>
#include <task.h>
#include "timers.h"
#include "app_bt_bonding.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#include "retarget_io_init.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
* Macro Definitions
*******************************************************************************/

/* Interrupt priority for GPIO connected to button */
#define GPIO_INTERRUPT_PRIORITY        (4U)

/* Stack size for Hello Sensor BTN task */
#define BTN_TASK_STACK_SIZE            (512U)

/* Task Priority of Hello Sensor BTN Task */
#define BTN_TASK_PRIORITY              ((configMAX_PRIORITIES) - 1U)
#define KEY_RESET_VALUE                (0U)
#define NO_BLE_CONNECTION              (0U)
#define INVALID_NUM                    (0U)
#define BLINK_INIT_VAL                 (1U)
#define BLINK_DELAY_MS                 (500U)
#define INTERRUPT_MASKED               (1U)
#define BUTTON_SCAN_DELAY_MSEC         (10U)
#define SHORT_PRESS_DELAY_MSEC         (10U)
#define LONG_PRESS_DELAY_MSEC          (3000U)
#define DEBOUNCE_TIME_MS               (300U)


/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/* This is a one shot timer that is used to blink the LED the number of times 
   written in the led_blink characteristic in GATT database  by the client */
TimerHandle_t timer_led_blink;

/* This timer is used to toggle LED to indicate the 10 sec button press
 * duration */
TimerHandle_t ms_timer_led_indicate;

/* ms_timer_btn is a periodic timer that ticks every millisecond.
 * This timer is used to measure the duration of button press events */
TimerHandle_t ms_timer_btn;

/* Variables to hold the timer count values */
uint8_t led_blink_count;

/* Handle of the button task */
TaskHandle_t button_handle;

/* To check if the device has entered pairing mode to connect and bond with a
 * new device */
bool pairing_mode = FALSE;

/*Flag to set based on USER BUTTON press */
bool gpio_intr_flag1 = FALSE;
bool gpio_intr_flag2 = FALSE;

/* Define the semaphore handle */
static SemaphoreHandle_t button_semaphore;

/* Variables for button debouncing */
volatile bool button_debouncing = false;
volatile uint32_t button_debounce_timestamp = 0;


/*******************************************************************************
* Function Name: app_bt_led_blink
********************************************************************************
* Summary:
*  The function blinks the LED at specified rate.
*
* Parameters:
*  uint8_t num_of_blinks: number written by peer to blink the LED
*
* Return:
*  None
*
*******************************************************************************/
void app_bt_led_blink(uint8_t num_of_blinks)
{
    for (int i=BLINK_INIT_VAL;i<=num_of_blinks;i++)
    {
         Cy_GPIO_Set(CYBSP_USER_LED_PORT,CYBSP_USER_LED_NUM);
         Cy_SysLib_Delay(BLINK_DELAY_MS);
         Cy_GPIO_Clr(CYBSP_USER_LED_PORT,CYBSP_USER_LED_NUM);
         Cy_SysLib_Delay(BLINK_DELAY_MS);
    }
}

/*******************************************************************************
* Function Name: user_btn_interrupt_config
********************************************************************************
* Summary:
*  This function initializes a pin as input that triggers interrupt on
*  falling edges.
*******************************************************************************/
void user_btn_interrupt_config(void)
{
    cy_rslt_t result=0;

    /* Interrupt config structure */
    cy_stc_sysint_t intrCfg =
    {
        .intrSrc =CYBSP_USER_BTN1_IRQ,
        .intrPriority = GPIO_INTERRUPT_PRIORITY
    };

    /* CYBSP_USER_BTN1 (SW2) and CYBSP_USER_BTN2 (SW4) share the same port and
     * hence they share the same NVIC IRQ line. Since both are configured in the
     * BSP via the Device Configurator, the interrupt flags for both the buttons
     * are set right after they get initialized through the call to cybsp_init().
     * The flags must be cleared before initializing the interrupt, otherwise
     * the interrupt line will be constantly asserted. 
     */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);
    NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

    /* Initialize the interrupt and register interrupt callback */
    cy_en_sysint_status_t btn_interrupt_init_status = Cy_SysInt_Init
            (&intrCfg, &app_bt_gpio_interrupt_handler);
    if(CY_SYSINT_SUCCESS != btn_interrupt_init_status)
    {
        handle_app_error();
    }

    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Create Button Task */
    result = xTaskCreate(button_task,"Button task",BTN_TASK_STACK_SIZE,NULL,
                 BTN_TASK_PRIORITY,&button_handle);
    if( pdPASS != result )
    {
        printf("Unable to create Button task\n");
    }
}

/*******************************************************************************
* Function Name: app_bt_gpio_interrupt_handler
********************************************************************************
* Summary:
*  GPIO interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  None
*
*******************************************************************************/
void app_bt_gpio_interrupt_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Check if the interrupt is from USER BUTTON 1 */
    if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN_PORT,
            CYBSP_USER_BTN_PIN))
    {
        /* Clear the interrupt and pending IRQ from NVIC*/
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN1_IRQ);

        /* Set the interrupt flag */
        gpio_intr_flag1 = true;
    }

    /* Check if the interrupt is from USER BUTTON 2 */
    if(INTERRUPT_MASKED == Cy_GPIO_GetInterruptStatusMasked(CYBSP_USER_BTN2_PORT,
            CYBSP_USER_BTN2_PIN))
    {
        /* Clear the interrupt and pending IRQ from NVIC*/
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
        NVIC_ClearPendingIRQ(CYBSP_USER_BTN2_IRQ);

        if (!button_debouncing)
        {
            /* Set the debouncing flag */
            button_debouncing = true;

            /* Record the current timestamp */
            button_debounce_timestamp = (uint32_t) (xTaskGetTickCount()
                * portTICK_PERIOD_MS);
        }

        if (button_debouncing && (((xTaskGetTickCount() * portTICK_PERIOD_MS)) -
                button_debounce_timestamp <= DEBOUNCE_TIME_MS *
                portTICK_PERIOD_MS))
        {
              button_debouncing = false;
              /* Set the interrupt flag */
              gpio_intr_flag2 = true;
        }
    }

    /* Give Semaphore from ISR to the button task */
    xSemaphoreGiveFromISR(button_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: button_task
********************************************************************************
* Summary:
*  This is a FreeRTOS task that handles the button press events.
*******************************************************************************/
void button_task(void *arg)
{
    wiced_result_t result;
    TickType_t last_button_press_time = 0;
    TickType_t long_press_start_time = 0;
    BaseType_t long_press_detected = pdFALSE;

    /* Create the semaphore */
    button_semaphore = xSemaphoreCreateBinary();
    for (;;)
    {
        /* Wait for the semaphore to be taken from the ISR */
        if (xSemaphoreTake(button_semaphore, portMAX_DELAY) == pdTRUE)
        {
            if(true == gpio_intr_flag2)
            {
                gpio_intr_flag2 = false;

                if (NO_BLE_CONNECTION == hello_sensor_state.conn_id)
                {
                    printf(" USER BUTTON 2 press is detected! \n");
                    printf("Entering Pairing Mode: Connect, Pair and Bond with a "
                       "new peer device...\n");

                    pairing_mode = TRUE;
                    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                       BLE_ADDR_PUBLIC,NULL);
                    result =
                       wiced_bt_ble_address_resolution_list_clear_and_disable();
                    if(WICED_BT_SUCCESS == result)
                    {
                        printf("Address resolution list cleared successfully "
                               "\n");
                    }
                    else
                    {
                        printf("Failed to clear address resolution list \n");
                    }

                    printf("Starting Undirected High Advertisement\n");
                    result = wiced_bt_start_advertisements
                                               (BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                BLE_ADDR_PUBLIC,NULL);
                    if (result != WICED_BT_SUCCESS)
                    {
                        printf("Start advertisement failed: %d\n", result);
                    }
                }
                else
                {
                    printf("Device already connected\n\n");
                }
            }
            else if(true == gpio_intr_flag1)
            {
                gpio_intr_flag1 = false;
                last_button_press_time = xTaskGetTickCount();
                long_press_start_time = xTaskGetTickCount();
                long_press_detected = pdFALSE;

                while (Cy_GPIO_Read(CYBSP_SW2_PORT, CYBSP_SW2_PIN) ==
                                       CYBSP_BTN_PRESSED)
                {
                    if(!long_press_detected &&
                          ((xTaskGetTickCount() - long_press_start_time) >=
                          pdMS_TO_TICKS(LONG_PRESS_DELAY_MSEC)))
                    {
                        long_press_detected = pdTRUE;
                        printf("USER BUTTON 1 long press is detected!\r\n");

                        /* Reset Kv-store library, this will clear the NVM */
                        cy_rslt_t cy_result = mtb_kvstore_reset(&kvstore_obj);
                        if(CY_RSLT_SUCCESS == cy_result)
                        {
                            printf("Successfully reset kv-store library, "
                                   "please reset the device to generate new "
                                   "keys!\n");
                        }
                        else
                        {
                            printf("failed to reset kv-store libray\n");
                        }

                        /* Clear peer link keys and identity keys structure */
                        memset(&bond_info, KEY_RESET_VALUE, sizeof(bond_info));
                        memset(&identity_keys, KEY_RESET_VALUE,
                               sizeof(identity_keys));
                        handle_app_error();
                    }

                    vTaskDelay(pdMS_TO_TICKS(BUTTON_SCAN_DELAY_MSEC));
                }

                if (!long_press_detected &&
                    ((xTaskGetTickCount() - last_button_press_time) >=
                      pdMS_TO_TICKS(SHORT_PRESS_DELAY_MSEC)))
                {
                    /* Check if button press is short */
                    printf(" USER BUTTON 1 short press is detected! \n");
                    if (NO_BLE_CONNECTION == hello_sensor_state.conn_id)
                    {
                        printf("Starting Undirected High Advertisement\n\n");
                        result = wiced_bt_start_advertisements
                                (BTM_BLE_ADVERT_UNDIRECTED_HIGH,BLE_ADDR_PUBLIC,
                                 NULL);

                        if (result != WICED_BT_SUCCESS)
                        {
                            printf("Start advertisement failed: %d\n", result);
                        }
                    }
                    else
                    {
                        /* Remember how many messages we need to send */
                        hello_sensor_state.num_to_send++;

                        /* Connection up.
                         * Send message if client registered to receive indication
                         * or notification. After we send an indication wait for
                         * the ack before we can send anything else */
                        printf("No. to write: %c\n",app_hello_sensor_notify
                                [app_hello_sensor_notify_len - 1]);
                        printf("flag_indication_sent: %d \n",
                               hello_sensor_state.flag_indication_sent);

                        while ((INVALID_NUM != hello_sensor_state.num_to_send) &&
                              (FALSE == hello_sensor_state.flag_indication_sent))
                        {
                            hello_sensor_state.num_to_send--;
                            app_bt_send_message();
                        }

                        /* Increment the last byte of the hello sensor
                         * notify value */
                        app_bt_gatt_increment_notify_value();
                    }
                }
            }
        }
    }
}


/* [] END OF FILE */