/*******************************************************************************
* File Name: app_bt_event_handler.h
*
* Description: This file is the public interface of app_bt_event_handler.c
*
* Related Document: See README.md
*
********************************************************************************
* (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#ifndef SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_

/*******************************************************************************
 * Structures
 *******************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;  /* remote peer device address */
    uint32_t  timer_count_s;                /* count of sec since the start
                                             * of seconds timer */
    uint32_t  timer_count_ms;               /* count of ms since the start of
                                             * ms timer */
    uint16_t  conn_id;                      /* connection ID of the peer */
    uint16_t  peer_mtu;                     /* peer MTU */
    uint8_t   flag_indication_sent;         /* to store the state of indication
                                             * confirmation */
    uint8_t   num_to_send;                  /* number of messages to send.
                                             * Incremented on each button
                                             * interrupt */
} hello_sensor_state_t;

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/* Holds the global state of the hello sensor application */
extern hello_sensor_state_t hello_sensor_state;
extern uint8_t bondindex;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);
void app_bt_application_init(void);

#endif /* SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_ */
