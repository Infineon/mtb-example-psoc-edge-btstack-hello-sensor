/*******************************************************************************
* File Name: app_bt_gatt_handler.c
*
* Description: This file contains the task that handles GATT events.
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
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "app_bt_bonding.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
* Macros
*******************************************************************************/
#define CCCD_INDEX_NOTIFY   (0U)
#define NOTIFY_NOT_ENABLED  (0U)
#define STARTING_VAL        ('0')
#define ENDING_VAL          ('9')
#define MINIMUM_LENGTH      (1U)
#define BLINK_INDEX         (0U)
#define ATTR_INDEX          (0U)
#define BLINK_INIT_VAL      (0U)
#define DOUBLE_DIGIT_VAL    (10U)
#define VALID_ATTR_LEN      (2U)
#define ATTR_TABLE_INIT_VAL (0U)
#define ATTR_INDEX_1        (1U)
#define LEFT_SHIFT_VAL      (8U)
#define RESET_VAL           (0U)
#define NOT_FILLED          (0U)
#define NOT_USED            (0U)

/*******************************************************************************
* Variable Definitions
*******************************************************************************/
pfn_free_buffer_t pfn_free;

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: app_bt_gatt_event_callback
********************************************************************************
* Summary:
*  This function handles GATT events from the BT stack.
*
* Parameters:
*  wiced_bt_gatt_evt_t event                : LE GATT event code of one
*                                             byte length
*  wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
*                                             structures
*
* Return:
*  wiced_bt_gatt_status_t                   : See possible status
*                                             codes in wiced_bt_gatt_status_e
*                                             in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                            wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;
    uint16_t error_handle = RESET_VAL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data
            ->attribute_request;

    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = app_bt_gatt_conn_status_cb( &p_event_data
                          ->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = app_bt_gatt_req_cb(p_attr_req, &error_handle);

            if(gatt_status!=WICED_BT_GATT_SUCCESS)
            {
               wiced_bt_gatt_server_send_error_rsp(p_attr_req->conn_id, 
                     p_attr_req->opcode,error_handle,gatt_status);
            }

            break; 

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt =
                    (void *)app_bt_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            pfn_free = \
            (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

        default:
            gatt_status = WICED_BT_GATT_ERROR;
            break;
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: app_bt_gatt_req_cb
********************************************************************************
* Summary:
*  This function handles GATT server events from the BT stack.
*
* Parameters:
*  p_attr_req             : Pointer to LE GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in
*                          wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_req_cb (wiced_bt_gatt_attribute_request_t
        *p_attr_req, uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_SUCCESS;

    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            gatt_status = app_bt_gatt_req_read_handler(p_attr_req->conn_id,
                          p_attr_req->opcode, &p_attr_req->data.read_req,
                          p_attr_req->len_requested, p_error_handle);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             gatt_status =
             app_bt_gatt_req_write_handler(p_attr_req->conn_id, 
                        p_attr_req->opcode, &p_attr_req->data.write_req,
                        p_attr_req->len_requested, p_error_handle);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == gatt_status))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->
                         data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }

             break;

        case GATT_REQ_MTU:
            gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                             p_attr_req->data.remote_mtu,CY_BT_RX_PDU_SIZE);
             break;

        case GATT_HANDLE_VALUE_NOTIF:
             printf("Notification send complete\n");
             break;

        case GATT_REQ_READ_BY_TYPE:
            gatt_status =app_bt_gatt_req_read_by_type_handler
                           (p_attr_req->conn_id,p_attr_req->opcode,
                           &p_attr_req->data.read_by_type,
                            p_attr_req->len_requested, p_error_handle);
            break;

        case GATT_HANDLE_VALUE_CONF:
            printf("Indication Confirmation received \n");
            hello_sensor_state.flag_indication_sent = FALSE;
            break;

        default:
            printf("ERROR: Unhandled GATT Connection Request case: %d\n",
                   p_attr_req->opcode);
            gatt_status = WICED_BT_GATT_ERROR;
            break;
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: app_bt_gatt_conn_status_cb
********************************************************************************
* Summary:
*  This callback function handles connection status changes.
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_conn_status : Pointer to data that
*                                                     has connection details
*
* Return:
*  wiced_bt_gatt_status_t   : See possible status codes in
*                             wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_conn_status_cb
                       (wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t result;

    if (p_conn_status->connected)
    {
        result= app_bt_gatt_connection_up(p_conn_status);
    }
    else
    {
        result= app_bt_gatt_connection_down(p_conn_status);
    }

    return result;
}

/*******************************************************************************
* Function Name: app_bt_gatt_req_read_handler
********************************************************************************
* Summary:
*  This function handles Read Requests received from the client device
*
* Parameters:
*  conn_id               : Connection ID
*  opcode                : LE GATT request type opcode
*  p_read_req            : Pointer to read request containing the handle to read
*  len_req               : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in
*                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler(uint16_t conn_id, 
                             wiced_bt_gatt_opcode_t opcode, 
                             wiced_bt_gatt_read_t *p_read_req, 
                             uint16_t len_req, 
                             uint16_t *p_error_handle)
{
    gatt_db_lookup_table_t  *puAttribute;
    int attr_len_to_copy;
    uint8_t *from;
    int to_send;
    *p_error_handle = p_read_req->handle;
    wiced_bt_gatt_status_t result;
    puAttribute = app_bt_find_by_handle(p_read_req->handle);

    if (NULL != puAttribute)
    {
        attr_len_to_copy = puAttribute->cur_len;
        printf("read_handler: conn_id:%d Handle:%x offset:%d len:%d\n ",
                conn_id, p_read_req->handle, p_read_req->offset,
                attr_len_to_copy);
        if (p_read_req->offset < puAttribute->cur_len)
        {
            to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
            from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
            result = wiced_bt_gatt_server_send_read_handle_rsp(conn_id,opcode,
                                to_send,from,NULL);
        }
        else
        {
            result = WICED_BT_GATT_INVALID_OFFSET;
        }
    }
    else
    {
        result = WICED_BT_GATT_INVALID_HANDLE;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_bt_gatt_req_write_handler
********************************************************************************
* Summary:
*  This function handles Write Requests received from the client device
*
* Parameters:
*  conn_id               : Connection ID
*  opcode                : LE GATT request type opcode
*  p_write_req           : Pointer to LE GATT write request
*  len_req               : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in
*                          wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id, 
                              wiced_bt_gatt_opcode_t opcode, 
                              wiced_bt_gatt_write_req_t *p_write_req, 
                              uint16_t len_req, 
                              uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    *p_error_handle = p_write_req->handle;

    printf("write_handler: conn_id:%d Handle:0x%x offset:%d len:%d\n ",
           conn_id, p_write_req->handle, p_write_req->offset,
           p_write_req->val_len );

    /* Attempt to perform the Write Request */
    gatt_status = app_bt_set_value(p_write_req->handle,p_write_req->p_val,
                                   p_write_req->val_len);
    if(WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }

    return (gatt_status);
}

/*******************************************************************************
* Function Name : app_bt_gatt_req_read_by_type_handler
********************************************************************************
* Summary:
*  Process read-by-type request from peer device
*
* Parameters:
*  uint16_t conn_id                       : Connection ID
*  wiced_bt_gatt_opcode_t opcode          : LE GATT request type opcode
*  wiced_bt_gatt_read_by_type_t p_read_req: Pointer to read request
*                                           containing the handle to read
*  uint16_t len_requested                 : Length of data requested
*
* Return:
*  wiced_bt_gatt_status_t                 : LE GATT status
*******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id, 
                                     wiced_bt_gatt_opcode_t opcode, 
                                     wiced_bt_gatt_read_by_type_t *p_read_req, 
                                     uint16_t len_requested, 
                                     uint16_t *p_error_handle)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = RESET_VAL;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = RESET_VAL;
    int used_len = RESET_VAL;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\n",len_requested);
        status = WICED_BT_GATT_INSUF_RESOURCE;
    }
    else
    {
        /* Read by type returns all attributes of the specified type,
         * between the start and end handles */
        while (WICED_TRUE)
        {
            *p_error_handle = attr_handle;
            last_handle = attr_handle;
            attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                            p_read_req->e_handle,
                                                            &p_read_req->uuid);
            if (RESET_VAL == attr_handle)
                break;

            if ( NULL == (puAttribute = app_bt_find_by_handle(attr_handle)))
            {
                printf("found type but no attribute for %d \n",last_handle);
                status = WICED_BT_GATT_INVALID_HANDLE;
                break;
            }

            int filled =
            wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                    len_requested - used_len, &pair_len, attr_handle,
                    puAttribute->cur_len, puAttribute->p_data);

            if (NOT_FILLED == filled)
            {
                break;
            }

            used_len += filled;

            /* Increment starting handle for next search to one past current */
            attr_handle++;
        }

        if (NOT_USED == used_len)
        {
            printf("attr not found  start_handle: 0x%04x"
                   "end_handle: 0x%04x  Type: 0x%04x\n", p_read_req->s_handle,
                   p_read_req->e_handle,p_read_req->uuid.uu.uuid16);
            status = WICED_BT_GATT_INVALID_HANDLE;
        }
        else
        {
            /* Send the response */
            status = wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,opcode,
                    pair_len, used_len, p_rsp,(void *)app_bt_free_buffer);
        }
    }

    if (p_rsp != NULL)
    {
        app_bt_free_buffer(p_rsp);
    }

    return status;
}

/*******************************************************************************
* Function Name: app_bt_gatt_connection_up
********************************************************************************
* Summary:
*  This function is invoked when connection is established
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_status : Pointer to data that has
*                                                connection details
*
* Return:
*  wiced_bt_gatt_status_t : See possible status codes in
*                           wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    printf("Connected to peer device: ");
    print_bd_address(p_status->bd_addr);
    printf("Connection ID '%d' \n", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr,
           sizeof(wiced_bt_device_address_t));
    app_bt_add_devices_to_address_resolution_db();
    pairing_mode = FALSE;

    /* Update the adv/conn state */
    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_bt_gatt_connection_down
********************************************************************************
* Summary:
*  This function is invoked when connection is disconnected
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_status : Pointer to data that has
*                                                connection details
*
* Return:
*  wiced_bt_gatt_status_t  : See possible status codes in wiced_bt_gatt_status_e
*                            in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    printf("Peer device disconnected: ");
    print_bd_address(p_status->bd_addr);
    printf("conn_id:%d reason:%s\n", p_status->conn_id,
           get_bt_gatt_disconn_reason_name(p_status->reason));

    /* Resetting the device info */
    memset(hello_sensor_state.remote_addr, RESET_VAL, BD_ADDR_LEN);
    hello_sensor_state.conn_id = RESET_VAL;

    /* Start advertisements after disconnection */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                           BLE_ADDR_PUBLIC,
                                           NULL);
    if(WICED_BT_SUCCESS != result)
    {
        printf("Start advertisement failed: %d\n", result);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name : app_bt_find_by_handle
********************************************************************************
* Summary:
*  Find attribute description by handle.
*
* Parameters:
*  uint16_t handle         : handle to look up.
*
* Return:
*  gatt_db_lookup_table_t  : pointer containing handle data.
*
*******************************************************************************/
gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle)
{
    int i;

    for (i = ATTR_TABLE_INIT_VAL; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }

    return NULL;
}

/*******************************************************************************
* Function Name: app_bt_set_value
********************************************************************************
* Summary:
*  This function handles writing to the attribute handle in the GATT
*  database using the data passed from the BT stack. The value to write is
*  stored in a buffer whose starting address is passed as one of the
*  function parameters
*
* Parameters:
*  uint16_t attr_handle : GATT attribute handle
*  uint8_t  p_val       : Pointer to LE GATT write request value
*  uint16_t len         : length of GATT write request
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in
*                          wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
*******************************************************************************/
wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                        uint8_t *p_val,uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    uint8_t *p_attr   = p_val;
    cy_rslt_t rslt;

    /* Check for a matching handle entry */
    for (int i = ATTR_TABLE_INIT_VAL; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                switch (attr_handle)
                {
                   /* By writing into Characteristic Client Configuration
                    * descriptor peer can enable or disable notification
                    * or indication */
                    case HDLD_HELLO_SENSOR_NOTIFY_CLIENT_CHAR_CONFIG:
                        if (VALID_ATTR_LEN != len)
                        {
                            gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
                        }
                        else
                        {
                            app_hello_sensor_notify_client_char_config
                            [CCCD_INDEX_NOTIFY] = p_attr[ATTR_INDEX];
                            peer_cccd_data[bondindex] = p_attr[ATTR_INDEX] |
                                    (p_attr[ATTR_INDEX_1]<< LEFT_SHIFT_VAL);
                            rslt = app_bt_update_cccd(peer_cccd_data[bondindex],
                                    bondindex);
                            if (CY_RSLT_SUCCESS != rslt)
                            {
                                printf("Failed to update CCCD Value in NVM! \n");
                            }
                            else
                            {
                                printf("CCCD value updated in NVM! \n");
                            }
                        }

                        break;

                    case HDLC_HELLO_SENSOR_BLINK_VALUE:
                        if (len != MINIMUM_LENGTH )
                        {
                            gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
                        }
                        else
                        {
                            app_hello_sensor_blink[BLINK_INDEX] =
                                    p_attr[ATTR_INDEX];
                            if (app_hello_sensor_blink[BLINK_INDEX] !=
                                    BLINK_INIT_VAL)
                            {
                                printf("hello_sensor_write_handler:num blinks: "
                                        "%d\n",app_hello_sensor_blink[BLINK_INDEX]);
                                /* Blink the LED only if the peer writes a single
                                 * digit */
                                if(app_hello_sensor_blink[BLINK_INDEX] <
                                        DOUBLE_DIGIT_VAL)
                                {
                                    app_bt_led_blink(app_hello_sensor_blink
                                        [BLINK_INDEX]);
                                }
                            }
                        }

                        break;

                    case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
                        gatt_status = WICED_BT_GATT_SUCCESS;
                        break;
                    
                    default:
                        gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                        break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }

            break;
        }
    }
    if (!isHandleInTable)
    {
        switch(attr_handle)
        {
            default:
                /* The write operation was not performed for the
                 * indicated handle */
                printf("Write Request to Invalid Handle: 0x%x\n", attr_handle);
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: app_bt_send_message
********************************************************************************
* Summary:
*  Check if client has registered for notification/indication and send
*  message if appropriate.
*******************************************************************************/
void app_bt_send_message(void)
{
    wiced_bt_gatt_status_t status;

    printf("hello_sensor_send_message: CCCD:%d\n",
            app_hello_sensor_notify_client_char_config[CCCD_INDEX_NOTIFY]);
    if(app_hello_sensor_notify_client_char_config[CCCD_INDEX_NOTIFY] &
            GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        status = wiced_bt_gatt_server_send_notification(hello_sensor_state.conn_id,
                    HDLC_HELLO_SENSOR_NOTIFY_VALUE,app_hello_sensor_notify_len,
                    app_hello_sensor_notify,NULL);
        printf("Notification Status: %d \n", status);
    }
    else if(!hello_sensor_state.flag_indication_sent)
    {
        hello_sensor_state.flag_indication_sent = TRUE;
        status = wiced_bt_gatt_server_send_indication(hello_sensor_state.conn_id,
                   HDLC_HELLO_SENSOR_NOTIFY_VALUE,app_hello_sensor_notify_len,
                   app_hello_sensor_notify,NULL);
        printf("Indication Status: %d \n", status);
    }

    /* If client has not registered for indication or notification, no action */
    else if(NOTIFY_NOT_ENABLED == app_hello_sensor_notify_client_char_config
            [CCCD_INDEX_NOTIFY])
    {
        return;
    }
}

/*******************************************************************************
* Function Name: app_bt_free_buffer
********************************************************************************
* Summary:
*  This function frees up the memory buffer.
*
* Parameters:
*  uint8_t *p_data: Pointer to the buffer to be free.
*
* Return:
*  None
*******************************************************************************/
void app_bt_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
* Function Name: app_bt_alloc_buffer
********************************************************************************
* Summary:
*  This function allocates a memory buffer.
*
* Parameters:
*  Int len: Length to allocate
*
* Return:
*  None
*******************************************************************************/
void* app_bt_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/*******************************************************************************
* Function Name: app_bt_gatt_increment_notify_value
********************************************************************************
* Summary:
*  Keep number of the button pushes in the last byte of the Hello
*  message.That will guarantee that if client reads it, it will have correct
*  data.
*******************************************************************************/
void app_bt_gatt_increment_notify_value(void)
{
    if(NOTIFY_NOT_ENABLED != app_hello_sensor_notify_client_char_config
       [CCCD_INDEX_NOTIFY])
    {
        /* Getting the last byte */
        int last_byte = app_hello_sensor_notify_len - 1 ;
        char notify_character = app_hello_sensor_notify[last_byte];
        notify_character++;

        if ((notify_character < STARTING_VAL) ||
                (notify_character > ENDING_VAL))
        {
            notify_character = STARTING_VAL;
        }
        
        app_hello_sensor_notify[last_byte] = notify_character;
    }
}


/* END OF FILE [] */