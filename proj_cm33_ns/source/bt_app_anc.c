/******************************************************************************
* File Name: bt_app_anc.c
*
* Description:
* This is the source code for the LE Alert Notification Client Example
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
* Includes
******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "bt_app_anc.h"
#include "uart_task.h"
#include "retarget_io_init.h"


/*******************************************************************************
* Macros
******************************************************************************/
#define MAX_KEY_SIZE                  (0x10U)
#define ANC_DISCOVERY_STATE_SERVICE   (0U)
#define ANC_DISCOVERY_STATE_ANC       (1U)
#define ANC_START_HANDLE              (1U)
#define ANC_END_HANDLE                (0xFFFF)
#define RESET_VAL                     (0U)

/*******************************************************************************
* Global Variables
******************************************************************************/
bt_app_anc_app_state_t anc_app_state;

/* Context of command that is failed due to gatt insufficient authentication.
 * Need to re trigger the command after anc establish authentication with ans
 *  anc_pending_cmd_context[0] is the opcode saved
 *  anc_pending_cmd_context[1] and anc_pending_cmd_context[2] used in case of
 *  only Control Alerts respectively hold
 *   wiced_bt_anp_alert_control_cmd_id_t and
 *   wiced_bt_anp_alert_category_id_t
 */
uint8_t anc_pending_cmd_context[3] = {0};
/* This is the index for the link keys, cccd and privacy mode of the host we are
 * currently bonded to */
uint8_t  bondindex = 0;

/******************************************************************************
* Function Definitions
*****************************************************************************/

/*******************************************************************************
* Function Name: bt_app_anc_set_advertisement_data
*******************************************************************************
* Summary:
*   Sets the advertisement data for ANC
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen((char *)cy_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data = (uint8_t *)cy_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*******************************************************************************
* Function Name: bt_app_clear_anc_pending_cmd_context
*******************************************************************************
* Summary:
*   Clear the pending context
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_clear_anc_pending_cmd_context( void )
{
    memset(anc_pending_cmd_context, RESET_VAL, sizeof(anc_pending_cmd_context));
}

/*******************************************************************************
* Function Name: bt_app_anc_start_pair
*******************************************************************************
* Summary:
*   Handles the ANC pairing start request
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_start_pair( void )
{
    wiced_result_t rc;

    rc = wiced_bt_dev_sec_bond(anc_app_state.remote_addr,
            anc_app_state.addr_type, BT_TRANSPORT_LE, RESET_VAL, NULL);
    WICED_BT_TRACE("Start bond result: 0x%X\n", rc);
}

/*******************************************************************************
* Function Name: bt_app_alert_type_name
*******************************************************************************
* Summary:
*   Returns Alert name for the given alert type
*
* Parameters:
*  id: Alert Category
*
* Return:
*  Returns alert type name in string format
*
******************************************************************************/
/*  */
static const char *bt_app_alert_type_name(wiced_bt_anp_alert_category_id_t id)
{
    switch (id)
    {
    case ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT:
        return "Simple Alert";
        break;

    case ANP_ALERT_CATEGORY_ID_EMAIL:
        return "Email";
        break;

    case ANP_ALERT_CATEGORY_ID_NEWS:
        return "News";
        break;

    case ANP_ALERT_CATEGORY_ID_CALL:
        return "Call";
        break;

    case ANP_ALERT_CATEGORY_ID_MISSED_CALL:
        return "Missed Call";
        break;

    case ANP_ALERT_CATEGORY_ID_SMS_OR_MMS:
        return "SMS/MMS";
        break;

    case ANP_ALERT_CATEGORY_ID_VOICE_MAIL:
        return "Voice Mail";
        break;

    case ANP_ALERT_CATEGORY_ID_SCHEDULE_ALERT:
        return "Scheduled Alert";
        break;

    case ANP_ALERT_CATEGORY_ID_HIGH_PRI_ALERT:
        return "High Priority Alert";
        break;

    case ANP_ALERT_CATEGORY_ID_INSTANT_MESSAGE:
        return "Instant Message";
        break;

    default:
        return "Unknown Alert Type";
        break;
    }
}

/*******************************************************************************
* Function Name: bt_app_anc_callback
*******************************************************************************
* Summary:
*   This is a Callback from the ANC Profile layer
*
* Parameters:
*   event: ANC callback event from profile
*   p_data: Pointer to the result of the operation initiated by ANC
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_callback(wiced_bt_anc_event_t event, 
        wiced_bt_anc_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    if (NULL == p_data)
    {
        WICED_BT_TRACE("GATT Callback Event Data is pointing to NULL \n");
        return;
    }
    switch (event)
    {
    case WICED_BT_ANC_DISCOVER_RESULT:
        WICED_BT_TRACE("ANC discover result: 0x%X \n",
                p_data->discovery_result.status);
        result = p_data->discovery_result.status;
        break;

    case WICED_BT_ANC_READ_SUPPORTED_NEW_ALERTS_RESULT:
        WICED_BT_TRACE("ANC read supported new alerts result: 0x%X\n",
                p_data->supported_new_alerts_result.status);
        printf("Supported New Alerts on ANS: 0x%X\n",
                p_data->supported_new_alerts_result.supported_alerts);
        result = p_data->supported_new_alerts_result.status;
        break;

    case WICED_BT_ANC_READ_SUPPORTED_UNREAD_ALERTS_RESULT:
        WICED_BT_TRACE("ANC read supported unread alerts result: 0x%X\n",
                p_data->supported_unread_alerts_result.status);
        printf("Supported Unread Alerts on ANS: 0x%X\n",
                p_data->supported_unread_alerts_result.supported_alerts);
        result = p_data->supported_unread_alerts_result.status;
        break;

    case WICED_BT_ANC_CONTROL_ALERTS_RESULT:
        result = p_data->control_alerts_result.status;
        WICED_BT_TRACE("ANC control alerts result: 0x%X\n",
                p_data->control_alerts_result.status);
        break;

    case WICED_BT_ANC_ENABLE_NEW_ALERTS_RESULT:
        WICED_BT_TRACE("ANC enable new alerts result: 0x%X\n",
                p_data->enable_disable_alerts_result.status);
        result = p_data->enable_disable_alerts_result.status;
        if(!result)
        {
            printf("ANC enabled New Alerts\n");
        }
        else
        {
            printf("ANC couldn't enable New Alerts\n");
        }
        break;

    case WICED_BT_ANC_DISABLE_NEW_ALERTS_RESULT:
        WICED_BT_TRACE("ANC disable new alerts result: 0x%X\n",
                p_data->enable_disable_alerts_result.status);
        result = p_data->enable_disable_alerts_result.status;
        if(!result)
        {
            printf("ANC disabled New Alerts\n");
        }
        else
        {
            printf("ANC couldn't disable New Alerts\n");
        }
        break;

    case WICED_BT_ANC_ENABLE_UNREAD_ALERTS_RESULT:
        WICED_BT_TRACE("ANC enable unread alerts result: 0x%X\n",
                p_data->enable_disable_alerts_result.status);
        result = p_data->enable_disable_alerts_result.status;
        if(!result)
        {
            printf("ANC enabled Unread Alerts\n");
        }
        else
        {
            printf("ANC couldn't enable Unread Alerts\n");
        }
        break;

    case WICED_BT_ANC_DISABLE_UNREAD_ALERTS_RESULT:
        WICED_BT_TRACE("ANC disable unread alerts result: 0x%X\n",
                p_data->enable_disable_alerts_result.status);
        result = p_data->enable_disable_alerts_result.status;
        if(!result)
        {
            printf("ANC disabled Unread Alerts\n");
        }
        else
        {
            printf("ANC couldn't disable Unread Alerts\n");
        }
        break;

    case WICED_BT_ANC_EVENT_NEW_ALERT_NOTIFICATION:
        printf("\nNew Alert type:%s Count:%u Last Alert Data:%s\n",
                bt_app_alert_type_name(p_data->new_alert_notification.new_alert_type),
                p_data->new_alert_notification.new_alert_count,
                p_data->new_alert_notification.p_last_alert_data);
        break;

    case WICED_BT_ANC_EVENT_UNREAD_ALERT_NOTIFICATION:
        printf("\nUnread Alert type: %s Count: %u \n",
                bt_app_alert_type_name(p_data->unread_alert_notification.unread_alert_type),
                p_data->unread_alert_notification.unread_count);
        break;

    default:
        break;
    }
    if (WICED_BT_GATT_INSUF_AUTHENTICATION == result)
    {
        bt_app_anc_start_pair();
    }
    else
    {
        /* Pending command no more valid other 
           than authentication failure cases */
        bt_app_clear_anc_pending_cmd_context();
    }
}


/*******************************************************************************
* Function Name: bt_app_anc_connection_up
********************************************************************************
* Summary:
*   This function will be called when a connection is established
*
* Parameters:
*   p_conn_status  : Current status of the Connection

* Return:
*  None
*
*******************************************************************************/
static void bt_app_anc_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status;

    if (TRUE == p_conn_status->connected)
    {
        printf("Connected to ANS \n");

        anc_app_state.conn_id = p_conn_status->conn_id;

        /* save address of the connected device. */
        memcpy(anc_app_state.remote_addr, p_conn_status->bd_addr, 
                sizeof(anc_app_state.remote_addr));
        anc_app_state.addr_type = p_conn_status->addr_type;

        /* need to notify ANC library that the connection is up */
        wiced_bt_anc_client_connection_up(p_conn_status);

        /* Initialize WICED BT ANC library Start discovery */
        anc_app_state.discovery_state = ANC_DISCOVERY_STATE_SERVICE;
        anc_app_state.anc_s_handle = 0;
        anc_app_state.anc_e_handle = 0;

        /* perform primary service search */
        status = wiced_bt_util_send_gatt_discover(anc_app_state.conn_id, 
                GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE,
                ANC_START_HANDLE, ANC_END_HANDLE);
        WICED_BT_TRACE("Start discover status: 0x%X\n", status);
    }
    else
    {
        WICED_BT_TRACE("Connection to ANS failed \n");
    }
}

/*******************************************************************************
* Function Name: bt_app_anc_connection_down
********************************************************************************
* Summary:
*   This function will be called when connection goes down
*
* Parameters:
*   p_conn_status  : Current status of the Connection

* Return:
*  None
*
*******************************************************************************/
static void bt_app_anc_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    anc_app_state.conn_id = 0;
    anc_app_state.anc_s_handle = 0;
    anc_app_state.anc_e_handle = 0;
    anc_app_state.discovery_state = ANC_DISCOVERY_STATE_SERVICE;

    printf("Connection Down \n");
    /* pending command no more valid now */
    bt_app_clear_anc_pending_cmd_context();

    memset(anc_app_state.remote_addr, 0, sizeof(wiced_bt_device_address_t));
    /* tell library that connection is down */
    wiced_bt_anc_client_connection_down(p_conn_status);
}

/*******************************************************************************
* Function Name: bt_app_anc_process_write_rsp
*******************************************************************************
* Summary:
*   Pass write response to appropriate client based on the attribute handle
*
* Parameters:
*  p_data     Pointer to GATT Operation Data
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("Write Response handle:0x%X\n", p_data->response_data.handle);

    /* Verify that write response is for our service */
    if ((p_data->response_data.handle >= anc_app_state.anc_s_handle) &&
            (p_data->response_data.handle <= anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_write_rsp(p_data);
    }
}

/*******************************************************************************
* Function Name: bt_app_anc_process_read_rsp
*******************************************************************************
* Summary:
*   Pass read response to appropriate client based on the attribute handle
*
* Parameters:
*  p_data     Pointer to GATT Operation Data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
static void bt_app_anc_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("Read Response Handle:0x%X\n", p_data->response_data.handle);

    /* Verify that write response is for our service */
    if ((p_data->response_data.handle >= anc_app_state.anc_s_handle) &&
            (p_data->response_data.handle <= anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_read_rsp(p_data);
    }
}

/*******************************************************************************
* Function Name: bt_app_anc_notification_handler
*******************************************************************************
* Summary:
*   Pass notification to appropriate client based on the attribute handle
*
* Parameters:
*  p_data     Pointer to GATT Operation Data
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("Notification Handle:0x%X\n", p_data->response_data.att_value.handle);

    /* Verify that notification is for ANCS service and if true pass to
     * the library for processing
     */
    if ((p_data->response_data.att_value.handle >= anc_app_state.anc_s_handle) &&
            (p_data->response_data.att_value.handle < anc_app_state.anc_e_handle))
    {
        wiced_bt_anc_client_process_notification(p_data);
    }
}

/*******************************************************************************
* Function Name: bt_app_anc_gatt_operation_complete
*******************************************************************************
* Summary:
*   GATT operation started by the client has been completed
*
* Parameters:
*  p_data     Pointer to LE operation data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
static wiced_bt_gatt_status_t bt_app_anc_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    if (NULL == p_data)
    {
        WICED_BT_TRACE("GATT Operation Complete Callback Event Data is pointing to NULL \n");
        return WICED_BT_GATT_SUCCESS;
    }
    
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
        bt_app_anc_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
        WICED_BT_TRACE("This app does not support optype:%u\n", p_data->op);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        bt_app_anc_notification_handler(p_data);
        break;

    case GATTC_OPTYPE_READ_HANDLE:
    case GATTC_OPTYPE_READ_BY_TYPE:
        bt_app_anc_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE("This app does not support optype:%u\n", p_data->op);
        break;

    default:
        break;
    }
    
    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: bt_app_anc_gatt_discovery_result
*******************************************************************************
* Summary:
*   Callback function to handle the result of GATT Discovery procedure
*
* Parameters:
*  p_data     Pointer to GATT Discovery Data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
static wiced_bt_gatt_status_t bt_app_anc_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    uint16_t alert_service_uuid = UUID_SERVICE_ALERT_NOTIFICATION;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    if (NULL == p_data)
    {
        WICED_BT_TRACE("GATT Discovery Result Data is pointing to NULL \n");
    }
    else
    {
        switch (anc_app_state.discovery_state)
        {
            case ANC_DISCOVERY_STATE_ANC:
                wiced_bt_anc_discovery_result(p_data);
                break;

            default:
                if (p_data->discovery_type == GATT_DISCOVER_SERVICES_ALL)
                {
                    if (p_data->discovery_data.group_value.service_type.len == LEN_UUID_16)
                    {
                        WICED_BT_TRACE("Start Handle: 0x%X End Handle: 0x%X \n",
                                p_data->discovery_data.group_value.s_handle,
                                p_data->discovery_data.group_value.e_handle);
                        if (memcmp(&p_data->discovery_data.group_value.service_type.uu,
                                &alert_service_uuid, LEN_UUID_16) == 0)
                        {
                            WICED_BT_TRACE("ANS Service found. Start Handle: 0x%X End Handle: 0x%X\n",
                                    p_data->discovery_data.group_value.s_handle,
                                    p_data->discovery_data.group_value.e_handle);
                            anc_app_state.anc_s_handle = p_data->discovery_data.group_value.s_handle;
                            anc_app_state.anc_e_handle = p_data->discovery_data.group_value.e_handle;
                        }
                    }
                }
                else
                {
                    WICED_BT_TRACE("!!!! Invalid operation: %u\n", p_data->discovery_type);
                    gatt_status = WICED_BT_GATT_ERROR;
                }
        }
    }
    
    return gatt_status;
}

/*******************************************************************************
* Function Name: bt_app_anc_gatt_discovery_complete
*******************************************************************************
* Summary:
*   Callback function to handle the Completion of GATT Discovery procedure
*
* Parameters:
*  p_data     Pointer to GATT Discovery Data
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
static wiced_bt_gatt_status_t bt_app_anc_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    if (NULL == p_data)
    {
        WICED_BT_TRACE("GATT Discovery Complete Data is pointing to NULL \n");
    }
    else
    {
        WICED_BT_TRACE("\nGATT Discovery: Connection Id 0x%X Type %u State 0x%X\n",
                p_data->conn_id, p_data->discovery_type, anc_app_state.discovery_state);

        switch (anc_app_state.discovery_state)
        {
            case ANC_DISCOVERY_STATE_ANC:
                wiced_bt_anc_client_discovery_complete(p_data);
                break;

            default:
                if (p_data->discovery_type == GATT_DISCOVER_SERVICES_ALL)
                {
                    WICED_BT_TRACE("ANS:Start Handle 0x%X- End Handle 0x%X\n",
                            anc_app_state.anc_s_handle, anc_app_state.anc_e_handle);

                    /* If Alert Notification Service found tell
                     * WICED BT anc library to start its discovery
                     */
                    if ((anc_app_state.anc_s_handle != 0) && (anc_app_state.anc_e_handle != 0))
                    {
                        anc_app_state.discovery_state = ANC_DISCOVERY_STATE_ANC;
                        if (wiced_bt_anc_discover(anc_app_state.conn_id, anc_app_state.anc_s_handle,
                                anc_app_state.anc_e_handle))
                        {
                            gatt_status = WICED_BT_GATT_ERROR;
                        }
                    }
                }
                else
                {
                    WICED_BT_TRACE("!!!! Invalid operation: %u\n", p_data->discovery_type);
                    gatt_status = WICED_BT_GATT_ERROR;
                }
                break;
        }
    }
    
    return gatt_status;
}

/*******************************************************************************
* Function Name: bt_app_anc_gatts_callback
*******************************************************************************
* Summary:
*   Callback for various GATT events.  As this application performs only as a
*   GATT server, some of the events are omitted.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one
*                                              byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
*                                              structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
static wiced_bt_gatt_status_t bt_app_anc_gatts_callback(wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    if (NULL == p_data)
    {
        WICED_BT_TRACE("GATT Callback Event Data is pointing to NULL \n");
        return result;
    }
    
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            bt_app_anc_connection_up(&p_data->connection_status);
        }
        else
        {
            bt_app_anc_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = bt_app_anc_gatt_operation_complete(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = bt_app_anc_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = bt_app_anc_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        /*Not supported for this as this acts as a client role */
        break;

    default:
        break;
    }

    return result;
}

/******************************************************************************
* Function Name: bt_app_handle_usr_cmd
******************************************************************************
* Summary:
*   Handles the ANC command and calls corresponding ANC API.
*
* Parameters:
*  cmd: User Command
*  cmd_id : Control Command ID in case of 'Control Required Alerts' option
*  alert_category: Alert category in case of 'Control Required Alerts' option
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e
*  in wiced_bt_gatt.h
*
******************************************************************************/
wiced_bt_gatt_status_t bt_app_handle_usr_cmd(uint8_t cmd, uint8_t cmd_id,
        uint8_t alert_categ)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    
    switch (cmd)
    {
    case USR_ANC_COMMAND_READ_SERVER_SUPPORTED_NEW_ALERTS:
        gatt_status = wiced_bt_anc_read_server_supported_new_alerts(anc_app_state.conn_id);
        break;

    case USR_ANC_COMMAND_READ_SERVER_SUPPORTED_UNREAD_ALERTS:
        gatt_status = wiced_bt_anc_read_server_supported_unread_alerts(anc_app_state.conn_id);
        break;

    case USR_ANC_COMMAND_CONTROL_ALERTS:
        gatt_status = wiced_bt_anc_control_required_alerts(anc_app_state.conn_id,
                (wiced_bt_anp_alert_control_cmd_id_t)cmd_id,
                (wiced_bt_anp_alert_category_id_t)alert_categ);
        break;

    case USR_ANC_COMMAND_ENABLE_NTF_NEW_ALERTS:
        gatt_status = wiced_bt_anc_enable_new_alerts(anc_app_state.conn_id);
        break;

    case USR_ANC_COMMAND_ENABLE_NTF_UNREAD_ALERT_STATUS:
        gatt_status = wiced_bt_anc_enable_unread_alerts(anc_app_state.conn_id);
        break;

    case USR_ANC_COMMAND_DISABLE_NTF_NEW_ALERTS:
        gatt_status = wiced_bt_anc_disable_new_alerts(anc_app_state.conn_id);
        break;

    case USR_ANC_COMMAND_DISABLE_NTF_UNREAD_ALERT_STATUS:
        gatt_status = wiced_bt_anc_disable_unread_alerts(anc_app_state.conn_id);
        break;

    default:
        WICED_BT_TRACE("Unknown pending command \n");
        break;
    }

    if (WICED_BT_GATT_INSUF_AUTHENTICATION == gatt_status)
    {
        bt_app_anc_start_pair();
        WICED_BT_TRACE("Starting Pairing process 0x%X \n", gatt_status);
        anc_pending_cmd_context[0] = cmd;
        anc_pending_cmd_context[1] = cmd_id;
        anc_pending_cmd_context[2] = alert_categ;
        return WICED_BT_GATT_SUCCESS;
    }

    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        WICED_BT_TRACE("Operation Result 0x%X \n", gatt_status);
    }
    
    return gatt_status;
}

/*******************************************************************************
* Function Name: bt_app_anc_trigger_pending_action
*******************************************************************************
* Summary:
*   Trigger the pending command
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
static void bt_app_anc_trigger_pending_action( void )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    
    if (!anc_pending_cmd_context[0])
    {
        WICED_BT_TRACE(" Trigger: No commands pending! \n");
        return;
    }
    
    gatt_status = bt_app_handle_usr_cmd(anc_pending_cmd_context[0],
            anc_pending_cmd_context[1],
            anc_pending_cmd_context[2]);

    if (gatt_status != WICED_BT_GATT_SUCCESS)
    {
        WICED_BT_TRACE("ANC trigger pending status 0x%X \n", gatt_status);
    }

    /* should not keep trying if fail in sending pending command */
    bt_app_clear_anc_pending_cmd_context();
}

/*******************************************************************************
* Function Name: bt_app_anc_application_init
*******************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called
*   from the BT management callback once the LE stack enabled event
*   (BTM_ENABLED_EVT) is triggered This function is executed in the
*   BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void bt_app_anc_application_init( void )
{
    wiced_bt_gatt_status_t gatt_status = 0;

    WICED_BT_TRACE("wiced_bt_gatt_register: 0x%X\n", gatt_status);
    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(bt_app_anc_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_db_init 0x%X\n", gatt_status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(true, false);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    app_bt_add_devices_to_address_resolution_db();

    /* Set the advertising params and make the device discoverable */
    bt_app_anc_set_advertisement_data();

    /* Create UART Task for processing UART commands */
    if (pdPASS != xTaskCreate(uart_task, "uart_task", UART_TASK_STACK_SIZE,
            NULL, UART_TASK_PRIORITY, &uart_task_handle))
    {
        printf("Failed to create UART task!\n");
        handle_app_error();
    }

}

/*******************************************************************************
* Function Name: bt_app_anc_start_advertisement
*******************************************************************************
* Summary:
*   Sets the advertisement data for ANC
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
void bt_app_anc_start_advertisement( void )
{
    wiced_result_t result;
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
            0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
}

/*******************************************************************************
* Function Name: bt_app_anc_management_callback
*******************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management
*   events from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte
*                                                 length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management
*                                                 event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
******************************************************************************/
wiced_result_t bt_app_anc_management_callback(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_device_address_t bda = {0};
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    wiced_result_t result = WICED_BT_SUCCESS;
    cy_rslt_t rslt;

    WICED_BT_TRACE("Bluetooth Management Event: 0x%X %s\n", event,
            (char *)get_btm_event_name(event));

    if (NULL == p_event_data)
    {
        WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
        result = WICED_BT_ERROR;
    }
    else
    {
        switch (event)
        {
            case BTM_ENABLED_EVT:

                /* Bluetooth Controller and Host Stack Enabled */
                if (WICED_BT_SUCCESS == p_event_data->enabled.status)
                {
                    wiced_bt_set_local_bdaddr(anc_bd_address, BLE_ADDR_PUBLIC);
                    wiced_bt_dev_read_local_addr(bda);

                    printf("Local Bluetooth Address (PSOC kit): ");
                    print_bd_address(bda);

                    /* Perform application-specific initialization */
                    wiced_bt_anc_init(&bt_app_anc_callback);
                    bt_app_anc_application_init();
                }
                else
                {
                    printf("Bluetooth Enabling Failed \n");
                }
                break;

            case BTM_DISABLED_EVT:
                printf("Bluetooth Disabled \n");
                break;

            case BTM_USER_CONFIRMATION_REQUEST_EVT:
                WICED_BT_TRACE("Numeric_value: %lu \n",
                        p_event_data->user_confirmation_request.numeric_value);
                wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                        p_event_data->user_confirmation_request.bd_addr);
                break;

            case BTM_PASSKEY_NOTIFICATION_EVT:
                WICED_BT_TRACE("PassKey Notification. Key: %lu, BDA: \n",
                        p_event_data->user_passkey_notification.passkey);
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->user_passkey_notification.bd_addr);
                }
                wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                        p_event_data->user_passkey_notification.bd_addr);
                break;

            case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
                p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
                p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
                p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_BOND;
                p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_KEY_SIZE;
                p_event_data->pairing_io_capabilities_ble_request.init_keys =
                        BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
                p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                        BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
                break;

            case BTM_PAIRING_COMPLETE_EVT:
                /* Update number of bonded devices and next free slot in slot data*/
                rslt = app_bt_update_slot_data();

                WICED_BT_TRACE("Pairing Complete: %d",
                        p_event_data->pairing_complete.pairing_complete_info.ble.reason);
                break;

            case BTM_ENCRYPTION_STATUS_EVT:
                WICED_BT_TRACE("Encryption Status Event: Result: %d BDA:",
                        p_event_data->encryption_status.result);
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->encryption_status.bd_addr);
                }
                if (p_event_data->encryption_status.result == WICED_BT_SUCCESS)
                    bt_app_anc_trigger_pending_action();
                else /* pending command no more valid to send if authentication fails */
                    bt_app_clear_anc_pending_cmd_context();
                break;

            case BTM_SECURITY_REQUEST_EVT:
                wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                        WICED_BT_SUCCESS);
                break;

            case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
                /* save device keys to NVM */
                rslt = app_bt_save_device_link_keys(&(p_event_data->paired_device_link_keys_update));
                if (CY_RSLT_SUCCESS == rslt)
                {
                    printf("Successfully Bonded to ");
                    print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
                }
                else
                {
                    printf("Failed to bond! \n");
                }
                break;

            case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
                /* Paired Device Link Keys Request */
                WICED_BT_TRACE("Paired device link key size: %u, BDA: ",
                        sizeof(wiced_bt_device_link_keys_t));
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->paired_device_link_keys_request.bd_addr);
                }
                /* Need to search to see if the BD_ADDR we are
                 * looking for is in NVM. If not, we return WICED_BT_ERROR
                 * and the stack will generate keys and will then call
                 * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they
                 * can be stored
                 */

                /* Assume the device won't be found.
                 * If it is, we will set this back to WICED_BT_SUCCESS */
                result = WICED_BT_ERROR;
                bondindex = app_bt_find_device_in_nvm(p_event_data->paired_device_link_keys_request.bd_addr);

                if(BOND_INDEX_MAX > bondindex)
                {
                    /* Copy the keys to where the stack wants it */
                    memcpy(&(p_event_data->paired_device_link_keys_request),
                            &bond_info.link_keys[bondindex],
                            sizeof(wiced_bt_device_link_keys_t));
                    result = WICED_BT_SUCCESS;
                }
                else
                {
                    WICED_BT_TRACE("Device Link Keys not found in the database! \n");
                }
                break;

            case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
                /* Update of local privacy keys - save to NVM */
                rslt = app_bt_save_local_identity_key(p_event_data->local_identity_keys_update);
                if (CY_RSLT_SUCCESS != rslt)
                {
                    result = WICED_BT_ERROR;
                }
                WICED_BT_TRACE("Local keys save to NVRAM result: 0x%X \n", result);
                break;

            case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
                /* Read Local Identity Resolution Keys if present in NVM*/
                rslt = app_bt_read_local_identity_keys();
                if(CY_RSLT_SUCCESS == rslt)
                {
                    memcpy(&(p_event_data->local_identity_keys_request),
                            &(identity_keys), sizeof(wiced_bt_local_identity_keys_t));
                    if(ENABLE_BT_VERBOSE_LOGS)
                    {
                        print_array(&identity_keys,
                                sizeof(wiced_bt_local_identity_keys_t));
                    }
                    result = WICED_BT_SUCCESS;
                }
                else
                {
                    result = WICED_BT_ERROR;
                }

                break;

            case BTM_BLE_SCAN_STATE_CHANGED_EVT:
                WICED_BT_TRACE("Scan State Change: %u\n",
                        p_event_data->ble_scan_state_changed);
                break;

            case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
                /* Advertisement State Changed */
                p_adv_mode = &p_event_data->ble_advert_state_changed;
                WICED_BT_TRACE("Advertisement State Change: %s\n",
                        get_bt_advert_mode_name(*p_adv_mode));

                if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
                {
                    /* Advertisement Stopped */
                    WICED_BT_TRACE("Advertisement stopped\n");
                }
                else
                {
                    /* Advertisement Started */
                    WICED_BT_TRACE("Advertisement started\n");
                }
                break;

            case BTM_BLE_CONNECTION_PARAM_UPDATE:
                WICED_BT_TRACE("Connection parameter update status:0x%X, "
                        "Connection Interval: %d, Connection Latency: %d, "
                        "Connection Timeout: %d\n",
                        p_event_data->ble_connection_param_update.status,
                        p_event_data->ble_connection_param_update.conn_interval,
                        p_event_data->ble_connection_param_update.conn_latency,
                        p_event_data->ble_connection_param_update.supervision_timeout);
                break;

            default:
                break;
        }
    }

    return result;
}

/* END OF FILE [] */
