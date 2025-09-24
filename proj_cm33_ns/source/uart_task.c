/*******************************************************************************
* File Name: uart_task.c
*
* Description: Implements the UART task for the Alert Notification Client (ANC),
*              handling input from the terminal to control alert settings and
*              receive notifications.
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
#include "uart_task.h"

/*******************************************************************************
* Macros
******************************************************************************/
#define CHAR_BUFFER_LENGTH      (5U)
#define CHAR_NULL               '\0'
#define CHAR_CARRIAGE_RETURN    '\r'
#define CHAR_NEWLINE            '\n'
#define MAX_HEX_INPUT_RANGE     (0x3FFU)
#define HEX_BASE                (16U)
 #define MAX_INPUT_VALUE         (3U)
 #define RESET_VAL_ZERO          (0U)

/*******************************************************************************
* Static Variables
******************************************************************************/
/* Menu string for the Alert Notification Client application */
static const char bt_app_anc_app_menu[] = "\n\
============================================================== \n\
  Alert Notification Client Menu \n\
---------------------------------------------------------------\n\
    1.  Start Advertising \n\
    2.  Read Supported New Alert Categories \n\
    3.  Read Supported Unread Alert Categories \n\
    4.  Configure Alert Notification Control Point (ANCP)\n\
    5.  Enable Notifications for New Alerts Category \n\
    6.  Enable Notifications for Unread Alerts Category \n\
    7.  Disable Notifications for New Alerts Category \n\
    8.  Disable Notifications for Unread Alerts Category \n\
 =============================================================\n\
 Select option (1-8): ";

/* String representing the alert IDs for selection */
static const char alert_ids[] = "\
    ----------------------------- \n\
    SIMPLE_ALERT          0 \n\
    EMAIL                 1 \n\
    NEWS                  2 \n\
    CALL                  3 \n\
    MISSED_CALL           4 \n\
    SMS_OR_MMS            5 \n\
    VOICE_MAIL            6 \n\
    SCHEDULE_ALERT        7 \n\
    HIGH_PRIORITY_ALERT   8 \n\
    INSTANT_MESSAGE       9 \n\
    ----------------------------- \n";

static const char control_cmds[] = "\
    -------------------------------------------\n\
    Enable New Alerts Category                0 \n\
    Enable Unread Alert Status Category       1 \n\
    Disable New Alerts Category               2 \n\
    Disable Unread Alert Status Category      3 \n\
    -------------------------------------------\n";

/*******************************************************************************
* Global variables
******************************************************************************/
uint8_t anc_bd_address[LOCAL_BDA_LEN] = {0x11, 0x12, 0x13, 0x51, 0x52, 0x53};
TaskHandle_t uart_task_handle;

/*******************************************************************************
* Function Definitions
******************************************************************************/

/******************************************************************************
* Function Name: uart_get_data()
*******************************************************************************
* Summary:
*         This function reads a single byte from the UART interface,
*         blocking until data is available.
*
* Parameters:
*          value : uart data
*
* Return:
*          None
*
******************************************************************************/
static void uart_get_data(uint8_t *value)
{
    uint32_t read_value = Cy_SCB_UART_Get(SCB2);
    while (CY_SCB_UART_RX_NO_DATA == read_value)
    {
        read_value = Cy_SCB_UART_Get(SCB2);
    }
    *value = (uint8_t)read_value;
}

/*******************************************************************************
* Function Name: read_hex_number
*******************************************************************************
* Summary:
*  Reads a hexadecimal number entered via UART terminal, terminating on "Enter".
*  It filters input for valid hexadecimal characters and converts the ASCII input
*  into a numerical value.
*
* Parameters:
*   unsigned int *hex_value : Pointer to store the hexadecimal number input
*
* Return:
*   cy_rslt_t
*
******************************************************************************/
cy_rslt_t read_hex_number(unsigned int *hex_value)
{
    char input_buffer[CHAR_BUFFER_LENGTH];
    int input_index = 0;
    char *conversion_endptr;
    uint8_t read_character;
    int i=0;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    while(i < MAX_INPUT_VALUE)
    {
        uart_get_data(&read_character);

        if(CHAR_CARRIAGE_RETURN  == read_character)
        {
            break;
        }
        if ( (CHAR_NEWLINE != read_character) &&
                (CHAR_CARRIAGE_RETURN  != read_character) &&
                ((CHAR_BUFFER_LENGTH - 1) > input_index) )
        {
            if (isxdigit((unsigned char)read_character))
            {
                input_buffer[input_index++] = read_character;
            }
        }
        i++;
    }
    input_buffer[input_index] = CHAR_NULL;

    /* Convert the hexadecimal string to a numerical value */
    *hex_value = (uint16_t)strtol(input_buffer, &conversion_endptr, HEX_BASE);

    if ( (CHAR_NULL != *conversion_endptr) ||
            (MAX_HEX_INPUT_RANGE < *hex_value) )
    {
        printf("Invalid input: out of range (0 to 0x3FF)\n");
        result = -1;
    }

    return result;
}


/*******************************************************************************
* Function Name: uart_task
*******************************************************************************
* Summary:
* This function runs the UART task which processes the received commands via
* Terminal.
*
* Parameters:
* void *pvParameters:  Not used
*
* Return:
* None
*
******************************************************************************/
void uart_task(void *pvParameters)
 {
     wiced_bt_gatt_status_t status = WICED_BT_SUCCESS;
     unsigned int alert_category = 0;
     unsigned int cmd_id = 0;
     uint8_t uart_response;
     uint8_t selected_cmd = 0;

     for (;;)
     {
         printf("%s\n", bt_app_anc_app_menu);

         uart_get_data(&uart_response);

         selected_cmd = CHAR_TO_DIGIT(uart_response);

         printf("Selected command: (%u) \n", selected_cmd);

         switch (selected_cmd)
         {
             case USR_ANC_COMMAND_START_ADVERTISEMENT: /* Start Advertising */
                 printf("Starting ANC Advertisement \n");
                 bt_app_anc_start_advertisement();
                 break;

                 /* Fall through as it is the same function called to execute command */
             case USR_ANC_COMMAND_READ_SERVER_SUPPORTED_NEW_ALERTS:
             case USR_ANC_COMMAND_READ_SERVER_SUPPORTED_UNREAD_ALERTS:
             case USR_ANC_COMMAND_ENABLE_NTF_NEW_ALERTS:
             case USR_ANC_COMMAND_ENABLE_NTF_UNREAD_ALERT_STATUS:
             case USR_ANC_COMMAND_DISABLE_NTF_NEW_ALERTS:
             case USR_ANC_COMMAND_DISABLE_NTF_UNREAD_ALERT_STATUS:
                 alert_category = RESET_VAL_ZERO;
                 cmd_id = RESET_VAL_ZERO;
                 status = bt_app_handle_usr_cmd(CHAR_TO_DIGIT(uart_response), cmd_id,
                         alert_category);
                 if (WICED_BT_SUCCESS == status)
                 {
                     printf("Command sent to Alert server successfully\n");
                 }
                 else
                 {
                     printf("Failed to send command. Status: 0x%X\n", status);
                 }
                 break;

             case USR_ANC_COMMAND_CONTROL_ALERTS:
                 printf("\n    Control Point Commands \n");
                 printf("%s", control_cmds);
                 printf("Enter Control Point Command and press Enter:\n");
                 if ( CY_RSLT_SUCCESS != read_hex_number(&cmd_id) )
                 {
                     printf("Invalid input. Please enter a valid control point command\n");
                     continue;
                 }
                 else
                 {
                     printf("Entered Control Point command: 0x%X\n", cmd_id);
                 }
                 printf("\n    Alert Categories \n");
                 printf("%s", alert_ids);
                 printf("Enter the Alert Category and press Enter:\n");
                 if ( CY_RSLT_SUCCESS != read_hex_number(&alert_category) )
                 {
                     printf("Invalid input. Please enter a valid alert category.\n");
                     continue;
                 }
                 else
                 {
                     printf("Entered Alert Category: 0x%X\n", alert_category);
                 }
                 status = bt_app_handle_usr_cmd(USR_ANC_COMMAND_CONTROL_ALERTS,
                         (uint8_t)cmd_id, (uint8_t)alert_category);
                 if (WICED_BT_SUCCESS == status)
                 {
                     printf("Command Sent to ANS successfully.\n");
                 }
                 else
                 {
                     printf("Failed to send Control Alert command. Status: 0x%X\n",
                             status);
                 }
                 break;

             default:
                 printf("Unknown ANC Command. Select option from the Menu \n");
                 break;
        }

         if (status != WICED_BT_SUCCESS)
         {
             printf("\n Command Failed. Status: 0x%X \n", status);
             status = WICED_BT_SUCCESS;
         }
     }
 }

 /* END OF FILE [] */
