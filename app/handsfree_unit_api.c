/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
*/

/******************************************************************************
 * File Name: handsfree_unit_api.c
 *
 * Description: This is the source code for the LE Handsfree Unit Example
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

/*******************************************************************************
*      INCLUDES
*******************************************************************************/

#include "handsfree_unit_api.h"

#include <stdio.h>
#include <stdlib.h>

#include "wiced_bt_stack.h"

#include "handsfree_unit.h"


/*******************************************************************************
*       MACROS
********************************************************************************/
#define BT_STACK_HEAP_SIZE              (0xF000)
#define HFU_INQUIRY_DURATION            (5)

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/
wiced_bt_heap_t *p_default_heap   = NULL;

/*******************************************************************************
*       FUNCTION PROTOTYPES
********************************************************************************/
static void handsfree_build_send_at_cmd
                            (
                                uint16_t handle,
                                char *cmd,
                                uint8_t arg_type,
                                uint8_t arg_format,
                                const char *p_arg,
                                int16_t int_arg
                            );

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/*******************************************************************************
* Function Name: handsfree_application_start
********************************************************************************
* Summary:
*  Set device configuration and start BT stack initialization. The actual
*  application initialization will happen when stack reports that BT device
*  is ready.
*
* Parameters: NONE
*
* Return: NONE
*
*******************************************************************************/
void handsfree_application_start()
{
    wiced_result_t wiced_result;

    WICED_BT_TRACE("************* Handsfree Unit Application Start ************************\n");

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init (handsfree_management_callback, &handsfree_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result)
    {
        WICED_BT_TRACE("Bluetooth Stack Initialization Successful \n");
        /* Create default heap */
        p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
        if (p_default_heap == NULL)
        {
            WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
            exit(EXIT_FAILURE);
        }
    }
    else
    {
       WICED_BT_TRACE("Bluetooth Stack Initialization failed!! \n");
       exit(EXIT_FAILURE);
    }
    wait_init_done();
}

/*******************************************************************************
* Function Name: handsfree_handle_set_visibility
********************************************************************************
* Summary:
*   Sets the device discoverable and connectability mode
*
* Parameters:
*   unsigned int discoverability : 1-discoverable, 0-Non-Discoverable
*   unsigned int connectability : 1-Connectable, 0-Non-Connectable
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_handle_set_visibility(unsigned int discoverability, unsigned int connectability)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    /* we cannot be discoverable and not connectable */
    if (((discoverability != 0) && (connectability == 0)) ||
           (discoverability > 1) ||
           (connectability > 1))
    {
        WICED_BT_TRACE("%s: Invalid Arguements: we cannot be discoverable and not connectable\n", __FUNCTION__);
    }
    else
    {
        result = wiced_bt_dev_set_discoverability((discoverability != 0) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);
        if (WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("%s: wiced_bt_dev_set_discoverability failed. Status = %x\n", __FUNCTION__, result);
        }

        result = wiced_bt_dev_set_connectability((connectability != 0) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);
        if (WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("%s: wiced_bt_dev_set_connectability failed. Status = %x\n", __FUNCTION__, result);
        }
    }
}

/*******************************************************************************
* Function Name: handsfree_handle_set_pairability
********************************************************************************
* Summary:
*   Enable or disable pairing
*
* Parameters:
*   unsigned int allowed : 1-pairable, 0-Not-Pairable
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_handle_set_pairability (unsigned int allowed)
{
    void *p1;

    if ( allowed > 1 )
    {
        WICED_BT_TRACE("%s: Invalid Argument\n", __FUNCTION__);
    }
    else if (handsfree_app_states.pairing_allowed != allowed)
    {
        handsfree_app_states.pairing_allowed = allowed;
        wiced_bt_set_pairable_mode(handsfree_app_states.pairing_allowed, 0);
        WICED_BT_TRACE(" Set the pairing allowed to %d \n", handsfree_app_states.pairing_allowed);
    }
}

/*******************************************************************************
* Function Name: handsfree_inquiry
********************************************************************************
* Summary:
*   Starts or Stops Device Inquiry
*
* Parameters:
*   unsigned int enable : 1 - starts Inquiry, 0 - Stops Inquiry
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_inquiry(unsigned int enable)
{
    wiced_result_t result;
    wiced_bt_dev_inq_parms_t params;

    if (enable == 1)
    {
        memset(&params, 0, sizeof(params));

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = HFU_INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry(&params, &handsfree_inquiry_result_cback);
        WICED_BT_TRACE("inquiry started:%d\n", result);
    }
    else if (enable == 0)
    {
        result = wiced_bt_cancel_inquiry();
        WICED_BT_TRACE("cancel inquiry:%d\n", result);
    }
    else
    {
        printf("%s: Invalid Argument\n", __FUNCTION__);
    }
    UNUSED_VARIABLE(result);
}

/*******************************************************************************
* Function Name: handsfree_build_send_at_cmd
********************************************************************************
* Summary:
*   Builds the AT command for the HF command
*
* Parameters:
*   uint16_t handle : Connection Handle
*   char *cmd : HF command to be sent
*   uint8_t arg_type : WICED_BT_HFP_HF_AT_SET - command for setting the value
*                      AT+<cmd>=<val>
*                      WICED_BT_HFP_HF_AT_READ - command for getting the value
*                      AT+<cmd>?
*   uint8_t arg_format : Format of the command value/data
*                       (WICED_BT_HFP_HF_AT_FMT_INT/WICED_BT_HFP_HF_AT_FMT_STR)
*   const char *p_arg : command value in case the arg_format is string
*   int16_t int_arg :  command value in case the arg_format is int
*
* Return:
*
*******************************************************************************/
static void handsfree_build_send_at_cmd (
                                            uint16_t handle,
                                            char *cmd,
                                            uint8_t arg_type,
                                            uint8_t arg_format,
                                            const char *p_arg,
                                            int16_t int_arg
                                        )
{
    char buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char *p = buf;

    if (NULL == cmd)
    {
        WICED_BT_TRACE("%s: error cmd to be sent is null\n", __FUNCTION__);
    }
    else
    {
        memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

        *p++ = 'A';
        *p++ = 'T';

        /* copy result code string */
        memcpy(p,cmd, strlen(cmd));
        p += strlen(cmd);

        if (arg_type == WICED_BT_HFP_HF_AT_SET)
        {
            *p++ = '=';

        }
        else if (arg_type == WICED_BT_HFP_HF_AT_READ)
        {
            *p++ = '?';

        }
        else if (arg_type == WICED_BT_HFP_HF_AT_TEST)
        {
            *p++ = '=';
            *p++ = '?';

        }

        /* copy argument if any */
        if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
        {
            p += utl_itoa((uint16_t) int_arg, p);
        }
        else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
        {
            utl_strcpy(p, (char *)p_arg);
            p += strlen(p_arg);
        }

        /* finish with \r*/
        *p++ = '\r';

        WICED_BT_TRACE("SENDING AT CMD << %s\n", buf);

        wiced_bt_hfp_hf_send_at_cmd(handle,buf);
    }
}

/*******************************************************************************
* Function Name: handsfree_send_at_command
********************************************************************************
* Summary:
*   calls handsfree_build_send_at_cmd to build and sent the AT command
*
* Parameters:
*   uint16_t handle : connection handle
*   uint8_t command : command to be sent
*   int num : AT command arguement / value in case of integer
*   uint8_t* p_data : AT command arguement / value in case of string
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_send_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data)
{
    switch (command)
    {
        case HF_AT_CMD_VGS:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_SPEAKER, num);
            break;

        case HF_AT_CMD_VGM:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_MIC, num);
            break;

        case HF_AT_CMD_BINP:
            handsfree_build_send_at_cmd(handle, "+BINP",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1);
            break;

        case HF_AT_CMD_CHLD:
            wiced_bt_hfp_hf_perform_call_action(handle,
                    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0 + num,(char *)p_data);
            break;

        case HF_AT_CMD_BVRA:
            handsfree_build_send_at_cmd(handle, "+BVRA",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_CMEE:
            handsfree_build_send_at_cmd(handle, "+CMEE",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_ATA:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_ANSWER,(char *)p_data);
            break;

        case HF_AT_CMD_CHUP:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_HANGUP,(char *)p_data);
            break;

        case HF_AT_CMD_CNUM:
            handsfree_build_send_at_cmd(handle, "+CNUM",
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_CLCC:
            handsfree_build_send_at_cmd(handle, "+CLCC",
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_CIND:
            handsfree_build_send_at_cmd(handle, "+CIND",
                    WICED_BT_HFP_HF_AT_READ, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HF_AT_CMD_D:
        case HF_AT_CMD_BLDN:
            wiced_bt_hfp_hf_perform_call_action (handle ,
                                        WICED_BT_HFP_HF_CALL_ACTION_DIAL ,(char *)p_data);
            break;

        case HF_AT_CMD_NREC:
            handsfree_build_send_at_cmd(handle, "+NREC",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 0);
            break;

        case HF_AT_CMD_VTS:
            handsfree_build_send_at_cmd(handle, "+VTS",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;

        case HF_AT_CMD_BTRH:
            handsfree_build_send_at_cmd(handle, "+BTRH",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HF_AT_CMD_BIEV:
            handsfree_build_send_at_cmd(handle, "+BIEV",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HF_AT_CMD_BIA:
            handsfree_build_send_at_cmd(handle, "+BIA",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HF_AT_CMD_BCC:
            handsfree_build_send_at_cmd(handle, "+BCC",
                            WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;
        case HF_AT_CMD_BAC:
            handsfree_build_send_at_cmd(handle, "+BAC",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
    }
}

/*******************************************************************************
* Function Name: handsfree_is_valid_state
********************************************************************************
* Summary:
*   This Function checks if the option can be handled in the current HF state
*
* Parameters:
*   int choice: user entered input
*
* Return:
*   uint8_t: returns if the user_input is valid in the current state
*            WICED_FALSE: for invalid state
*            WICED_TRUE: for valid state
*
*******************************************************************************/
uint8_t handsfree_is_valid_state (int user_input)
{
    uint8_t valid = WICED_FALSE;

    switch(handsfree_ctxt_data.connection_status)
    {
    case WICED_BT_HFP_HF_STATE_DISCONNECTED:
        WICED_BT_TRACE("Current State: WICED_BT_HFP_HF_STATE_DISCONNECTED\n");
        switch(user_input)
        {
        case EXIT:
        case PRINT_MENU:
        case SET_VISIBILITY:
        case SET_PAIRING_MODE:
        case SET_INQUIRY:
        case HF_CONNECT:
        case HF_PRINT_CONNECTION_DETAILS:
        case AUDIO_PLAYBACK_TEST:
        case AUDIO_RECORD_TEST:
            valid = WICED_TRUE;
            break;
        }
        break;
    case WICED_BT_HFP_HF_STATE_CONNECTED:
        WICED_BT_TRACE("Current State: WICED_BT_HFP_HF_STATE_CONNECTED\n");
        switch(user_input)
        {
        case EXIT:
        case PRINT_MENU:
        case HF_DISCONNECT:
        case HF_PRINT_CONNECTION_DETAILS:
        case AUDIO_PLAYBACK_TEST:
        case AUDIO_RECORD_TEST:
            valid = WICED_TRUE;
            break;
        }
        break;
    case WICED_BT_HFP_HF_STATE_SLC_CONNECTED:
        WICED_BT_TRACE("Current State: WICED_BT_HFP_HF_STATE_SLC_CONNECTED\n");
        switch(user_input)
        {
        case EXIT:
        case PRINT_MENU:
        case HF_DISCONNECT:
        case AVAILABLE_CODEC:
        case HF_PRINT_CONNECTION_DETAILS:
        case ANSWER_CALL:
        case HANGUP_CALL:
        case DIAL_NUM:
        case REDIAL:
        case QUERY_CUR_CALLS:
        case SET_SPK_VOL:
        case SET_MIC_VOL:
        case SUBSCIBER_NUM_INFO:
        case ENABLE_AG_ERROR_CMEE:
        case AUDIO_PLAYBACK_TEST:
        case AUDIO_RECORD_TEST:
            valid = WICED_TRUE;
            break;
        }
        break;
    }
    return valid;
}

/* END OF FILE [] */
