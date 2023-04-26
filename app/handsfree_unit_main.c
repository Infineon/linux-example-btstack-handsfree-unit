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
 * File Name: handsfree_unit_main.c
 *
 * Description: This is the source code for Linux CE Handsfree Unit project.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

/******************************************************************************
*       INCLUDES
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

#include "utils_arg_parser.h"
#include "wiced_memory.h"
#include "platform_linux.h"
#include "wiced_bt_cfg.h"

#include "audio_platform_common.h"
#include "handsfree_unit.h"
#include "handsfree_unit_api.h"
/******************************************************************************
*       MACROS
******************************************************************************/
#define MAX_PATH            (256)
#define BDA_LEN             (6)
#define IP_ADDR_LEN         (16)
#define MAX_PHONE_NUM_LEN   (16)
#define SW_VERSON "[SW_VERSON]v1.0.0"

/******************************************************************************
*       VARIABLES
******************************************************************************/
extern wiced_bt_device_address_t bt_device_address;

static const char app_menu[] = "\n\
------------HFU MENU-----------------------\n\n\
    0.  Exit \n\
    1.  Print Menu \n\
    2.  Set Visibility \n\
    3.  Set Pairing Mode\n\
    4.  Set Inquiry \n\
    5.  HF Connect \n\
    6.  HF Disconnect \n\
    7.  Update Available Codec \n\
    8.  Print HF Connection Details\n\
    9.  Answer Call\n\
    10. Hangup Call\n\
    11. Dial Number\n\
    12. Redial\n\
    13. Query Current Calls\n\
    14. Set Speaker Volume\n\
    15. Set Microphone Volume\n\
    16. Get Subscriber Info\n\
Choose option -> ";

/******************************************************************************
*       FUNCTION DECLARATIONS
******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_buffer, uint32_t length);
void APPLICATION_START(void);

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/******************************************************************************
* Function Name: hci_control_proc_rx_cmd()
*******************************************************************************
* Summary:
*   Function to handle HCI receive
*
* Parameters:
*   uint8_t* p_buffer   : rx buffer
*   uint32_t length     : rx buffer length
*
* Return:
*  status code
*
******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_buffer, uint32_t length)
{
    return 0;
}

/*******************************************************************************
* Function Name: APPLICATION_START()
********************************************************************************
* Summary:
*   BT stack initialization function wrapper
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void APPLICATION_START(void)
{
    handsfree_application_start();
}

/*******************************************************************************
* Function Name: my_atoi()
********************************************************************************
* Summary:
*   check input is number or not
*
* Parameters:
*   numArray: input string
*   value: atoi value
*
* Return:
*   None
*
*******************************************************************************/
void my_atoi(char *numArray, int *value)
{
    int i;
    const int len = strlen(numArray);

    for (i = 0; i < len; i++)
    {
        if (!isdigit(numArray[i])) break;
    }
    *value = len == i ? atoi(numArray) : PRINT_MENU;
}

/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   Application entry function
*
* Parameters:
*   int argc            : argument count
*   char *argv[]        : list of arguments
*
* Return:
*   None
*
*******************************************************************************/
int main(int argc, char* argv[])
{
    int len = 0;
    char patchFile[MAX_PATH];
    char device[MAX_PATH];
    char local_bda[BDA_LEN];
    char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
    uint32_t baud =0 ,patch_baud = 0;
    int spy_inst = 0;
    uint8_t is_socket_tcp = 0;
    char input_choice[3] = {0,0,0};
    unsigned int choice = 0;
    wiced_result_t result = WICED_BT_SUCCESS;
    unsigned int handle;
    uint8_t command = HF_AT_CMD_MAX;

    cybt_controller_autobaud_config_t autobaud;

    /* Parse the arguments */
    memset(patchFile,0,MAX_PATH);
    memset(device,0,MAX_PATH);
    memset(local_bda,0,BDA_LEN);
    if (EXIT_FAILURE == arg_parser_get_args(argc,
                                  argv,
                                  device,
                                  bt_device_address,
                                  &baud,
                                  &spy_inst,
                                  peer_ip_addr,
                                  &is_socket_tcp,
                                  patchFile,
                                  &patch_baud,
                                  &autobaud))
    {
        return EXIT_FAILURE;
    }

    cy_bt_spy_comm_init(is_socket_tcp, spy_inst, peer_ip_addr);
    cy_platform_bluetooth_init(patchFile, device, baud, patch_baud, &autobaud);
    printf("Linux CE Handsfree Unit project initialization complete...\n");

    for(;;)
    {
        printf("%s%s",SW_VERSON, app_menu);
        fflush(stdin);
        if (scanf("%2s", input_choice) == EOF){
            printf( "Enter input cmd fail!!\n");
            choice = PRINT_MENU;
        }
        else{

            my_atoi(input_choice, &choice);
        }

        if(handsfree_is_valid_state(choice))
        {
            switch(choice)
            {
                case EXIT:
                    handsfree_terminate();
                    exit(EXIT_SUCCESS);
                case PRINT_MENU:
                    {
                        printf("%s%s",SW_VERSON, app_menu);
                    }
                    break;
                case SET_VISIBILITY:
                    {
                        unsigned int discoverable;
                        unsigned int connectable;
                        printf("Enter discoverability: 0:Non Discoverable, 1: Discoverable\n");
                        if (scanf("%u", &discoverable) == EOF){
                            printf("Enter discoverability input error!\n");
                            break;
                        }
                        printf("\nEnter connectability: 0:Non Connectable, 1: Connectable\n");
                        if (scanf("%u", &connectable) == EOF){
                            printf("Enter connectability input error!\n");
                            break;
                        }
                        handsfree_handle_set_visibility(discoverable, connectable);
                    }
                    break;
                case SET_PAIRING_MODE:
                    {
                        unsigned int allowed;
                        printf("Enter if pairing is allowed: 0: Not allowed, 1: Allowed\n");
                        if (scanf("%u", &allowed) == EOF){
                            printf("Enter allowed input error!\n");
                            break;
                        }
                        handsfree_handle_set_pairability(allowed);
                    }
                    break;
                case SET_INQUIRY:
                    {
                        unsigned int enable;
                        printf("Enter if Inquiry has to be enabled/disabled: 0: Disabled, 1: Enabled\n");
                        if (scanf("%u", &enable) == EOF){
                            printf("Enter Inquiry input error!\n");
                            break;
                        } 
                        handsfree_inquiry(enable);
                    }
                    break;
                case HF_CONNECT:
                    {
                        BD_ADDR peer_bd_addr;
                        int i = 0;
                        unsigned int read;
                        printf("Enter Peer BD Address:\n");
                        for(i = 0; i < BDA_LEN; i++)
                        {
                            if (scanf("%x", &read) == EOF){
                                printf("Enter BD_ADDR input error!\n");
                                break;
                            }
                            peer_bd_addr[i] = (unsigned char)read;
                        }
                        result = wiced_bt_hfp_hf_connect(peer_bd_addr);
                        printf("HF connect status = %x\n", result);
                    }
                    break;
                case HF_DISCONNECT:
                    {
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle to disconnect: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter disconnect Handle input error!\n");
                            break;
                        }
                        (void) wiced_bt_hfp_hf_disconnect((uint16_t)handle);
                    }
                    break;
                case AVAILABLE_CODEC:
                    {
                        uint8_t p_data[10];
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle:");
                        if( scanf("%x", &handle) == EOF){
                            printf("Enter AVAILABLE_CODEC Handle input error!\n");
                            break;
                        }
                        printf("Enter Available Codecs (CVSD: 1, mSBC: 2) in the format <codec1>,<codec2>: ");
                        if (scanf("%9s", p_data) == EOF){
                            printf("Enter Available Codecs input error!\n");
                            break;
                       }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_BAC, 0, p_data);
                    }
                    break;
               case HF_PRINT_CONNECTION_DETAILS:
                    handsfree_print_hfp_context();
                    break;
                case ANSWER_CALL:
                    {
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle:");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter ANSWER_CALL Handle input error!\n");
                            break;
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_ATA, 0, NULL);
                    }
                    break;
                case HANGUP_CALL:
                    {
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle:");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter HANGUP_CALL Handle input error!\n");
                            break;
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_CHUP, 0, NULL);
                    }
                    break;
                case DIAL_NUM:
                case REDIAL:
                    {
                        uint8_t p_data[MAX_PHONE_NUM_LEN];
                        memset(p_data, 0, strlen(p_data));
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if(scanf("%x", &handle) == EOF){
                            printf("Enter Handle input error!\n");
                            break;
                        }
                        if(DIAL_NUM == choice)
                        {
                            printf("Enter the Number to be Dialed: ");
                            if (scanf("%15s", p_data) == EOF){
                                printf("Enter Number input error!\n");
                                break;
                            }
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_D, 0, p_data);
                    }
                    break;
                case QUERY_CUR_CALLS:
                    {
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter Connection Handle input error!\n");
                            break;
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_CLCC, 0, NULL);
                    }
                    break;
                case SET_SPK_VOL:
                    {
                        int vol = 0;
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter Connection Handle input error!\n");
                            break;
                        }
                        printf("Enter Volume: 0~15:\n");
                        if(scanf("%d", &vol) == EOF){                      
                            printf("Enter Volume input error!\n");
                            break;
                        }
                        else{
                            if ((vol < 0) || (vol > 15)){
                                vol = 8;
                            }
                            printf("Volume input: %d\n", vol);
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_VGS, vol, NULL);
                    }
                    break;
                case SET_MIC_VOL:
                    {
                        int vol = 0;
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter Connection Handle input error!\n");
                            break;
                        }
                        printf("Enter Volume: 0~15\n");
                        if (scanf("%d", &vol) == EOF){
                            printf("Enter Volume input error!\n");
                            break;
                        }
                        else{
                            if ((vol < 0) || (vol > 15)){
                                vol = 8;
                            }
                            printf("Volume input: %d\n", vol);
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_VGM, vol, NULL);
                    }
                    break;
                case SUBSCIBER_NUM_INFO:
                    {
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter Connection Handle input error!\n");
                            break;
                        }
                        handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_CNUM, 0, NULL);
                    }
                    break;
                case ENABLE_AG_ERROR_CMEE:
                    {
                        int enable = 0;
                        handsfree_print_hfp_context();
                        printf("Enter the Connection Handle: ");
                        if (scanf("%x", &handle) == EOF){
                            printf("Enter Connection Handle input error!\n");
                            break;
                        }
                        printf("Enter 1 for enabling, 0 for disabling CMEE: ");
                        if (scanf("%x", &enable) == EOF){
                            printf("Enter ERROR_CMEE input error!\n");
                            break;
                        }
                        if((enable < 0) || (enable > 1))
                        {
                            printf("Invalid arguement\n");
                        }
                        else
                        {
                            handsfree_send_at_command((uint16_t)handle, HF_AT_CMD_CMEE, enable, NULL);
                        }
                    }
                    break;    
                default:
                    printf("Invalid Input\n");
                    break;
            }
        }
        else
        {
            printf("--------------------------------------------\n");
            printf("User Input Disallowed in the Current State\n");
            printf("--------------------------------------------\n");
        }
    }
    return EXIT_SUCCESS;
}
