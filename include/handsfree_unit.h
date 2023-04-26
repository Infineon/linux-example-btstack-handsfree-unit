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
 * File Name: handsfree_unit.h
 *
 * Description: This is the source code for Linux CE Handsfree Unit project.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

#ifndef __APP_HANDSFREE_H__
#define __APP_HANDSFREE_H__

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include <stdio.h>
#include "wiced_bt_cfg.h"
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_bt_utils.h"


/*******************************************************************************
*       MACROS
********************************************************************************/

//Debug Use
//#define AUDIO_DEBUG
//#define DUMP_SCO_TO_FILE /* Debug SCO from HF AG, dump to file: sco_ouput.raw */
//#define DUMP_MIC_DATA_TO_FILE  /* Debug mic data, dump to file: mic_capture.raw */

//#define SCO_LOOPBACK  //Have issue: do not enable

/*USER INPUT COMMANDS*/
#define EXIT                            (0)
#define PRINT_MENU                      (1)
#define SET_VISIBILITY                  (2)
#define SET_PAIRING_MODE                (3)
#define SET_INQUIRY                     (4)
#define HF_CONNECT                      (5)
#define HF_DISCONNECT                   (6)
#define AVAILABLE_CODEC                 (7)
#define HF_PRINT_CONNECTION_DETAILS     (8)
#define ANSWER_CALL                     (9)
#define HANGUP_CALL                     (10)
#define DIAL_NUM                        (11)
#define REDIAL                          (12)
#define QUERY_CUR_CALLS                 (13)
#define SET_SPK_VOL                     (14)
#define SET_MIC_VOL                     (15)
#define SUBSCIBER_NUM_INFO              (16)
#define ENABLE_AG_ERROR_CMEE            (17)
#define AUDIO_PLAYBACK_TEST             (18)
#define AUDIO_RECORD_TEST               (19)
#define NO_SUPPORT_CMD                  (20)

#define HF_AT_CMD_VGS                       0x00    /* Update speaker volume */
#define HF_AT_CMD_VGM                       0x01    /* Update microphone volume */
#define HF_AT_CMD_ATA                       0x02    /* Answer incoming call */
#define HF_AT_CMD_BINP                      0x03    /* Retrieve number from voice tag */
#define HF_AT_CMD_BVRA                      0x04    /* Enable/Disable voice recognition */
#define HF_AT_CMD_BLDN                      0x05    /* Last Number redial */
#define HF_AT_CMD_CHLD                      0x06    /* Call hold command */
#define HF_AT_CMD_CHUP                      0x07    /* Call hang up command */
#define HF_AT_CMD_CIND                      0x08    /* Read Indicator Status */
#define HF_AT_CMD_CNUM                      0x09    /* Retrieve Subscriber number */
#define HF_AT_CMD_D                         0x0A    /* Place a call using a number or memory dial */
#define HF_AT_CMD_NREC                      0x0B    /* Disable Noise reduction and echo canceling in AG */
#define HF_AT_CMD_VTS                       0x0C    /* Transmit DTMF tone */
#define HF_AT_CMD_BTRH                      0x0D    /* CCAP incoming call hold */
#define HF_AT_CMD_COPS                      0x0E    /* Query operator selection */
#define HF_AT_CMD_CMEE                      0x0F    /* Enable/disable extended AG result codes */
#define HF_AT_CMD_CLCC                      0x10    /* Query list of current calls in AG */
#define HF_AT_CMD_BIA                       0x11    /* Activate/Deactivate indicators */
#define HF_AT_CMD_BIEV                      0x12    /* Send HF indicator value to peer */
#define HF_AT_CMD_BCC                       0x13    /* Initiate Codec Connection */
#define HF_AT_CMD_BCS                       0x14    /* Codec Selection */
#define HF_AT_CMD_BAC                       0x15    /* Updating Available Codec */
#define HF_AT_CMD_MAX                       0x16    /* For Command validation */

/* SDP Record for Hands-Free Unit */
#define HDLR_HANDS_FREE_UNIT                    0x10001
#define HDLR_HEADSET_UNIT                       0x10002
#define HANDS_FREE_SCN                          0x01
#define HEADSET_SCN                             0x02
#define HANDS_FREE_DEVICE_NAME                  "Hands-Free Unit CE"
#define BT_AUDIO_HFP_VOLUME_MIN                 1
#define BT_AUDIO_HFP_VOLUME_MAX                 15
#define TRANS_UART_BUFFER_SIZE                  1024
#define BT_AUDIO_INVALID_SCO_INDEX              0xFFFF
#define HANDSFREE_SINK_NVRAM_ID                 WICED_NVRAM_VSID_START

#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE               145 /* Size of the buffer used for holding the peer device key info */
#define KEY_INFO_POOL_BUFFER_COUNT              10  /* Correspond's to the number of peer devices */
#define DATA_BUFFER_SIZE                        1024

#ifndef BTM_SCO_PKT_TYPES_MASK_HV1
#define BTM_INVALID_SCO_INDEX           0xFFFF
#define BTM_SCO_LINK_ALL_PKT_MASK       0x003F
#define BTM_SCO_LINK_ONLY_MASK          0x0007
#define BTM_SCO_PKT_TYPES_MASK_HV3      0x0004
#define BTM_SCO_PKT_TYPES_MASK_EV3      0x0008
#define BTM_SCO_PKT_TYPES_MASK_EV4      0x0010
#define BTM_SCO_PKT_TYPES_MASK_EV5      0x0020
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200
#define BTM_ESCO_RETRANS_POWER          2       /* For S4 and T2 Retransmission_Effort=2 */
#define BTM_ESCO_RETRANS_QUALITY        2
#endif

#define SCO_CONNECTION_WAIT_TIMEOUT     1000    /* If AG won't trigger sco connection in 1000msec of time, we will initiate SCO connection. */

#define HANDS_FREE_SCO_PKT_TYPES (BTM_SCO_PKT_TYPES_MASK_HV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV4 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV5 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV5)

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define SUPPORTED_FEATURES_ATT            (WICED_BT_HFP_HF_SDP_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_VRECG | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH)

#define BT_AUDIO_HFP_SUPPORTED_FEATURES   (WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL | \
                                           WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                           WICED_BT_HFP_HF_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION)
#else
#define SUPPORTED_FEATURES_ATT            (WICED_BT_HFP_HF_SDP_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_VRECG | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL)

#define BT_AUDIO_HFP_SUPPORTED_FEATURES   (WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL | \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                           WICED_BT_HFP_HF_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_VOICE_RECOGNITION)
#endif

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/
/** HF device indicators. */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_IND     =   1,
    WICED_BT_HFP_HF_CALL_IND        =   2,
    WICED_BT_HFP_HF_CALL_SETUP_IND  =   3,
    WICED_BT_HFP_HF_CALL_HELD_IND   =   4,
    WICED_BT_HFP_HF_SIGNAL_IND      =   5,
    WICED_BT_HFP_HF_ROAM_IND        =   6,
    WICED_BT_HFP_HF_BATTERY_IND     =   7
} wiced_bt_hfp_hf_indicator_t;

typedef struct
{
    wiced_bt_device_address_t               peer_bd_addr;
    wiced_bt_hfp_hf_connection_state_t      connection_status;
    int                                     call_active;
    int                                     call_held;
    wiced_bt_hfp_hf_callsetup_state_t       call_setup;
    wiced_bt_hfp_hf_inband_ring_state_t     inband_ring_status;
    uint8_t                                 mic_volume;
    uint8_t                                 spkr_volume;
    uint16_t                                sco_index;
    uint16_t                                rfcomm_handle;
    wiced_bool_t                            init_sco_conn;
    wiced_bool_t                            is_sco_connected;
    wiced_bt_voice_path_setup_t             sco_voice_path;
} bluetooth_hfp_context_t;


/* data associated with HF_OPEN_EVT */
typedef struct
{
    BD_ADDR             bd_addr;
    uint8_t             status;
} hci_control_hf_open_t;

/* data associated with AT command response event */
typedef struct
{
    uint16_t            num;
    char                str[WICED_BT_HFP_HF_MAX_AT_CMD_LEN];
} hci_control_hf_value_t;

/* data associated with HF_CONNECTED_EVT */
typedef struct
{
    uint32_t           peer_features;
    uint8_t            profile_selected;
} hci_control_hf_connect_t;

/* union of data associated with HS callback */
typedef union
{
    hci_control_hf_open_t    open;
    hci_control_hf_connect_t conn;
    hci_control_hf_value_t   val;
} hci_control_hf_event_t;

typedef struct
{
    uint8_t pairing_allowed;
    hci_control_hf_connect_t connect;
    wiced_timer_t hfp_timer;
} handsfrees_app_globals;

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/
extern const wiced_bt_cfg_settings_t handsfree_cfg_settings;
#ifndef BTSTACK_VER
extern const wiced_bt_cfg_buf_pool_t handsfree_cfg_buf_pools[];
#endif
extern const uint8_t handsfree_sdp_db[];
extern bluetooth_hfp_context_t handsfree_ctxt_data;
extern handsfrees_app_globals handsfree_app_states;
extern wiced_bt_sco_params_t handsfree_esco_params;

/******************************************************************************
*       FUNCTION PROTOTYPES
******************************************************************************/

/* External Function Definitions */
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);

/*******************************************************************************
* Function Name: handsfree_management_callback
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management
*   events from the Bluetooth stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event : BT event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data : Pointer to BT management
*                                                  event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_bt_dev_status_t  handsfree_management_callback
                            (
                                wiced_bt_management_evt_t event,
                                wiced_bt_management_evt_data_t *p_event_data
                            );

/*******************************************************************************
* Function Name: handsfree_inquiry_result_cback
********************************************************************************
* Summary:
*   Callback function called from stack for Inquiry Results
*
* Parameters:
*   wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result : Inquiry results
*   uint8_t *p_eir_data : EIR data
*
* Return:
*
*******************************************************************************/
void handsfree_inquiry_result_cback
                            (
                                wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
                                uint8_t *p_eir_data
                            );

/*******************************************************************************
* Function Name: handsfree_print_hfp_context
********************************************************************************
* Summary:
*   This Function prints the current HFP connection status
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_print_hfp_context(void);


/*******************************************************************************
* Function Name: wait_init_done
********************************************************************************
* Summary:
*   This Function block to wait the BT stack FW download done (notify_init_done)
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void wait_init_done();

/*******************************************************************************
* Function Name: notify_init_done
********************************************************************************
* Summary:
*   This Function notify and release the blocking of wait_init_done() 
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void notify_init_done();

/*******************************************************************************
* Function Name: handsfree_terminate
********************************************************************************
* Summary:
*   This Function should be call when program close 
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_terminate();


#endif /* __APP_HANDSFREE_UNIT_H__ */
