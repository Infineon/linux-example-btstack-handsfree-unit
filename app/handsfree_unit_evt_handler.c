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
 * File Name: handsfree_unit_evt_handler.c
 *
 * Description: This is the source code for the Handsfree Unit Code Example
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "wiced_bt_stack.h"
#include <string.h>
#include <stdlib.h>
#include "wiced_memory.h"
#include "stdio.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "handsfree_unit.h"
#include "wiced_memory.h"
#include "wiced_hal_nvram.h"
#include "audio_platform_common.h" /* ALSA */
#include <pthread.h>
#include <time.h>
/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/
wiced_bt_sco_params_t handsfree_esco_params =
{
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        0x000D,  /* Latency: 13 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( T2 ) */
#else
        0x000C, /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ) */
#endif
        HANDS_FREE_SCO_PKT_TYPES,
        /* Retrans Effort ( At least one retrans, opt for power ) ( S4 ) */
        BTM_ESCO_RETRANS_POWER,
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        WICED_TRUE
#else
        WICED_FALSE
#endif
};

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/
bluetooth_hfp_context_t handsfree_ctxt_data;
handsfrees_app_globals handsfree_app_states;

extern int PcmBytesPerFrame;
extern wiced_bt_device_address_t bt_device_address;
int period_time;

// declaring mutex
pthread_cond_t cond_call_initial = PTHREAD_COND_INITIALIZER;
pthread_mutex_t cond_lock_initial = PTHREAD_MUTEX_INITIALIZER;

#ifdef DUMP_SCO_TO_FILE
FILE *fp;
uint8_t sco_data_copy[DATA_BUFFER_SIZE];
#endif
#ifdef DUMP_MIC_DATA_TO_FILE
FILE *fp_mic;
uint8_t mic_data_copy[DATA_BUFFER_SIZE];
#endif

snd_pcm_uframes_t mic_frames = ALSA_MIC_FRAMES_SIZE;
int buffer_size = ALSA_MIC_FRAMES_SIZE * 4;
uint8_t *mic_data = NULL;
/*******************************************************************************
*       FUNCTION PROTOTYPES
*******************************************************************************/
void sleep_us(unsigned long microseconds);
void handsfree_post_bt_init (wiced_bt_management_evt_data_t *p_event_data);
void handsfree_hfp_init (void);
void handsfree_write_eir (void);
void handsfree_init_context_data (void);
void hf_sco_management_callback
                (
                    wiced_bt_management_evt_t event,
                    wiced_bt_management_evt_data_t *p_event_data
                );

static const char *handsfree_get_indicator_name (wiced_bt_hfp_hf_event_t event);
static void handsfree_call_setup_event_handler
                (
                    wiced_bt_hfp_hf_call_data_t* call_data
                );
static void handsfree_event_callback
                (
                    wiced_bt_hfp_hf_event_t event,
                    wiced_bt_hfp_hf_event_data_t* p_data
                );
static void handsfree_connection_event_handler
                (
                    wiced_bt_hfp_hf_event_data_t* p_data
                );
static void hfp_timer_expiry_handler (TIMER_PARAM_TYPE arg);
static int handsfree_write_nvram (int nvram_id, int data_len, void *p_data);
static int handsfree_read_nvram (int nvram_id, void *p_data, int data_len);
static void handsfree_sco_data_app_callback
                (
                    uint16_t sco_channel,
                    uint16_t length,
                    uint8_t* p_data
                );

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

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
wiced_result_t handsfree_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    int nvram_id;
    int bytes_written, bytes_read;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl;
    uint8_t pairing_result;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    const uint8_t *link_key;

    WICED_BT_TRACE("handsfree_management_callback. Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr((uint8_t *)bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* disable pairing, pairing has to be explicitly enabled by user */
                wiced_bt_set_pairable_mode(0,0);

                handsfree_post_bt_init(p_event_data);
                notify_init_done();
            }
            else
            {
                WICED_BT_TRACE("Bluetooth Enable Failed \n");
            }

            break;
        case BTM_DISABLED_EVT:
            WICED_BT_TRACE("Bluetooth Disabled \n");
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
            hf_sco_management_callback(event, p_event_data);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (handsfree_app_states.pairing_allowed)
            {
                wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            }
            else
            {
                /* Pairing not allowed, return error */
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;

            if (p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE("pairing_result = 0x%x\n",pairing_result);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT\n");

            /* This application supports a single paired host, we can save keys
             * under the same NVRAM ID overwriting previous pairing if any */
            (void)handsfree_write_nvram
                            (
                                HANDSFREE_SINK_NVRAM_ID,
                                sizeof(wiced_bt_device_link_keys_t),
                                &p_event_data->paired_device_link_keys_update
                            );
            link_key = p_event_data->paired_device_link_keys_update.key_data.br_edr_key;
            WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    link_key[0], link_key[1], link_key[2], link_key[3], link_key[4], link_key[5], link_key[6],
                    link_key[7], link_key[8], link_key[9], link_key[10], link_key[11], link_key[12], link_key[13],
                    link_key[14], link_key[15]);
            break;


        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            if (handsfree_read_nvram (
                        HANDSFREE_SINK_NVRAM_ID,
                        &p_event_data->paired_device_link_keys_request,
                        sizeof(wiced_bt_device_link_keys_t)) != 0)
            {
                WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: successfully read\n");
                result = WICED_BT_SUCCESS;
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n",
                                                p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     =
                                                BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            WICED_BT_TRACE("Encryption Status:(%B) res:%d\n",
                    p_encryption_status->bd_addr, p_encryption_status->result);
            break;

        default:
            break;
    }
    UNUSED_VARIABLE(p_encryption_status);
    UNUSED_VARIABLE(bytes_read);
    UNUSED_VARIABLE(bytes_written);
    return result;
}

/*******************************************************************************
* Function Name: handsfree_post_bt_init
********************************************************************************
* Summary:
*   Called after the stack init is success. Sets up EIR data, SDP database and
*   hfp init
*
* Parameters:
*   wiced_bt_management_evt_data_t *p_event_data : Pointer to BT management
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_post_bt_init(wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_ERROR;
    if ((p_event_data) && (p_event_data->enabled.status == WICED_BT_SUCCESS))
    {
        WICED_BT_TRACE("Bluetooth stack initialized\n");

        handsfree_app_states.pairing_allowed = WICED_FALSE;
        wiced_init_timer(&handsfree_app_states.hfp_timer, hfp_timer_expiry_handler, 0,
                        WICED_MILLI_SECONDS_TIMER);

        /* Set-up EIR data */
        handsfree_write_eir();
        /* Set-up SDP database */
        wiced_bt_sdp_db_init((uint8_t *)handsfree_sdp_db, wiced_app_cfg_sdp_record_get_size());

        handsfree_hfp_init();
    }
    else
    {
        WICED_BT_TRACE("Bluetooth stack initialization failure!!\n");
        return;
    }
}

/*******************************************************************************
* Function Name: handsfree_hfp_init
********************************************************************************
* Summary:
*   Configures the HF params, sets up voice data path, configures and
*   initiatizes alsa
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_hfp_init(void)
{
    bool res = 0;
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;
    PLAYBACK_CONFIG_PARAMS pb_config_params;

    handsfree_init_context_data();

    config.feature_mask     = BT_AUDIO_HFP_SUPPORTED_FEATURES;
    config.speaker_volume   = handsfree_ctxt_data.spkr_volume;
    config.mic_volume       = handsfree_ctxt_data.mic_volume;
    config.num_server       = 1;
    config.scn[0]           = HANDS_FREE_SCN;
    config.uuid[0]          = UUID_SERVCLASS_HF_HANDSFREE;

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);

    /* Setup SCO PATH */
    handsfree_ctxt_data.sco_voice_path.path = WICED_BT_SCO_OVER_HCI;
    handsfree_ctxt_data.sco_voice_path.p_sco_data_cb = &handsfree_sco_data_app_callback;
    result = wiced_bt_sco_setup_voice_path(&handsfree_ctxt_data.sco_voice_path);

    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);


#ifdef DUMP_SCO_TO_FILE
    fp = fopen("sco_ouput.raw", "wb");
    if (fp == NULL)
    {
        perror("fopen error: ");
    }
#endif
#ifdef DUMP_MIC_DATA_TO_FILE
    WICED_BT_TRACE("DUMP_MIC_DATA_TO_FILE Enable\n");
    fp_mic = fopen("mic_capture.raw", "wb");
    if (fp_mic == NULL)
    {
        perror("fopen error: ");
    }
#endif
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    WICED_BT_TRACE("%s WBS enabled\n",__func__);
    if (handsfree_esco_params.use_wbs == WICED_TRUE)
    {
        pb_config_params.samplingFreq = WBS_SAMPLE_RATE;
    }
    else
#endif
    {
        pb_config_params.samplingFreq = NBS_SAMPLE_RATE;
    }
    pb_config_params.channelMode = 0; /* Mono */
    pb_config_params.numOfSubBands = 8;
    pb_config_params.numOfChannels = 1;
    pb_config_params.allocationMethod = 0; /* Loudness */
    pb_config_params.bitPool = 26;
    pb_config_params.numOfBlocks = 15;

    /* ALSA INIT */
    res = init_alsa(pb_config_params); //ALSA

    /* assert if alsa pcm init fail */
    assert(res);

    /* init mic data buffer */
    mic_data = (uint8_t *)malloc(buffer_size);
    memset(mic_data, 0, buffer_size);
}

/*******************************************************************************
* Function Name: hfp_timer_expiry_handler
********************************************************************************
* Summary:
*   If sco is not created as an acceptor then this function removes the sco and
*   create it as initiator
*
* Parameters:
*
* Return:
*   NONE
*
*******************************************************************************/
static void hfp_timer_expiry_handler(TIMER_PARAM_TYPE arg)
{
    /* if sco is not created as an acceptor then remove the sco and create it as initiator. */
    if (handsfree_ctxt_data.call_active && !handsfree_ctxt_data.is_sco_connected)
    {
        wiced_bt_sco_remove(handsfree_ctxt_data.sco_index);
        wiced_bt_sco_create_as_initiator(handsfree_ctxt_data.peer_bd_addr, &handsfree_ctxt_data.sco_index,
                                            (wiced_bt_sco_params_t *) &handsfree_esco_params);
    }
}

/*******************************************************************************
* Function Name: handsfree_write_eir
********************************************************************************
* Summary:
*   Sets the EIR data to the stacl
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer (WICED_HS_EIR_BUF_MAX_SIZE);

    if (!pBuf)
    {
        WICED_BT_TRACE("%s: wiced_bt_get_buffer returned NULL\n", __FUNCTION__);
    }
    else
    {
        p = pBuf;

        length = strlen((char *)handsfree_cfg_settings.device_name);

        *p++ = length + 1;
        *p++ = 0x09;            /* EIR type full name */
        memcpy(p, handsfree_cfg_settings.device_name, length);
        p += length;
        *p++ = (1 * 2) + 1;     /* length of services + 1 */
        *p++ =   0x02;            /* EIR type full list of 16 bit service UUIDs */
        *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
        *p++ = (UUID_SERVCLASS_HF_HANDSFREE >> 8) & 0xff;
        *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
        *p++ = (UUID_SERVCLASS_GENERIC_AUDIO >> 8) & 0xff;
        *p++ = 0;

        /* print EIR data */
        WICED_BT_TRACE_ARRAY((uint8_t*)(pBuf+1), MIN(p-(uint8_t*)pBuf,100), "EIR :");
        wiced_bt_dev_write_eir(pBuf, (uint16_t)(p - pBuf));
    }
    return;
}

/*******************************************************************************
* Function Name: handsfree_init_context_data
********************************************************************************
* Summary:
*   initializes the hanffree context data
*
* Parameters:
*   NONE
*
* Return:
*   NONE
*
*******************************************************************************/
void handsfree_init_context_data(void)
{
    handsfree_ctxt_data.call_active         = 0;
    handsfree_ctxt_data.call_held           = 0;
    handsfree_ctxt_data.call_setup          = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
    handsfree_ctxt_data.connection_status   = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    handsfree_ctxt_data.spkr_volume         = 8;
    handsfree_ctxt_data.mic_volume          = 8;
    handsfree_ctxt_data.sco_index           = BT_AUDIO_INVALID_SCO_INDEX;
    handsfree_ctxt_data.init_sco_conn       = WICED_FALSE;
}

/*******************************************************************************
* Function Name: handsfree_event_callback
********************************************************************************
* Summary:
*   Callback from the stack for Handsfree events
*
* Parameters:
*   wiced_bt_hfp_hf_event_t event : handsfree events
*   wiced_bt_hfp_hf_event_data_t* p_data : handsfree event data
*
* Return:
*   NONE
*
*******************************************************************************/
static void handsfree_event_callback(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    hci_control_hf_event_t p_val;
    int res = 0;

    memset(&p_val,0,sizeof(hci_control_hf_event_t));

    switch(event)
    {
        case WICED_BT_HFP_HF_CONNECTION_STATE_EVT:
            handsfree_connection_event_handler(p_data);
            break;
        case WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT:
            p_val.conn.peer_features = p_data->ag_feature_flags;

            if (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY)
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
            }
            else
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
            }
            WICED_BT_TRACE("%s : AG Feature Flags = 0x%X\n", handsfree_get_indicator_name(event), p_data->ag_feature_flags);
            break;

        case WICED_BT_HFP_HF_SERVICE_STATE_EVT:
            WICED_BT_TRACE("%s : Service = %d\n", handsfree_get_indicator_name(event), p_data->service_state);
            break;
        case WICED_BT_HFP_HF_RSSI_IND_EVT:
            WICED_BT_TRACE("%s : RSSI = %d\n", handsfree_get_indicator_name(event), p_data->rssi);
            break;
        case WICED_BT_HFP_HF_SERVICE_TYPE_EVT:
            WICED_BT_TRACE("%s : Service Type = %d\n", handsfree_get_indicator_name(event), p_data->service_type);
            break;
        case WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT:
            WICED_BT_TRACE("%s : Battery Level = %d\n", handsfree_get_indicator_name(event), p_data->battery_level);
            break;
        case WICED_BT_HFP_HF_CALL_SETUP_EVT:
            WICED_BT_TRACE("%s :", handsfree_get_indicator_name(event));
            handsfree_call_setup_event_handler(&p_data->call_data);
            break;
        case WICED_BT_HFP_HF_RING_EVT:
        case WICED_BT_HFP_HF_OK_EVT:
        case WICED_BT_HFP_HF_ERROR_EVT:
            WICED_BT_TRACE("%s\n", handsfree_get_indicator_name(event));
            break;

        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT:
            handsfree_ctxt_data.inband_ring_status = p_data->inband_ring;
            WICED_BT_TRACE("%s : %d\n", handsfree_get_indicator_name(event), p_data->inband_ring);
            break;

        case WICED_BT_HFP_HF_CME_ERROR_EVT:
            WICED_BT_TRACE("%s : Error = %d\n", handsfree_get_indicator_name(event), p_data->error_code);
            break;

        case WICED_BT_HFP_HF_CLIP_IND_EVT:
            WICED_BT_TRACE("%s : Number = %s, Type = %d\n",
                    handsfree_get_indicator_name(event), p_data->clip.caller_num, p_data->clip.type);
            break;

        case WICED_BT_HFP_HF_BINP_EVT:
            WICED_BT_TRACE("%s : Number = %s, Type = %d\n",
                    handsfree_get_indicator_name(event), p_data->binp_data.caller_num, p_data->binp_data.type);
            break;

        case WICED_BT_HFP_HF_VOLUME_CHANGE_EVT:
            WICED_BT_TRACE("%s : %s VOLUME = %d \n", handsfree_get_indicator_name(event),
                    (p_data->volume.type == WICED_BT_HFP_HF_SPEAKER)?"SPK (+VGS)":"MIC (+VGS)",  p_data->volume.level);
            break;

        case WICED_BT_HFP_HFP_CODEC_SET_EVT:
            if (p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC)
            {
                handsfree_esco_params.use_wbs = WICED_TRUE;
            }
            else
            {
                handsfree_esco_params.use_wbs = WICED_FALSE;
            }
            WICED_BT_TRACE("%s : %d \n", handsfree_get_indicator_name(event), p_data->selected_codec);
            break;

        case WICED_BT_HFP_HFP_ACTIVE_CALL_EVT:
            WICED_BT_TRACE("%s : idx = %d, dir = %d, status = %d, mode = %d, is_conference = %d, num = %s, type = %d\n",
                                                            handsfree_get_indicator_name(event),
                                                            p_data->active_call.idx,
                                                            p_data->active_call.dir,
                                                            p_data->active_call.status,
                                                            p_data->active_call.mode,
                                                            p_data->active_call.is_conference,
                                                            p_data->active_call.num,
                                                            p_data->active_call.type);
            break;

        case WICED_BT_HFP_HF_CNUM_EVT:
            WICED_BT_TRACE("%s : cnum data = %s\n", handsfree_get_indicator_name(event), p_data->cnum_data);
            break;

        case WICED_BT_HFP_HF_BIND_EVT:
            WICED_BT_TRACE("%s : id = %d, val = %d\n",
                    handsfree_get_indicator_name(event), p_data->bind_data.ind_id, p_data->bind_data.ind_value);
            break;

        case WICED_BT_HFP_HF_COPS_EVT:
            break;
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: handsfree_connection_event_handler
********************************************************************************
* Summary:
*   Hander funtion for connection events (Connected/Disconnected/SLC connected)
*
* Parameters:
*   wiced_bt_hfp_hf_event_data_t* p_data : Connection Event data
*
* Return:
*
*******************************************************************************/
static void handsfree_connection_event_handler(wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_dev_status_t status;

    if (NULL != p_data)
    {
        handsfree_ctxt_data.connection_status = p_data->conn_data.conn_state;

        if (p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_CONNECTED)
        {
            hci_control_hf_open_t    open;
            wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (p_data->conn_data.remote_address);
            memcpy(open.bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
            open.status = WICED_BT_SUCCESS;
            handsfree_ctxt_data.rfcomm_handle = p_scb->rfcomm_handle;

            if (p_data->conn_data.connected_profile == WICED_BT_HFP_PROFILE)
            {
                handsfree_app_states.connect.profile_selected = WICED_BT_HFP_PROFILE;
            }
            else
            {
                handsfree_app_states.connect.profile_selected = WICED_BT_HSP_PROFILE;
                memcpy
                    (
                        handsfree_ctxt_data.peer_bd_addr,
                        p_data->conn_data.remote_address,
                        sizeof(wiced_bt_device_address_t)
                    );
            }

            status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
            WICED_BT_TRACE("%s: Connection: status [%d] Connection Handle [%x] SCO INDEX [%d] \n",
                                        __func__, status, p_scb->rfcomm_handle, handsfree_ctxt_data.sco_index);
        }
        else if (p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
        {
            WICED_BT_TRACE("%s: SLC Connected : Peer BD Addr [%B]\n", __func__,p_data->conn_data.remote_address);

            memcpy
                (
                    handsfree_ctxt_data.peer_bd_addr,
                    p_data->conn_data.remote_address,
                    sizeof(wiced_bt_device_address_t)
                );
            handsfree_print_hfp_context();
        }
        else if (p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_DISCONNECTED)
        {
            memset(handsfree_ctxt_data.peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
            handsfree_ctxt_data.rfcomm_handle = 0;
            if (handsfree_ctxt_data.sco_index != BT_AUDIO_INVALID_SCO_INDEX)
            {
                status = wiced_bt_sco_remove(handsfree_ctxt_data.sco_index);
                handsfree_ctxt_data.sco_index = BT_AUDIO_INVALID_SCO_INDEX;
                WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
            }
            handsfree_print_hfp_context();

#ifdef DUMP_SCO_TO_FILE
            if (fp) {
                fclose(fp);
            }
#endif
#ifdef DUMP_MIC_DATA_TO_FILE
            if (fp_mic) {
                fclose(fp_mic);
                WICED_BT_TRACE("[DEBUG] alsa_mic Close.\n");
            }

#endif
        }
    }
    else
    {
        WICED_BT_TRACE("%s: p_data is NULL\n", __FUNCTION__);
    }
    UNUSED_VARIABLE(status);
}

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
void handsfree_terminate(){
    WICED_BT_TRACE("%s\n", __func__);
#ifdef DUMP_MIC_DATA_TO_FILE
    if (fp_mic) {
        fclose(fp_mic);
        WICED_BT_TRACE("[DEBUG] alsa_mic Close.\n");
    }

#endif


}

/*******************************************************************************
* Function Name: hf_sco_management_callback
********************************************************************************
* Summary:
*   Callback from Stack for SCO events
*
* Parameters:
*   wiced_bt_management_evt_t event : SCO events
*   wiced_bt_management_evt_data_t *p_event_data : SCO Event data
*
* Return:
*   NONE
*
*******************************************************************************/
void hf_sco_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (handsfree_ctxt_data.peer_bd_addr);
    int status;

    WICED_BT_TRACE("hf_sco_management_callback handsfree_management_callback. Event: 0x%x %s\n",
                                                                event, get_bt_event_name(event));
    switch (event)
    {
        /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
        case BTM_SCO_CONNECTED_EVT:
            WICED_BT_TRACE("%s: SCO Audio connected, sco_index = %d [in context sco index=%d]\n",
                    __func__, p_event_data->sco_connected.sco_index, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_TRUE;
            handsfree_ctxt_data.sco_index = p_event_data->sco_connected.sco_index;
            handsfree_print_hfp_context();

            break;
        /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
        case BTM_SCO_DISCONNECTED_EVT:
            WICED_BT_TRACE("%s: SCO disconnection change event handler\n", __func__);

            status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
            WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_FALSE;
            handsfree_ctxt_data.sco_index = BT_AUDIO_INVALID_SCO_INDEX;
            handsfree_print_hfp_context();
            break;
        /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
        case BTM_SCO_CONNECTION_REQUEST_EVT:
            WICED_BT_TRACE("%s: SCO connection request event handler \n", __func__);

            if (wiced_is_timer_in_use(&handsfree_app_states.hfp_timer))
            {
                wiced_stop_timer(&handsfree_app_states.hfp_timer);
            }
            if (handsfree_app_states.connect.profile_selected == WICED_BT_HFP_PROFILE)
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS,
                                                            (wiced_bt_sco_params_t *) &handsfree_esco_params);
            }
            break;
        /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            WICED_BT_TRACE("%s: SCO connection change event handler\n", __func__);
            break;
    }
    UNUSED_VARIABLE(status);
}

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
void handsfree_inquiry_result_cback(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
                                                                    uint8_t *p_eir_data)
{
    if (p_inquiry_result == NULL)
    {
        WICED_BT_TRACE("Inquiry Complete \n");
    }
    else
    {
        WICED_BT_TRACE("\n--------------------------------------------\n");
        WICED_BT_TRACE("Inquiry Result: %02X %02X %02X %02X %02X %02X\n",
                            p_inquiry_result->remote_bd_addr[0], p_inquiry_result->remote_bd_addr[1],
                            p_inquiry_result->remote_bd_addr[2], p_inquiry_result->remote_bd_addr[3],
                            p_inquiry_result->remote_bd_addr[4], p_inquiry_result->remote_bd_addr[5]);
        WICED_BT_TRACE("Clock Offset = 0x%x\n", p_inquiry_result->clock_offset);
        WICED_BT_TRACE("RSSI = %d\n", p_inquiry_result->rssi);
        WICED_BT_TRACE("--------------------------------------------\n");
    }
}

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
void handsfree_print_hfp_context(void)
{

    WICED_BT_TRACE("\n----------------HF CONNECTION DETAILS----------------------------\n");
    WICED_BT_TRACE("BD ADDRESS \t\t CONNECTION HANDLE \t SCO INDEX\n");
    WICED_BT_TRACE("%02X %02X %02X %02X %02X %02X \t %X \t\t\t %X \n",
                                        handsfree_ctxt_data.peer_bd_addr[0],
                                        handsfree_ctxt_data.peer_bd_addr[1],
                                        handsfree_ctxt_data.peer_bd_addr[2],
                                        handsfree_ctxt_data.peer_bd_addr[3],
                                        handsfree_ctxt_data.peer_bd_addr[4],
                                        handsfree_ctxt_data.peer_bd_addr[5],
                                        handsfree_ctxt_data.rfcomm_handle,
                                        handsfree_ctxt_data.sco_index);
    WICED_BT_TRACE("-----------------------------------------------------------------\n");
}

/*******************************************************************************
* Function Name: handsfree_get_indicator_name
********************************************************************************
* Summary:
*   Prints details about the indicator / Events received from AG
*
* Parameters:
*   wiced_bt_hfp_hf_event_t event : Indicator / Event received from AG
*
* Return:
*   const char *: Returns details about the indicator / Events received from AG
*   as a constant string
*
*******************************************************************************/
static const char *handsfree_get_indicator_name(wiced_bt_hfp_hf_event_t event)
{
    WICED_BT_TRACE("RCVD FROM AG >> ");
    switch(event)
    {
        case WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT: return "+BRSF";
        /*WICED_BT_HFP_HF_SERVICE_STATE_EVT - received for both +CIEV and +CIND*/
        case WICED_BT_HFP_HF_SERVICE_STATE_EVT: return "Service Availability Indicator";
        case WICED_BT_HFP_HF_RSSI_IND_EVT: return "Signal Indicator"; /*received for both +CIEV and +CIND*/
        case WICED_BT_HFP_HF_SERVICE_TYPE_EVT: return "Service Type"; /*received for both +CIEV and +CIND*/
        /*WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT - received for both +CIEV and +CIND*/
        case WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT: return "Battery Charge Indicator";
        case WICED_BT_HFP_HF_CALL_SETUP_EVT: return "+CIEV";
        case WICED_BT_HFP_HF_RING_EVT: return "RING";
        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT: return "+BSIR: Inband Ring Tone Status";
        case WICED_BT_HFP_HF_OK_EVT: return "OK";
        case WICED_BT_HFP_HF_ERROR_EVT: return "AT+ERROR";
        case WICED_BT_HFP_HF_CME_ERROR_EVT: return "+CME";
        case WICED_BT_HFP_HF_CLIP_IND_EVT: return "+CLIP";
        case WICED_BT_HFP_HF_BINP_EVT: return "+BNEP";
        case WICED_BT_HFP_HF_VOLUME_CHANGE_EVT: return "Audio Volume Gain";
        case WICED_BT_HFP_HFP_CODEC_SET_EVT: return "+BCS: Codec Selection";
        case WICED_BT_HFP_HFP_ACTIVE_CALL_EVT: return "+CLCC: Active Call Status";
        case WICED_BT_HFP_HF_CNUM_EVT: return "+CNUM: Subscriber Number Info";
        case WICED_BT_HFP_HF_BIND_EVT: return "+BIND: AG Supported Indicators";
        default: return "Indicator Unknown";
    }
}

/*******************************************************************************
* Function Name: handsfree_call_setup_event_handler
********************************************************************************
* Summary:
*   Prints the Call Status details based on the call setup event received from
*   AG
*
* Parameters:
*   wiced_bt_hfp_hf_call_data_t* call_data : call setup data
*
* Return:
*   NONE
*
*******************************************************************************/
static void handsfree_call_setup_event_handler(wiced_bt_hfp_hf_call_data_t* call_data)
{
    if (NULL != call_data)
    {
        if (handsfree_ctxt_data.call_active != call_data->active_call_present)
        {
            WICED_BT_TRACE("Call Status = %d\n", call_data->active_call_present);
        }
        if (handsfree_ctxt_data.call_held != call_data->held_call_present)
        {
            WICED_BT_TRACE("Call Held = %d\n", call_data->held_call_present);
        }
        if (handsfree_ctxt_data.call_setup != call_data->setup_state)
        {
            WICED_BT_TRACE("Call Setup = %d\n", call_data->setup_state);
        }
        WICED_BT_TRACE("\n-----------CALL SETUP DETAILS------------------\n");
        /* Third party call (Call Held) events are not handled */
        switch (call_data->setup_state)
        {
            case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
                WICED_BT_TRACE("Call(incoming) setting-up\n");
                break;

            case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
                if (call_data->active_call_present == 0)
                {
                    if (handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING ||
                            handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING ||
                            handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING)
                    {
                        WICED_BT_TRACE("Call: Inactive; Call Set-up: IDLE\n");
                        break;
                    }
                    /* If previous context has an active-call and active_call_present is 0 */
                    if (handsfree_ctxt_data.call_active == 1)
                    {
                        WICED_BT_TRACE("Call Terminated\n");
                        stop_alsa_capture();
                        break;
                    }
                }
                /* On call active, 1st we receive +CIEV:<CALL>,1 and later call set up DONE
                 * (+CIEV:<CALLSETUP>,0) is sent */
                else if (call_data->active_call_present == 1)
                {
                    /* Following case if for +CIEV:<CALL>,1 when call is picked up */
                    if (call_data->active_call_present != handsfree_ctxt_data.call_active)
                    {
                        WICED_BT_TRACE("Call: Active\n");
                        handsfree_ctxt_data.call_active = call_data->active_call_present;
                        //Start Capture
                        start_alsa_capture();

                    }
                    /* Following case if for +CIEV:<CALLSETUP>,0 when call set up done is sent */
                    else
                    {
                        WICED_BT_TRACE("Call-setup: DONE\n");
                    }
                }
                break;

            case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
                WICED_BT_TRACE("Call(outgoing) setting-up\n");
                break;

            case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
                WICED_BT_TRACE("Remote(outgoing) ringing\n");
                break;

            default:
                break;
        }
        handsfree_ctxt_data.call_active = call_data->active_call_present;
        handsfree_ctxt_data.call_setup  = call_data->setup_state;
        handsfree_ctxt_data.call_held   = call_data->held_call_present;
        WICED_BT_TRACE("-----------------------------------------------\n");
    }
    else
    {
        WICED_BT_TRACE("%s: call data is NULL\n",__FUNCTION__);
    }
}

/*******************************************************************************
* Function Name: handsfree_write_nvram
********************************************************************************
* Summary:
*   Write NVRAM function is called to store information in the NVRAM.
*
* Parameters:
*   int nvram_id : Volatile Section Identifier. Application can use
*                  the VS ids from WICED_NVRAM_VSID_START to
*                  WICED_NVRAM_VSID_END
*   int data_len : Length of the data to be written to the NVRAM
*   void *p_data : Pointer to the data to be written to the NVRAM
*
* Return:
*   int : number of bytes written, 0 on error
*
*******************************************************************************/
int handsfree_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t result;
    int bytes_written = 0;

    if (NULL != p_data)
    {
        bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    }
    return (bytes_written);
}

/*******************************************************************************
* Function Name:
********************************************************************************
* Summary:
*   Read data from the NVRAM and return in the passed buffer
*
* Parameters:
*   int nvram_id : Volatile Section Identifier. Application can use
*                  the VS ids from WICED_NVRAM_VSID_START to
*                  WICED_NVRAM_VSID_END
*   int data_len : Length of the data to be written to the NVRAM
*   void *p_data : Pointer to the data to be written to the NVRAM

* Return:
*   int : number of bytes written, 0 on error
*
*******************************************************************************/
int handsfree_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t read_bytes = 0;
    wiced_result_t result;

    if ((NULL != p_data) && (data_len >= sizeof(wiced_bt_device_link_keys_t)))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n",
                                        nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result);
    }
    return (read_bytes);
}

/*******************************************************************************
* Function Name: handsfree_sco_data_app_callback
********************************************************************************
* Summary:
*   callback function called for incoming pcm data
*
* Parameters:
*   uint16_t sco_channel : sco channel
*   uint16_t length : SCO data callback length
*   uint8_t* p_data : incoming SCO pcm data
*
* Return:
*   NONE
*
*******************************************************************************/
static void handsfree_sco_data_app_callback(uint16_t sco_channel, uint16_t length, uint8_t* p_data)
{
    uint32_t mic_data_len = 0;
    wiced_result_t result = WICED_ERROR;
#ifdef AUDIO_DEBUG
    WICED_BT_TRACE("sco_data_app_callback-length =  (%d)\n", length);
#endif
    if (length)
    {
#ifdef DUMP_SCO_TO_FILE
        /* You can play the audio file generated (audio_mic.raw) using 
         * the following aplay command:
         * aplay mic_capture.raw -c 1 -f S16_LE -r 8000
         * Replace 8000 with any frequency with which the alsa 
         * is configured
         */
        if (fp){
            memset(sco_data_copy, 0, DATA_BUFFER_SIZE);
            memcpy(sco_data_copy, p_data, length);
            fwrite(sco_data_copy, sizeof(unsigned char), length, fp);
        }
#endif
        alsa_write_pcm_data(p_data, length);
    }
#ifdef SCO_LOOPBACK
    /* loop back sco data */
    if (handsfree_ctxt_data.call_active)
    {
        result = wiced_bt_sco_write_buffer(handsfree_ctxt_data.sco_index, p_data, length);
        if (WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("wiced_bt_sco_write_buffer error, sco_index = %d, result = %d\n", handsfree_ctxt_data.sco_index, result);
        }
    }
#else
    if (handsfree_ctxt_data.call_active && (handsfree_ctxt_data.is_sco_connected == WICED_TRUE)){
        //WICED_BT_TRACE("mic phone-length =  (%d)\n", length);
        mic_data_len = alsa_capture_mic_data(mic_data, length);
        if (mic_data_len > 0){
                //WICED_BT_TRACE("mic get-length =  (%d)\n", mic_data_len); 
                result = wiced_bt_sco_write_buffer(handsfree_ctxt_data.sco_index, mic_data, mic_data_len);
                if (WICED_BT_SUCCESS != result)
                {
                    WICED_BT_TRACE("wiced_bt_sco_write_buffer error, sco_index = %d, result = %d\n", handsfree_ctxt_data.sco_index, result);
                }
    #ifdef DUMP_MIC_DATA_TO_FILE
                memset(mic_data_copy, 0, mic_data_len);
                memcpy(mic_data_copy, mic_data, mic_data_len);
                if (fp_mic){
                    fwrite(mic_data_copy, sizeof(unsigned char), mic_data_len, fp_mic);
                }
    #endif
        }
    }
#endif //SCO_LOOPBACK
}

void sleep_us(unsigned long microseconds)
{
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000ul;            // whole seconds
    ts.tv_nsec = (microseconds % 1000000ul) * 1000;  // remainder, in nanoseconds
    nanosleep(&ts, NULL);
}

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
void wait_init_done(){
    WICED_BT_TRACE("Waiting FW Download");
    pthread_mutex_lock(&cond_lock_initial);
    pthread_cond_wait(&cond_call_initial, &cond_lock_initial);
    pthread_mutex_unlock(&cond_lock_initial);
    WICED_BT_TRACE("wait_init_done[Release...]");
}

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
void notify_init_done(){
    WICED_BT_TRACE("notify_init_done");
    pthread_mutex_lock(&cond_lock_initial);
    pthread_cond_signal(&cond_call_initial);
    pthread_mutex_unlock(&cond_lock_initial);

}


/* END OF FILE [] */
