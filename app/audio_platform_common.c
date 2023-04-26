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
 * File Name: audio_platform_common.c
 *
 * Description: This file contains the definitions of functions related to
 * Linux platform specific audio.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

/*******************************************************************************
*      INCLUDES
*******************************************************************************/

#include "audio_platform_common.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "alsa/asoundlib.h"
#include "sbc_decoder.h"
#include "sbc_dec_func_declare.h"
#include "sbc_dct.h"
#include "sbc_types.h"
#include "wiced_bt_trace.h"

#include "alsa_capture.h"
/*******************************************************************************
*       MACROS
********************************************************************************/
#define MSBC_STATIC_MEM_SIZE      1920 /* BYTES */
#define MSBC_SCRATCH_MEM_SIZE     2048  /* BYTES */
#define ALSA_LATENCY             80000 /* value based on audio playback*/

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/
static SINT32 staticMem[MSBC_STATIC_MEM_SIZE / sizeof (SINT32)];
static SINT32 scratchMem[MSBC_SCRATCH_MEM_SIZE / sizeof (SINT32)];
static char *alsa_playback_device = "default";
static char *alsa_capture_device = "default";
int  PcmBytesPerFrame;
static  SBC_DEC_PARAMS  strDecParams = { 0 };
snd_pcm_t *p_alsa_handle = NULL;
snd_pcm_t *p_alsa_capture_handle = NULL; /* Handle for capturing mic data */
snd_pcm_hw_params_t *params;
snd_pcm_format_t format;
static snd_mixer_elem_t* snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_max;

//capture use
alsa_config_t  ac_config;
alsa_capture_t *ac_device;

/*******************************************************************************
*       FUNCTION DECLARATION
********************************************************************************/
static void alsa_volume_driver_deinit(void);

/*******************************************************************************
*       FUNCTION DEFINITION
*******************************************************************************/

/*****************************************************************************
* function name: alsa_volume_driver_deinit
*******************************************************************************
* summary:
*   de-initializes the alsa volume driver
*
* parameters:
*   none
*
* return:
*   none
*
*****************************************************************************/
static void alsa_volume_driver_deinit(void)
{
    if (snd_mixer_handle != NULL)
    {
        snd_mixer_close(snd_mixer_handle);
        snd_mixer_handle = NULL;
    }
    if (snd_sid != NULL)
    {
        snd_mixer_selem_id_free(snd_sid);
        snd_sid = NULL;
    }
    snd_mixer_elem = NULL;
}

/*****************************************************************************
* Function Name: alsa_volume_driver_init
*******************************************************************************
* Summary:
*   Initializes the ALSA Volume Driver
*
* Parameters:
*   None
*
* Return:
*   None
*
****************************************************************************/
static void alsa_volume_driver_init(void)
{
    long vol_min;
    WICED_BT_TRACE("alsa_volume_driver_init\n");

    alsa_volume_driver_deinit();

    snd_mixer_open(&snd_mixer_handle, 0);
    if (snd_mixer_handle == NULL)
    {
        WICED_BT_TRACE("alsa_volume_driver_init snd_mixer_open Failed\n");
        return;
    }
    snd_mixer_attach(snd_mixer_handle, "default");
    snd_mixer_selem_register(snd_mixer_handle, NULL, NULL);
    snd_mixer_load(snd_mixer_handle);

    snd_mixer_selem_id_malloc(&snd_sid);
    if (snd_sid == NULL)
    {
        alsa_volume_driver_deinit();
        WICED_BT_TRACE("alsa_volume_driver_init snd_mixer_selem_id_alloca Failed\n");
        return;
    }
    snd_mixer_selem_id_set_index(snd_sid, 0);
    snd_mixer_selem_id_set_name(snd_sid, "Master");

    snd_mixer_elem = snd_mixer_find_selem(snd_mixer_handle, snd_sid);

    if (snd_mixer_elem)
    {
        snd_mixer_selem_get_playback_volume_range(snd_mixer_elem, &vol_min, &vol_max);
        WICED_BT_TRACE("min volume %ld max volume %ld\n", vol_min, vol_max);
    }
    else
    {
        alsa_volume_driver_deinit();
        WICED_BT_TRACE("alsa_volume_driver_init snd_mixer_find_selem Failed\n");
    }
}

/*****************************************************************************
* Function Name: alsa_set_volume
*******************************************************************************
* Summary:
*   Sets the required volume level in the ALSA driver
*
* Parameters:
*   Required volume
*
* Return:
*   none
*
****************************************************************************/
void alsa_set_volume(uint8_t volume)
{
    WICED_BT_TRACE("alsa_set_volume volume %d",volume);
    if (snd_mixer_elem)
    {
        snd_mixer_selem_set_playback_volume_all(snd_mixer_elem,  volume * vol_max / 100);
    }
}

/*****************************************************************************
* Function Name: init_alsa
*******************************************************************************
* Summary:
*   Initializes the ALSA driver
*
* Parameters:
*   PLAYBACK_CONFIG_PARAMS pb_config_params : alsa configurations to be set
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool init_alsa(PLAYBACK_CONFIG_PARAMS pb_config_params)
{
    int status;
    int latency = ALSA_LATENCY;
    uint16_t sample_rate = 0;
    
    snd_pcm_uframes_t buffer_size = 0;
    snd_pcm_uframes_t period_size = 0;
    snd_pcm_uframes_t buffer_mic_size = 0;
    snd_pcm_uframes_t period_mic_size = 0;

    memset (staticMem, 0, sizeof (staticMem));
    memset (scratchMem, 0, sizeof (scratchMem));
    memset (&strDecParams, 0, sizeof (strDecParams));

    strDecParams.s32StaticMem  = staticMem;
    strDecParams.s32ScratchMem = scratchMem;

    strDecParams.numOfBlocks = pb_config_params.numOfBlocks;
    strDecParams.numOfChannels = pb_config_params.numOfChannels;
    strDecParams.numOfSubBands =  pb_config_params.numOfSubBands;
    strDecParams.allocationMethod = pb_config_params.allocationMethod;
    sample_rate = pb_config_params.samplingFreq;
    format = SND_PCM_FORMAT_S16_LE; /* SND_PCM_FORMAT_U8; */

    SBC_Decoder_decode_Init (&strDecParams);

    WICED_BT_TRACE("nblocks %d nchannels %d nsubbands %d ameth %d freq %d format %d latency = %d",
                        strDecParams.numOfBlocks , strDecParams.numOfChannels, strDecParams.numOfSubBands,
                        strDecParams.allocationMethod, sample_rate, format, latency);

    PcmBytesPerFrame = strDecParams.numOfBlocks * strDecParams.numOfChannels * strDecParams.numOfSubBands * 2;
    WICED_BT_TRACE("PcmBytesPerFrame = %d\n",PcmBytesPerFrame);

    /* If ALSA PCM driver was already open => close it */
    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE("[DEBUG] p_alsa_handle already one");
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
        return false;
    }
    status = snd_pcm_open(&(p_alsa_handle), alsa_playback_device, SND_PCM_STREAM_PLAYBACK, SND_PCM_ASYNC);  //SND_PCM_ASYNC SND_PCM_NONBLOCK

    if (status < 0)
    {
        WICED_BT_TRACE("[DEBUG]snd_pcm_open failed: %s", snd_strerror(status));
    }
    else
    {
        /* Configure ALSA driver with PCM parameters */
        status = snd_pcm_set_params(p_alsa_handle,
                                    format,
                                    SND_PCM_ACCESS_RW_INTERLEAVED,
                                    strDecParams.numOfChannels,
                                    sample_rate,
                                    1,
                                    latency);

        if (status < 0)
        {
            WICED_BT_TRACE("snd_pcm_set_params failed: %s", snd_strerror(status));
            return false;
        }
        /* In snd_pcm_get_params:
         * buffer_size : PCM ring buffer size in frames
         * period_size: PCM period size in frames
         */
        snd_pcm_get_params(p_alsa_handle, &buffer_size, &period_size);
        WICED_BT_TRACE("snd_pcm_get_params %d ms bs %d ps %d",(latency / 1000), buffer_size, period_size);
    }

    //****************************************************
    // Setup ALSA for capturing mic data

    // Open PCM device for recording (capture).
    status = snd_pcm_open(&(p_alsa_capture_handle), alsa_capture_device, SND_PCM_STREAM_CAPTURE, SND_PCM_ASYNC);
    if (status < 0) {
        WICED_BT_TRACE("unable to open pcm device: %s", snd_strerror(status));
        return false;
    }
    return true;
}

/*****************************************************************************
* Function Name: deinit_alsa
*******************************************************************************
* Summary:
*   De-initializes the ALSA driver
*
* Parameters:
*   None
*
* Return:
*   None
****************************************************************************/
void deinit_alsa(void)
{
    WICED_BT_TRACE("deinit_alsa");
    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE("snd_pcm_close");
        snd_pcm_close(p_alsa_handle);
        p_alsa_handle = NULL;
    }
    alsa_volume_driver_deinit();
}

/*****************************************************************************
* Function Name: alsa_write_pcm_data
*******************************************************************************
* Summary:
*   Writes the supplied PCM frames to ALSA driver
*
* Parameters:
*   p_rx_media: The PCM buffer to be written
*   media_len : Length of p_rx_media data
*
* Return:
*   NONE
*
****************************************************************************/
void alsa_write_pcm_data(uint8_t* p_rx_media, uint16_t media_len)
{
    uint8_t *pOut = p_rx_media;

    snd_pcm_sframes_t alsa_frames = 0;
    snd_pcm_sframes_t alsa_frames_to_send = 0;

    alsa_frames_to_send = media_len / strDecParams.numOfChannels;
    alsa_frames_to_send = alsa_frames_to_send / format; /*Bits per sample is 16 */

    if (p_alsa_handle == NULL)
    {
        WICED_BT_TRACE("ALSA is not configured, dropping the data pkt!!!!");
        return;
    }
#ifdef AUDIO_DEBUG
    WICED_BT_TRACE("alsa_write_pcm_data : Recd a2dp data len %d, alsa_frames_to_send %d\n",
                                                                media_len, alsa_frames_to_send);
#endif
    while(1)
    {
        alsa_frames = snd_pcm_writei(p_alsa_handle, pOut, alsa_frames_to_send);
#ifdef AUDIO_DEBUG
        WICED_BT_TRACE("alsa_frames written = %d\n", alsa_frames);
#endif

        if (alsa_frames < 0)
        {
            alsa_frames = snd_pcm_recover(p_alsa_handle, alsa_frames, 0);
        }
        if (alsa_frames < 0)
        {
            WICED_BT_TRACE("snd_pcm_writei failed %s", snd_strerror(alsa_frames));
            break;
        }
        if (alsa_frames > 0 && alsa_frames < alsa_frames_to_send)
        {
            WICED_BT_TRACE("Short write (expected %li, wrote %li)",
                (long) alsa_frames_to_send, alsa_frames);
        }
        if(alsa_frames == alsa_frames_to_send)
        {
            break;
        }
        pOut += (alsa_frames*format*strDecParams.numOfChannels);
        alsa_frames_to_send = alsa_frames_to_send - alsa_frames;
    }
}


/*****************************************************************************
* Function Name: alsa_capture_mic_data
*******************************************************************************
* Summary:
*   Reads mic data from ALSA driver
*
* Parameters:
*   uint8_t *mic_data : buffer to fill the mic data (Out Parameter)
*
* Return:
*   uint32_t : return the actual bytes read from mic
*
****************************************************************************/
uint32_t alsa_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames){
    int rc = 0;
    rc = alsa_capture_pcm_read(ac_device, mic_data, _frames);
    if (rc < 0) {
        WICED_BT_TRACE("error %d\n", rc);
        return 0;
    }
    else{
        return rc;
    }
}

/*****************************************************************************
* Function Name: start_alsa_capture
*******************************************************************************
* Summary:
*   start to capture audio from alsa 
*
* Parameters:
*   void: none
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool start_alsa_capture(void){
    bool ret;
    WICED_BT_TRACE("%s\n",__func__);
    memset(&ac_config, 0, sizeof(ac_config));
    ac_device = alsa_capture_open(alsa_capture_device);
    if (!ac_device){
        WICED_BT_TRACE("alsa_capture_open fail \n");
        return false;
    }
    
    ac_config.sample_rate = MIC_CAPTURE_SAMPLE;
    ac_config.channels = MIC_CAPTURE_CHANNEL;
    ac_config.format = ALSA_INT16;

    ret = alsa_capture_config_set(ac_device, &ac_config);
    if (ret != true){
        WICED_BT_TRACE("alsa_capture_config_set fail \n");
        return false;
    }
    alsa_capture_start(ac_device);
    return true;
}


/******************************************************************************
* Function Name: stop_alsa_capture
*******************************************************************************
* Summary:
*   Stop to capture audio from alsa 
*
* Parameters:
*   void: none
*
* Return:
*   bool : true: successful, false: failed
*
****************************************************************************/
bool stop_alsa_capture(void){
    WICED_BT_TRACE("%s\n",__func__);
    return alsa_capture_close(&ac_device);
}