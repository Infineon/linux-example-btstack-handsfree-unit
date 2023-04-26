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
 * File Name: audio_platform_common.h
 *
 * Description: This file contains the declarations of data types and functions
 * for Linux platform specific audio.
 * 
 * Related Document: See README.md
 *
 *****************************************************************************/

#ifndef AUDIO_PLATFORM_COMMON_H_
#define AUDIO_PLATFORM_COMMON_H_

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>

#include "wiced_memory.h"
#include "alsa/asoundlib.h"
/*******************************************************************************
*       MACROS
********************************************************************************/
#define ALSA_MIC_FRAMES_SIZE     60
#define NBS_SAMPLE_RATE        8000
#define WBS_SAMPLE_RATE       16000
#define MIC_CAPTURE_SAMPLE     8000
#define MIC_CAPTURE_CHANNEL       2

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/
typedef struct PLAYBACK_CONFIG_PARAMS_TAG
{
    int16_t samplingFreq;        /*16k, 32k, 44.1k or 48k*/
    int16_t channelMode;         /*mono, dual, streo or joint streo*/
    int16_t numOfSubBands;       /*4 or 8*/
    int16_t numOfChannels;
    int16_t numOfBlocks;         /*4, 8, 12 or 16*/
    int16_t allocationMethod;    /*loudness or SNR*/
    int16_t bitPool;
} PLAYBACK_CONFIG_PARAMS;

enum
{
    WICED_BT_ALSA_SUCCESS           =   0,
    WICED_BT_ALSA_INIT_PCM_FAIL     =   1,
    WICED_BT_ALSA_INIT_CAP_FAIL     =   2
};

/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/

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
bool init_alsa(PLAYBACK_CONFIG_PARAMS pb_config_params);

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
void deinit_alsa(void);

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
void alsa_write_pcm_data(uint8_t* p_rx_media, uint16_t media_len);

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
uint32_t alsa_capture_mic_data(uint8_t *mic_data, snd_pcm_sframes_t _frames);

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
void alsa_set_volume(uint8_t volume);

/******************************************************************************
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
bool start_alsa_capture(void);

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
bool stop_alsa_capture(void);

#endif /* AUDIO_PLATFORM_COMMON_H_ */
