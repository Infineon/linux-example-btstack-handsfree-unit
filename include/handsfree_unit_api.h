/*
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * File Name: handsfree_unit_api.h
 *
 * Description: This is the header file of handsfree_unit_api for Linux CE Handsfree Unit project.
 *
 * Related Document: See README.md
 *
 *****************************************************************************/

#ifndef __HANDSFREE_UNIT_API_H__
#define __HANDSFREE_UNIT_API_H__

/*******************************************************************************
*      INCLUDES
*******************************************************************************/
#include "wiced_bt_utils.h"


/*******************************************************************************
*       MACROS
********************************************************************************/

/*******************************************************************************
*       STRUCTURES AND ENUMERATIONS
*******************************************************************************/


/*******************************************************************************
*       VARIABLE DEFINITIONS
*******************************************************************************/

/******************************************************************************
*       FUNCTION PROTOTYPES
******************************************************************************/

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
void handsfree_application_start (void);

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
void handsfree_handle_set_visibility (unsigned int discoverability, unsigned int connectability);

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
void handsfree_handle_set_pairability (unsigned int allowed);

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
void handsfree_inquiry (unsigned int enable);

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
void handsfree_send_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data);

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
uint8_t handsfree_is_valid_state (int user_input);

#endif /* __APP_HANDSFREE_UNIT_H__ */
