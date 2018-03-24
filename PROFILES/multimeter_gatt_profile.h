/******************************************************************************

 @file  multimeter_gatt_profile.h

 @brief This file contains the Multimeter GATT profile definitions and prototypes
        prototypes.

 Group: WCS, BTS
 Target Device: CC1350

 ******************************************************************************

 Copyright (c) 2010-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ti-ble-2.3.2-stack-sdk_1_50_xx
 Release Date: 2017-09-27 14:52:17
 *****************************************************************************/

#ifndef MULTIMETERGATTPROFILE_H
#define MULTIMETERGATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define MULTIMETERPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value
#define MULTIMETERPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value

// Multimeter Service UUID
#define MULTIMETER_SERV_UUID               0xFFF0

// Key Pressed UUID
#define MULTIMETERPROFILE_CHAR1_UUID            0xFFF1
#define MULTIMETERPROFILE_CHAR4_UUID            0xFFF4

// Multimeter Keys Profile Services bit fields
#define MULTIMETER_SERVICE               0x00000001

// Length of Characteristic 4 in bytes
#define MULTIMETERPROFILE_CHAR4_LEN           4

/*********************************************************************
 * TYPEDEFS
 */

/*!
 *  @def    MultimeterMode
 *  @brief  Enum of Multimeter working modes
 */
typedef enum MultimeterMode {
    MultimeterMode_Off = 0,
    MultimeterMode_3V,
    MultimeterMode_10V,
    MultimeterMode_500mA,
    MultimeterMode_Ohm
} MultimeterMode;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*multimeterProfileChange_t)( uint8 paramID );

typedef struct
{
  multimeterProfileChange_t        pfnMultimeterProfileChange;  // Called when characteristic value changes
} multimeterProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * MultimeterProfile_AddService- Initializes the Multimeter GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t MultimeterProfile_AddService( uint32 services );

/*
 * MultimeterProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t MultimeterProfile_RegisterAppCBs( multimeterProfileCBs_t *appCallbacks );

/*
 * MultimeterProfile_SetParameter - Set a Multimeter GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MultimeterProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * MultimeterProfile_GetParameter - Get a Multimeter GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MultimeterProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MULTIMETERGATTPROFILE_H */
