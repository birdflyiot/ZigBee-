/**************************************************************************************************
  Filename:       SampleApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef SAMPLEAPP_H
#define SAMPLEAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

#define SAMPLEAPP_MAX_CLUSTERS       3
//#define SAMPLEAPP_PERIODIC_CLUSTERID 11
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID01 22
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID02 23
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID03 24
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID 12
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID05 25
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID06 26
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID07 27
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID08 28
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID09 29
#define SAMPLEAPP_POINT_TO_POINT_CLUSTERID10 21
#define SAMPLEAPP_POINT_TO_POINT_Pi_CLUSTERID 13
  
#define SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID 1
#define SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID  8
#define SAMPLEAPP_CANCEL_CLUSTERID   9
#define SAMPLEAPP_OK_CLUSTERID       10
#define SAMPLEAPP_FLASH_CLUSTERID    2
#define SAMPLEAPP_COM_CLUSTERID      3

//AI DEFINE  
#define WEBEE_GROUP_CLUSTERID        4
#define WEBEE_GROUP_SWITCH04OFFCLUSTERID  5
#define WEBEE_GROUP_SWITCH04ONCLUSTERID  6
#define WEBEE_GROUP_SWITCH10OFFCLUSTERID  14
#define WEBEE_GROUP_SWITCH10ONCLUSTERID  15
  
#define WEBEE_GROUP_WHOLE1CLUSTERID  7

// Send Message Timeout
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT   20000     // Every 20 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT01   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT02   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT03   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT04   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT05   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT06   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT07   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT08   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT09   29000     // Every 29 seconds
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT10   29000     // Every 29 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT01     0x0003
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT02     0x0006
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT03     0x0007
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT04       0x0004
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT05       0x0008
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT06       0x0009
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT07       0x0010
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT08       0x0011
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT09       0x0012
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT10       0x0005
  
// Group ID for Flash Command
#define SAMPLEAPP_FLASH_GROUP                  0x0001
  
// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION               1000

  #define WEBEE_GROUP 0x0002
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void SampleApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */
