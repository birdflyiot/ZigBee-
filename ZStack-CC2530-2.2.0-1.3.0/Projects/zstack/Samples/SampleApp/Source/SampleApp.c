/**************************************************************************************************
  选择coordinator协调器时注销gprs系列函数
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

#include "string.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

#include "DHT11.h"  //温湿度传感器头文件


/*LED*/
#define LED1R P1_0
#define LED2Y P1_1

/*直流电机*/
#define A1 P1_2	
#define A2 P1_3

/*KEY*/
#define KEYC P0_4

char Txdata[255];
char SensorData01[6];
char SensorData02[6];
char SensorData03[6];
char SensorData[6];
char SensorData05[6];
char SensorData06[6];
char SensorData07[6];
char SensorData08[6];
char SensorData09[6];
char SensorData10[6];

uint8 Pi[11]; //接收树莓派数据的数组
char PiData[11];

void delay(int xms);
void Delay(int i);

void gprs01(char* SensorDatas);
void gprs02(char* SensorDatas);
void gprs03(char* SensorDatas);
void gprs4(char* SensorDatas);
void gprs05(char* SensorDatas);
void gprs06(char* SensorDatas);
void gprs07(char* SensorDatas);
void gprs08(char* SensorDatas);
void gprs09(char* SensorDatas);
void gprs10(char* SensorDatas);
void gprs4_Pi(char* PiDatas);

void restart(void);
void gprs40(void);
void gprs41(void);
void gprs100(void);
void gprs101(void);
void gprs70(void);
void gprs71(void);
void UartSend_String(char *Data,int len);


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID,
  SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
afAddrType_t Point_To_Point_DstAddr; //点对点通信定义

afAddrType_t Group_DstAddr;//网蜂组播通信定义
afAddrType_t CancelBack_DstAddr;
afAddrType_t OKBack_DstAddr;


aps_Group_t SampleApp_Group;
aps_Group_t WEBEE_Group;  //分组内容

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

uint8 count04 = 0xFF;
uint8 flag04 = 0xFF;

uint8 count10 = 0xFF;

uint8 count07 = 0xFF;
uint8 flag07 = 0xFF;

uint8 count=0xFF;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );

void SampleApp_SendPointToPointMessage01( void );
void SampleApp_SendPointToPointMessage02( void );
void SampleApp_SendPointToPointMessage03( void );
void SampleApp_SendPointToPointMessage04( void );
void SampleApp_SendPointToPointMessage05( void );
void SampleApp_SendPointToPointMessage06( void );
void SampleApp_SendPointToPointMessage07( void );
void SampleApp_SendPointToPointMessage08( void );
void SampleApp_SendPointToPointMessage09( void );
void SampleApp_SendPointToPointMessage10( void );
void SampleApp_SendPointToPointMessage04_Pi( void );

void SampleApp_SendSwitch04backMessage( void );
void SampleApp_SendSwitch10backMessage( void );

void SampleApp_SendCancelbackMessage( void );
void SampleApp_SendOKbackMessage( void );
void SampleApp_SendGroupMessageOK(void); //网蜂组播通讯定义

void SampleApp_SendGroupMessageSWITCH04OFF(void);
void SampleApp_SendGroupMessageSWITCH04ON(void);

void SampleApp_SendGroupMessageSWITCH10OFF(void);
void SampleApp_SendGroupMessageSWITCH10ON(void);

//void SampleApp_SendGroupMessageWHOLE1(void);

void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
 /***********串口初始化************/
  MT_UartInit();//初始化
  MT_UartRegisterTaskID(task_id);//登记任务号
//  HalUARTWrite(0,"Hello World\n",12);
  
  /*DHT11初始化*/
  P0SEL&=0XBF;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  CancelBack_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  CancelBack_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  CancelBack_DstAddr.addr.shortAddr = 0x0000;
  
  OKBack_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  OKBack_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  OKBack_DstAddr.addr.shortAddr = 0x0000;
  
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //(afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0x0000 ; //0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  // 网蜂点对点通讯定义 
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;//点播 
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000; //发给协调器
  
    // 网蜂组播通讯定义
  Group_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  Group_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Group_DstAddr.addr.shortAddr = WEBEE_GROUP;
  

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
  WEBEE_Group.ID = 0x0002;// 组ID
  osal_memcpy( SampleApp_Group.name, "Group 2", 7  );//组名称
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &WEBEE_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        
        case CMD_SERIAL_MSG:  //串口收到数据后由MT_UART层传递过来的数据，编译时不定义MT_TASK，则由MT_UART层直接传递到此应用层
       // 如果是由MT_UART层传过来的数据，则上述例子中29 00 14 31都是普通数据，串口控制时候用的。   
        SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
        break;
        
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
        
        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
//          if ( (SampleApp_NwkState == DEV_ZB_COORD)
//              || (SampleApp_NwkState == DEV_ROUTER)
//              || (SampleApp_NwkState == DEV_END_DEVICE) )
#if defined(coordinator)
          osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
#endif
          
          if(SampleApp_NwkState == DEV_END_DEVICE)
          {
            // Start sending the periodic message in a regular interval. 
#if defined(Sensor01)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT01,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT01 );
#endif
            
#if defined(Sensor02)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT02,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT02 );
#endif
            
#if defined(Sensor03)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT03,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT03 );
#endif
            
#if defined(Sensor04)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT04,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT04 );
#endif
            
#if defined(Sensor05)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT05,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT05 );
#endif
            
#if defined(Sensor06)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT06,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT06 );
#endif
            
#if defined(Sensor07)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT07,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT07 );
#endif
            
#if defined(Sensor08)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT08,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT08 );
#endif
            
#if defined(Sensor09)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT09,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT09 );
#endif
            
#if defined(Sensor10)
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT10,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT10 );
#endif
            

          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    //AT+CIPSTART=\"TCP\",\"14i8247o19.iask.in\",10908
    strcpy(Txdata,"AT+CIPSTART=\"TCP\",\"14i8247o19.iask.in\",29642\r\n");     //将发送内容copy到Txdata;
    UartSend_String(Txdata,46); //串口发送数据
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }  
  
#if defined(Sensor01) 
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT01 )
  {    
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage01();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT01,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT01 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT01);
  }
#endif  
  
#if defined(Sensor02) 
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT02 )
  {    
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage02();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT02,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT02 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT02);
  }
#endif  
  
#if defined(Sensor03) 
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT03 )
  {    
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage03();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT03,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT03 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT03);
  }
#endif  
  
#if defined(Sensor04)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT04 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage04();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT04,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT04 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT04);
  }
#endif  

#if defined(Sensor05)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT05 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage05();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT05,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT05 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT05);
  }
#endif 
  
#if defined(Sensor06)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT06 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage06();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT06,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT06 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT06);
  }
#endif 
  
#if defined(Sensor07)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT07 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage07();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT07,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT07 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT07);
  }
#endif
  
#if defined(Sensor08)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT08 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage08();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT08,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT08 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT08);
  }
#endif 
  
#if defined(Sensor09)
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT09 )
  {
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage09();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT09,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT09 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT09);
  }
#endif 
  
#if defined(Sensor10) 
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT10 )
  {    
    uint8 T[8]; //温度+提示符
    DHT11_TEST(); //温度检测 
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48; 
    T[2]=' '; 
    T[3]=shidu_shi+48; 
    T[4]=shidu_ge+48; 
    T[5]=' '; 
    T[6]=' ';
    T[7]=' '; 

    HalLcdWriteString( T, HAL_LCD_LINE_3 );//LCD显示
    
    SampleApp_SendPointToPointMessage10();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT10,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT10 + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT10);
  }
#endif  
   
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_6 )//KEY OK
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    
  }
  
  if ( keys & HAL_KEY_SW_7 )  //KEY CANCEL
  {
    
  }

  if ( keys & HAL_KEY_SW_5 )  //S5 继电器
  {
    
  }
  
  
  if ( keys & HAL_KEY_SW_4 )  //S4 总开关1
  {
    
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{  
  uint16 flashTime;  
  switch ( pkt->clusterId )
  {    
  case WEBEE_GROUP_CLUSTERID:
    break;

#if defined(Controller4)    
    //继电器04关
  case WEBEE_GROUP_SWITCH04OFFCLUSTERID:  
    //初始化LED和继电器端口
    P1DIR |= 0x07;        
    
    LED1R=1;
    LED2Y=0;
    
    //继电器04
    if(pkt->cmd.Data[0]==4)
    {  
      P1_4=1;
      count04 = 0xFF;    //标志位   
      LED2Y=1;
    }
    SampleApp_SendSwitch04backMessage(); 
    break;
    
    //继电器04开
  case WEBEE_GROUP_SWITCH04ONCLUSTERID:   
    //初始化LED和继电器端口
    P1DIR |= 0x07;        
    
    LED1R=1;
    LED2Y=1;
    
    //继电器04
    if(pkt->cmd.Data[0]==4)
    {     
      P1_4=0;
      count04 = 0x00;    //标志位        
    }
    SampleApp_SendSwitch04backMessage(); 
    
    break;
#endif

#if defined(Controller10)    
     //继电器10关
  case WEBEE_GROUP_SWITCH10OFFCLUSTERID:    
    //初始化LED和继电器端口
    P1DIR |= 0x07;        
    
    LED1R=1;
    LED2Y=0;
    
    //继电器10
    if(pkt->cmd.Data[0]==10)
    {  
      P1_4=1;
      count10 = 0xFF;    //标志位   
      LED2Y=1;
    }
    SampleApp_SendSwitch10backMessage(); 
    break;
    
    //继电器10开
  case WEBEE_GROUP_SWITCH10ONCLUSTERID:     
    //初始化LED和继电器端口
    P1DIR |= 0x07;        
    
    LED1R=1;
    LED2Y=1;
    
    //继电器10
    if(pkt->cmd.Data[0]==10)
    {   
      P1_4=0;
      count10 = 0x00;    //标志位        
    }
    SampleApp_SendSwitch10backMessage(); 
    
    break;
#endif
    
  case WEBEE_GROUP_WHOLE1CLUSTERID:   
    break;
    
  case SAMPLEAPP_OK_CLUSTERID:
    P1DIR |= 0x07;            
    LED2Y=0;
    HalUARTWrite(0,"CON",3);
    HalUARTWrite(0,"\n",1);
    break;
    
  case SAMPLEAPP_CANCEL_CLUSTERID:
    P1DIR |= 0x07;            
    LED1R=0;
    HalUARTWrite(0,"COFF",4);
    HalUARTWrite(0,"\n",1);
    break;
  
  case SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID:
    P1DIR |= 0x07;            
    
    if(pkt->cmd.Data[0]==4)
    { 
      //选择coordinator协调器时注销gprs系列函数
      gprs40();
    }
    
        if(pkt->cmd.Data[0]==10)
    {
      //选择coordinator协调器时注销gprs系列函数
      gprs100();
    }
    
    break;
    
  case SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID:
  P1DIR |= 0x07;   
    
    if(pkt->cmd.Data[0]==4)
    {   
      //选择coordinator协调器时注销gprs系列函数
      gprs41();
    }
    
    if(pkt->cmd.Data[0]==10)
    { 
      //选择coordinator协调器时注销gprs系列函数
      gprs101(); 
    }
    
    break;
    
 
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID01:
   
    SensorData01[0]=pkt->cmd.Data[0];
    SensorData01[1]=pkt->cmd.Data[1];
    SensorData01[2]='#';
    SensorData01[3]=pkt->cmd.Data[2];
    SensorData01[4]=pkt->cmd.Data[3];
    SensorData01[5]='#';
    HalLcdWriteString( SensorData01, HAL_LCD_LINE_2 );
    gprs01(SensorData01);
    break; 
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID02:
   
    SensorData02[0]=pkt->cmd.Data[0];
    SensorData02[1]=pkt->cmd.Data[1];
    SensorData02[2]='#';
    SensorData02[3]=pkt->cmd.Data[2];
    SensorData02[4]=pkt->cmd.Data[3];
    SensorData02[5]='#';
    HalLcdWriteString( SensorData02, HAL_LCD_LINE_2 );
    gprs02(SensorData02);
    break; 
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID03:
   
    SensorData03[0]=pkt->cmd.Data[0];
    SensorData03[1]=pkt->cmd.Data[1];
    SensorData03[2]='#';
    SensorData03[3]=pkt->cmd.Data[2];
    SensorData03[4]=pkt->cmd.Data[3];
    SensorData03[5]='#';
    HalLcdWriteString( SensorData03, HAL_LCD_LINE_2 );
    gprs03(SensorData03);
    break; 
    
     case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
//    /***************湿度打印****************/ 
    
    SensorData[0]=pkt->cmd.Data[0];
    SensorData[1]=pkt->cmd.Data[1];
    SensorData[2]='#';
    SensorData[3]=pkt->cmd.Data[2];
    SensorData[4]=pkt->cmd.Data[3];
    SensorData[5]='#';
    HalLcdWriteString( SensorData, HAL_LCD_LINE_3 );
    gprs4(SensorData);
    break; 
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID05:
   
    SensorData05[0]=pkt->cmd.Data[0];
    SensorData05[1]=pkt->cmd.Data[1];
    SensorData05[2]='#';
    SensorData05[3]=pkt->cmd.Data[2];
    SensorData05[4]=pkt->cmd.Data[3];
    SensorData05[5]='#';
    HalLcdWriteString( SensorData05, HAL_LCD_LINE_2 );
    gprs05(SensorData05);
    break;
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID06:
   
    SensorData06[0]=pkt->cmd.Data[0];
    SensorData06[1]=pkt->cmd.Data[1];
    SensorData06[2]='#';
    SensorData06[3]=pkt->cmd.Data[2];
    SensorData06[4]=pkt->cmd.Data[3];
    SensorData06[5]='#';
    HalLcdWriteString( SensorData06, HAL_LCD_LINE_2 );
    gprs06(SensorData06);
    break;
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID07:
   
    SensorData07[0]=pkt->cmd.Data[0];
    SensorData07[1]=pkt->cmd.Data[1];
    SensorData07[2]='#';
    SensorData07[3]=pkt->cmd.Data[2];
    SensorData07[4]=pkt->cmd.Data[3];
    SensorData07[5]='#';
    HalLcdWriteString( SensorData07, HAL_LCD_LINE_2 );
    gprs07(SensorData07);
    break;
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID08:
   
    SensorData08[0]=pkt->cmd.Data[0];
    SensorData08[1]=pkt->cmd.Data[1];
    SensorData08[2]='#';
    SensorData08[3]=pkt->cmd.Data[2];
    SensorData08[4]=pkt->cmd.Data[3];
    SensorData08[5]='#';
    HalLcdWriteString( SensorData08, HAL_LCD_LINE_2 );
    gprs08(SensorData08);
    break;
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID09:
   
    SensorData09[0]=pkt->cmd.Data[0];
    SensorData09[1]=pkt->cmd.Data[1];
    SensorData09[2]='#';
    SensorData09[3]=pkt->cmd.Data[2];
    SensorData09[4]=pkt->cmd.Data[3];
    SensorData09[5]='#';
    HalLcdWriteString( SensorData09, HAL_LCD_LINE_2 );
    gprs09(SensorData09);
    break;
    
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID10:
   
    SensorData10[0]=pkt->cmd.Data[0];
    SensorData10[1]=pkt->cmd.Data[1];
    SensorData10[2]='#';
    SensorData10[3]=pkt->cmd.Data[2];
    SensorData10[4]=pkt->cmd.Data[3];
    SensorData10[5]='#';
    HalLcdWriteString( SensorData10, HAL_LCD_LINE_2 );
    gprs10(SensorData10);
    break; 
    
  case SAMPLEAPP_POINT_TO_POINT_Pi_CLUSTERID:
    PiData[0]=pkt->cmd.Data[0];
    PiData[1]=pkt->cmd.Data[1];
    PiData[2]=pkt->cmd.Data[2];
    PiData[3]=pkt->cmd.Data[3];
    PiData[4]=pkt->cmd.Data[4];
    PiData[5]=pkt->cmd.Data[5];
    PiData[6]=pkt->cmd.Data[6];
    PiData[7]=pkt->cmd.Data[7];
    PiData[8]=pkt->cmd.Data[8];
    PiData[9]=pkt->cmd.Data[9];
    PiData[10]=pkt->cmd.Data[10];
    
    HalLcdWriteString( PiData, HAL_LCD_LINE_3 );//LCD显示
    
    gprs4_Pi(PiData);
    
    break;
    
//  case SAMPLEAPP_PERIODIC_CLUSTERID:
//  break;  
  
  case SAMPLEAPP_FLASH_CLUSTERID:
    flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
    HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
    break;
  }
}

void SampleApp_SendGroupMessageOK( void )
{
  uint8 data[2]={0,1};
  if ( AF_DataRequest( &Group_DstAddr,
                      &SampleApp_epDesc,
                      WEBEE_GROUP_CLUSTERID,
                      2,
                      data,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }

}

void SampleApp_SendGroupMessageSWITCH04OFF( void )
{
  uint8 data[1]={4};
  if ( AF_DataRequest( &Group_DstAddr,
                      &SampleApp_epDesc,
                      WEBEE_GROUP_SWITCH04OFFCLUSTERID,
                      1,
                      data,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }

}

void SampleApp_SendGroupMessageSWITCH04ON( void )
{
  uint8 data[1]={4};  
  if ( AF_DataRequest( &Group_DstAddr,
                       &SampleApp_epDesc,
                       WEBEE_GROUP_SWITCH04ONCLUSTERID,
                       1,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_SendGroupMessageSWITCH10OFF( void )
{
  uint8 data[1]={10};
  if ( AF_DataRequest( &Group_DstAddr,
                      &SampleApp_epDesc,
                      WEBEE_GROUP_SWITCH10OFFCLUSTERID,
                      1,
                      data,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }

}

void SampleApp_SendGroupMessageSWITCH10ON( void )
{
  uint8 data[1]={10};  
  if ( AF_DataRequest( &Group_DstAddr,
                      &SampleApp_epDesc,
                      WEBEE_GROUP_SWITCH10ONCLUSTERID,
                      1,
                      data,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_SendOKbackMessage( void )
{
  uint8 data[1]={0};
    if ( AF_DataRequest(&OKBack_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_OK_CLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
}

void SampleApp_SendCancelbackMessage( void )
{
  if(count07 == 0)
  {
    uint8 data[1]={7};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
  else
  {
    uint8 data[1]={7};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
}

void SampleApp_SendSwitch04backMessage( void )
{
  if(count04==0)
  {
    uint8 data[1]={4};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
  else
  {
    uint8 data[1]={4};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
}

void SampleApp_SendSwitch10backMessage( void )
{
  if(count10==0)
  {
    uint8 data[1]={10};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHONCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
  else
  {
    uint8 data[1]={10};
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, 
                        &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_SWITCHOFFCLUSTERID,
                        1,
                        data,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
}

void Delay(int i )
{
  int k;
  for(k=0;k<i;k++)
  {
    delay(30000);
  }
}

void delay(int xms)
{
  int i,j;
  for(i=xms;i>0;i--)
    for(j=587;j>0;j--);
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage01
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage01( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID01,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage02
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage02( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID02,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage03
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage03( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID03,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage04
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage04( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage05
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage05( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID05,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage06
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage06( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID06,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage07
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage07( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID07,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage08
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage08( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID08,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage09
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage09( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID09,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage10
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage10( void ) 
{ 
  //  flag=0x0A;
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                      &SampleApp_epDesc, 
                      SAMPLEAPP_POINT_TO_POINT_CLUSTERID10,
                      4, 
                      T_H, 
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  { 
  } 
  else 
  { 
    // Error occurred in request to send. 
  } 
}

/*********************************************************************
 * @fn      SampleApp_SendPointToPointMessage04_Pi
 *
 * @brief   Send the Point To Point message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPointToPointMessage04_Pi( void )
{
   if ( AF_DataRequest( &Point_To_Point_DstAddr,
                          &SampleApp_epDesc, 
                          SAMPLEAPP_POINT_TO_POINT_Pi_CLUSTERID,
                          11, 
                          Pi, 
                          &SampleApp_TransID,
                          AF_DISCV_ROUTE,
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      { 
      } 
      else 
      { 
        // Error occurred in request to send. 
      } 
}
  
/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{

}



void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)//发送 FE 02 01 F1  ,则返回01 F1
{
  uint8 i,len,*str=NULL;
//  uint8 count=0;
  str=cmdMsg->msg;
  len=*str; //msg里的第1个字节代表后面的数据长度
  
  
  for(i=1;i<=len;i++)
  { 
    /*发送注册信息*/
    if((*(str+i)=='C')&&(*(str+i+1)=='O'))
    {
      
      //AT+CIPSEND
      strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
      UartSend_String(Txdata,12); //串口发送数据
      Delay(15);                  //延时
      
      //8
      strcpy(Txdata,"8\x1A");     //将发送内容copy到Txdata;
      UartSend_String(Txdata,2); //串口发送数据        

      break;      
    }   
  
    /*发送传感器当前值*/
    
    /*接收指令*/
    //开
    //if((*(str+i)=='4')&&(*(str+i+1)=='1'))
    if((*(str+i)=='0')&&(*(str+i+1)=='4')&&(*(str+i+2)=='1'))
    {
      SampleApp_SendGroupMessageSWITCH04ON();
      break;
    }
  
    //关
    //if((*(str+i)=='4')&&(*(str+i+1)=='0'))
    if((*(str+i)=='0')&&(*(str+i+1)=='4')&&(*(str+i+2)=='0'))
    {
      SampleApp_SendGroupMessageSWITCH04OFF();
      break;
    }  
    
    /*接收指令*/
    //开
    if((*(str+i)=='1')&&(*(str+i+1)=='0')&&(*(str+i+2)=='1'))
    {
      SampleApp_SendGroupMessageSWITCH10ON();
      break;
    }
  
    //关
    if((*(str+i)=='1')&&(*(str+i+1)=='0')&&(*(str+i+2)=='0'))
    {
      SampleApp_SendGroupMessageSWITCH10OFF();
      break;
    }  
    
    /*非摄像头连接点注释掉*/
    if((*(str+i)=='#')&&(*(str+i+5)=='#')&&(*(str+i+8)=='#'))
    {       
      Pi[0]=*(str+1);
      Pi[1]=*(str+2);
      Pi[2]=*(str+3);
      Pi[3]=*(str+4);
      Pi[4]=*(str+5);
      Pi[5]=*(str+6);
      Pi[6]=*(str+7);
      Pi[7]=*(str+8);
      Pi[8]=*(str+9);
      Pi[9]=*(str+10);
      Pi[10]=*(str+11);  

      HalLcdWriteString( Pi, HAL_LCD_LINE_3 );//LCD显示            
      SampleApp_SendPointToPointMessage04_Pi();    
     
      break;
    }  
  }
}

void UartSend_String(char *Data,int len)
{
  int j=0;

  while(j<len)
  {
    j++;
    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

void restart()
{
  strcpy(Txdata,"At+CIPCLOSE\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,13); //串口发送数据
  Delay(15); 
  
}

void gprs01(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"01#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs02(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"02#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs03(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"03#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

//发送数据格式 Controller编号#温度#DO#PH#待定
void gprs4(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //04#温度#湿度#待定 
  strcpy(Txdata,"04#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs05(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"05#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs06(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"06#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs07(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"07#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs08(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"08#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs09(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"09#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs10(char* SensorDatas)
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //10#温度#湿度#待定 
  strcpy(Txdata,"10#");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3);
  
  strcpy(Txdata,SensorDatas);     //将发送内容copy到Txdata;  
  UartSend_String(Txdata,6); //串口发送数据
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15);   
}

void gprs4_Pi(char* PiDatas)
{
   //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
   
  //04#人数#人流方向
  strcpy(Txdata,PiDatas);     //将发送内容copy到Txdata;
  UartSend_String(Txdata,11);
  
  //CTRL+Z
  strcpy(Txdata,"\x1A\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,3); //串口发送数据
  Delay(15); 
}

void gprs40()
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
  
  //0
  strcpy(Txdata,"0\x1A");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,2); //串口发送数据
  Delay(120); 
}

void gprs41()
{
   //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
  
  //1
  strcpy(Txdata,"1\x1A");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,2); //串口发送数据
  Delay(120); 
}

void gprs100()
{  
  //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
  
  //0
  strcpy(Txdata,"0\x1A");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,2); //串口发送数据
  Delay(120); 
}

void gprs101()
{
   //AT+CIPSEND
  strcpy(Txdata,"AT+CIPSEND\r\n");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,12); //串口发送数据
  Delay(15);                  //延时
  
  //1
  strcpy(Txdata,"1\x1A");     //将发送内容copy到Txdata;
  UartSend_String(Txdata,2); //串口发送数据
  Delay(120); 
}

/*********************************************************************
*********************************************************************/  