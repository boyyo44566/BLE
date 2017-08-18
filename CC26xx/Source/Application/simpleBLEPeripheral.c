/**************************************************************************************************
30 5A 26 01 58 01 01 00 /7A
20 00 00 00 26 05 00 00 00 58 15 26 20 58 20 07
30 5A 61 95 27 31 00 7A
53 53 01 01 00 00 00 01 01 09
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2015-07-13 11:43:11 -0700 (Mon, 13 Jul 2015) $
  Revision:       $Revision: 44336 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2650 Bluetooth Low Energy Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/BIOS.h>
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"
#include "math.h"
#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "simpleBLEPeripheral.h"

#include <ti/drivers/lcd/LCDDogm1286.h>
#include <ti/drivers/UART.h>

//UART M0
#include "scif.h"

//PWM
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Log.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/PWM2.h>

//®É¶¡
#include <time.h>
#include <ti/sysbios/hal/Seconds.h>
#include <UTC_Clock.h>
#include <GUA_RTC.h>
/*********************************************************************
 * CONSTANTS
*/

//PWM
#define PERIOD_US 1000 

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160//20msec
 
// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          20

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               100
#define NOTI_PERIOD                                          5000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

#define SNV_IDX_DEVICENAME 0x82

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                0x0001
#define SBP_CHAR_CHANGE_EVT                 0x0002
#define SBP_CONN_EVT_END_EVT                0x0004
#define SBP_PERIODIC_EVT                           0x0008
#define SBP_NOTIFY                                       0x0010
#define CLEAR_EVT                                        0x0020
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;


/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// GAP GATT Attributes  ³]³Æ¦W¦r
char attDeviceName[GAP_DEVICE_NAME_LEN] = "Wistron_B_Node";//¹w³]ªºdevice name

//notify
uint16 gapConnHandle;

//PWM
PWM_Handle hPWM;

//®É¶¡
int systimelink_now[4];
int systimelink_over[4];
uint16 mins;
int systime[4];
int time_link_now,time_link_over,timeAll,time_now;
UTCTimeStruct GUA_Timer;
uint8 bBuf[20] = {0};     
uint8 *p = bBuf;
uint8 sysovertime[3];
bool sys_F=0;//¦P¨BºX¼Ð  0=¥¼¶}©l

bool itself=0;//uart ¦Û¤v§_

int tx=0;
//UART
uint8 rxbuf[100]={0};
bool dontsave=0;
bool dontsave_1=0;
bool repeat_flag=0;
bool self_falg=0;
bool linked = 0;
//¼s¼½buffer

typedef struct 
{
  uint8 devicecount;            //¼s¼½¬í¼Æ
  bool data_flag;               //¬O§_¦³¸ê®Æ
  bool CT_flag;                // ±±¿O 1 = ¦Û¤v
  char data[31];             //data   
  bool self;
}devicedata; 

char txbuf[31]={0};
bool anss;
char tsss[31];
static devicedata device[36];

static uint8 newValue3[20];       //¦¬¨ì¼s¼½¤§¦ì¸m
int  flash[20]={0};//¦Û²`¦a§}+²×ÂI


//¤T¨¤©w¦ì
int Pr0;
int  E0_1;
int  E0_2;

//±±¿O
uint8 hoppinglight[40]; 
int controltime=0;


uint8 uartover[31]={0};
bool OAD_FLAG = 0;

///////////////¸g½n«×
uint8_t MyLight_latSum_1,MyLight_latSum_2,MyLight_latSum_3,MyLight_latSum_4,MyLight_latSum_5;
uint8_t MyLight_lonSum_1,MyLight_lonSum_2,MyLight_lonSum_3,MyLight_lonSum_4,MyLight_lonSum_5;
int Final_latSum;
int Final_lonSum;
int MyLight_latSum;
int MyLight_lonSum;
double longitude;      //¸g«×
double latitude;        //½n«×
uint8 MyLight_latSum_printf_lat_H_1,MyLight_latSum_printf_lat_H_2,MyLight_latSum_printf_lat_H;
uint8 MyLight_latSum_printf_lat_L_1,MyLight_latSum_printf_lat_L_2,MyLight_latSum_printf_lat_L,MyLight_lonSum_printf_lon_H,MyLight_lonSum_printf_lon_L;
bool starting_point=0; //¼g¸g½n«×ºX¼Ð
bool end=0;//¼g²×ÂIºX¼Ð

char DeviceName[10]={0};

uint8_t  addrbuf[31]={0};//¸g½n
uint8_t  addrbufack[31]={0};//¦^¶Ç¸g½n
 
//MAC
uint8_t nGUA_Address[6];
char MAC_Address[8];



int jzx=0;//Åª¨úµ§¼Æ
int once=0;//¦³¨S¦³¦¬¨ìrxbuf
int order=0;//¼s¼½¶¶§Ç
bool scanp = 1;//¤Á´«¼s¼½data
//UART



// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct SAVEClock;
//static Clock_Struct CLEARClock;
//static Clock_Struct NOTI;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

static PIN_Handle hSbpPins;


//PWM
char        PWM_taskStack[512];
Task_Struct PWM_taskStruct;

uint16 handle  ;

//delay
static void delaysec(int sec_use);
uint32_t sec;
time_t t;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

//devicename
uint8_t set_snv_devicename(void) //device name length is define as 21
{
  char attDeviceName_change[7] = "W_Node";//¹w³]ªºdevice name
  
  osal_snv_read(0x81,sizeof(DeviceName),DeviceName);
  
  if(   DeviceName[7]!=    0xFF    ||      DeviceName[8]!=    0xFF    ||    DeviceName[9]!=    0xFF      )
  {
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, DeviceName);
    return (FALSE);
  }
  else
  {
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName_change);
    return (FALSE); 
  }
  

  /*
       if(osal_snv_read(SNV_IDX_DEVICENAME,sizeof(uint8_t)*GAP_DEVICE_NAME_LEN, attDeviceName) == ICALL_INVALID_ENTITY_ID)//it means the there isn't any device name on snv
       {
           //set thedevice name as default setting.
           GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
           return (FALSE);
       }
       else
       {
           GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
           return (TRUE);
       }*/
}

//SCS_Uart_banana///////////////////////////////////////////////////////////////
Task_Struct my_UART_Task;
Char my_UART_TaskStack[512];
static Semaphore_Struct semScTaskAlert;

void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback


//UART inhere
void scTaskAlertCallback(void) {

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));
    
    // Wait for an ALERT callback
    Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    // Echo all characters currently in the RX FIFO
    int rxFifoCount = scifUartGetRxFifoCount();
    
    //Åª¥X³o¦¸buffer
    for(int a=0;a<rxFifoCount;a++)
     rxbuf[a] =(      (char) scifUartRxGetChar()      );

    
    // Clear the events that triggered this
    scifUartClearEvents();

    // Acknowledge the alert event
    scifAckAlertEvents();    
    ////////////////////////////////////Åª
    
    if(rxbuf[0]=='$')
    {
      //§PÂ_data¬O§_­«½Æ
      for( int a=0; a<36; a++)
      {
        //¤âÀô§PÂ_1~24bytes¬O§_¤@¼Ë  ¿é¥X1->¤@¼Ë
        if(
            rxbuf[0] == device[a].data[0] &&
            rxbuf[1] == device[a].data[1] &&
            rxbuf[2] == device[a].data[2] &&
            rxbuf[3] == device[a].data[3] &&              
            rxbuf[4] == device[a].data[4] &&
            rxbuf[5] == device[a].data[5] &&           
            rxbuf[6] == device[a].data[6] &&            
            rxbuf[7] == device[a].data[7] &&           
            rxbuf[8] == device[a].data[8] &&            
            rxbuf[9] == device[a].data[9] &&                 
            rxbuf[10] == device[a].data[10] &&                  
            rxbuf[11] == device[a].data[11] &&            
            rxbuf[12] == device[a].data[12] &&              
            rxbuf[13] == device[a].data[13] &&               
            rxbuf[14] == device[a].data[14] &&                 
            rxbuf[15] == device[a].data[15] &&              
            rxbuf[16] == device[a].data[16] &&               
            rxbuf[17] == device[a].data[17] &&                 
            rxbuf[18] == device[a].data[18] &&              
            rxbuf[19] == device[a].data[19] &&               
            rxbuf[20] == device[a].data[20] &&                             
            rxbuf[21] == device[a].data[21] &&                
            rxbuf[22] == device[a].data[22] &&                 
            rxbuf[23] == device[a].data[23] &&                 
            rxbuf[24] == device[a].data[24] &&                 
            rxbuf[25] == device[a].data[25] &&    
            rxbuf[26] == device[a].data[26] &&                
            rxbuf[27] == device[a].data[27] &&                 
            rxbuf[28] == device[a].data[28] &&                  
            rxbuf[29] == device[a].data[29] &&    
            rxbuf[30] == device[a].data[30]   
           )
        {
          repeat_flag = 1;//­«½Æ¸õ¥X
          break;
        }
      }
      
      
      if( repeat_flag == 0)
      {
          //¼s¼½buffer ¦s¨úÅª¨ìªº­È
          for(int z=0;z<31;z++)
          {
            device[jzx].data[z]=rxbuf[z];
          }
                       
          device[jzx].data_flag=1;//¦³¸ê®Æ              
          //¶ñ¼g¦Û¤v²×ÂI

          jzx++;//·suartµ§¼Æ¼W¥[
          if(jzx>35)
            jzx=0;
      }
    }
    
    repeat_flag = 0;
    
    
    //¦s·s¸ê®Æ
    if(   (rxbuf[0]=='P')&&(rxbuf[30]==0x07)
          ||(rxbuf[0]=='T')&&(rxbuf[30]==0x07)
          ||(rxbuf[0]=='R')&&(rxbuf[30]==0x07)   
          ||(rxbuf[0]==0xE0)&&(rxbuf[30]==0x07)
          ||(rxbuf[0]=='C')&&(rxbuf[1]=='T') )
      {
        //ÀË¬d¤£¤@¼Ë¤~¦s
        for(int d=0;d<36;d++)
        {
        if( (rxbuf[07] == device[d].data[07])       &&
             (rxbuf[10] == device[d].data[10])       && 
             (rxbuf[11] == device[d].data[11])       &&                            
             (rxbuf[12] == device[d].data[12])       &&                            
             (rxbuf[13] == device[d].data[13])       )
          {
            //¤£¦sºX¼Ð
            dontsave=1;
            break;
          }
        }
        
        
        if(dontsave==0)
        {
        //±½´y§PÂ_¬O§_¦Û¤v  «G·À¿O+¼s¼½
          if( 
              rxbuf[10] == MyLight_lonSum_printf_lon_H  &&
              rxbuf[11] == MyLight_lonSum_printf_lon_L  &&
              rxbuf[12] == MyLight_latSum_printf_lat_H  &&
              rxbuf[13] == MyLight_latSum_printf_lat_L  
             )//¬O¦Û¤v
          {
            itself=1;
            device[jzx].CT_flag=1;
            
            //¼s¼½buffer ¦s¨úÅª¨ìªº­È
            memcpy(device[jzx].data,rxbuf,31);
                        
            device[jzx].data_flag=1;//¦³¸ê®Æ              
            device[jzx].CT_flag = 1;

            jzx++;//·suartµ§¼Æ¼W¥[   
            
            if(jzx>=35)
              jzx=0;   
            
            if(rxbuf[23] == 0x01)//«G¿O
            {
              PIN_setOutputValue(hSbpPins, Board_LED6, 1);   
            }
            else//·À¿O
            {
              PIN_setOutputValue(hSbpPins, Board_LED6, 0);  
            }
        }
      }
      if(itself==0)//«D¦Û¤v
      {
        //ÀË¬d¤£¤@¼Ë¤~¦s
        for(int d=0;d<36;d++)
        {
          if(       (rxbuf[07] == device[d].data[07])       &&
                     (rxbuf[10] == device[d].data[10])       && 
                     (rxbuf[11] == device[d].data[11])       &&                            
                     (rxbuf[12] == device[d].data[12])       &&                            
                     (rxbuf[13] == device[d].data[13])       )
          {
            //¤£¦sºX¼Ð
            dontsave=1;
            break;
          }
        }
        
        if(dontsave==0)//save
        {
          //¼s¼½buffer ¦s¨úÅª¨ìªº­È
          memcpy(device[jzx].data,rxbuf,31);
                   
          device[jzx].data_flag=1;//¦³¸ê®Æ              
          device[jzx].CT_flag = 1;
          
          jzx++;//·suartµ§¼Æ¼W¥[
          
          if(jzx>=35)
            jzx=0;
          
        }      
      }
     }
    
    ///////////////////////////////////¦¬¨ì§R°£DATA
    /*
    if(rxbuf[0]==0xAC)
    {
      for(int u=0;u<36;u++)
      {
        if(
           (device[u].data[1]    ==      rxbuf[1])  &&
           (device[u].data[2]    ==      rxbuf[2])  &&
           (device[u].data[3]    ==      rxbuf[3])  &&
           (device[u].data[4]    ==      rxbuf[4])  &&
           (device[u].data[5]    ==      rxbuf[5])  &&
           (device[u].data[6]    ==      rxbuf[6])  &&  
           (device[u].data[7]    ==      rxbuf[7])  &&  
           (device[u].data[8]    ==      rxbuf[8])  &&  
           (device[u].data[9]    ==      rxbuf[9])  &&  
           (device[u].data[10]    ==      rxbuf[10])  &&  
           (device[u].data[11]    ==      rxbuf[11])  &&  
           (device[u].data[12]    ==      rxbuf[12])  &&  
           (device[u].data[13]    ==      rxbuf[13])  &&  
           (device[u].data[14]    ==      rxbuf[14])  &&  
           (device[u].data[15]    ==      rxbuf[15])  &&  
           (device[u].data[16]    ==      rxbuf[16])  &&
           (device[u].data[17]    ==      rxbuf[17])  &&  
           (device[u].data[18]    ==      rxbuf[18])  &&  
           (device[u].data[19]    ==      rxbuf[19])  &&  
           (device[u].data[20]    ==      rxbuf[20])  &&  
           (device[u].data[21]    ==      rxbuf[21])  &&  
           (device[u].data[22]    ==      rxbuf[22])  &&  
           (device[u].data[23]    ==      rxbuf[23])  && 
           (device[u].data[24]    ==      rxbuf[24])  &&  
           (device[u].data[25]    ==      rxbuf[25])  &&  
           (device[u].data[26]    ==      rxbuf[26])  &&  
           (device[u].data[27]    ==      rxbuf[27])  && 
           (device[u].data[28]    ==      rxbuf[28])  && 
           (device[u].data[29]    ==      rxbuf[29])  //&&  
           //(device[u].data[30]    ==      rxbuf[30])    06.27
           )
        {
            device[u].data_flag=0;//²MªÅbuffer
            device[u].devicecount=0;
            
            for(int s=0;s<31;s++)
              device[u].data[s]=0; 
            
            order++;

        }
      }
 

      dontsave = 0;
      if(order >= 35)
        order = 0;
      
      if(jzx >= 35)
        jzx = 0;

      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
    }
    */
    if(rxbuf[0] == 'W'  &&  rxbuf[1] == 'W')  //¹ïÀ³slave 892
    {
      scifUartTxPutChars(MAC_Address,8);
    }
    
    
    
    dontsave_1=0;
    dontsave=0;

} // scTaskAlertCallback
//SCS_Uart_banana///////////////////////////////////////////////////////////////
    
    
// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)

static uint8_t scanRspData[] =
{
  // complete name
  0x0B,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 
  0x69,   // 
  0x6d,   // 
  0x70,   // 
  0x6c,   // 
  0x65,   // 
  0x42,   // 
  0x4c,   // 
  0x45,   // 
  0x50,   // 
  
  0x07,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'


  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

static uint8 advertData_IOS[] = 
{ 
   // 25 byte ibeacon advertising data
  // Preamble: 0x4c000215
  // UUID: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
  // Major: 1 (0x0001)
  // Minor: 1 (0x0001)
  // Measured Power: -59 (0xc5)
  0x1A, // length of this data including the data type byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific advertisement data type

  0x4c,
  0x00,
  0x02,
  0x15,
  
  
  0xe2,// 6 start  (0¶}©lºâ) 
  0xc5,
  0x6d,
  0xb5,
  0xdf,
  0xfb,
  0x00,
  0x00,
  0x00,                                                                  
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  
  
  0x00,//22 start
  0x9,
  0x8,
  0x9,
  
  0xc5
};
static uint8_t  ios_data[31]={0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAB,0xAB,0xAB,0x24,0x25,0x26,0x27,0x28,0x29,0x30};

//static uint8_t  hand_data[14]={'V','C',0x25,0x04,0x26,0x05,0x01,0x21,0x53,0x58,0x15,0x00,0x07,0x00};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)

static uint8_t advertData[] =//¤â¾÷³s½u­n¤@¼Ë ­n©ñ¦badver¸Ì­±
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //!FEATURE_OAD
};



// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//UART_M0
static void Uart_taskFxn (UArg a0, UArg a1);
//PWM
static void PWM_taskFxn (UArg a0, UArg a1);
//®É¶¡
static void systimeAns(int getTicks);
//check
bool check_buffer( char *inputbuffer);
//³]³Æ¦WºÙ
uint8_t set_snv_devicename(void);

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);
//MAC
static void GUA_Read_Mac(uint8 *pGUA_Address);  
char *GUA_Addr2Str(uint8 *pGUA_Addr);  
//MAC
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);

static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);

static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void SimpleBLEPeripheral_clockHandler(UArg arg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks

static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};


#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      PWM_Task_banana
 *
 * @brief   Task creation function for the SES_UART
 *
 * @param   None.
 *
 * @return  None.
 */
void PWM_Task(void)
{
  Task_Params params;
  Task_Params_init(&params);
  params.priority = 1;
  params.stackSize = sizeof(PWM_taskStack);
  params.stack = PWM_taskStack;

  Task_construct(&PWM_taskStruct, PWM_taskFxn, &params, NULL);
}
/*********************************************************************

********************************************************************
 * @fn      UART_creatTask_banana
 *
 * @brief   Task creation function for the SES_UART
 *
 * @param   None.
 *
 * @return  None.
 */
void UART_creatTask(void)
{
  Task_Params taskParams;

  Task_Params_init(&taskParams);
  taskParams.stack = my_UART_TaskStack;
  taskParams.stackSize = sizeof(my_UART_TaskStack);
  taskParams.priority = 3;
  Task_construct(&my_UART_Task, Uart_taskFxn, &taskParams, NULL);

  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  semParams.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&semScTaskAlert, 0, &semParams);
  
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{

    //clear
    for(int x=0;x<36;x++)
    {
      device[x].devicecount=0;
      device[x].data_flag=0;
      device[x].CT_flag=0;
      for(int k=0;k<31;k++)
      {
        device[x].data[k]=0;
      }
    }
    PIN_setOutputValue(hSbpPins, Board_LED6, 0);
    PIN_setOutputValue(hSbpPins, Board_LED0, 0);
  
    
    //UART
    /*
    UART_Params_init(&SbpUartParams);
    SbpUartParams.baudRate=57600;
    //SbpUartParams.readTimeout=60000;//UART_WAIT_FOREVER
    SbpUartParams.writeTimeout=30000;
    SbpUartHandle = UART_open(CC2650_UART0, &SbpUartParams);  
  */
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 self_address[6] = { 0x02,0x00,0x01,0x7D,0x3A,0x50};
  //HCI_EXT_SetBDADDRCmd(self_address);
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(80);

  //HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_1_DBM);
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
 
  // Create one-shot clocks for internal periodic events.
  //Util_constructClock(&NOTI, SimpleBLEPeripheral_clockHandler,0,0, false, SBP_NOTIFY);
  Util_constructClock(&SAVEClock, SimpleBLEPeripheral_clockHandler,SBP_PERIODIC_EVT_PERIOD,100, true, SBP_PERIODIC_EVT);
  //Util_startClock(&SAVEClock);//­«·s±Ò°Ê®ÉÄÁ
  //Util_constructClock(&CLEARClock, SimpleBLEPeripheral_clockHandler,SBP_PERIODIC_EVT_PERIOD,0, false, CLEAR_EVT);
  //Util_startClock(&CLEARClock);//­«·s±Ò°Ê®ÉÄÁ  
  
/*
#ifndef SENSORTAG_HW
  Board_openLCD();
#endif //SENSORTAG_HW
  */
#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW
  
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    //uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    //GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
    //                     &advertOffTime);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),scanRspData);//©w¦ìdata
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(ios_data), ios_data);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  uint8_t set_snv_devicename(void);
  // Set the GAP Characteristics
  //GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);//device ID

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service


  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Register callback with SimpleGATTprofile
  
#endif //!FEATURE_OAD

  
  //RTCªì©l¤Æ  
  GUA_RTC_Init();  
  


  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
  //HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_2_DBM);
  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);//¶}±Ò¸Ë¸m

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
    
  osal_snv_read(0x80, sizeof(int)*20, flash);   
  
  //¸g
  MyLight_lonSum_printf_lon_H = flash[14];
  MyLight_lonSum_printf_lon_L = flash[15];

  //½n
  MyLight_latSum_printf_lat_H = flash[9];
  MyLight_latSum_printf_lat_L = flash[10];   
  
  //¦Û¨­¦ì¸m
  MyLight_lonSum  = ((MyLight_lonSum_printf_lon_H & 0xF0) >>4)*1000  +   (MyLight_lonSum_printf_lon_H & 0x0F)*100 +  ((MyLight_lonSum_printf_lon_L & 0xF0) >>4)*10 + (MyLight_lonSum_printf_lon_L & 0x0F);
  MyLight_latSum  = ((MyLight_latSum_printf_lat_H & 0xF0) >>4)*1000  +   (MyLight_latSum_printf_lat_H & 0x0F)*100 +  ((MyLight_latSum_printf_lat_L & 0xF0) >>4)*10 + (MyLight_latSum_printf_lat_L & 0x0F);

  Final_lonSum  = ((flash[2] & 0xF0) >>4)*1000  +   (flash[2] & 0x0F)*100 +  ((flash[3] & 0xF0) >>4)*10 + (flash[3] & 0x0F);     
  Final_latSum  = (( flash[0] & 0xF0) >>4)*1000  +   ( flash[0] & 0x0F)*100 +  (( flash[1] & 0xF0) >>4)*10 + ( flash[1] & 0x0F);
  
  
   ios_data[24]     =       flash[17]; //Pr0
   ios_data[25]     =       flash[18]; //E0_1
   ios_data[26]     =       flash[19]; //E0_2

   
   ios_data[27]     =       MyLight_lonSum_printf_lon_H; 
   ios_data[28]     =       MyLight_lonSum_printf_lon_L;
   ios_data[29]     =       MyLight_latSum_printf_lat_H;
   ios_data[30]     =       MyLight_latSum_printf_lat_L;     

   advertData_IOS[22]     =       MyLight_lonSum_printf_lon_H; 
   advertData_IOS[23]     =       MyLight_lonSum_printf_lon_L;
   advertData_IOS[24]     =       MyLight_latSum_printf_lat_H;
   advertData_IOS[25]     =       MyLight_latSum_printf_lat_L;      
   
   int  UTLtxpower[1];
   osal_snv_read(0x82, sizeof(int)*1, UTLtxpower);
   HCI_EXT_SetTxPowerCmd(UTLtxpower[0]);


    

  //PIN_setOutputValue(hSbpPins, Board_LED6, 0);
  //PIN_setOutputValue(hSbpPins, Board_LED0, 0);
/*
  if(ios_data!=0x00)
  {
  ios_data[0]=0x1A;
  }  */
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  LCD_WRITE_STRING("BLE Peripheral A", LCD_PAGE0);
#else
  LCD_WRITE_STRING("BLE Peripheral B", LCD_PAGE0);
#endif // HAL_IMAGE_A
#else
  LCD_WRITE_STRING("BLE Peripheral", LCD_PAGE0);
#endif // FEATURE_OAD
}

/*********************************************************************
 * @fn      PWM_taskFxn
 *
 * @brief   Application task entry point for the Uart_taskFxn.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void PWM_taskFxn (UArg a0, UArg a1) {
  
  PWM_Params pwmParams;
  PWM_Params_init(&pwmParams);
  pwmParams.idleLevel  = PWM_IDLE_LOW;
    
  /* PWM in microseconds with period in microseconds */
  
  uint32_t dutyValue = 0; /* 0us (0%) initial duty cycle */
   
  pwmParams.periodUnit  = PWM_PERIOD_US;
  pwmParams.periodValue = PERIOD_US;
  pwmParams.dutyUnit    = PWM_DUTY_US; 
  pwmParams.dutyValue   = dutyValue;
    
  /* PWM open should will set to pin to idle level  */
  hPWM = PWM_open(CC2650_PWM0, &pwmParams);

  if(hPWM == NULL) 
  {
    Log_error0("Opening PWM failed");
    while(1);
  }
  
  PWM_start(hPWM);
  
  bool direction = 1; /* Initially increase duty */
  

    /* Sleep 10 PWM periods */
    Task_sleep(10 * PERIOD_US / Clock_tickPeriod);
    /* Change duty cycle with 1% of period */
    if(direction)
    {
      dutyValue += PERIOD_US / 100; 
    }
    else {
     dutyValue -= PERIOD_US / 100; 
    }
    
    if(dutyValue >= PERIOD_US)
    {
      dutyValue = PERIOD_US;
      direction = 0;
    }
    else if(dutyValue <= 0) {
      dutyValue = 0;
      direction = 1;
    }
    PWM_setDuty(hPWM, dutyValue);
    

}

/*********************************************************************
 * @fn      Uart_taskFxn
 *
 * @brief   Application task entry point for the Uart_taskFxn.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */


static void Uart_taskFxn (UArg a0, UArg a1) {
  
    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Start the UART emulator
    scifExecuteTasksOnceNbl(BV(SCIF_UART_EMULATOR_TASK_ID));

    // Enable baud rate generation
    scifUartSetBaudRate(57600);

    // Enable RX (10 idle bit periods required before enabling start bit detection)
    scifUartSetRxFifoThr(SCIF_UART_RX_FIFO_MAX_COUNT / 2);
    scifUartSetRxTimeout(10 * 2);
    scifUartSetRxEnableReqIdleCount(10 * 2);
    scifUartRxEnable(1);

    // Enable events (half full RX FIFO or 10 bit period timeout
    scifUartSetEventMask(BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR | BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT);


} // taskFxn
/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();
  
  set_snv_devicename();//device name
  GUA_Read_Mac(nGUA_Address);//mac
  
  memcpy(scanRspData+14,nGUA_Address,6);
  memcpy(advertData_IOS+6,nGUA_Address,6);
  //¶ÇMACµ¹Master
  MAC_Address[0]=0xAB;
  MAC_Address[1]=0xAB;
  MAC_Address[2]=nGUA_Address[0];
  MAC_Address[3]=nGUA_Address[1];
  MAC_Address[4]=nGUA_Address[2];
  MAC_Address[5]=nGUA_Address[3];
  MAC_Address[6]=nGUA_Address[4];
  MAC_Address[7]=nGUA_Address[5];

  for(int a=0;a<8;a++)
    scifUartTxPutChar(MAC_Address[a]);
  
  memcpy(scanRspData+2,DeviceName,10);
  
  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
    
    if( events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;
      if(linked != 1)
        PIN_setOutputValue(hSbpPins, Board_LED0,0);
      if(OAD_FLAG == 0)
      {
        once=1;   
        
        //ÀË¬dbuffer¤º¦³¸ê®Æ§_  hopping+©w¦ì¼s¼½
        for(int o=0;o<36;o++)
        {
          if((device[o].data_flag == 1) && (once == 1))
          {
            once=0;//¥u¯à¶i¤@¦¸
            PIN_setOutputValue(hSbpPins, Board_LED0,1);
            if(scanp == 0)
            {
              scanp=1;
              GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(device[o].data), device[o].data);  
              device[o].devicecount++;
              Task_sleep(50000);
            }
            else
            {
              scanp=0;
              GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(ios_data), ios_data);  
              device[o].devicecount++;
              Task_sleep(50000);     
            }
            
            if(device[o].devicecount>3)
            {
              device[o].data_flag=0;//²MªÅbuffer
              device[o].devicecount=0;
              
              for(int s=0;s<31;s++)
                device[o].data[s]=0; 
              
              order++; 
              
              if(order >= 35)
                order = 0;
              
              if(jzx >= 35)
                jzx = 0;
              
            } 
          }
          //hopping
          GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
          
          if(device[o].CT_flag == 1 && device[o].devicecount > 5)//¬O±±¿O¦Û¤v,§R°£
          {
            for(int a=0;a<31;a++)
            {
              device[o].data[a] = 0;
            }
            device[o].CT_flag = 0;
            device[o].data_flag = 0;
            device[o].devicecount = 0;
          }
        }

        //ios¼s¼½+©w¦ì
        if(once==1)
        {
          once = 0;
          if(scanp==1)          
          {
            scanp=0;
            GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(ios_data),ios_data);  
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(scanRspData), scanRspData);
            Task_sleep(50000);
          }
          else
          {
            scanp=1;
            GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(advertData_IOS),advertData_IOS);
            GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
            Task_sleep(50000);
          }
        }
      }
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      OAD_FLAG = 1;
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;
      
    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                           10, LCD_PAGE5);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
      }
      break;

    case GAPROLE_ADVERTISING:
      LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
        advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER   

    case GAPROLE_CONNECTED://³Q³s½u¤¤ ¥¼¼g¤J
      {
        linked = 1;
        //PWM_setDuty(hPWM, 1000);
        PIN_setOutputValue(hSbpPins, Board_LED0, 1);
        //Util_stopClock(&SAVEClock);//°±¤î
        //time_link_now        =       Clock_getTicks();//ºâminsec
        
        uint8_t peerAddress[B_ADDR_LEN];
        
        GUA_Timer.day = 0;
        GUA_Timer.hour = 0;
        GUA_Timer.minseconds = 0;
        GUA_Timer.minutes = 0;
        GUA_Timer.month = 0;    
        GUA_Timer.seconds = 0;    
        GUA_RTC_Set(&GUA_Timer);
        
        GUA_RTC_Get(&GUA_Timer);//±o¨ì®É¤À¬í
        
        systimelink_now[0] = GUA_Timer.hour;         
        systimelink_now[1] = GUA_Timer.minutes;       
        systimelink_now[2] = GUA_Timer.seconds;       
        systimelink_now[3] = GUA_Timer.minseconds;
   


        PIN_setOutputValue(hSbpPins, Board_LED0, 1);
        
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
        
        Util_startClock(&periodicClock);

        LCD_WRITE_STRING("Connected", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
        linked = 0;
      //PWM_setDuty(hPWM, 0);
      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
      Util_stopClock(&periodicClock);//³Q³s½u ¨S¼g­È´NÂ_½u
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      /*
      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
      */
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
        linked = 0;
      //PWM_setDuty(hPWM, 0);
      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      
      LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      LCD_WRITE_STRING("Error", LCD_PAGE2);
      break;

    default:
      LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{

  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue3);
      

      if(       newValue3[0]    ==      0xDD    &&      newValue3[1]    ==      0xDD   &&      newValue3[2]    ==      0xCD  )//§ïÅÜ¥\²v
      { 
        int UTLtxpower[1];
        /*
      #define LL_EXT_TX_POWER_MINUS_21_DBM                   0
      #define LL_EXT_TX_POWER_MINUS_18_DBM                   1
      #define LL_EXT_TX_POWER_MINUS_15_DBM                   2
      #define LL_EXT_TX_POWER_MINUS_12_DBM                   3
      #define LL_EXT_TX_POWER_MINUS_9_DBM                    4
      #define LL_EXT_TX_POWER_MINUS_6_DBM                    5
      #define LL_EXT_TX_POWER_MINUS_3_DBM                    6
      #define LL_EXT_TX_POWER_0_DBM                          7
      #define LL_EXT_TX_POWER_1_DBM                          8
      #define LL_EXT_TX_POWER_2_DBM                          9
      #define LL_EXT_TX_POWER_3_DBM                          10
      #define LL_EXT_TX_POWER_4_DBM                          11
      #define LL_EXT_TX_POWER_5_DBM                          12
        */
        GAPRole_TerminateConnection();      
        UTLtxpower[0] = newValue3[3];
        osal_snv_write(0x82, sizeof(int)*1, UTLtxpower);
        
        HCI_EXT_SetTxPowerCmd(UTLtxpower[0]);
      } 
      
      
      
      if(       newValue3[0]    ==      0x53    &&      newValue3[1]    ==      0x53    )//®É¶¡¦P¨B
      {
       // Util_stopClock(&SAVEClock);//°±¤î
        GUA_RTC_Get(&GUA_Timer);//±o¨ì®É¤À¬í
        
        systimelink_over[0] = GUA_Timer.hour;
        systimelink_over[1] = GUA_Timer.minutes;      
        systimelink_over[2] = GUA_Timer.seconds;
        systimelink_over[3] = GUA_Timer.minseconds;
        
        
        
        
        
        GUA_Timer.hour                =       newValue3[2]+systimelink_over[0] - systimelink_now[0];//®É
        GUA_Timer.minutes           =       newValue3[3]+systimelink_over[1] - systimelink_now[1];//¤À
        GUA_Timer.seconds           =       newValue3[4]+systimelink_over[2] - systimelink_now[2];//¬í
        //¦X¦¨minsec

        mins     =       ((newValue3[5]<<8) |newValue3[6]);                                                       //±µ¦¬¶Ç¨Óªºminsec_1
        //mins     =       mins<< 8        +      newValue3[6];                        //±µ¦¬¶Ç¨Óªºminsec_2      
        GUA_Timer.minseconds     =       mins + systimelink_over[3] - systimelink_now[3];//±µ¦¬¶Ç¨Óªºminsec + ©µ¿ð 
       //¦P¨B§¹¦¨®É¶¡
       sysovertime[0]   =       newValue3[7];
       sysovertime[1]   =       newValue3[8];
       sysovertime[2]   =       newValue3[9];     

        sys_F   = 1;
        //ÅÜ¦¨¦P¨B«á
        //hand_data[1]='V';

        PIN_setOutputValue(hSbpPins, Board_LED0, 0);

       //®É¶¡¶i¦ì
       if(GUA_Timer.minseconds>=1000)
       {
          GUA_Timer.minseconds = GUA_Timer.minseconds-1000;
          GUA_Timer.seconds = GUA_Timer.seconds+1;
       }     
       if(GUA_Timer.seconds>=60)
       {
          GUA_Timer.seconds = GUA_Timer.seconds-60;
          GUA_Timer.minutes = GUA_Timer.minutes+1;
       }         
       if(GUA_Timer.minutes>=60)
       {
          GUA_Timer.minutes = GUA_Timer.minutes-60;
          GUA_Timer.hour = GUA_Timer.hour+1;
       }      
       if(GUA_Timer.hour>=24)
       {
          GUA_Timer.hour = GUA_Timer.hour-24;
       }         
 
       
        GUA_RTC_Set(&GUA_Timer);//³]©w®É¶¡

        GUA_RTC_Get(&GUA_Timer);//®³®É¶¡

        uint8 systxbuf[20];

        newValue3[2]    =       GUA_Timer.hour;
        newValue3[3]    =       GUA_Timer.minutes; 
        newValue3[4]    =       GUA_Timer.seconds; 
        newValue3[5]    =       ((uint16)(GUA_Timer.minseconds) >>8);           //©î1
        newValue3[6]    =       ((uint16)(GUA_Timer.minseconds) & 0x00FF); //©î2
        
        //¶Ç°e®É¶¡uartµ¹Master ¨Ã¶}©l¦P¨B
        for(int a=0;a<20;a++)
        {
          systxbuf[a]   =       newValue3[a];
          scifUartTxPutChar(systxbuf[a]);
        }
        
        //////////notify
        uint8 state1;
        uint16 len;
        //bStatus_t status;
        attHandleValueNoti_t notify_back;

        GAPRole_GetParameter(GAPROLE_CONNHANDLE,&handle);
        
        notify_back.pValue = (uint8 *)GATT_bm_alloc( gapConnHandle, ATT_HANDLE_VALUE_NOTI,
                                        GATT_MAX_MTU, &len );//1.4 & 2.1¥H«á³W©w
        
        
       if ( notify_back.pValue != NULL )
       {
          notify_back.handle = 0x0027;//notify  //
          
          notify_back.len = 3;
          notify_back.pValue[0] = newValue3[4]; //sec
          notify_back.pValue[1] = newValue3[5]; //minsec_1
          notify_back.pValue[2] = newValue3[6]; //minsec_2
          state1 = GATT_Notification(gapConnHandle,&notify_back,FALSE);        
          
       }
       else
       {
          GAPRole_TerminateConnection();
       }    
        //////////notify 
        
        
        GAPRole_TerminateConnection();
        
        //scifUartTxPutChars(newValue3,20); //newValue3 ­n¬°char
        
        //GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(hand_data), hand_data);
        PIN_setOutputValue(hSbpPins, Board_LED6, 1);
        //Util_startClock(&SAVEClock);//­«·s±Ò°ÊCLEAR®ÉÄÁ
      }///////////////////////////////sys_end

      
      
      if(newValue3[0]==0x30)//±±¿O
      {
        if(newValue3[7]=='z')//±±¿Oµ²§ô  Z-->5A  Z-->7A
        {
         controltime = 0;
         PIN_setOutputValue(hSbpPins, Board_LED0, 0);
         GAPRole_TerminateConnection();        
        // Util_startClock(&SAVEClock);//­«·s±Ò°ÊCLEAR®ÉÄÁ
        }
        if(newValue3[1]=='Z')//¦¬¸g½n
        {
          //¬O¦Û¤v¥ô°È§_
          if(   (newValue3[2])  ==      MyLight_lonSum_printf_lon_H        &&
                 (newValue3[3])  ==      MyLight_lonSum_printf_lon_L        &&
                 (newValue3[4])  ==      MyLight_latSum_printf_lat_H          &&
                 (newValue3[5])  ==      MyLight_latSum_printf_lat_L        )
          {
            self_falg=1;//¬O¦Û¤v
          }
          else
          {
            self_falg=0;//§O¤H¥ô°È
          }
        }
        if(self_falg == 0 )//§O¤H¥ô°È
        {
         //save
         device[jzx].data_flag=1;//¦³¸ê®Æ
         //23¶}orÃö
         device[jzx].data[23] = newValue3[6];
         //¦Û¤v¸g½n
         device[jzx].data[25] = MyLight_lonSum_printf_lon_H;
         device[jzx].data[26] = MyLight_lonSum_printf_lon_L;
         device[jzx].data[27] = MyLight_latSum_printf_lat_H;
         device[jzx].data[28] = MyLight_latSum_printf_lat_L;
         //¥Ø¼Ð¸g½n
         device[jzx].data[10] = newValue3[2];  
         device[jzx].data[11] = newValue3[3];  
         device[jzx].data[12] = newValue3[4];  
         device[jzx].data[13] = newValue3[5];
         
         device[jzx].data[0] = 'C';  
         device[jzx].data[1] = 'T'; 
         device[jzx].data[30] = 0x07;      
         
         memcpy(txbuf,device[jzx].data,31);
         scifUartTxPutChars(txbuf,31);

         jzx++;  //¤U¤@­Ó        
         
         if(jzx>=35)
           jzx=0;
   
        }
        else//¦Û¤v«G·À¿O  ¼s¤@¤U§Y¥i
        {
          //save
          device[jzx].CT_flag = 1;
          device[jzx].data_flag = 1;//¦³¸ê®Æ
          //23¶}orÃö
          device[jzx].data[23] = newValue3[6];
          //¦Û¤v¸g½n
          device[jzx].data[25] = MyLight_lonSum_printf_lon_H;
          device[jzx].data[26] = MyLight_lonSum_printf_lon_L;
          device[jzx].data[27] = MyLight_latSum_printf_lat_H;
          device[jzx].data[28] = MyLight_latSum_printf_lat_L;
          //¥Ø¼Ð¸g½n
          device[jzx].data[10] = newValue3[2];  
          device[jzx].data[11] = newValue3[3];  
          device[jzx].data[12] = newValue3[4];  
          device[jzx].data[13] = newValue3[5];

          device[jzx].data[0] = 'C';  
          device[jzx].data[1] = 'T'; 
          device[jzx].data[30] = 0x07;      
       


          jzx++;  //¤U¤@­Ó        

          if(jzx>=35)
           jzx=0;

          if(newValue3[6] == 0x01)//«G 
          {
            PIN_setOutputValue(hSbpPins, Board_LED6, 1); 
          }
          else
          {
            PIN_setOutputValue(hSbpPins, Board_LED6, 0);           
          }
        }
       }

      else if( newValue3[0] == 0x26  &&
                   newValue3[2] == 0x12 &&
                   newValue3[3] == 0x34 &&
                   newValue3[4] == 0x56 &&
                   newValue3[5] == 0x78   )//³]©w¸g½n or ²×ÂI
      {
        //PWM_setDuty(hPWM, 0);
        if( newValue3[1] == 0x00 )
        {
  
          flash[9]   = newValue3[9] ;  //½n«×4 ¥D
          flash[10] = newValue3[10] ;//½n«×5 ¥D
          
          flash[14] = newValue3[14] ;//¸g«×4¥D
          flash[15] = newValue3[15] ;//¸g«×5¥D

          
          GAPRole_TerminateConnection();
          
          //½n
          MyLight_latSum_printf_lat_H = flash[9];
          MyLight_latSum_printf_lat_L =  flash[10]; 
          
          //¸g
          MyLight_lonSum_printf_lon_H = flash[14];
          MyLight_lonSum_printf_lon_L = flash[15];

         advertData_IOS[22]     =       MyLight_lonSum_printf_lon_H; 
         advertData_IOS[23]     =       MyLight_lonSum_printf_lon_L;
         advertData_IOS[24]     =       MyLight_latSum_printf_lat_H;
         advertData_IOS[25]     =       MyLight_latSum_printf_lat_L;       
         
         ios_data[27]     =       MyLight_lonSum_printf_lon_H; 
         ios_data[28]     =       MyLight_lonSum_printf_lon_L;
         ios_data[29]     =       MyLight_latSum_printf_lat_H;
         ios_data[30]     =       MyLight_latSum_printf_lat_L;     
                  

          //¦Û¨­¦ì¸m
          MyLight_lonSum  = ((MyLight_lonSum_printf_lon_H & 0xF0) >>4)*1000  +   (MyLight_lonSum_printf_lon_H & 0x0F)*100 +  ((MyLight_lonSum_printf_lon_L & 0xF0) >>4)*10 + (MyLight_lonSum_printf_lon_L & 0x0F);
          MyLight_latSum  = ((MyLight_latSum_printf_lat_H & 0xF0) >>4)*1000  +   (MyLight_latSum_printf_lat_H & 0x0F)*100 +  ((MyLight_latSum_printf_lat_L & 0xF0) >>4)*10 + (MyLight_latSum_printf_lat_L & 0x0F);
          end =1;
        }
        else if( newValue3[1] == 0x01 ) //¬ö¿ý²×ÂI
        {
          GAPRole_TerminateConnection();
          
          flash[0] = newValue3[9];   
          flash[1] = newValue3[10];
          flash[2] = newValue3[14];
          flash[3] = newValue3[15];

          Final_lonSum  = ((flash[2] & 0xF0) >>4)*1000  +   (flash[2] & 0x0F)*100 +  ((flash[3] & 0xF0) >>4)*10 + (flash[3] & 0x0F);     
          Final_latSum  = (( flash[0] & 0xF0) >>4)*1000  +   ( flash[0] & 0x0F)*100 +  (( flash[1] & 0xF0) >>4)*10 + ( flash[1] & 0x0F);
          
          starting_point = 1 ;
        }
        else if(newValue3[1] == 0xAA)//¬ö¿ý Pr0 E
        {
          GAPRole_TerminateConnection();
        
          Pr0 = newValue3[6];
          
          E0_1 = newValue3[7];
          E0_2 = newValue3[8];    
          
          flash[17] = newValue3[6];//Pr0
          flash[18] = newValue3[7];//E0_1
          flash[19] = newValue3[8];//E0_2
          
          osal_snv_write(0x80, sizeof(int)*20, flash);
          
          ios_data[24] = Pr0;
          ios_data[25] = E0_1; //­Ó¦ì¼Æ
          ios_data[26] = E0_2; //¤p¼ÆÂI

        }
        else if(newValue3[1] == 0xBB)//¬ö¿ý¸Ë¸m¦WºÙ
        {
          GAPRole_TerminateConnection();
          char attDeviceName_change[10] = "W_Node";//¹w³]ªºdevice name
          char getchar[4] = "";   
          
          getchar[0] = newValue3[6];
          getchar[1] = newValue3[7];
          getchar[2] = newValue3[8];

         strcat(attDeviceName_change,getchar);//¦r¦ê¬Û¥[
         
         for(int i=0;i<10;i++)
          DeviceName[i] = attDeviceName_change[i];
         
         osal_snv_write(0x81, sizeof(int)*10, DeviceName);
         
         memcpy(scanRspData+2,DeviceName,10);
           
         GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, DeviceName);//´«¦W¦r
        }

        if( end  == 1 && starting_point == 1 )
        {
          osal_snv_write(0x80, sizeof(int)*20, flash);
          osal_snv_read(0x80, sizeof(int)*20, flash);
          end  = 0 ;
          starting_point = 0;

          for(int k=0;k<20;k++)
          {
            addrbuf[k]=flash[k];
          }
          for(int p=20;p<31;p++)
          {
            addrbuf[p]=0x22;
          }        
          for(int a=0;a<31;a++)
            scifUartTxPutChar(addrbuf[a]);//¼g¸g½n        
        }

      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
      //Util_startClock(&NOTI);//©I¥s¦^¶ÇMAC³¡¤À

    }
    else
    {
      PIN_setOutputValue(hSbpPins, Board_LED6, 0);
      GAPRole_TerminateConnection();
    //  Util_startClock(&SAVEClock);//­«·s±Ò°ÊCLEAR®ÉÄÁ
    }
      //LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

      default:
      // should not reach here!
      break;
  }

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */

static void SimpleBLEPeripheral_performPeriodicTask(void)
{
  
  uint8 MacAddress_notify;
  
  if( SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR4, &MacAddress_notify) == SUCCESS )
  {
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8),
                          &MacAddress_notify);
  }
/*
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    
    
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
*/
}


#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  OAD_FLAG = 1;
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
*********************************************************************/
//******************************************************************************            
//name:             GUA_Read_Mac           

//******************************************************************************   
static void GUA_Read_Mac(uint8 *pGUA_Address)        
{    
  uint32_t nGUA_Mac0 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);  
  uint32_t nGUA_Mac1 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);  
    
  pGUA_Address[5] = nGUA_Mac0;  
  pGUA_Address[4] = nGUA_Mac0 >> 8;  
  pGUA_Address[3] = nGUA_Mac0 >> 16;  
  pGUA_Address[2] = nGUA_Mac0 >> 24;  
  pGUA_Address[1] = nGUA_Mac1;  
  pGUA_Address[0] = nGUA_Mac1 >> 8;  
}
    




/*********************************************************************
*********************************************************************/
//******************************************************************************            
//name:             GUA_Addr2Str           
         
//******************************************************************************   
char *GUA_Addr2Str(uint8 *pGUA_Addr)    
{    
  uint8         i;    
  char          bGUA_Hex[] = "0123456789ABCDEF";    
  static char   bGUA_Str[B_ADDR_LEN*2];    
  char          *pGUA_Str = bGUA_Str;    
    
  for(i = B_ADDR_LEN; i > 0; i--)    
  {    
    *pGUA_Str++ = bGUA_Hex[*pGUA_Addr >> 4];    
    *pGUA_Str++ = bGUA_Hex[*pGUA_Addr++ & 0x0F];    
  }    
    
  return bGUA_Str;    
}

///dealy  sec
static void delaysec(int sec_use)
{
  uint32_t timeNow = 1426692075;
  Seconds_set(timeNow);
  

  while( (sec = Seconds_get()) != timeNow + sec_use) {} // <- This actually waits for 5 seconds!

  t   = time(NULL);
  
}
static void systimeAns(int getTicks)
{
  int Allsec       =       getTicks/100000;   //©Ò¦³¬í¼Æ
  int minsec      =       getTicks%1000;    //minsec
  
  
  systime[0]    =       Allsec/3600;    //®É
  systime[1]    =       Allsec/60;       //¤À
  systime[2]    =       Allsec%60;     //¬í
  systime[3]    =       minsec;          //minsec  
}

bool check_buffer( char inputbuffer[])
{
  bool answer = 1;
  
  for(int a=0; a<36; a++)
  {
    if( memcmp( device[a].data, inputbuffer,31 ) == 0) // °²¦p¦³¤@¼Ò¤@¼Ëªº ¿é¥X0
    {
      answer = 0;
      return answer; //¦³§¹¥þ¤@¼Ëªº
    }
  }
  return answer;//³£¤£¤@¼Ë ¿é¥X1
}

