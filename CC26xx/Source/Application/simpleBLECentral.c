/*******************************************************************************
  Filename:       SimpleBLECentral_Central.c
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    This file contains the Simple BLE Central sample application 
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
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_key.h"
#include "board_lcd.h"
#include "Board.h"

#include "simpleBLECentral.h"

#include "bleUserConfig.h"

#include <ti/drivers/lcd/LCDDogm1286.h>


/*********************************************************************
 * ¦Û©w¸q
 */
#include <time.h>                       //®É¶¡©µ¿ð
#include <ti/sysbios/hal/Seconds.h>     //®É¶¡©µ¿ð
#include <math.h>
#include <ti/sysbios/BIOS.h>            //M0
#include "scif.h"                               //M0

/*********************************************************************
 * ¦Û©w¸q°Ñ¼Æ
 */
Task_Struct my_UART_Task;
Char my_UART_TaskStack[512];
static Semaphore_Struct semScTaskAlert;

/*********************************************************************
Buffer¬ÛÃöÅÜ¼Æ
*/
#define devicenumbers 36                        //¸Ë¸mBuffer­Ó¼Æ
#define CLK            8*2*3                        //´X­Ó´`Àô«á²M°£¸ê°T  48­Ó³æ¦ì¬ù5¬í
typedef struct 
{
  char data[31];
  short count;
  bool protection;
  bool save;
  bool wait;
  bool wait_over;
  bool self;
}devicedata;
bool self;
bool checksame;
static devicedata device[36];

uint8 rxbuf[100];

/*********************************************************************
±½´yÅÜ¼Æ
*/
int nowbuffer = 0;                              //Buffer«ü¼Ð
bool repeat_flag = 0;                          //¸ê®Æ­«½Æ»P§_
bool closer_flag = 0;                          //¸ê®Æ­«½Æ»P§_
int  flash[20]={0};                              //flash
/*********************************************************************
¼s¼½ÅÜ¼Æ
*/
static uint8_t  ios_data[31]={0x00,0xFF,0x4c,0x00,0x02,0x15,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0xc5};//²Ä7­Óbytes
/*********************************************************************
µwÅé¥\¯àÅÜ¼Æ
*/
static PIN_Handle hSbpPins;                  //IO¸}¶}ÃöªºPIN
bool ledvale;                                           //LED6ªºflag
bool led2vale;                                          //LED0ªºflag
uint8_t nGUA_Address[6];                     //¦Û¨­MAC
uint8_t MAC_ad[6];                               //Slave¶Ç¨ÓªºMAC
bool flash_flag = 0;                               //Åª¨úflash¦Û¨­¸g½n«×¤F¨S
bool MAC_flag = 0;                              //Åª¨úSlave¤§MAC¤F¨S
bool MAC_over = 0;                              //Åª¨úSlaveµ²§ô

/*********************************************************************
Delay¥\¯àÅÜ¼Æ
*/
uint32_t sec;
time_t t;

/*********************************************************************
 * ¸g½n«×¬ÛÃö
 */
uint8 MyHigh;              //¦Û¨­°ª«×
uint8 MyLight_lat_H;   //¦Û¨­½n«×H¦ì
uint8 MyLight_lat_L;    //¦Û¨­½n«×L¦ì   
uint8 MyLight_lon_H;  //¦Û¨­¸g«×H¦ì
uint8 MyLight_lon_L;  //¦Û¨­¸g«×L¦ì
int MyLight_lat;       //¦Û¨­½n«×      
int MyLight_lon;      //¦Û¨­¸g«×

uint8 Data_lat_H;        //±½´y¤§¸ê®Æ½n«×H¦ì      
uint8 Data_lat_L;        //±½´y¤§¸ê®Æ½n«×L¦ì   
   
uint8 Data_lon_H;       //±½´y¤§¸ê®Æ¸g«×H¦ì
uint8 Data_lon_L;       //±½´y¤§¸ê®Æ¸g«×L¦ì
uint8 Data_High_H;    //±½´y¤§°ª«×H¦ì
uint8 Data_High_L;    //±½´y¤§°ª«×L¦ì
int Data_lat;            //¸ê®Æ½n«×
int Data_lon;           //¸ê®Æ¸g«×


uint8 CT_lat_H;        //±±¿O¤§¸ê®Æ½n«×H¦ì      
uint8 CT_lat_L;        //±±¿O¤§¸ê®Æ½n«×L¦ì   
   
uint8 CT_lon_H;       //±±¿O¤§¸ê®Æ¸g«×H¦ì
uint8 CT_lon_L;       //±±¿O¤§¸ê®Æ¸g«×L¦ì
uint8 CT_High_H;    //±±¿O¤§°ª«×H¦ì
uint8 CT_High_L;    //±±¿O¤§°ª«×L¦ì
int CT_target_lat;            //¸ê®Æ½n«×
int CT_target_lon;           //¸ê®Æ¸g«×





uint8 Router_lat_H;     //²×ÂI¤§½n«×H¦ì
uint8 Router_lat_L;     //²×ÂI¤§½n«×L¦ì
uint8 Router_lon_H;    //²×ÂI¤§¸g«×H¦ì
uint8 Router_lon_L;    //²×ÂI¤§¸g«×L¦ì
int Router_lat;         //²×ÂI½n«×
int Router_lon;        //²×ÂI¸g«×

float FinalOthLight_dis_float_1=0; //¶ZÂ÷¹Bºâ¥Î
float FinalOthLight_dis_float_2=0; //¶ZÂ÷¹Bºâ¥Î
float FinalOthLight_dis;                 //¹Bºâ¶ZÂ÷

float FinalMylight_dis_float_1=0;  //¶ZÂ÷¹Bºâ¥Î
float FinalMylight_dis_float_2=0;  //¶ZÂ÷¹Bºâ¥Î
float FinalMylight_dis;                  //¹Bºâ¶ZÂ÷


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT     0x0001
#define SBC_PAIRING_STATE_EVT            0x0002
#define SBC_PASSCODE_NEEDED_EVT    0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                0x0010
#define SBC_STATE_CHANGE_EVT            0x0020

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 200  //±½´y§¹¦¨®É¶¡
// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           200  //±½´y¦h¤[´Nª½±µ¶i¤JINFO


// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is 
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update 
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY



// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct 
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data 
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle 
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// Connection handle of current connection 
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];



/*********************************************************************
 * ·s¼W¤§LOCAL FUNCTIONS
 */
static void Uart_taskFxn (UArg a0, UArg a1); //SCE UART
static void GUA_Read_Mac(uint8 *pGUA_Address);  //MAC
static void delaysec(int sec_use);              //©µ¿ð
bool check_buffer( char inputbuffer[] , int count);//¤ñ¸û°}¦C


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, 
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period);
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle);
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status, 
                                           uint8_t *pData);




void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback


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

    //¦¬¸g½n«×  
    if((rxbuf[29]==0x22)&&(rxbuf[30]==0x22))//Åª¸g½n ¦¬§À 
    {
      for(int i=0;i<20;i++)
        flash[i]=rxbuf[i];

      Router_lon  = ((flash[2] & 0xF0) >>4)*1000  +   (flash[2] & 0x0F)*100 +  ((flash[3] & 0xF0) >>4)*10 + (flash[3] & 0x0F);     
      Router_lat  = (( flash[0] & 0xF0) >>4)*1000  +   ( flash[0] & 0x0F)*100 +  (( flash[1] & 0xF0) >>4)*10 + ( flash[1] & 0x0F);
       
      MyLight_lon = ((flash[14] & 0xF0) >>4)*1000  +   (flash[14] & 0x0F)*100 +  ((flash[15] & 0xF0) >>4)*10 + (flash[15] & 0x0F); 
      MyLight_lat = (( flash[9] & 0xF0) >>4)*1000  +   ( flash[9] & 0x0F)*100 +  (( flash[10] & 0xF0) >>4)*10 + ( flash[10] & 0x0F);
      
      MyHigh = flash[16];
      
      MyLight_lon_H = flash[14];
      MyLight_lon_L = flash[15];
      
      MyLight_lat_H = flash[9];
      MyLight_lat_L = flash[10];  
      
      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
      
      for(int x=0;x<20;x++)
        rxbuf[x]=0;
      
      //osal_snv_write(0x80, sizeof(int)*20, flash);
      //osal_snv_read(0x80, sizeof(int)*20, flash);
      
     flash_flag=1;    //§ó·s¸g½n«×
     
     delaysec(1);
     PIN_setOutputValue(hSbpPins, Board_LED0, 1);
    }
    if(rxbuf[0]==0xAB  &&  rxbuf[1]==0xAB)  //rxbuf[0]¸ò[1] ¬O³]©w¬°ÅªMACªº±K½X   ¹ïÀ³slave 621
    { 
      for(int a=0;a<6;a++)
        MAC_ad[a]=rxbuf[a+2];   //rxbuf[2]¶}©l¤~¬OMACªº¦ì§}
      
      //osal_snv_write(0x81, sizeof(int)*6, MAC_ad);
      //osal_snv_read(0x81, sizeof(int)*6, MAC_ad);  
      
      MAC_flag = 1;           //­YÅª¨ú¨ì«á¬°1

      
      if(MAC_ad[3]!=0)
      {
        delaysec(1);
        PIN_setOutputValue(hSbpPins, Board_LED0, 0);
        delaysec(1);
        PIN_setOutputValue(hSbpPins, Board_LED0, 1);
      }
    }

    //±±¿O  CT¦s  
    if(rxbuf[0] == 'C' && rxbuf[1] == 'T')   //rxbuf[0]¸ò[1] ³]©w¬°±±¿Oªº±K½X
    {
      for( int a=0;a<31;a++)
        device[nowbuffer].data[a] = rxbuf[a];
      
      //½á¤©¸g½n«×
      device[nowbuffer].data[25] = MyLight_lon_H;
      device[nowbuffer].data[26] = MyLight_lon_L;
      device[nowbuffer].data[27] = MyLight_lat_H;
      device[nowbuffer].data[28] = MyLight_lat_L;
      device[nowbuffer].data[30] = MyHigh;
      device[nowbuffer].wait = 1;
      device[nowbuffer].save = 1;

      
      nowbuffer++;
      if( nowbuffer > 35)   //buffer³Ì¤j­­¨î  ¶W¹L«hÂÐ»\*/
      nowbuffer = 0; 
    }
    
    
    
    
    
    
    
    for(int a=0;a<rxFifoCount;a++)
      rxbuf[a]=0;
} // scTaskAlertCallback





/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */



/*********************************************************************
 * @fn      UART_creatTask
 *
 * @brief   SCS ªº UART.
 *
 * @param   none
 *
 * @return  none
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
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;
  
  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
  uint8_t i;
  
  GUA_Read_Mac(nGUA_Address);   //MAC
  
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);
  
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
     
  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 1 , true, 0);
  
  Board_initKeys(SimpleBLECentral_keyChangeHandler);
  
  //Board_openLCD();
  
  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }
  
  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    
    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t), 
                                &scanRes);
  }
  
  // Setup GAP ±½´y

  GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT,100);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND,100);
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND,100);
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT,100);
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, 
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;
    
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), 
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications  
  GATT_RegisterForInd(selfEntity);
  
    osal_snv_read(0x80, sizeof(int)*20, flash);
    osal_snv_read(0x81, sizeof(int)*6, MAC_ad);
    
    MyHigh = flash[16];
    
    Router_lon  = ((flash[2] & 0xF0) >>4)*1000  +   (flash[2] & 0x0F)*100 +  ((flash[3] & 0xF0) >>4)*10 + (flash[3] & 0x0F);     
    Router_lat  = (( flash[0] & 0xF0) >>4)*1000  +   ( flash[0] & 0x0F)*100 +  (( flash[1] & 0xF0) >>4)*10 + ( flash[1] & 0x0F);
     
    MyLight_lon = ((flash[14] & 0xF0) >>4)*1000  +   (flash[14] & 0x0F)*100 +  ((flash[15] & 0xF0) >>4)*10 + (flash[15] & 0x0F); 
    MyLight_lat = (( flash[9] & 0xF0) >>4)*1000  +   ( flash[9] & 0x0F)*100 +  (( flash[10] & 0xF0) >>4)*10 + ( flash[10] & 0x0F);
    
    MyLight_lon_H = flash[14];
    MyLight_lon_L = flash[15];
    
    MyLight_lat_H = flash[9];
    MyLight_lat_L = flash[10];    
    
  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  
  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
  LCD_WRITE_STRING("BLE Central", LCD_PAGE0);
}

/*********************************************************************
 * @fn      Uart_taskFxn
 *
 * @brief   SCS Uartªº«H¸¹¼Ð.
 *
 * @param   none
 *
 * @return  events not processed
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


}


/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();
  PIN_setOutputValue(hSbpPins, Board_LED0, 1);   
 
  
  //¶}©l±½´y one shot
  GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                DEFAULT_DISCOVERY_WHITE_LIST);  
  
  
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
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);
        
        // Free the space from the message
        ICall_free(pMsg);
      }
    }
    
    if(flash_flag == 1)  //§ó·s¸g½n«×
    {
      osal_snv_write(0x80, sizeof(int)*20, flash);
      flash_flag = 0;
    }
    
    
    
    //¸òSlave¯Á¨úMAC§¹²¦
    if(MAC_flag == 1 && MAC_over == 0)
    {
      PIN_setOutputValue(hSbpPins, Board_LED0, 1);   
      delaysec(1);
      PIN_setOutputValue(hSbpPins, Board_LED0, 0);
      delaysec(1);
      PIN_setOutputValue(hSbpPins, Board_LED0, 1);   

      osal_snv_write(0x81, sizeof(int)*6, MAC_ad);
      osal_snv_read(0x81, sizeof(int)*6, MAC_ad);  
      MAC_over = 1;
    }
    //¸òSlave¯Á¨úMAC¤¤  ¹ïÀ³slave 621
    else if (MAC_flag == 0 && MAC_over == 0)
    {
      char wantMAC[5];
      
      for(int a=0;a<5;a++)
        wantMAC[a] = 'W';
      
      scifUartTxPutChars(wantMAC,5);
    }     
    
    
    
    
    
    
    /*
    if (events & SBC_START_DISCOVERY_EVT)
    {      
      events &= ~SBC_START_DISCOVERY_EVT;
      GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);
      SimpleBLECentral_startDiscovery();
    }*/
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;
      
    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;
            
          default:
            break;
        }
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);
      
      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;
      
    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state); 
      break;
      
    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;
      
    // Pairing event  
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }
      
    // Passcode event    
    case SBC_PASSCODE_NEEDED_EVT:
      {     
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);
        
        ICall_free(pMsg->pData);
        break;
      }
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        maxPduSize = pEvent->initDone.dataPktLen;
        
        LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->initDone.devAddr),
                         LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
      if(pEvent->deviceInfo.eventType == 0x04 || pEvent->deviceInfo.eventType == 0x00)
       {
         //Á×¶}¦Û¨­Slaveªº¼s¼½
         if(   
              pEvent->deviceInfo.addr[0] != MAC_ad[5]  ||
              pEvent->deviceInfo.addr[1] != MAC_ad[4]  ||
              pEvent->deviceInfo.addr[2] != MAC_ad[3]  ||
              pEvent->deviceInfo.addr[3] != MAC_ad[2]  ||
              pEvent->deviceInfo.addr[4] != MAC_ad[1]  ||
              pEvent->deviceInfo.addr[5] != MAC_ad[0] 
            )
         {
           //§PÂ_¬O¤âÀô or hopping¸ê°T or ºÏÂ®¶}Ãö
           //¤âÀô or hopping
           if( (
                pEvent->deviceInfo.pEvtData[0] == '$'             &&
                pEvent->deviceInfo.pEvtData[16] == 0x00       &&
                pEvent->deviceInfo.pEvtData[17] == 0x00       &&
                pEvent->deviceInfo.pEvtData[18] == 0x00       &&
                pEvent->deviceInfo.pEvtData[19] == 0x00       &&
                pEvent->deviceInfo.pEvtData[20] == 0x00       &&
                pEvent->deviceInfo.pEvtData[21] == 0x00       &&
                pEvent->deviceInfo.pEvtData[22] == 0x00       && 
                pEvent->deviceInfo.pEvtData[23] == 0x00       )       ||
                //ºÏÂ®¶}Ãö
                (
                pEvent->deviceInfo.pEvtData[0] == '$'                   &&              
               (pEvent->deviceInfo.pEvtData[16] >>4 ==0x01 )     &&
                pEvent->deviceInfo.pEvtData[17] == 0x00              &&
                pEvent->deviceInfo.pEvtData[18] == 0x00              &&
                pEvent->deviceInfo.pEvtData[19] == 0x00              &&
                pEvent->deviceInfo.pEvtData[20] == 0x00              &&
                pEvent->deviceInfo.pEvtData[21] == 0x00              &&
                pEvent->deviceInfo.pEvtData[22] == 0x00              && 
                pEvent->deviceInfo.pEvtData[24] == 0xCC                 
                )
              )
           {
             //¤âÀô
             if(
                pEvent->deviceInfo.pEvtData[25] == 0x00        &&
                pEvent->deviceInfo.pEvtData[26] == 0x00        &&
                pEvent->deviceInfo.pEvtData[27] == 0x00        &&
                pEvent->deviceInfo.pEvtData[28] == 0x00        &&
                pEvent->deviceInfo.pEvtData[29] == 0x00        &&
                pEvent->deviceInfo.pEvtData[30] == 0x00   
                )
             {
               //§PÂ_buffer¤º¬O§_¦³­«½Æ¸ê°T
               for(int a=0;a<devicenumbers;a++)
               {
                  if(
                      pEvent->deviceInfo.pEvtData[0] == device[a].data[0]     &&
                      pEvent->deviceInfo.pEvtData[1] == device[a].data[1]     &&
                      pEvent->deviceInfo.pEvtData[2] == device[a].data[2]     &&
                      pEvent->deviceInfo.pEvtData[3] == device[a].data[3]     &&
                      pEvent->deviceInfo.pEvtData[4] == device[a].data[4]     &&
                      pEvent->deviceInfo.pEvtData[5] == device[a].data[5]     &&
                      pEvent->deviceInfo.pEvtData[6] == device[a].data[6]     &&
                      pEvent->deviceInfo.pEvtData[7] == device[a].data[7]     &&
                      pEvent->deviceInfo.pEvtData[8] == device[a].data[8]     &&
                      pEvent->deviceInfo.pEvtData[9] == device[a].data[9]     &&
                      pEvent->deviceInfo.pEvtData[10] == device[a].data[10] &&
                      pEvent->deviceInfo.pEvtData[11] == device[a].data[11] &&
                      pEvent->deviceInfo.pEvtData[12] == device[a].data[12] &&
                      pEvent->deviceInfo.pEvtData[13] == device[a].data[13] &&
                      pEvent->deviceInfo.pEvtData[14] == device[a].data[14] &&
                      pEvent->deviceInfo.pEvtData[15] == device[a].data[15] &&
                      pEvent->deviceInfo.pEvtData[16] == device[a].data[16] &&
                      pEvent->deviceInfo.pEvtData[17] == device[a].data[17] &&
                      pEvent->deviceInfo.pEvtData[18] == device[a].data[18] &&
                      pEvent->deviceInfo.pEvtData[19] == device[a].data[19] &&
                      pEvent->deviceInfo.pEvtData[20] == device[a].data[20] &&
                      pEvent->deviceInfo.pEvtData[21] == device[a].data[21] &&
                      pEvent->deviceInfo.pEvtData[22] == device[a].data[22] &&
                      pEvent->deviceInfo.pEvtData[23] == device[a].data[23] &&
                      pEvent->deviceInfo.pEvtData[24] == device[a].data[24]   
                     )
                  {
                    //µo²{­«½Æ ¸õ¥X ¤£²z
                    repeat_flag = 1;
                    break;
                  }
               }
               //¦s­È
               if( repeat_flag == 0 ) 
               {
                  for( int a=0;a<31;a++)
                    device[nowbuffer].data[a] = pEvent->deviceInfo.pEvtData[a];

                  //½á¤©¸g½n«×
                  device[nowbuffer].data[25] = MyLight_lon_H;
                  device[nowbuffer].data[26] = MyLight_lon_L;
                  device[nowbuffer].data[27] = MyLight_lat_H;
                  device[nowbuffer].data[28] = MyLight_lat_L;
                  device[nowbuffer].data[30] = MyHigh;
                  device[nowbuffer].save = 1;
                  device[nowbuffer].wait = 1;
                  
                  
                  nowbuffer++;
                
                  if( nowbuffer > 35)
                    nowbuffer = 0;
               }
               repeat_flag = 0;
             }
             //hopping¸ê°T
             else
             {
                //¥ý²Õ¦X±½´y¤§¸ê°T
                Data_lon_H = pEvent->deviceInfo.pEvtData[25];
                Data_lon_L = pEvent->deviceInfo.pEvtData[26];
                Data_lon = ((Data_lon_H & 0xF0)>>4)*1000 + (Data_lon_H & 0x0F)*100 + ((Data_lon_L & 0xF0)>>4)*10 + (Data_lon_L & 0x0F);//¸g«×
                
                Data_lat_H = pEvent->deviceInfo.pEvtData[27];
                Data_lat_L = pEvent->deviceInfo.pEvtData[28];
                Data_lat = ((Data_lat_H & 0xF0)>>4)*1000 + (Data_lat_H & 0x0F)*100 + ((Data_lat_L & 0xF0)>>4)*10 + (Data_lat_L & 0x0F);//½n«×
                
                Data_High_H = pEvent->deviceInfo.pEvtData[29];
                Data_High_L = pEvent->deviceInfo.pEvtData[30];
                
                //¨ú¦¹¸ê®Æ»P²×ÂI¦h»·
                FinalOthLight_dis_float_1 = pow( Router_lat-Data_lat,2);//½n
                FinalOthLight_dis_float_2 = pow( Router_lon-Data_lon,2);//¸g
                FinalOthLight_dis = sqrt(FinalOthLight_dis_float_1+FinalOthLight_dis_float_2);
                
                //¨ú§Ú¶ZÂ÷²×ÂI¦h»·
                FinalMylight_dis_float_1 = pow( Router_lon-MyLight_lon,2);
                FinalMylight_dis_float_2 = pow( Router_lat-MyLight_lat,2);
                FinalMylight_dis = sqrt(FinalMylight_dis_float_1+FinalMylight_dis_float_2);       
                
                //¤ñ¶ZÂ÷                ¥L»P²×ÂI¶ZÂ÷  §Ú»P²×ÂI¶ZÂ÷  
                if( FinalOthLight_dis >= FinalMylight_dis )
                {
                  closer_flag = 1;    //§Ú¤ñ¸ûªñ ¥L¤ñ¸û»· ¦s©Î¤£²z
                }
                else
                {
                  closer_flag = 0;    //§Ú¤ñ¸û»· ¥L¤ñ¸ûªñ  §R°£©Î¤£²z
                }
                for( int a=0; a<devicenumbers; a++)
                {
                  if(
                      pEvent->deviceInfo.pEvtData[0] == device[a].data[0] &&
                      pEvent->deviceInfo.pEvtData[1] == device[a].data[1] &&
                      pEvent->deviceInfo.pEvtData[2] == device[a].data[2] &&
                      pEvent->deviceInfo.pEvtData[3] == device[a].data[3] &&
                      pEvent->deviceInfo.pEvtData[4] == device[a].data[4] &&
                      pEvent->deviceInfo.pEvtData[5] == device[a].data[5] &&
                      pEvent->deviceInfo.pEvtData[6] == device[a].data[6] &&
                      pEvent->deviceInfo.pEvtData[7] == device[a].data[7] &&
                      pEvent->deviceInfo.pEvtData[8] == device[a].data[8] &&
                      pEvent->deviceInfo.pEvtData[9] == device[a].data[9] &&
                      pEvent->deviceInfo.pEvtData[10] == device[a].data[10] &&
                      pEvent->deviceInfo.pEvtData[11] == device[a].data[11] &&
                      pEvent->deviceInfo.pEvtData[12] == device[a].data[12] &&
                      pEvent->deviceInfo.pEvtData[13] == device[a].data[13] &&
                      pEvent->deviceInfo.pEvtData[14] == device[a].data[14] &&
                      pEvent->deviceInfo.pEvtData[15] == device[a].data[15] &&
                      pEvent->deviceInfo.pEvtData[16] == device[a].data[16] &&
                      pEvent->deviceInfo.pEvtData[17] == device[a].data[17] &&
                      pEvent->deviceInfo.pEvtData[18] == device[a].data[18] &&
                      pEvent->deviceInfo.pEvtData[19] == device[a].data[19] &&
                      pEvent->deviceInfo.pEvtData[20] == device[a].data[20] &&
                      pEvent->deviceInfo.pEvtData[21] == device[a].data[21] &&
                      pEvent->deviceInfo.pEvtData[22] == device[a].data[22] &&
                      pEvent->deviceInfo.pEvtData[23] == device[a].data[23] &&
                      pEvent->deviceInfo.pEvtData[24] == device[a].data[24] 
                     )//§PÂ_0~31 bytes ¬O§_¤@¼Ë  ¿é¥X1->¤@¼Ë
                  {
                    repeat_flag = 1;//­«½Æ¸õ¥X
                    break;
                  }
                }              
                if( closer_flag == 1 && repeat_flag == 0 )//ªñ + ¨S­«½Æ = ¦s­È
                {
                  for( int a=0;a<31;a++)
                    device[nowbuffer].data[a] = pEvent->deviceInfo.pEvtData[a];
                  
                  //½á¤©¸g½n«×
                  device[nowbuffer].data[25] = MyLight_lon_H;
                  device[nowbuffer].data[26] = MyLight_lon_L;
                  device[nowbuffer].data[27] = MyLight_lat_H;
                  device[nowbuffer].data[28] = MyLight_lat_L;
                  device[nowbuffer].data[30] = MyHigh;
                  device[nowbuffer].wait = 1;
                  device[nowbuffer].save = 1;
                  
                  
                  
                  nowbuffer++;
                  if( nowbuffer > 35)
                    nowbuffer = 0;
                }
                if( closer_flag == 0 )//»· + ­«½Æ = §R°£data
                {
                  for( int a=0;a<devicenumbers;a++ )//§PÂ_¨ì²Ä´X­ÓBuffer·|­«½Æ
                  {
                  if(
                      pEvent->deviceInfo.pEvtData[0] == device[a].data[0] &&
                      pEvent->deviceInfo.pEvtData[1] == device[a].data[1] &&
                      pEvent->deviceInfo.pEvtData[2] == device[a].data[2] &&
                      pEvent->deviceInfo.pEvtData[3] == device[a].data[3] &&
                      pEvent->deviceInfo.pEvtData[4] == device[a].data[4] &&
                      pEvent->deviceInfo.pEvtData[5] == device[a].data[5] &&
                      pEvent->deviceInfo.pEvtData[6] == device[a].data[6] &&
                      pEvent->deviceInfo.pEvtData[7] == device[a].data[7] &&
                      pEvent->deviceInfo.pEvtData[8] == device[a].data[8] &&
                      pEvent->deviceInfo.pEvtData[9] == device[a].data[9] &&
                      pEvent->deviceInfo.pEvtData[10] == device[a].data[10] &&
                      pEvent->deviceInfo.pEvtData[11] == device[a].data[11] &&
                      pEvent->deviceInfo.pEvtData[12] == device[a].data[12] &&
                      pEvent->deviceInfo.pEvtData[13] == device[a].data[13] &&
                      pEvent->deviceInfo.pEvtData[14] == device[a].data[14] &&
                      pEvent->deviceInfo.pEvtData[15] == device[a].data[15] &&
                      pEvent->deviceInfo.pEvtData[16] == device[a].data[16] &&
                      pEvent->deviceInfo.pEvtData[17] == device[a].data[17] &&
                      pEvent->deviceInfo.pEvtData[18] == device[a].data[18] &&
                      pEvent->deviceInfo.pEvtData[19] == device[a].data[19] &&
                      pEvent->deviceInfo.pEvtData[20] == device[a].data[20] &&
                      pEvent->deviceInfo.pEvtData[21] == device[a].data[21] &&
                      pEvent->deviceInfo.pEvtData[22] == device[a].data[22] &&
                      pEvent->deviceInfo.pEvtData[23] == device[a].data[23] &&
                      pEvent->deviceInfo.pEvtData[24] == device[a].data[24] 
                     )
                    {
                      device[a].protection = 1;//   x­ÓCLK´`Àô«á¬í²M°£
                      break;
                    }
                  }//§PÂ_¨ì²Ä´X­ÓBuffer·|­«½Æµ²§À
                }   
             }
           }
            else if(pEvent->deviceInfo.pEvtData[0] == 'C' && pEvent->deviceInfo.pEvtData[1] == 'T')
            {
              //­«½Æ§_
              if( check_buffer(pEvent->deviceInfo.pEvtData,24)==0 )
              {
                checksame = 0;//¤@¼Ë
              }
              
              else
              {
                checksame = 1;//¤£¤@¼Ë
              }
              
              //¦Û¤v§_
              if(
                 pEvent->deviceInfo.pEvtData[10] ==  MyLight_lon_H      &&
                 pEvent->deviceInfo.pEvtData[11] ==  MyLight_lon_L      &&
                 pEvent->deviceInfo.pEvtData[12] ==  MyLight_lat_H       &&
                 pEvent->deviceInfo.pEvtData[13] ==  MyLight_lat_L
                 )
              {
                self = 1;
              }
              //«D¦Û¤v
              else
              {
                self = 0;
              }
              
              
              if(checksame == 1)//°²³]¸ê®ÆµL­«½Æ
              {
                if(self == 1)//°²³]¸ê®ÆµL­«½Æ,¬O¦Û¤v¥ô°È
                {
                  memcpy(device[nowbuffer].data,pEvent->deviceInfo.pEvtData,31);    
                  
                  //±N«á­±´«¦¨¦Û§Ú¸g½n°ª
                  device[nowbuffer].data[25] = MyLight_lon_H;
                  device[nowbuffer].data[26] = MyLight_lon_L;
                  device[nowbuffer].data[27] = MyLight_lat_H;
                  device[nowbuffer].data[28] = MyLight_lat_L;
                  device[nowbuffer].data[30] = MyHigh;
                  device[nowbuffer].save = 1;
                  device[nowbuffer].wait = 1;
                  device[nowbuffer].self = 1;
                  device[nowbuffer].protection = 1;//ÀH«á§R°£
                  
                  nowbuffer++;
                  if( nowbuffer > 35)
                    nowbuffer = 0; 
                }
                else//°²³]¸ê®ÆµL­«½Æ,¤£¬O¦Û¤v¥ô°È,­n¬Ý»·ªñ¨M©w°Ê§@»P§_
                {
                  //¥ý§ì¥X¸Ó¸ê®Æ¤§²×ÂI
                  CT_lon_H = pEvent->deviceInfo.pEvtData[10];
                  CT_lon_L = pEvent->deviceInfo.pEvtData[11];
                  CT_lat_H = pEvent->deviceInfo.pEvtData[12];
                  CT_lat_L = pEvent->deviceInfo.pEvtData[13];
                  CT_High_H = pEvent->deviceInfo.pEvtData[14];
                  CT_High_L = pEvent->deviceInfo.pEvtData[15];
                  
                  //¦X¦¨¸g½n«×
                  CT_target_lon  = ((CT_lon_H & 0xF0)>>4)*1000 + (CT_lon_H & 0x0F)*100 + ((CT_lon_L & 0xF0)>>4)*10 + (CT_lon_L & 0x0F);
                  CT_target_lat  = ((CT_lat_H & 0xF0)>>4)*1000 + (CT_lat_H & 0x0F)*100 + ((CT_lat_L & 0xF0)>>4)*10 + (CT_lat_L & 0x0F);
                     
                  //§ì¥X¸Ó¸ê®Æ²{¦b¶Ç¨ì­þ
                  Data_lon_H = pEvent->deviceInfo.pEvtData[25];
                  Data_lon_L = pEvent->deviceInfo.pEvtData[26];
                  Data_lat_H = pEvent->deviceInfo.pEvtData[27];
                  Data_lat_L = pEvent->deviceInfo.pEvtData[28];
                  Data_High_H = pEvent->deviceInfo.pEvtData[29];
                  Data_High_L = pEvent->deviceInfo.pEvtData[30];
                  
                  //¦X¦¨¸g½n«×
                  Data_lon = ((Data_lon_H & 0xF0)>>4)*1000 + (Data_lon_H & 0x0F)*100 + ((Data_lon_L & 0xF0)>>4)*10 + (Data_lon_L & 0x0F);//¸g«×
                  Data_lat = ((Data_lat_H & 0xF0)>>4)*1000 + (Data_lat_H & 0x0F)*100 + ((Data_lat_L & 0xF0)>>4)*10 + (Data_lat_L & 0x0F);//½n«×
                  

                  //¤ñ¸û¸ê®Æ²{¦b©M¥Ø¼Ð®t¦h»·
                  FinalOthLight_dis_float_2 = pow( CT_target_lon-Data_lon,2);//¸g
                  FinalOthLight_dis_float_1 = pow( CT_target_lat-Data_lat,2);//½n
                  FinalOthLight_dis = sqrt(FinalOthLight_dis_float_1+FinalOthLight_dis_float_2);
                  
                  //¤ñ¸û§Ú¥»¨­©M¥Lªº¥Ø¼Ð®t¦h»·
                  FinalMylight_dis_float_1 = pow( CT_target_lon-MyLight_lon,2);//¸g
                  FinalMylight_dis_float_2 = pow( CT_target_lat-MyLight_lat,2);//½n
                  FinalMylight_dis = sqrt(FinalMylight_dis_float_1+FinalMylight_dis_float_2);    
                  
                  
                  //¤ñ¸û¥L¦Û¤v©M¥Ø¼Ðªº¶ZÂ÷ ¬O§_¦³¤ñ§Ú¨ì¥Ø¼ÐÁÙªñ
                  //¦pªG¥L©M¥Ø¼Ðªº¶ZÂ÷®t  ¤ñ§Ú¨ì¥L¥Ø¼Ðªº¶ZÂ÷®tÁÙ¤p ¥Nªí¥L¤ñ§Úªñ
                  
                  if(FinalOthLight_dis < FinalMylight_dis)
                    closer_flag = 1;//³o¸ê®Æªº¶ZÂ÷®t¤ñ§Úªñ §Ú¥u¯à®³¨Ó§R°£©Î¤£²z
                  else
                    closer_flag = 0;//³o¸ê®Æªº¶ZÂ÷®t¤ñ§Ú»· §Ú¸ÓÀ°¥L°e
                  
                  if(closer_flag == 0)
                  {
                    memcpy(device[nowbuffer].data,pEvent->deviceInfo.pEvtData,31);
                    device[nowbuffer].data[25] = MyLight_lon_H;
                    device[nowbuffer].data[26] = MyLight_lon_L;
                    device[nowbuffer].data[27] = MyLight_lat_H;
                    device[nowbuffer].data[28] = MyLight_lat_L;
                    device[nowbuffer].data[30] = MyHigh;
                    device[nowbuffer].save = 1;
                    device[nowbuffer].wait = 1;
                    device[nowbuffer].self = 1;
                    
                    nowbuffer++;
                    if( nowbuffer > 35)
                      nowbuffer = 0;
                  }
                }
              }
              else//°²³]¸ê®Æ­«½Æ,¥u­n¤ñ»·ªñ,§ä¥X¶ZÂ÷®t»·ªº,¨Ó§R°£ACK
              {  
                //¥ý§ì¥X¸Ó¸ê®Æ¤§²×ÂI
                CT_lon_H = pEvent->deviceInfo.pEvtData[10];
                CT_lon_L = pEvent->deviceInfo.pEvtData[11];
                CT_lat_H = pEvent->deviceInfo.pEvtData[12];
                CT_lat_L = pEvent->deviceInfo.pEvtData[13];
                CT_High_H = pEvent->deviceInfo.pEvtData[14];
                CT_High_L = pEvent->deviceInfo.pEvtData[15];
                
                //¦X¦¨¸g½n«×
                CT_target_lon  = ((CT_lon_H & 0xF0)>>4)*1000 + (CT_lon_H & 0x0F)*100 + ((CT_lon_L & 0xF0)>>4)*10 + (CT_lon_L & 0x0F);
                CT_target_lat  = ((CT_lat_H & 0xF0)>>4)*1000 + (CT_lat_H & 0x0F)*100 + ((CT_lat_L & 0xF0)>>4)*10 + (CT_lat_L & 0x0F);
                   
                //§ì¥X¸Ó¸ê®Æ²{¦b¶Ç¨ì­þ
                Data_lon_H = pEvent->deviceInfo.pEvtData[25];
                Data_lon_L = pEvent->deviceInfo.pEvtData[26];
                Data_lat_H = pEvent->deviceInfo.pEvtData[27];
                Data_lat_L = pEvent->deviceInfo.pEvtData[28];
                Data_High_H = pEvent->deviceInfo.pEvtData[29];
                Data_High_L = pEvent->deviceInfo.pEvtData[30];
                
                //¦X¦¨¸g½n«×
                Data_lon = ((Data_lon_H & 0xF0)>>4)*1000 + (Data_lon_H & 0x0F)*100 + ((Data_lon_L & 0xF0)>>4)*10 + (Data_lon_L & 0x0F);//¸g«×
                Data_lat = ((Data_lat_H & 0xF0)>>4)*1000 + (Data_lat_H & 0x0F)*100 + ((Data_lat_L & 0xF0)>>4)*10 + (Data_lat_L & 0x0F);//½n«×
                

                //¤ñ¸û¸ê®Æ²{¦b©M¥Ø¼Ð®t¦h»·
                FinalOthLight_dis_float_2 = pow( CT_target_lon-Data_lon,2);//¸g
                FinalOthLight_dis_float_1 = pow( CT_target_lat-Data_lat,2);//½n
                FinalOthLight_dis = sqrt(FinalOthLight_dis_float_1+FinalOthLight_dis_float_2);
                
                //¤ñ¸û§Ú¥»¨­©M¥Lªº¥Ø¼Ð®t¦h»·
                FinalMylight_dis_float_1 = pow( CT_target_lon-MyLight_lon,2);//¸g
                FinalMylight_dis_float_2 = pow( CT_target_lat-MyLight_lat,2);//½n
                FinalMylight_dis = sqrt(FinalMylight_dis_float_1+FinalMylight_dis_float_2);    
                
                
                
                if(FinalOthLight_dis < FinalMylight_dis)
                {
                  //§ì¥X­þ­Óbuffer­«½Æ
                  for(int i=0;i<devicenumbers;i++)
                  {
                    if(        
                        pEvent->deviceInfo.pEvtData[0] == device[i].data[0] &&
                        pEvent->deviceInfo.pEvtData[1] == device[i].data[1] &&
                        pEvent->deviceInfo.pEvtData[2] == device[i].data[2] &&
                        pEvent->deviceInfo.pEvtData[3] == device[i].data[3] &&
                        pEvent->deviceInfo.pEvtData[4] == device[i].data[4] &&
                        pEvent->deviceInfo.pEvtData[5] == device[i].data[5] &&
                        pEvent->deviceInfo.pEvtData[6] == device[i].data[6] &&
                        pEvent->deviceInfo.pEvtData[7] == device[i].data[7] &&
                        pEvent->deviceInfo.pEvtData[8] == device[i].data[8] &&
                        pEvent->deviceInfo.pEvtData[9] == device[i].data[9] &&
                        pEvent->deviceInfo.pEvtData[10] == device[i].data[10] &&
                        pEvent->deviceInfo.pEvtData[11] == device[i].data[11] &&
                        pEvent->deviceInfo.pEvtData[12] == device[i].data[12] &&
                        pEvent->deviceInfo.pEvtData[13] == device[i].data[13] &&
                        pEvent->deviceInfo.pEvtData[14] == device[i].data[14] &&
                        pEvent->deviceInfo.pEvtData[15] == device[i].data[15] &&
                        pEvent->deviceInfo.pEvtData[16] == device[i].data[16] &&
                        pEvent->deviceInfo.pEvtData[17] == device[i].data[17] &&
                        pEvent->deviceInfo.pEvtData[18] == device[i].data[18] &&
                        pEvent->deviceInfo.pEvtData[19] == device[i].data[19] &&
                        pEvent->deviceInfo.pEvtData[20] == device[i].data[20] &&
                        pEvent->deviceInfo.pEvtData[21] == device[i].data[21] &&
                        pEvent->deviceInfo.pEvtData[22] == device[i].data[22] &&
                        pEvent->deviceInfo.pEvtData[23] == device[i].data[23] &&
                        pEvent->deviceInfo.pEvtData[24] == device[i].data[24] 
                        )
                    {
                      device[i].protection = 1;
                      break;
                    }
                  }       
                }    
              }  
            }
         }
        }

        /*
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr, 
                                           pEvent->deviceInfo.addrType);
          }
        }*/
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;
        
        for(int a=0;a<devicenumbers;a++)//¥Nªí¸ê®Æ¥Hªñ¨Ó ¶}©l­pºâ¸ê®Æ«OÅ@®É¶¡
        {
          if(device[a].wait == 1)
          {
            device[a].count++;
          }
        }
        
        for(int a=0;a<devicenumbers;a++)
        {
          if(device[a].save == 1)//¥Nªí±N­n¶Ç¹L¥h¼s¼½
          {
            scifUartTxPutChars(device[a].data,31);
            device[a].save = 0;
            device[a].protection = 1;
          }
          else if(device[a].protection == 1)//¥Nªí±N­n§âCLR¶Ç¹L¥h
          {
            device[a].data[0] = 0xAC;
            scifUartTxPutChars(device[a].data,31);
            device[a].protection = 0;
            device[a].wait_over = 1;
          }
          
          if( device[a].count > CLK && device[a].wait_over == 1)//¥Nªí¥ô°Èµ²§ô,¨Ã¥B¸g¹LCLK­Ó´`Àô
          {
            for(int b=0;b<31;b++)
              device[a].data[b] = 0;

            device[a].count = 0;
            device[a].protection = 0;
            device[a].save = 0;  
            device[a].wait = 0;
            device[a].wait_over = 0;
          }
        }
        
        

        
        
        
/*
        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }
        
        LCD_WRITE_STRING_VALUE("Devices Found", scanRes, 10, LCD_PAGE2);
        
        if (scanRes > 0)
        {
          LCD_WRITE_STRING("<- To Select", LCD_PAGE3);
        }

        // initialize scan index to last device
        scanIdx = scanRes;
*/ 
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST); 
        
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          LCD_WRITE_STRING("Connected", LCD_PAGE2);
          LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr),
                           LCD_PAGE3);   
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING("Connect Failed", LCD_PAGE2);
          LCD_WRITE_STRING_VALUE("Reason:", pEvent->gap.hdr.status, 10, 
                                 LCD_PAGE3);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;
        procedureInProgress = FALSE;
          
        // Cancel RSSI reads
        SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);
        
        LCD_WRITE_STRING("Disconnected", LCD_PAGE2);
        LCD_WRITE_STRING_VALUE("Reason:", pEvent->linkTerminate.reason,
                                10, LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING_VALUE("Param Update:", pEvent->linkUpdate.status,
                                10, LCD_PAGE2);
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
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
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Display discovery results
    if (!scanningStarted && scanRes > 0)
    {
      // Increment index of current result (with wraparound)
      scanIdx++;
      if (scanIdx >= scanRes)
      {
        scanIdx = 0;
      }

      LCD_WRITE_STRING_VALUE("Device", (scanIdx + 1), 10, LCD_PAGE2);
      LCD_WRITE_STRING(Util_convertBdAddr2Str(devList[scanIdx].addr), LCD_PAGE3);
    }

    return;
  }

  if (keys & KEY_UP)
  {
    // Start or stop discovery
    if (state != BLE_STATE_CONNECTED)
    {
      if (!scanningStarted)
      {
        scanningStarted = TRUE;
        scanRes = 0;
        
        LCD_WRITE_STRING("Discovering...", LCD_PAGE2);
        LCD_WRITE_STRING("", LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
        
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    else if (state == BLE_STATE_CONNECTED &&
             charHdl != 0                 &&
             procedureInProgress == FALSE)
    {
      uint8_t status;

      // Do a read or write as long as no other read or write is in progress
      if (doWrite)
      {
        // Do a write
        attWriteReq_t req;

        req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
        if ( req.pValue != NULL )
        {
          req.handle = charHdl;
          req.len = 1;
          req.pValue[0] = charVal;
          req.sig = 0;
          req.cmd = 0;

          status = GATT_WriteCharValue(connHandle, &req, selfEntity);
          if ( status != SUCCESS )
          {
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
          }
        }
        else
        {
          status = bleMemAllocError;
        }
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = charHdl;
        status = GATT_ReadCharValue(connHandle, &req, selfEntity);
      }

      if (status == SUCCESS)
      {
        procedureInProgress = TRUE;
        doWrite = !doWrite;
      }
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    // Connection update
    if (state == BLE_STATE_CONNECTED)
    {
      GAPCentralRole_UpdateLink(connHandle,
                                DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                DEFAULT_UPDATE_SLAVE_LATENCY,
                                DEFAULT_UPDATE_CONN_TIMEOUT);
    }
    
    return;
  }

  if (keys & KEY_SELECT)
  {
    uint8_t addrType;
    uint8_t *peerAddr;
    
    // Connect or disconnect
    if (state == BLE_STATE_IDLE)
    {
      // if there is a scan result
      if (scanRes > 0)
      {
        // connect to current device in scan result
        peerAddr = devList[scanIdx].addr;
        addrType = devList[scanIdx].addrType;
      
        state = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);
  
        LCD_WRITE_STRING("Connecting", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddr), LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
      }
    }
    else if (state == BLE_STATE_CONNECTING ||
              state == BLE_STATE_CONNECTED)
    {
      // disconnect
      state = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink(connHandle);
      
      LCD_WRITE_STRING("Disconnecting", LCD_PAGE2);
      LCD_WRITE_STRING("", LCD_PAGE4);
    }

    return;
  }

  if (keys & KEY_DOWN)
  {
    // Start or cancel RSSI polling
    if (state == BLE_STATE_CONNECTED)
    {
      if (SimpleBLECentral_RssiFind(connHandle) == NULL)
      {
        SimpleBLECentral_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
      }
      else
      {
        SimpleBLECentral_CancelRssi(connHandle);

        LCD_WRITE_STRING("RSSI Cancelled", LCD_PAGE4);
      }
    }

    return;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      LCD_WRITE_STRING_VALUE("ATT Rsp dropped", pMsg->method, 10, LCD_PAGE4);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {      
        LCD_WRITE_STRING_VALUE("Read Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE4);
      }
      else
      {
        // After a successful read, display the read value
        LCD_WRITE_STRING_VALUE("Read rsp:", pMsg->msg.readRsp.pValue[0], 10,
                               LCD_PAGE4);
      }
      
      procedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {     
        LCD_WRITE_STRING_VALUE("Write Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE4);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        LCD_WRITE_STRING_VALUE("Write sent:", charVal++, 10, LCD_PAGE4);
      }
      
      procedureInProgress = FALSE;    

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.
      
      // Display the opcode of the message that caused the violation.
      LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                             10, LCD_PAGE4);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {   
      // MTU size updated
      LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE4);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleBLECentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.
  
  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        int8 rssi = (int8)pMsg->pReturnParam[3];

        LCD_WRITE_STRING_VALUE("RSSI -dB:", (uint32_t)(-rssi), 10, LCD_PAGE4);
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_StartRssi
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   period - RSSI read period in ms
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static bStatus_t SimpleBLECentral_StartRssi(uint16_t connHandle, uint16_t period)
{
  readRssi_t *pRssi;

  // Verify link is up
  if (!linkDB_Up(connHandle))
  {
    return bleIncorrectMode;
  }

  // If already allocated
  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);
    
    pRssi->period = period;
  }
  // Allocate structure
  else if ((pRssi = SimpleBLECentral_RssiAlloc(connHandle)) != NULL)
  {
    pRssi->period = period;
  }
  // Allocate failed
  else
  {
    return bleNoResources;
  }

  // Start timer
  Util_restartClock(pRssi->pClock, period);

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimpleBLECentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle)
{
  readRssi_t *pRssi;

  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    // Free RSSI structure
    SimpleBLECentral_RssiFree(connHandle);

    return SUCCESS;
  }

  // Not found
  return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiAlloc
 *
 * @brief   Allocate an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if allocation failed.
 */
static readRssi_t *SimpleBLECentral_RssiAlloc(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == GAP_CONNHANDLE_ALL)
    {
      readRssi_t *pRssi = &readRssi[i];
      
      pRssi->pClock = (Clock_Struct *)ICall_malloc(sizeof(Clock_Struct));
      if (pRssi->pClock)
      {
        Util_constructClock(pRssi->pClock, SimpleBLECentral_readRssiHandler,
                            0, 0, false, i);
        pRssi->connHandle = connHandle;
        
        return pRssi;
      }
    }
  }

  // No free structure found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      return &readRssi[i];
    }
  }

  // Not found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleBLECentral_RssiFree(uint16_t connHandle)
{
  uint8_t i;

  // Find RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      readRssi_t *pRssi = &readRssi[i];
      if (pRssi->pClock)
      {
        Clock_destruct(pRssi->pClock);
        
        // Free clock struct
        ICall_free(pRssi->pClock);
        pRssi->pClock = NULL;
      }
      
      pRssi->connHandle = GAP_CONNHANDLE_ALL;
      break;
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    LCD_WRITE_STRING("Pairing started", LCD_PAGE2);
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Pairing success", LCD_PAGE2);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Pairing fail:", status, 10, LCD_PAGE2);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Bonding success", LCD_PAGE2);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Bond save success", LCD_PAGE2);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Bond save failed:", status, 10, LCD_PAGE2);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    LCD_WRITE_STRING_VALUE("Passcode:", passcode, 10, LCD_PAGE4);
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;
  
  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;
    
  discState = BLE_DISC_STATE_MTU;
  
  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;
  
  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{ 
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
      
      // Just in case we're using the default MTU size (23 octets)
      LCD_WRITE_STRING_VALUE("MTU Size:", ATT_MTU_SIZE, 10, LCD_PAGE4);
        
      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }
    
    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) && 
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;
          
        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && 
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);
      
      LCD_WRITE_STRING("Simple Svc Found", LCD_PAGE2);
      procedureInProgress = FALSE;
    }
    
    discState = BLE_DISC_STATE_IDLE;
  }    
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, 
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) || 
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;
  
  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }
    
    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;
    
    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT, 
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;  
  
    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;
    
    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
  SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS, 
                              (uint8_t *)&readRssi[a0]);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state, 
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));
  
  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;
    
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }
  
  return FALSE;
}
/*********************************************************************
 * @fn      GUA_Read_Mac
 *
 * @brief   Read MAC
 *
 * @return  none
 */
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

static void delaysec(int sec_use)
{
  uint32_t timeNow = 1426692075;
  Seconds_set(timeNow);

  while( (sec = Seconds_get()) != timeNow + sec_use) {} // <- This actually waits for 5 seconds!

  t   = time(NULL);
}


bool check_buffer( char inputbuffer[] , int count)
{
  bool answer = 1;
  
  for(int a=0; a<36; a++)
  {
    if( memcmp( device[a].data, inputbuffer,count ) == 0) // °²¦p¦³¤@¼Ò¤@¼Ëªº ¿é¥X0
    {
      answer = 0;
      return answer; //¦³§¹¥þ¤@¼Ëªº ¿é¥X0
    }
  }
  return answer;//³£¤£¤@¼Ë ¿é¥X1
}