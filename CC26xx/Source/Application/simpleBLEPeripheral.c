/*******************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2016-01-07 16:59:59 -0800 (Thu, 07 Jan 2016) $
  Revision:       $Revision: 44594 $

  Description:    This file contains the Simple BLE Peripheral sample 
                  application for use with the CC2650 Bluetooth Low Energy 
                  Protocol Stack.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

int i =0;

#include <string.h>



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
#include "osal_snv.h"   //FLASH
#include "ICallBleAPIMSG.h"
#include "util.h"
#include "board_lcd.h"
#include "board_key.h"


//ADC
#include "Board.h"
#include "simpleBLEPeripheral.h"
#include <ti/drivers/lcd/LCDDogm1286.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>

#include <ti/drivers/PIN/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include <inc/hw_aux_evctl.h>

//PWM
#include <ti/drivers/PIN.h>
#include <ti/drivers/PWM2.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Log.h>

//UART
#include <scif.h>

//ÅªMAC ADDRESS
static void readMac(uint8_t * mac_address);

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

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
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

//2017.06.13
#define SBP_GUA_PERIODIC_EVT_PERIOD           2000
//2017.06.26 ¼s¼½¨Æ¥ó
#define SBP_LIN_PERIODIC_EVT_PERIOD           5000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
//06.13
#define SBP_GUA_PERIODIC_EVT                  0x0020
//06.13
#define SBP_LIN_PERIODIC_EVT                  0x0010


#define LED_ON                        1               /* LEDs on CC2650 are active high */
#define LED_OFF                       0
 uint16   gapConnHandle;
 uint8 temp[SIMPLEPROFILE_CHAR6_LEN]; 
 
 //UART
 
Task_Struct my_UART_Task;
Char my_UART_TaskStack[512];
static Semaphore_Struct semScTaskAlert;
int rxbuf[80];
int txbuf[80];
 
 //PWM
 
PWM_Handle  hPWM;
char  PWM_taskStack[512];
Task_Struct PWM_taskStruct;

#define PERIOD_US 1000
 
 //ADC
#define SAMPLECOUNT 8
#define SAMPLETYPE uint16_t
#define SAMPLESIZE sizeof(SAMPLETYPE)
#define TASK_STACK_SIZE 512
#define TASK_PRI        1

Semaphore_Struct sem_1;
Semaphore_Handle hSem;
Hwi_Struct hwi;
SAMPLETYPE adcSamples[SAMPLECOUNT];
SAMPLETYPE singleSample;


const PIN_Config alsPins[] = {
  ALS_POWER       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  ALS_OUTPUT      | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS ,
  PIN_TERMINATE
};

PIN_Handle pinHandle;
PIN_State  pinState_1;
UART_Handle uHandle;
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;



//PWM

typedef struct   
{  
    uint8 PWM_P10;              //P10ªºpwm­È  
    uint8 PWM_P11;              //P11ªºpwm­È         
}LED_CONFIG;  
LED_CONFIG led_config;  

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

//2017.06.13
//static Clock_Struct GUA_periodicClock;
//2017.06.26
static Clock_Struct LIN_periodicClock;

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

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];


/*********************************************************************
 *  ·s¼W
 *
 */
 char flash_buf[252]={0};   



//ADC
char taskStack[TASK_STACK_SIZE];
Task_Struct taskStruct;


// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,

  
  0x53,   // 'S' 
  0x69,   // 'i' 
  0x6d,   // 'm' 
  0x70,   // 'p' 
  0x6c,   // 'l' 
  0x65,   // 'e' 
  // mac address
  0x07,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  
  /*
  0x42,   // 'B' 
  0x4c,   // 'L' 
  0x45,   // 'E' 
  0x50,   // 'P' 
  0x65,   // 'e' 
  0x72,   // 'r' 
  0x69,   // 'i' 
  0x70,   // 'p' 
  0x68,   // 'h' 
  0x65,   // 'e' 
  0x72,   // 'r' 
  0x61,   // 'a' 
  0x6c,   // 'l' 
  */
 
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

static uint8_t scanRspData_LIN[] =
{
  0xAA,
  0xBB,
  0xCC,
  0xFF
};

uint8_t newValue_1[20];
// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
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
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */


static void SimpleBLEPeripheral_init( void )    ;
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);
static void taskFxn(UArg a0, UArg a1);
//PWM
static void PWM_taskFxn(UArg a0, UArg a1);
//UART
static void Uart_taskFxn(UArg a0, UArg a1);

static void adcIsr(UArg a0);
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(
                                                     gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);

//2017.06.13
//static void GUA_performPeriodicTask(void);  
//2017.06.26
//static void LIN_performPeriodicTask(void);  

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
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
#ifndef FEATURE_OAD
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

//UART
void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback


void scTaskAlertCallback(void) {
  
    int a;

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

    // Wait for an ALERT callback
    Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    // Echo all characters currently in the RX FIFO
    int rxFifoCount = scifUartGetRxFifoCount();

    //Åª¥X³o¦¸buffer
    for(a=0;a<rxFifoCount;a++)
    {
     rxbuf[a] =(      (char) scifUartRxGetChar()      );
     
    }
    // Clear the events that triggered this
    scifUartClearEvents();

    // Acknowledge the alert event
    scifAckAlertEvents();   
    
    for(int b=0;b<rxFifoCount;b++)

    (    scifUartTxPutChar(rxbuf[b]) )   ; 
    
    

} // scTaskAlertCallback

  


/**********************************************************************
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

  //ADC
void ADC_createTask(void)
{
//Initialize task
  Task_Params params;
  Task_Params_init(&params);
  params.priority = TASK_PRI;
  params.stackSize = TASK_STACK_SIZE;
  params.stack = taskStack;

  Task_construct(&taskStruct, taskFxn, &params, NULL);

 
  // Construct semaphore used for pending in task
  Semaphore_Params sParams;
  Semaphore_Params_init(&sParams);
  sParams.mode = Semaphore_Mode_BINARY;

  Semaphore_construct(&sem_1, 0, &sParams);
  hSem = Semaphore_handle(&sem_1);
  
}

  
//PWM
void PWM_createTask(void)
{
  Task_Params params;
  Task_Params_init(&params);
  params.priority = 1;
  params.stackSize = sizeof(PWM_taskStack);
  params.stack = PWM_taskStack;

  Task_construct(&PWM_taskStruct, PWM_taskFxn, &params, NULL);
}

//UART
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
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  
  ICall_registerApp(&selfEntity, &sem);
 osal_snv_read(0x80,  sizeof(char)*252, flash_buf);
  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(500);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
 /*  //2017.06.13  
 Util_constructClock(&GUA_periodicClock, SimpleBLEPeripheral_clockHandler,  
                    SBP_GUA_PERIODIC_EVT_PERIOD,500, false, SBP_GUA_PERIODIC_EVT); 
 
 Util_startClock(&GUA_periodicClock);    */
 
   /*//2017.06.26
 Util_constructClock(&LIN_periodicClock, SimpleBLEPeripheral_clockHandler,  
                    SBP_LIN_PERIODIC_EVT_PERIOD,0, false, SBP_LIN_PERIODIC_EVT); 
 
 Util_startClock(&LIN_periodicClock);   */
  
#ifndef SENSORTAG_HW
  Board_openLCD();
#endif //SENSORTAG_HW
  
#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW
  
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    
    uint8_t initialAdvertEnable = TRUE; //¶}±Ò¼s¼½

      // By setting this to zero, the device will go into the waiting state after
      // being discoverable for 30.72 second, and will not being advertising again
      // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    
    readMac(&scanRspData[10]);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
 
    
    
    
    
    
    

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);
    
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), //¤£­«­n©ñscanrs
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData); //­«­n¸ê®Æ©ñadver
    
    
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA,sizeof(scanRspData_LIN), //¤£­«­n©ñscanrs
                         scanRspData_LIN);

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

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

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

#ifndef FEATURE_OAD
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD

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
    uint8_t charValue3[SIMPLEPROFILE_CHAR3_LEN] ={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}; 
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8_t charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 1, 2, 3, 4, 5 ,6 ,7 ,8,9,10,11,12,13,14,15,16,17,18,19,20}; 

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN,
                               charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN,
                               charValue6);
    
    
    
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD
  
  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
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

//UART
static void Uart_taskFxn (UArg a0, UArg a1) {

    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Start the UART emulator
    scifExecuteTasksOnceNbl(BV(SCIF_UART_EMULATOR_TASK_ID));

    // Enable baud rate generation
    scifUartSetBaudRate(57600);   //¥]²v

    // Enable RX (10 idle bit periods required before enabling start bit detection)
    scifUartSetRxFifoThr(SCIF_UART_RX_FIFO_MAX_COUNT / 2);
    scifUartSetRxTimeout(10 * 2);
    scifUartSetRxEnableReqIdleCount(10 * 2);
    scifUartRxEnable(1);

    // Enable events (half full RX FIFO or 10 bit period timeout
    scifUartSetEventMask(BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR | BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT);


}



  //PWM
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
 
  if(hPWM == NULL) {
    Log_error0("Opening PWM failed");
    while(1);
  }

  PWM_start(hPWM);


  bool direction = 1; /* Initially increase duty */
 
while(1){
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
 
}}
//ADC
void taskFxn(UArg a0, UArg a1) {


  Hwi_Params hwiParams;
  Hwi_Params_init(&hwiParams);
  hwiParams.enableInt = true;
  Hwi_construct(&hwi, INT_AUX_ADC, adcIsr, &hwiParams, NULL);
  
  
   // Set up pins
  pinHandle = PIN_open(&pinState_1, alsPins);
  
  // Enable clock for ADC digital and analog interface (not currently enabled in driver)

  AUXWUCClockEnable(AUX_WUC_ADI_CLOCK|AUX_WUC_SOC_CLOCK);

  // Connect AUX IO7 (DIO23) as analog input. Light sensor on SmartRF06EB

  AUXADCSelectInput(ADC_COMPB_IN_AUXIO7);

  // Set up ADC
  AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
  
  uint8_t currentSample = 0;
 
    //while(currentSample < SAMPLECOUNT) 
  while(1) 
    {


    //Sleep 100ms in IDLE mode
    Task_sleep(100 * 1000 / Clock_tickPeriod);

    // Trigger ADC sampling
    AUXADCGenManualTrigger();
    // Wait in IDLE until done
    Semaphore_pend(hSem, BIOS_WAIT_FOREVER );

    adcSamples[currentSample++] = singleSample;

    if (currentSample ==8) currentSample=0;
  }
/*
  // Disable ADC
  AUXADCDisable();
  // Allow STANDBY mode again
  Power_releaseConstraint(Power_SB_DISALLOW);

  // Restore pins to values in BoardGpioTable
  PIN_close(pinHandle);

  // Log data through UART
  
  for(uint8_t i = 0; i<SAMPLECOUNT; i++ ) {
    UART_write(uHandle, &adcSamples[i], SAMPLESIZE);

  }

  // Goto STANDBY forever
  Task_sleep(BIOS_WAIT_FOREVER);
*/

}

void adcIsr(UArg a0) 
{

  // Pop sample from FIFO to allow clearing ADC_IRQ event
  singleSample = AUXADCReadFifo();
  // Clear ADC_IRQ flag. Note: Missing driver for this.
  HWREGBITW(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR, AUX_EVCTL_EVTOMCUFLAGSCLR_ADC_IRQ_BITN) = 1;

  // Post semaphore to wakeup task
  Semaphore_post(hSem);

}



PIN_Config pinConfig[] =  { PIN_ID(Board_PWMPIN0) | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX ,
                            PIN_TERMINATE };

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
            safeToDealloc = SimpleBLEPeripheral_processStackMsg(
                                                             (ICall_Hdr *)pMsg);
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

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      Util_startClock(&periodicClock);
 
      // Perform periodic application task
      SimpleBLEPeripheral_performPeriodicTask();
    }
//2017.06.13
    //¿O¸¹°{
    /*
     if (events & SBP_GUA_PERIODIC_EVT)
    {
 
      events &= ~SBP_GUA_PERIODIC_EVT;

          // Perform periodic application task
        Util_startClock(&GUA_periodicClock);

    
     GUA_performPeriodicTask();

    }*/

 
    /*
    int i= 0;
while(1)
{   
  PIN_setOutputValue(hSbpPins, Board_LED3,i=~i); 
  PIN_setOutputValue(hSbpPins, Board_LED4,i); 
  Task_sleep(10000); 
 }
 */
    
/*    //2017.06.26
    
     if (events & SBP_LIN_PERIODIC_EVT)
    {
 
      events &= ~SBP_LIN_PERIODIC_EVT;

          // Perform periodic application task
     Util_startClock(&LIN_periodicClock);

    
     LIN_performPeriodicTask();

    }*/


#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
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
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg(
                                                        (gattMsgEvent_t *)pMsg);
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
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, 
                          &(pAttRsp->msg));
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

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, 
                             systemId);

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
          SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
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

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
        Util_startClock(&periodicClock);
        
        
//³s½u«G¿O
        PIN_setOutputValue(hSbpPins, Board_LED3,1);
        
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
      Util_stopClock(&periodicClock);
      
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      LCD_WRITE_STRING("Disconnected", LCD_PAGE2);
     
      
      // Clear remaining lines
      LCD_WRITE_STRING("", LCD_PAGE3);
      LCD_WRITE_STRING("", LCD_PAGE4);
      LCD_WRITE_STRING("", LCD_PAGE5);
          //Â_½u·À¿O
      PIN_setOutputValue(hSbpPins, Board_LED3,0);
      PIN_setOutputValue(hSbpPins, Board_LED4,0);
      
      
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
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

#ifndef FEATURE_OAD
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
#endif //!FEATURE_OAD

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
  uint8_t buf[7];
 //uint8 temp[SIMPLEPROFILE_CHAR6_LEN];  
  

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);  
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &flash_buf); 
      
      if(newValue_1[0]==0x0011) //ÀË¬d¼g¤J­È¬O§_²Å¦X
       {
         
      // uint8 statel;
       uint16 len;
       
       uint8 handle;
       attHandleValueNoti_t notify_back;
       GAPRole_GetParameter(GAPROLE_CONNHANDLE,&handle);
       
       notify_back.pValue=(uint8 *)GATT_bm_alloc(gapConnHandle,ATT_HANDLE_VALUE_NOTI,
                           GATT_MAX_MTU,&len); //1.4&2.1¥H«á¶·¥[
              
       PIN_setOutputValue(hSbpPins, Board_LED3,0);
       PIN_setOutputValue(hSbpPins, Board_LED4,0);
       
       if(notify_back.pValue!=NULL)
       {
       notify_back.len=6;
       notify_back.handle = 0x0027;  //¼g¤J­È­Y¥¿½T¡A¦^¶ÇCHAR4³q¹D
       notify_back.pValue[0]=0x99;
       notify_back.pValue[1]=0x88;
       notify_back.pValue[2]=0x77;
       notify_back.pValue[3]=0x66;
       notify_back.pValue[4]=0x55;
       notify_back.pValue[5]=0x44;
       
 
       GATT_Notification(gapConnHandle,&notify_back,FALSE);

       }
       else
       {
       GAPRole_TerminateConnection();
       }

       
       }
      
      
      
      if(flash_buf[0]==0x0081)
      {
        for(int b=0 ; b<252 ; b++)
        {
    
       flash_buf[b]=flash_buf[0];
       
        }
          osal_snv_write(0x80,  sizeof(char)*252, flash_buf);
          osal_snv_read(0x80,  sizeof(char)*252, flash_buf);
      }
      

      LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      
      break;

   case SIMPLEPROFILE_CHAR6:
       SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR6, &buf);
     
   
       
   
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
#ifndef FEATURE_OAD
  //uint8_t valueToCopy;

   //Call to retrieve the value of the third characteristic in the profile
     
     
 /* if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
  

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
    
  }*/
  
  

  
  
#endif //!FEATURE_OAD
}


//2017.06.13
/*
bool j=1;
static void GUA_performPeriodicTask(void)  
{  



PIN_setOutputValue(hSbpPins, Board_LED3,j?j--:j++); 
PIN_setOutputValue(hSbpPins, Board_LED4,j); 
  

   
}*/
 
//2017.06.26
/*
static void LIN_performPeriodicTask(void)  
{  
  
   

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA,sizeof(scanRspData_LIN), //¤£­«­n©ñscanrs
                         scanRspData_LIN);
    scanRspData_LIN[0] +=1;

}
 */

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

static void readMac(uint8_t * mac_address)  //¨úMAC ADDRESS
{    
	uint32_t mac0 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);  
	uint32_t mac1 = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);  

	*mac_address++ = HI_UINT16(mac1);
	*mac_address++ = LO_UINT16(mac1);
	*mac_address++ = BREAK_UINT32(mac0, 3);
	*mac_address++ = BREAK_UINT32(mac0, 2);
	*mac_address++ = BREAK_UINT32(mac0, 1);
	*mac_address++ = BREAK_UINT32(mac0, 0);
}
/*********************************************************************
*********************************************************************/

