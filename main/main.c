/***************************************************************************
* File name :   main.c                                           *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/i2c.h"
#include "SCI.h"
#include "CircularBuffer.h"
#include "FrontPanel.h"
#include "MotionSensor.h"
#include "ADC.h"
#include "DeepSleep.h"

#include "freertos/event_groups.h"
#include "esp_event.h"
#include <nvs_flash.h>

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_wifi.h"
#include "esp_bt.h"
// #include "driver/adc.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "esp_private/phy.h"


#define BTBUFF_SIZE 500
unsigned char BTBuff[BTBUFF_SIZE];
unsigned short BTBuffIndex = 0;


static bool notify_state;

static uint16_t conn_handle;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;


static TimerHandle_t blehr_tx_timer;

uint16_t HR=9;
/*****************************************************************************
* Function Name : ResetBTBuffer                                              *
*                                                                            *
* Description   : This function is used to reset the BT  buffer              *
*                                                                            *
* Arguments     : None                                                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void ResetBTBuffer(void)
{
    unsigned short i;
    //USART_ITConfig(USART2,USART_IT_RXNE, DISABLE);
    for(i = 0; i < BTBUFF_SIZE; i++){ BTBuff[i] = 0; }
    BTBuffIndex = 0;
    //ProcessPacket = 0;
    //USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
}
//extern static PeerToPeerContext_t aPeerToPeerContext;
void putcharBT(char Data)
{
    
    char TResponsePacket[2];
    TResponsePacket[0] = 0x01; 
    TResponsePacket[1] = Data;            
    //P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)TResponsePacket);
    osDelay(100); // Needed for app thread character collection, app cant process all in sequence due to threading.
    
//         result = aci_gatt_update_char_value(aPeerToPeerContext.PeerToPeerSvcHdle,
//                             aPeerToPeerContext.P2PNotifyServerToClientCharHdle,
//                              0, /* charValOffset */
//                             2, /* charValueLen */
//                             (uint8_t *)  pPayload);
}
void PrintBT(char *pData)
{
    unsigned short i;   
 
    for(i = 0; i < strlen((void*)pData); i++)
    {
        putcharBT(pData[i]);
    }
        
}

//char *TAG = "BLE-Server";
uint8_t ble_addr_type;
void ble_app_advertise(void);
int c=0;
char ParamBeingRead[20];
//char strb[500];

char tbstr[300];
char tbstr1[300];

void splitString(char *input, char *delimiter) {
    char *token = strtok(input, delimiter);

    while (token != NULL) {
        printf("%s\n", token);
        token = strtok(NULL, delimiter);
    }
}
// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char ParamBeingWritten[100];
    char delimiter[] = " | ";
    c+=ctxt->om->om_len;
    memset(tbstr1,0,sizeof(tbstr1));
    memcpy(tbstr1,ctxt->om->om_data,ctxt->om->om_len);
   
    
  
    ESP_LOGW(TAG,"Received BT data##########################");
    //printf("\r\nlen = %d\r\n",ctxt->om->om_len);   
   
    char *token = strtok(tbstr1, delimiter);

  
    sprintf(ParamBeingWritten,"%s", token);
    
    token = strtok(NULL, delimiter);
    //sprintf(ParamBeingWritten,"%s", token);
    //printf("\nWriting --> %s---%s\n",ParamBeingWritten,token);
    
    StoreParamString(ParamBeingWritten,token);

    //StoreEEParams();
    //char s[30]="Trying";//{8,8,8,8,8,8};
    //snprintf(s,_countof(s),"888888");
    // ESP_LOGW(TAG,"Sending BT data************************************");
    // os_mbuf_append(ctxt->om, "Trying", strlen("Trying"));
    //memset()
    return 0;
}
int btx=0;
    // Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char s[30]="Hello";//{8,8,8,8,8,8};
    //snprintf(s,_countof(s),"888888");
    ESP_LOGW(TAG,"Sending BT data************************************");
    GetParams(&Params);
    os_mbuf_append(ctxt->om, Params.Fields.APNName, strlen(Params.Fields.APNName));
        // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    // os_mbuf_append(ctxt->om, s, strlen(s));
    return 0;
}

//device_notify is named by Ravi from gatt_svr_chr_access_heart_rate
static int
device_notify(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Sensor location, set to "Chest" */
    static uint8_t body_sens_loc = 0x01;
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == 0xDEAF) {
        rc = os_mbuf_append(ctxt->om, &body_sens_loc, sizeof(body_sens_loc));

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}


/* This function simulates heart beat and notifies it to the client */
static void
//blehr_tx_hrate(TimerHandle_t ev)
SendParam(char *pValue)
{
    // static uint8_t hrm[2];
    int rc;
    struct os_mbuf *om;

    if (!notify_state) {
        //blehr_tx_hrate_stop();
        //heartrate = 90;
        return;
    }

    // hrm[0] = 0x06; /* contact of a sensor */
    // hrm[1] = heartrate; /* storing dummy data */

    /* Simulation of heart beats */
    // heartrate++;
    // if (heartrate == 160) {
    //     heartrate = 90;
    // }
    printf("Notifying\n");
    // sprintf(tbstr,"%s",heartrate);
    //om = ble_hs_mbuf_from_flat("hrm", sizeof(hrm));
    //om = ble_hs_mbuf_from_flat(tbstr, strlen(tbstr));
    om = ble_hs_mbuf_from_flat(pValue, strlen(pValue));
    rc = ble_gatts_notify_custom(conn_handle, HR, om);

    assert(rc == 0);

    // blehr_tx_hrate_reset();
}

// Write data to ESP32 defined as server
static int device_command(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //char s[100];
    //c+=ctxt->om->om_len;
    //sprintf(s,"%.*s", ctxt->om->om_len, ctxt->om->om_data);
    //memcpy(s,ctxt->om->om_data,ctxt->om->om_len);
    //strcat(strb,s);
    
    //if(c>=100)
    // {
    //     printf(strb);c=0;
    // }
    // printf("%d-",c);
    //ESP_LOGW(TAG,"Received BT data##########################");
    //printf("\r\nlen = %d\r\n",ctxt->om->om_len);
    // for(int i =0;i<ctxt->om->om_len;i++)
    //     printf("%c,",s[i]);
    //strcpy(ParamBeingRead,(char*)ctxt->om->om_data);
    memcpy(ParamBeingRead,(char*)ctxt->om->om_data,ctxt->om->om_len);
    ParamBeingRead[ctxt->om->om_len] = 0;
    printf("ParamBeingRead = %s\n",ParamBeingRead);
    memset(tbstr,0,sizeof(tbstr));
    GetParamString(ParamBeingRead,tbstr);
    //SendParam(ParamBeingRead);
    SendParam(tbstr);
    //StoreEEParams();
    //char s[30]="Trying";//{8,8,8,8,8,8};
    //snprintf(s,_countof(s),"888888");
    // ESP_LOGW(TAG,"Sending BT data************************************");
    // os_mbuf_append(ctxt->om, "Trying", strlen("Trying"));
    
    return 0;
}
// static void
// blehr_tx_hrate_stop(void)
// {
//     xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
// }

// /* Reset heart rate measurement */
// static void
// blehr_tx_hrate_reset(void)
// {
//     int rc;

//     if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
//         rc = 0;
//     } else {
//         rc = 1;
//     }

//     assert(rc == 0);

// }


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_command},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
           {.uuid = BLE_UUID16_DECLARE(0xDEAF),           // Define UUID for writing
           .val_handle = &HR,
          .flags = BLE_GATT_CHR_F_NOTIFY,
          .access_cb = device_notify},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI("GAP", "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, HR);
        if (event->subscribe.attr_handle == HR) {
            notify_state = event->subscribe.cur_notify;
            //blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != HR) {
            notify_state = event->subscribe.cur_notify;
            //blehr_tx_hrate_stop();
        }
        ESP_LOGI("GAP", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

#ifdef VALTRACK_V4_VTS
const char *TAG = "VALTRACK-V4-VTS";
#else
const char *TAG = "VALTRACK-V4-MF";
#endif


#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define TIMER_TASK_STACK_SIZE   (CONFIG_EXAMPLE_TASK_STACK_SIZE)`

// UART1 TX-----0
// UART1 RX-----1
// ANALOG IN----2
// INT1---------3
// TPS-ENABLE---4
// IIC-DATA-----5
// IIC-CLOCK----6
// PWRKEY-------7
// LED-SIGNAL---8
// SWITCH-SW2---9
// GSM-ENABLE---10
// USB DN-------18
// USB_DP-------19
// UART0 RX-----20
// UART0 TX-----21




#define EEPROM_FIFO


void SystemClock_Config(void);
void Error_Handler(void);
void DeepSleep (void);
void InitRTCAlarm(void);

const StringType ETypes[]=
{
    {"EVENT_ENGINE_OFF"}                     ,   // 0  00 
    {"EVENT_ENGINE_ON"}                     ,    // 1  01 
    {"REGULAR_TEST"}                        ,    // 2  02 
    {"RANDOM_TEST"}                         ,    // 3  03 
    {"EVENT_BYPASS"}                        ,    // 4  04 
    {"EVENT_LOG_CLEARED"}                   ,    // 5  05 
    {"EVENT_SERVICE_PEROD_SET"}             ,    // 6  06 
    {"EVENT_RESET_FOR_VIOLATION"}           ,    // 7  07 
    {"EVENT_USE_ONE_TIME_CODE"}             ,    // 8  08 
    {"EVENT_RE_DO"}                         ,    // 9  09 
    {"EVENT_12V_POWER_REMOVED"}             ,    // 10  0A
    {"EVENT_LOG_READ"}                      ,    // 11  0B
    {"EVENT_CONFIG_CHANGED"}                ,    // 12  0C
    {"EVENT_LOG_FULL"}                      ,    // 13  0D
    {"EVENT_SERVICE_ALERT"}                 ,    // 14  0E
    {"not_used"}                            ,    // 15  0F
    {"EVENT_WEAK_BLOW"}                     ,    // 16  10
    {"EVENT_WARNING_FAIL_RR"}               ,    // 17  11
    {"EVENT_WARNING_REFUSED"}               ,    // 18  12
    {"EVENT_WARNING_SERVICE_PERIOD_END"}    ,    // 19  13
    {"EVENT_CAR_BATTERY_ON"}                ,    // 20  14
    {"EVENT_CAR_BATTERY_OFF"}               ,    // 21  15
    {"EVENT_CALIBATION_DONE"}               ,    // 22  16
    {"EVENT_IGNITION_KEYED"}                ,    // 23  17
    {"EVENT_WARNING_GIVEN"}                 ,    // 24  18
    {"EVENT_STATER_NOT_ACTIVE"}             ,    // 25  19
    {"EVENT_INSUFFICIENT_BLOW"}             ,    // 26  1A
    {"EVENT_COOL_SAMPLE"}                   ,    // 27  1B
    {"EVENT_TAMPERED"}                      ,    // 28  1C
    {"EVENT_START_TEST_ATTEMPT"}            ,    // 29  1D
    {"EVENT_AB_FC_CONNECTED"}               ,    // 30  1E
    {"EVENT_AB_FC_REMOVED"}	              ,    // 31  1F
    {"EVENT_CAL_CHK_PASS"}	              ,    // 32  20
    {"EVENT_CAL_CHK_FAIL"}	              ,    // 33  21
    {"EVENT_ENGINE_NOT_STARTED"}			  ,    // 34  22
    {"G_PING"}							  ,	   // 35  23
    {"GPRS_PING"}							  ,	   // 36  24
    {"MOTION"}					          ,	   // 37  25
    {"REBOOT"}					          ,	   // 38  26
    {"SOS"}					              ,	   // 39  27
    
};
const StringType BootReasons[]= 
{
    {"RTC_RESET"},
    {"INT1_RESET"},
    {"BUFF2_RESET"},
    {"NETWORK_RESET"},
    {"LPUART_TIMER_RESET"},
    {"MOTION_RESET"},
    {"HARDFAULT_RESET"},
    {"DEEP_SLEEP_RESET"},
    {"CHARGER_RESET"}

};
const StringType Results[] = 
{
    {"FAIL"},
    {"PASS"},
    {"NA"}
};
const EngineStatusStringType EngineStatusStrings[]=
{
    {"OFF"}                     ,   // 0  00 
    {"ON"}                      ,    // 1  01 
};
/* USER CODE END 0 */


char tBuff[TBUFF_SIZE];
unsigned char x,SMSSent=1;
extern unsigned char RingCount;

unsigned char NeedBTAttention=0;
unsigned short millis = 0;
unsigned short tSeconds;
unsigned short SystemTimer;
unsigned short InactivityTimer;
unsigned long  IntervalTimer;
unsigned short MotionTimer=0;
unsigned short HeartBeatTimer = 0;
unsigned short NoSignalTimer=0;
unsigned short ButtonPressTimer=0;
unsigned short AuthenticationTimer = 0;
unsigned short BatteryCheckTimer=950;
unsigned char BatteryTimeout=0;
unsigned short minVal,maxVal;
unsigned char ADCRunning = 0;
char savedNumber[11];
char rxNumber[16];
unsigned char ActivityLevel = 0;
unsigned short AlertTimer,SOSTimer,BatteryTimer,DebounceTimer,GSMResetTimer,LPUARTTimer,ConnectivityTimer=0;
int FrontPanelTimer=0;
unsigned short EEPROMReadTimer = 0;
unsigned char ticks;
unsigned char DeviceStatus = 1;
unsigned char SysClockConfigFlag=0;
unsigned char SleepModeEnabled=0,RTCSleepModeEnabled = 0;
unsigned char PowerButtonSleep = 0;

int ChargeStatus,ChargeLevel,ChargeVoltage,ChargingState;
float ChargeVoltageF=0;
unsigned long RTCTimeout=0;
const char ATCmgsToken[] = {"AT+CMGS=\""};
const char SetMsgTypeCmd[] = {"AT+CMGF=1\r\n"};
const char NewLineCmd[] = {"\r\n"};
const char NewLine2Cmd[] = {"\"\r\n"};
//unsigned char Count;


unsigned char FirstTime=0;
unsigned char SMSNumber=0;
unsigned char GPSAwake=0;

char tmpbuff[100];
unsigned char tmpvar;
uint16_t Counter = 0;
// HAL_StatusTypeDef Result = HAL_OK;
unsigned char VALREAD;
unsigned char iAddr; 

MotionStatusType INT1;
unsigned char SOS,POWER_BUTTON;
unsigned char GSM_STATUS;
unsigned char PG_STATUS;
unsigned char CHG_STATUS;

RTC_TimeTypeDef R;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
unsigned char StandbyReset=0;
unsigned char LEDInhibit = 1;
RTC_DATA_ATTR BootReasonType BootReason;

unsigned char AlignInterval = 1;


#define DEVICE_ID "864287038316376"
//#define WATCHDOG_ENABLED
//#define SOSALERT


unsigned char SosAlert=0;

//#define ACCLEROMETER_I2C_ADDRESS 0x1D<<1   //MMA854

unsigned char ClockSource=0;

const char SYS_VERSION[]="VALTRACK-V4-19-02-23";

char devid[16] = DEVICE_ID;
char bstr[250];

#ifdef VALETRON_SYSTEMS

const char CUSTOMER[]="VALETRON_SYSTEMS";
#endif

unsigned char LoadDefaultParams = 0;
///NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
#ifdef PARAMS_NORMAL
const ParamsType DefaultParams = {
    
    .Fields=
    {
         10,//unsigned int PingInterval;
        "HTTP",//"TCP",//char WorkingMode[5];
        "NONE",//"CALL",//char MotionAlertMode[5];
        0x12,//char MotionThreshold;
        "http://domain.com/api/update",//char HTTPURL[150];,
        "Your-Api-Key: 1234456789",///char HTTPKey[100];
        "www",//"iot.1nce.net",//char APNName[20];
        "",//char APNUsername[20];
        "",//char APNPassword[20];
        "EGSM_MODE,ALL_BAND",//char Band[30];
        "1234567890",//char rxNumber[16];
        "test.mosquitto.com",//char MQTTHost[30];
        "1883",//char MQTTPort[10];
        "708a964577c84f5abac",//char MQTTClientID[20];
        "valtrack",//char MQTTTopic[30];
        "MQIsdp",//char MQTTProtocolName[10];
        0x03,//unsigned char MQTTLVL;
        0xC2,//unsigned char MQTTFlags;
        60,//unsigned int MQTTKeepAlive;
        "txzborlq",//char MQTTUsername[30];
        "Rr-kEclgx_M2"//char MQTTPassword[35];
    }
};
#endif
//NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
//ParamsType Params;


MotionStatusType INT1;



#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

//#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define ACCLEROMETER_WHO_AM_I_REG_ADDR           0x0F        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7





unsigned long Count,Count2,LoopTimeout1,LoopTimeout2,LoopTimeout3;

int echo = 1;
int counter=0;
int temp;
unsigned char retVal;


char Speed[10]="",tSpeed[10]="";;


unsigned long PLTime,LTime,LDate;
double tfLat,tfLong;    
unsigned int fuel;
unsigned char QueEmpty;


 uint16_t  ADC1C7 = 0;
 uint16_t  ADC1C6 = 0;
 uint16_t  ADC1C5 = 0;

#define ADC1_DR_Address                0x40012440


 uint16_t RegularConvData_Tab[3];


unsigned char ISRstatus;
char RFIDCardNumber[20];
char ValidRFIDCardNumber[20];
unsigned char ValidRFIDCardLength;

unsigned char ImageSent = 0, ImageIndex = 0,ImagePresent=0,SendingImageIndex=0,ImageStored=0;    
//IDPacketType DeviceID;
GeneralEventType GEvent;
char IMEI[16],IMSI[16],/*time[21],ctime[21],*/etype,*presult,bac,filename[32];
//PackedGeneralEventType PGEvent;
HWEventDataType CPacket,GPacket;
const char dummyevent[]={"0D00000000000000000033002300000000"};
unsigned long CheckSum,RLength;
unsigned short datalength;
unsigned short topiclength;
char topic[30];

HWEventDataType *pPacket;
char str[2500];
EventCodeType EventType;
char ImageRetryCount[64];
char Version[50];
char RFID[50];
char query[30];
unsigned char RFIDDataPresent;
unsigned char EngineStatus=0;

void XCheckGPS(void);


unsigned char MCU_ACOK,MCU_CHGOK,EnableCharge=0;
char Test1='C';
char ACCTest='T';
unsigned char LED=0,GSP=0;


extern unsigned char GSM_STATUS;
extern unsigned char PG_STATUS;
extern unsigned char CHG_STATUS;
 
extern unsigned short millis;
extern unsigned short tSeconds;
extern unsigned short InactivityTimer;
extern unsigned short MotionTimer;
extern unsigned short NoSignalTimer;
extern unsigned char SMSSent;
extern unsigned short BatteryCheckTimer;
extern unsigned char BatteryTimeout;
extern int FrontPanelTimer;

//unsigned short ButtonPressTimer = 0;
unsigned char pSOS = 0;
unsigned char tmpFlagClearer;
unsigned char BTInitialized = 0;



unsigned long lastTickValue=0;
//unsigned long ButtonLastTickValue=0;
char MResponsePacket[2] = {0x01,0x00};
extern unsigned char AppAuthenticated;
void DisconnectDevice(void);

unsigned short BTReceiveTimer = 0;
unsigned char SOSActivated = 0;
extern unsigned char AnswerCall;
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t motion_sensor_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ACCLEROMETER_I2C_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t motion_sensor_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ACCLEROMETER_I2C_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}


void I2C_WrReg(uint8_t Reg, uint8_t Data)
{
    motion_sensor_register_write_byte(Reg,Data);

    
    //osDelay(100);
}
uint8_t I2C_RdReg(uint8_t Reg)
{
    //HAL_StatusTypeDef tResult = HAL_OK;
    uint8_t Data;
    motion_sensor_register_read(Reg,&Data,1);
    

    //osDelay(100);
    
    return Data;
    
}
typedef enum SensorTypes
{
    NOT_DETECTED = 0,
    MMA8652_SENSOR,
    MMA8653_SENSOR,
    MMA8452_SENSOR,
    LIS3DH_SENSOR
}SensorType;
SensorType MotionSensor = NOT_DETECTED;
unsigned char VALREAD=0;
void InitAccelerometer_LIS3D(void)
{
     unsigned char i;
    char ReadBuff[100];
    
  
//    I2C_WrReg(MMA8652_CTRL_REG2, RST_MASK);
    //DDelay();
    VALREAD = I2C_RdReg(0x0F);
    
   if(VALREAD == 0x33)
    {
        MotionSensor = LIS3DH_SENSOR;
        ESP_LOGI(TAG, "LIS3DH_SENSOR");
    }
    if(VALREAD == 0x4A)
    {
        MotionSensor = MMA8652_SENSOR;
        ESP_LOGI(TAG, "MMA8652_SENSOR");
    }
    if(VALREAD == 0x5A)
    {
        MotionSensor = MMA8653_SENSOR;
        ESP_LOGI(TAG, "MMA8653_SENSOR");
    }
    if(VALREAD == 0x2A)
    {
        MotionSensor = MMA8452_SENSOR;
        ESP_LOGI(TAG, "MMA8452_SENSOR");
    }


    VALREAD = I2C_RdReg(0x26);
    VALREAD = I2C_RdReg(0x0F);//VALREAD = I2C_RdReg(0x0D);
    I2C_WrReg(REG_CTRL_REG1, 0x57);// LPEN bit 3
    I2C_WrReg(REG_CTRL_REG4, 0x08);// HR bit 3
    
    osDelay(200);
   //  VALREAD = I2C_RdReg(REG_CTRL_REG1);
    I2C_WrReg(REG_CTRL_REG2, 0x05);
    I2C_WrReg(REG_CTRL_REG3, 0x40);//    I2C_WrReg(MMA8652_CTRL_REG3, 0x39);
    
    I2C_WrReg(REG_CTRL_REG5, 0x08);
   // VALREAD = I2C_RdReg(REG_CTRL_REG5);
    I2C_WrReg(REG_CTRL_REG6, 0x02);
    //I2C_WrReg(REG_CTRL_REG6, 0xFF);
    I2C_WrReg(REG_INT1_THS,Params.Fields.MotionThreshold);
    I2C_WrReg(REG_INT1_DURATION,0x00);
    I2C_WrReg(REG_INT1_CFG,0x2A);
    
    
     for(i=0x07;i<=0x3F;i++)
    {
       ReadBuff[i] = I2C_RdReg(i);
        
    }
    ReadBuff[0]=ReadBuff[0];
    
}



void InitAccelerometer_MMA8XXX(void)
{
   
  
}
void InitAccelerometer(void)
{
    #ifdef LIS3DH_ENABLED
        InitAccelerometer_LIS3D();
    #else
        InitAccelerometer_MMA8XXX();
    #endif
    
    
}
/////////////////////////////////////////I2C

unsigned char SendATCommand(char *pCommand,char *pResponse1,char *pResponse2, unsigned short Timeout)
{
    unsigned char Response1Length,Response2Length;
    //osDelay(2000); // Between command delay
    osDelay(200);
    Response1Length = strlen((void*)pResponse1);
    Response2Length = strlen((void*)pResponse2);
    ResetBuffer();
    Print(pCommand);
    LoopTimeout1 = 0;
    while(1)
    {
        vTaskDelay(1);
        if(MapForward(Buff2,BUFF2_SIZE,(char*)pResponse1,Response1Length) != NULL)
            return 1;
        if(MapForward(Buff2,BUFF2_SIZE,(char*)pResponse2,Response2Length) != NULL)
            return 2;
        if(LoopTimeout1>Timeout)
            return 3; 
        
    }
    
}
char BatteryString[25];
void CheckBattery(void)
{
    char*pToken;
    unsigned char i;

                         
    i=0;
 
    SendATCommand("AT+CBC\r\n","+CBC:","ERROR",5);
    //#ifdef SIM800
    osDelay(500);
    //#endif
    pToken = MapForward(Buff2,70,(char*)"+CBC:",5);
    //pToken+=5;
    if(pToken != NULL)
    {
        
        //while(*pToken != '"')pToken--;
        //pToken++;
        
        // save sender number
        while(pToken[i] != '\n')
        {
            BatteryString[i] = pToken[i];
            i++;
            if(i>=20)break;
        }
        BatteryString[i] = '\0';
        BatteryString[20] = '\0';
        
        #ifdef SIM800
            sscanf( (void*)pToken, "+CBC: %d,%d,%d\r", &ChargeStatus,&ChargeLevel,&ChargeVoltage);
            ChargeVoltageF = ((float)ChargeVoltage/1000);
        #else
            sscanf( (void*)pToken, "+CBC: %f\r", &ChargeVoltageF);
        #endif
    }
    ChargeVoltageF = ADCBatteryVoltage; //OVERRIDING WITH ADC DATA. May be add app control later
   
 
        
    
}
char SignalStrength[25];
void CheckSignalStrength(void)
{
    char*pToken;
    unsigned char i;

  
    i=0;
    SendATCommand("AT+CSQ\r\n","+CSQ:","ERROR",5);
    //#ifdef SIM800
        osDelay(500);
    //#endif
    pToken = MapForward(Buff2,70,(char*)"+CSQ:",5);
    pToken+=6;
    if(pToken != NULL)
    {
        
        //while(*pToken != '"')pToken--;
        //pToken++;
        
        // save sender number
        while(pToken[i] != '\r')
        {
            SignalStrength[i] = pToken[i];
            i++;
            if(i>=20)break;
        }
        //SignalStrength[2] = '/';
        SignalStrength[i] = '\0';
        SignalStrength[20] = '\0';
        

    }

    
}
//float nLat=0,nLon=0;
float NLat,NLong;
NetworkLocationStatusType NStatus;
int NAccuracy;
char NLPacket[80];
char TowerPacket[100];
void CheckNetworkLocation(void)
{
    char*pToken;
    //unsigned char i;

    CheckSignalStrength();
    //i=0;
    

    SendATCommand((void*)"AT+CLBS=1\r\n","+CLBS:","ERROR",50);
    //#ifdef SIM800
        osDelay(500);
    //#endif
    pToken = MapForward(Buff2,70,(char*)"+CLBS:",6);
    if(pToken != NULL)
    {
        sscanf((void*)pToken,"+CLBS: %d",(int*)&NStatus);
        if(NStatus == NL_SUCCESS)
        {
            pToken+=6;
            //osDelay(1000);//DelayProc(50000);
            sscanf((void*)pToken,"%d,%f,%f,%d",(int*)&NStatus,&NLat,&NLong,(int*)&NAccuracy);
            snprintf((void*)NLPacket,_countof(NLPacket),",\"nlat\":\"%f\",\"nlon\":\"%f\",\"ncsq\":\"%s\"",NLat,NLong,SignalStrength);
            //sprintf((void*)NLPacket,",\"nlat\":\"%f\",\"nlon\":\"%f\",\"ltype\":\"NL\"",NLat,NLong);
        }
    }
    
        
    
}

void  Binary2Ascii(unsigned long HexValue);
int GetLength(char *p)
{
   int Length;
   Length=0;
   while(*p != '\0')
   {
     Length++;
     p++;
   }
   
   return Length;
  
}
void WakeUp(void);




unsigned char GSMEnabled = 0;
unsigned char GPSEnabled = 0;
void EnableGSM(void)
{
    // Enable power to GSM
    gpio_set_level(GPIO_GSM_ENABLE, 1); 
    GSMEnabled = 1;
    //osDelay(8000);
} 

void DisableGSM(void)
{
    // #ifdef SIM7600
    // SendATCommand("AT+CPOF\r\n","OK","ERROR",10);// Disconnect network and shutdown
    // #endif
    // #ifdef SIM7070
    // SendATCommand("AT+CPOWD=1\r\n","NORMAL","ERROR",10);// Disconnect network and shutdown
    // #endif

    //osDelay(1000);
    // Disable power to GSM
    gpio_set_level(GPIO_GSM_ENABLE, 0);   
    GSMEnabled = 0;
    //osDelay(3000);
    
}
void EnableMainPower(void)
{
    #ifdef ENABLE_TPS_CONTROL
    // Disable power to GSM
    gpio_set_level(GPIO_TPS_ENABLE, 1);   
    #endif
        
}
void DisableMainPower(void)
{
    #ifdef ENABLE_TPS_CONTROL
    // Disable power to GSM
    gpio_set_level(GPIO_TPS_ENABLE, 0);   
    #endif    
    
}
// void EnableCharger(void)
// {
//     // Enable Battery charger -> LOW = ENABLE
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); //EN
//     ChargingState = 1;
// }
// void DisableCharger(void)
// {
//     // Enable Battery charger -> HIGH = DISABLE
//     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //EN
//     ChargingState = 0;
// }

void HandleChargingState(void)
{
    #ifdef BATTERY_PRESENT
    
    //if(BatteryTimer > BATTERY_READ_INTERVAL)
    //{
            
        DisableCharger();
        osDelay(1000);
        CheckBattery();
     
        //if(ChargingState == 1)
        //    EnableCharger();
        //else
        //    DisableCharger();
        
        
        if(ChargeVoltage < 3750)
        {
            EnableCharger();
            ChargingState = 1;
        }
        else if(ChargeVoltage > 4100)
        {
            DisableCharger();
            ChargingState = 0;
        }
        
        //BatteryTimer = 0;
    //}    
    #endif
}
void LoadTimeStamp(HWEventDataType *pPacket)
{
    //FunctionCode;
    //EventType;
    pPacket->GEvent.Hours   = R.Hours;
    pPacket->GEvent.Minutes = R.Minutes;
    pPacket->GEvent.Seconds = R.Seconds;
    pPacket->GEvent.Month   = sDate.Month;
    pPacket->GEvent.Date    = sDate.Date;
    pPacket->GEvent.Year    = sDate.Year;
    pPacket->GEvent.Voltage = ChargeVoltageF;
    pPacket->GEvent.Lat     = fLat;
    pPacket->GEvent.Long    = fLong;
    pPacket->GEvent.Speed   = fSpeed;
}
void LoadGPSTimeStamp(HWEventDataType *pPacket)
{
    
    //FunctionCode;
    //EventType;
    pPacket->GEvent.Hours   = GPSHours;
    pPacket->GEvent.Minutes = GPSMinutes;
    pPacket->GEvent.Seconds = GPSSeconds;
    pPacket->GEvent.Month   = GPSMonth;
    pPacket->GEvent.Date    = GPSDay;
    pPacket->GEvent.Year    = GPSYear;
    pPacket->GEvent.Voltage = ChargeVoltageF;
    pPacket->GEvent.Lat     = fLat;
    pPacket->GEvent.Long    = fLong;
    pPacket->GEvent.Speed   = fSpeed;
}
void PostMotionEvent(void)
{
    
    //unsigned char CheckSum;
    #ifdef DEBUG_PRINT
        DebugPrint("Entered-PostMotionEvent\r\n"); 
    #endif

    LoadTimeStamp(&CPacket);
    CPacket.GEvent.EventType = MOTION_PING;
    //CheckSum = GetCheckSum(CPacket.Bytes,34);
    //CPacket.GEvent.CheckSum[1]=GetAscii(CheckSum&0x0F);  
    //CPacket.GEvent.CheckSum[0]=GetAscii((CheckSum>>4) & 0x0F);  

	  PostEvent( &CPacket);
   // if(PostEvent( &CPacket) == FIFO_OVERRUN_OCCURED)
      //  PostEEEvent(&CPacket);
    
    
}

void PostReboot(void)
{
    #ifdef DEBUG_PRINT
        DebugPrint("Entered-PostReboot\r\n"); 
    #endif

    LoadTimeStamp(&CPacket);
    CPacket.GEvent.EventType = REBOOT_PING;
    //CheckSum = GetCheckSum(CPacket.Bytes,34);
    //CPacket.GEvent.CheckSum[1]=GetAscii(CheckSum&0x0F);  
    //CPacket.GEvent.CheckSum[0]=GetAscii((CheckSum>>4) & 0x0F);  

	  
    PostEvent( &CPacket);
//    if(PostEvent( &CPacket) == FIFO_OVERRUN_OCCURED)
//        PostEEEvent(&CPacket);
   
}
void PostGPing(void)
{
    
    //unsigned char CheckSum;//,str[50];
	#ifdef DEBUG_PRINT
        DebugPrint("Entered-PostGPing\r\n"); 
    #endif

    LoadGPSTimeStamp(&CPacket);
    CPacket.GEvent.EventType = G_PING;
    //CheckSum = GetCheckSum(CPacket.Bytes,34);
    //CPacket.GEvent.CheckSum[1]=GetAscii(CheckSum&0x0F);  
    //CPacket.GEvent.CheckSum[0]=GetAscii((CheckSum>>4) & 0x0F);  

    PostEvent( &CPacket);
//    if(PostEvent( &CPacket) == FIFO_OVERRUN_OCCURED)
//        PostEEEvent(&CPacket);
   
}

void PostGPRSPing(void)
{
    #ifdef DEBUG_PRINT
        DebugPrint("Entered-PostGPRSPing\r\n"); 
    #endif
    LTime = InactivityTimer;
    if(LTime-PLTime>=Params.Fields.PingInterval)
    {

        LoadTimeStamp(&CPacket);
        CPacket.GEvent.EventType = GPRS_PING;
        //CheckSum = GetCheckSum(CPacket.Bytes,34);
        //CPacket.GEvent.CheckSum[1]=GetAscii(CheckSum&0x0F);  
        //CPacket.GEvent.CheckSum[0]=GetAscii((CheckSum>>4) & 0x0F);  

        PostEvent( &CPacket);
//        if(PostEvent( &CPacket) == FIFO_OVERRUN_OCCURED)
//            PostEEEvent(&CPacket);
        PLTime=LTime;
    }
}

void PostSOSPing(void)
{
    #ifdef DEBUG_PRINT
        DebugPrint("Entered-PostSOSPing\r\n"); 
    #endif
    //unsigned char CheckSum;//,str[50];
	
    LoadTimeStamp(&CPacket);
    CPacket.GEvent.EventType = SOS_PING;
    //CheckSum = GetCheckSum(CPacket.Bytes,34);
    //CPacket.GEvent.CheckSum[1]=GetAscii(CheckSum&0x0F);  
    //CPacket.GEvent.CheckSum[0]=GetAscii((CheckSum>>4) & 0x0F);  

    PostEvent( &CPacket);
//    if(PostEvent( &CPacket) == FIFO_OVERRUN_OCCURED)
//        PostEEEvent(&CPacket);
   
}
void DDelay()
{
 
    osDelay(500);
}
void WakeUp(void)
{
    Print("AT\r\n");
    DDelay();//DelayProc(10000);
    //DDelay();
}

unsigned char CheckNetwork(void)
{
    //unsigned long LoopCount = 0;
    unsigned char Retries=0;
    char*pToken1,*pToken2,*pToken3, *pToken4;
 
    CHECK_NETWORK_AGAIN:    

    ResetBuffer();
    Print("AT+CREG?\r\n");
 
	  LoopTimeout1 = 0;
    while(1)
    {
			if(MapForward(Buff2,BUFF2_SIZE,(char*)"+CREG:",6) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }
		osDelay(500);
	
    pToken1 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,1",3);
    pToken2 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,5",3);
    
    if( (pToken1 != NULL) || (pToken2 != NULL))
    {
        ResetBuffer();
        ESP_LOGI(TAG,"Network registered/roaming");

        
        return 0;
        
    }

    pToken1 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,0",3);
    pToken2 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,2",3);
    pToken3 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,3",3);
    pToken4 = MapForward(Buff2,BUFF2_SIZE,(char*)"0,4",3);
    
    if( (pToken1 != NULL) || (pToken2 != NULL) || (pToken3 != NULL) || (pToken4 != NULL) )
    {            
        ResetBuffer();
        ESP_LOGI(TAG,"Network not registered");
        #ifdef DEBUG_PRINT
            DebugPrint("Network Not registered -CheckNetwork\r\n"); 
        #endif

        
        if(++Retries<20)
        {
            osDelay(1000);
            goto CHECK_NETWORK_AGAIN;
        }
        else
            return 1;
    }
    
    return 1;
    
}

void DeleteAllSMS(void)
{
    unsigned char i;
    ResetBuffer();
    Print("AT\r\n");
    DelayProc(10000);
    for(i=1;i<10;i++)
    {
        ResetBuffer();
        Print("AT+CMGD=");
        WriteUART2(i|0x30);
        Print("\r\n");
        DelayProc(850000);
    }
}
unsigned char SendPMTKCommand(char *pCommand,char *pResponse1,char *pResponse2, unsigned short Timeout)
{
    unsigned char Response1Length,Response2Length;
    osDelay(200);
    Response1Length = strlen((void*)pResponse1);
    Response2Length = strlen((void*)pResponse2);
    ResetBuffer1();
    Print1(pCommand);
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff,BUFF_SIZE,(char*)pResponse1,Response1Length) != NULL)
            return 1;
        if(MapForward(Buff,BUFF_SIZE,(char*)pResponse2,Response2Length) != NULL)
            return 2;
        if(LoopTimeout1>Timeout)
            return 3; 
        
    }
    
}
 

void SystemPower_Config(void);
//static void SystemClock_ConfigMSI(void);
char tmpeebuff[10]="123456789";
unsigned char tmpeeaddr;


typedef struct
{
    char t1[10];
    char t2[10];
    char t3[100];
    char t4[10];
}TempType;

TempType ts;
char*pVar;

void CheckBLE(void)
{
	
RTC_TimeTypeDef BTime;
RTC_DateTypeDef BDate;
int Year=0,Month=0,Date=0,Hour=0,Minute=0,Sec=0;
    #ifdef BLUETOOTH_ENABLED
    
    char*pToken;
    unsigned char i;
    SOS = gpio_get_level(GPIO_SOS);
    if(SOS == 0)
    {
        LEDInhibit=1;
        osDelay(3000);
        SOS = gpio_get_level(GPIO_SOS);
        if(SOS == 0)
        {
            
            if(DeviceStatus == 0)
                return;
            
            FrontPanelTimer = -600;
            UpdateBluetooth(1);

            
            NeedBTAttention = 1;
            //osDelay(10000);
            //osDelay(10000);
            
            WakeUp();
            ResetBuffer();
            Print("AT+BTPOWER=1\r\n");
            osDelay(5000);
     
 
            
            ResetBuffer();
            Print("AT+BTPAIRCFG=1\r\n");
            osDelay(1000);
            
            ResetBuffer();
            Print("AT+BTVIS=1\r\n");
            osDelay(1000);
        
            while(1)
            {
                WakeUp();
                osDelay(1000);
                SOS = gpio_get_level(GPIO_SOS);
                if(SOS == 0) 
                {
                    goto TURN_OFF_BLE;
                }
                if(MapForward(Buff2,BUFF2_SIZE,(char*)"CONNECTING",10) != NULL)
                {
                    
                    Print("AT+BTACPT=1\r\n");
                    osDelay(1000);
                    while(1)
                    {
                        FrontPanelTimer = -600;
                        
                        // TBD//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
                        osDelay(500);
                        if(SystemTimer%10 == 0)
                        {
                            WakeUp();
                            ResetBuffer();
                            Print("AT+BTSPPSEND\r\n");
                            osDelay(1000);
                            
                            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5);
                            if(pToken != NULL)
                            {
                                goto TURN_OFF_BLE;
                            }
                            //sprintf(ts.t1,"WM: %d\n",Params.Fields.WorkingMode);
                            //sprintf(ts.t2,"AM: %d\n",Params.Fields.MotionAlertMode);
                            //sprintf(ts.t3,"URL: %s\n",Params.Fields.HTTPURL);
                            //sprintf(ts.t4,"PI: %d\n",Params.Fields.PingInterval);
                            UpdateBluetooth(2);
                            Print("Am Listening#\n");
                            
                            sprintf((void*)bstr, "\nBand: %s",Params.Fields.Band);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nWM: %s",Params.Fields.WorkingMode);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMAlert: %s",Params.Fields.MotionAlertMode);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMThresh: %hhu",Params.Fields.MotionThreshold);   
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nContact: %s",Params.Fields.rxNumber);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nAPN: %s",Params.Fields.APNName);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nAPNUser: %s",Params.Fields.APNUsername);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nAPNPass: %s",Params.Fields.APNPassword);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nHTTPURL: %s",Params.Fields.HTTPURL);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nHTTPKey: %s",Params.Fields.HTTPKey);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nInterval: %u",Params.Fields.PingInterval);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMHost: %s",Params.Fields.MQTTHost);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMPort: %s",Params.Fields.MQTTPort);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMCID: %s",Params.Fields.MQTTClientID);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMTopic: %s",Params.Fields.MQTTTopic);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMProt: %s",Params.Fields.MQTTProtocolName);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMLVL: 0x%hhX",Params.Fields.MQTTLVL);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMFlag: 0x%hhX",Params.Fields.MQTTFlags);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMKAlive: %u",Params.Fields.MQTTKeepAlive);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMUser: %s",Params.Fields.MQTTUsername);
                            Print((void*)bstr);
                            sprintf((void*)bstr, "\nMPass: %s",Params.Fields.MQTTPassword);
                            Print((void*)bstr);
                            sprintf((void*)bstr,"\nTime: 20%02d-%02d-%02d  %02d:%02d:%02d",sDate.Year,sDate.Month,sDate.Date,R.Hours,R.Minutes,R.Seconds);
                            Print((void*)bstr);
                            //Print(ts.t1);
                            //Print(ts.t2);
                            //Print(ts.t3);
                            //Print(ts.t4);
                            
                            putcchar(0x1A);
                            UpdateBluetooth(1);
                            
                        }
                        //TBD// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
                        osDelay(500);
                        //ResetBuffer();
                        Count=0;
                        pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"VALETRON",8);
                        if(pToken != NULL)
                        {
                            osDelay(2000);
                            memset((void*)bstr,0,sizeof(bstr));
                            i=0;
                            while(pToken[11+i] != '#')
                            {
                                bstr[i] = pToken[11+i];
                                i++;
                                if(i>250)goto DISCARD;
                            }
                            bstr[i] = '\0';
                            switch(pToken[9])
                            {
                                case '0':
                                    Params.Fields.Band[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.Band);
                                break;
                                case '1':
                                    sscanf((void*)bstr, "%s",Params.Fields.WorkingMode);
                                break;
                                case '2':
                                    sscanf((void*)bstr, "%s",Params.Fields.MotionAlertMode);
                                break;
                                case '3':
                                    //bstr[i] = ' ';
                                    sscanf((void*)bstr, "%hhu",(char*)&Params.Fields.MotionThreshold);   
//                                    #ifdef LIS3DH_ENABLED
//                                    ISRstatus = I2C_RdReg(REG_INT1_SRC);
//                                    InitAccelerometer();
//                                    #else
//                                    InitAccelerometer_mma84();
//                                    #endif
                                      InitAccelerometer();
                                    //InitAccelerometer();
                                    //Params.Fields.MotionThreshold = strtoul((void*)bstr);
                                break;
                                case '4':
                                    sscanf((void*)bstr, "%s",Params.Fields.rxNumber);
                                break;
                                case '5':
                                    Params.Fields.APNName[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.APNName);
                                break;
                                case '6':
                                    Params.Fields.APNUsername[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.APNUsername);
                                break;
                                case '7':
                                    Params.Fields.APNPassword[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.APNPassword);
                                break;
                                case '8':
                                    Params.Fields.HTTPURL[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.HTTPURL);
                                break;
                                case '9':
                                    Params.Fields.HTTPKey[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.HTTPKey);
                                break;
                                case 'A':
                                    //bstr[i] = ' ';
                                    sscanf((void*)bstr, "%u",&Params.Fields.PingInterval);
                                    //Params.Fields.PingInterval = strtoul((void*)bstr);
                                break;
                                case 'B':
                                    Params.Fields.MQTTHost[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTHost);
                                break;
                                case 'C':
                                    Params.Fields.MQTTPort[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTPort);
                                break;
                                case 'D':
                                    Params.Fields.MQTTClientID[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTClientID);
                                break;
                                case 'E':
                                    Params.Fields.MQTTTopic[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTTopic);
                                break;
                                case 'F':
                                    Params.Fields.MQTTProtocolName[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTProtocolName);
                                break;
                                case 'G':
                                    sscanf((void*)bstr, "%hhx",(char*)&Params.Fields.MQTTLVL);
                                break;
                                case 'H':
                                    sscanf((void*)bstr, "%hhx",(char*)&Params.Fields.MQTTFlags);
                                break;
                                case 'I':
                                    sscanf((void*)bstr, "%u",&Params.Fields.MQTTKeepAlive);
                                break;
                                case 'J':
                                    Params.Fields.MQTTUsername[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTUsername);
                                break;
                                case 'K':
                                    Params.Fields.MQTTPassword[0] = '\0';
                                    sscanf((void*)bstr, "%s",Params.Fields.MQTTPassword);
                                break;
																
                                case 'L':
                                    sscanf((void*)bstr,"20%02d-%02d-%02d  %02d:%02d:%02d",
                                    &Year,&Month,&Date,&Hour,&Minute,&Sec);
                                BDate.Year=Year;
                                BDate.Month=Month;
                                BDate.Date=Date;
                                BTime.Hours=Hour;
                                BTime.Minutes=Minute;
                                BTime.Seconds=Sec;
                                //TBD//HAL_RTC_SetDate(&hrtc, &BDate, RTC_FORMAT_BIN);
                                //TBD//HAL_RTC_SetTime(&hrtc, &BTime, RTC_FORMAT_BIN);
                                break;

                            }
                            
                            /*sscanf( (void*)&pToken[10], 
                                "%d %d %s %d", 
                                (void*)&Params.Fields.Params.Fields.WorkingMode ,
                                (void*)&Params.Fields.Params.Fields.MotionAlertMode,
                                (void*)Params.Fields.API_URL,
                                (void*)&Params.Fields.Params.Fields.PingInterval);
                            osDelay(100);*/
                            /*pVar = strtok(&pToken[10], ",");                               
                            sscanf(pVar, "%s",ts.t1);
                            pVar = strtok(NULL, ",");
                            sscanf(pVar, "%s",ts.t2);
                            pVar = strtok(NULL, ",");
                            sscanf(pVar, "%s",ts.t3);
                            pVar = strtok(NULL, "\n");
                            sscanf(pVar, "%s",ts.t4);
                            
                            //sscanf(ts.t1, "%1d",&Params.Fields.Params.Fields.WorkingMode);
                            //sscanf(ts.t2, "%1d",&Params.Fields.Params.Fields.MotionAlertMode);
                            ts.t1[0]-=0x30;
                            if(ts.t1[0] < 3)
                            Params.Fields.WorkingMode = ts.t1[0];
                            ts.t2[0]-=0x30;
                            if(ts.t2[0] < 3)
                            Params.Fields.MotionAlertMode = ts.t2[0];
                            
                            sscanf(ts.t3, "%s",Params.Fields.HTTPURL);
                            sscanf(ts.t4, "%u",&Params.Fields.PingInterval);
                            
                            Params.Fields.HTTPURL[149] = '\0';
                            */
                            StoreEEParams();
                        
                        GetEEParams();
                            InitRTCAlarm();
                    DISCARD: 
                            ResetBuffer();
                            //__disable_irq();
                            //if( (pToken[8] == 'A') && (pToken[9] == 'P') )
                            /*{
                                
                                i=0;
                                j=0;
                                while(pToken[10+i] != '#')
                                {                    
                                    pEEData->APN[j] = pToken[10+i];
                                    i++;j++;
                                    if(i>=100)break;
                                }                
                                pEEData->APN[j] = '\0';
                                //i=0;
                                i++;
                                j=0;
                                while(pToken[10+i] != '#')
                                {                    
                                    pEEData->username[j] = pToken[10+i];
                                    i++;j++;
                                    if(i>=150)break;
                                }
                                i++;                        
                                pEEData->username[j] = '\0';
                                j=0;
                                while(pToken[10+i] != '#')
                                {                    
                                    pEEData->password[j] = pToken[10+i];
                                    i++;
                                    j++;
                                    if(i>=200)break;
                                }                
                                pEEData->password[j] = '\0';
                                
                                
                                
                                UpdateFlash();
                                
                                Print("AT+BTSPPSEND\r\n");
                                DDelay();
                                DDelay();
                                Print("Settings changed#");
                                putcchar(0x1A);
                                ResetBuffer();
                                NeedBTAttention = 0;
                                break;
                                
                            }*/
                        }
                        pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"DISCONN",7);
                        if(pToken != NULL)
                        {
                            goto TURN_OFF_BLE;
                        }
                        SOS = gpio_get_level(GPIO_SOS);
                        if(SOS == 0) 
                        {
                            TURN_OFF_BLE:
                            UpdateBluetooth(0);
                            FrontPanelTimer = 0;
                            //TBD//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
                            NeedBTAttention = 0;
                            WakeUp();
                            ResetBuffer();
                            Print("AT+BTPOWER=0\r\n");
                            osDelay(3000);
                            
                            goto EXIT_BLE;
                        }
                         
                    }                        
                }
            }
        }
    }
    EXIT_BLE:
    LEDInhibit = 0;
    #endif
}
unsigned char ResponsePacket[2] = {0x01,0x00};
void WriteBTParams(char Byte)
{
    char *pToken;
    char bstr[250];
    unsigned char i;    
    
    if(Byte == '$')
    {
        BTBuffIndex = 0;
    }
    BTBuff[BTBuffIndex] = Byte;
    
    ResponsePacket[0] = 0x01; ResponsePacket[1] = 0x01;        
    ResponsePacket[0] = '$'; 
    //HAL_NVIC_DisableIRQ(USART1_IRQn);
    //HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    //P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
    //HAL_NVIC_EnableIRQ(LPUART1_IRQn);
//            
//            for(unsigned char i = 0;i<15;i++)
//            {
//                ResponsePacket[0] = IMEI[i]; 
//                P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
//            }
//            ResponsePacket[0] = '#'; 
//            P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
    

    
    if(BTBuff[BTBuffIndex] == '#')
    {
        pToken = MapForward((char*)BTBuff,BTBUFF_SIZE,(char*)"VALETRON",8);
        if(pToken != NULL)
        {
            
            //HAL_Delay(2000);
            memset(bstr,0,sizeof(bstr));
            i=0;
            while(pToken[11+i] != '#')
            {
                bstr[i] = pToken[11+i];
                i++;
                if(i>250)goto DISCARD;
            }
            bstr[i] = '\0';
            //FieldUpdated = pToken[9];
            switch(pToken[9])
            {
                case '0':
                    Params.Fields.Band[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.Band);
                break;
                case '1':
                    sscanf((void*)bstr, "%s",Params.Fields.WorkingMode);
                    BTReceiveTimer = 0;
                break;
                case '2':
                    sscanf((void*)bstr, "%s",Params.Fields.MotionAlertMode);
                break;
                case '3':
                    //bstr[i] = ' ';
                    sscanf((void*)bstr, "%hhu",(char*)&Params.Fields.MotionThreshold);   
//                            #ifdef LIS3DH_ENABLED
//                                ISRstatus = I2C_RdReg(REG_INT1_SRC);
//                                InitAccelerometer();
//                            #else
//                                InitAccelerometer_mma84();
//                            #endif
                    InitAccelerometer();
                break;
                case '4':
                    sscanf((void*)bstr, "%s",Params.Fields.rxNumber);
                break;
                case '5':
                    Params.Fields.APNName[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.APNName);
                break;
                case '6':
                    Params.Fields.APNUsername[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.APNUsername);
                break;
                case '7':
                    Params.Fields.APNPassword[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.APNPassword);
                break;
                case '8':
                    Params.Fields.HTTPURL[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.HTTPURL);
                break;
                case '9':
                    Params.Fields.HTTPKey[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.HTTPKey);
                break;
                case 'A':
                    //bstr[i] = ' ';
                    sscanf((void*)bstr, "%u",&Params.Fields.PingInterval);
                    //Params.Fields.PingInterval = strtoul(bstr);
                break;
                case 'B':
                    Params.Fields.MQTTHost[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTHost);
                break;
                case 'C':
                    Params.Fields.MQTTPort[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTPort);
                break;
                case 'D':
                    Params.Fields.MQTTClientID[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTClientID);
                break;
                case 'E':
                    Params.Fields.MQTTTopic[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTTopic);
                break;
                case 'F':
                    Params.Fields.MQTTProtocolName[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTProtocolName);
                break;
                case 'G':
                    sscanf((void*)bstr, "%hhx",(char*)&Params.Fields.MQTTLVL);
                break;
                case 'H':
                    sscanf((void*)bstr, "%hhx",(char*)&Params.Fields.MQTTFlags);
                break;
                case 'I':
                    sscanf((void*)bstr, "%u",&Params.Fields.MQTTKeepAlive);
                break;
                case 'J':
                    Params.Fields.MQTTUsername[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTUsername);
                break;
                case 'K':
                    Params.Fields.MQTTPassword[0] = '\0';
                    sscanf((void*)bstr, "%s",Params.Fields.MQTTPassword);
                break;
                case 'L':
                        //     sscanf((void*)bstr,"20%02d-%02d-%02d  %02d:%02d:%02d",
                        //     &Year,&Month,&Date,&Hour,&Minute,&Sec);
                        // BDate.Year=Year;
                        // BDate.Month=Month;
                        // BDate.Date=Date;
                        // BTime.Hours=Hour;
                        // BTime.Minutes=Minute;
                        // BTime.Seconds=Sec;
                        // HAL_RTC_SetDate(&hrtc, &BDate, RTC_FORMAT_BIN);
                        // HAL_RTC_SetTime(&hrtc, &BTime, RTC_FORMAT_BIN);
                        break;
                    case 'Z':
                    return;
                break;
                
            }
            StoreEEParams();
            GetEEParams();
            
            
            // HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN);
            // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
            
            
            
            putcharBT('$');
            
            switch(pToken[9])
            {
                case '0':
                    sprintf((void*)bstr, "Band: %s",Params.Fields.Band);
                    PrintBT(bstr);
                break;
                case '1':
                    sprintf((void*)bstr, "WM: %s",Params.Fields.WorkingMode);
                    PrintBT(bstr);
                break;
                case '2':
                    sprintf((void*)bstr, "MAlert: %s",Params.Fields.MotionAlertMode);
                    PrintBT(bstr);
                break;
                case '3':
                    sprintf((void*)bstr, "MThresh: %hhu",Params.Fields.MotionThreshold);   
                    PrintBT(bstr);
                    
                break;
                case '4':
                    sprintf((void*)bstr, "Contact: %s",Params.Fields.rxNumber);
                    PrintBT(bstr);
                break;
                case '5':
                    sprintf((void*)bstr, "APN: %s",Params.Fields.APNName);
                    PrintBT(bstr);
                break;
                case '6':
                    sprintf((void*)bstr, "APNUser: %s",Params.Fields.APNUsername);
                    PrintBT(bstr);
                break;
                case '7':
                    sprintf((void*)bstr, "APNPass: %s",Params.Fields.APNPassword);
                    PrintBT(bstr);
                break;
                case '8':
                    sprintf((void*)bstr, "HTTPURL: %s",Params.Fields.HTTPURL);
                    PrintBT(bstr);
                break;
                case '9':
                    sprintf((void*)bstr, "HTTPKey: %s",Params.Fields.HTTPKey);
                    PrintBT(bstr);
                break;
                case 'A':
                    sprintf((void*)bstr, "Interval: %u",Params.Fields.PingInterval);
                    PrintBT(bstr);
                break;
                case 'B':
                    sprintf((void*)bstr, "MHost: %s",Params.Fields.MQTTHost);
                    PrintBT(bstr);
                break;
                case 'C':
                    sprintf((void*)bstr, "MPort: %s",Params.Fields.MQTTPort);
                    PrintBT(bstr);
                break;
                case 'D':
                    sprintf((void*)bstr, "MCID: %s",Params.Fields.MQTTClientID);
                    PrintBT(bstr);
                break;
                case 'E':
                    sprintf((void*)bstr, "MTopic: %s",Params.Fields.MQTTTopic);
                    PrintBT(bstr);
                break;
                case 'F':
                    sprintf((void*)bstr, "MProt: %s",Params.Fields.MQTTProtocolName);
                    PrintBT(bstr);
                break;
                case 'G':
                    sprintf((void*)bstr, "MLVL: 0x%hhX",Params.Fields.MQTTLVL);
                    PrintBT(bstr);
                break;
                case 'H':
                    sprintf((void*)bstr, "MFlag: 0x%hhX",Params.Fields.MQTTFlags);
                    PrintBT(bstr);
                break;
                case 'I':
                    sprintf((void*)bstr, "MKAlive: %u",Params.Fields.MQTTKeepAlive);
                    PrintBT(bstr);
                break;
                case 'J':
                    sprintf((void*)bstr, "MUser: %s",Params.Fields.MQTTUsername);
                    PrintBT(bstr);
                break;
                case 'K':
                    sprintf((void*)bstr, "MPass: %s",Params.Fields.MQTTPassword);
                    PrintBT(bstr);
                break;
                case 'L':
                        sprintf((void*)bstr,"Time: 20%02d-%02d-%02d  %02d:%02d:%02d",sDate.Year,sDate.Month,sDate.Date,R.Hours,R.Minutes,R.Seconds);
                        PrintBT(bstr);
                        break;
                    case 'Z':
                    return;
                break;
                
            }
                    
            putcharBT('#');
            putcharBT('\n');
        }
        
        
    }
        
    BTBuffIndex++;
    if(BTBuffIndex >= BTBUFF_SIZE-1)
        BTBuffIndex = 0;
    
    return;
    //DisconnectDevice();
    //UpdateLED3(RED_COLOR);
    //SystemState = State_IdleState;
DISCARD: ResetBTBuffer();
        
        
    
}
void ReadBTParams(char Byte)
{
    char *pToken;
    char bstr[250];
    unsigned char i;

    if(Byte == '$')
    {
        BTBuffIndex = 0;
    }
    BTBuff[BTBuffIndex] = Byte;
    
    ResponsePacket[0] = 0x02; ResponsePacket[1] = 0x01;        
    ResponsePacket[0] = '$'; 
    //HAL_NVIC_DisableIRQ(USART1_IRQn);
    //HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    //P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
    //HAL_NVIC_EnableIRQ(LPUART1_IRQn);
//            
//            for(unsigned char i = 0;i<15;i++)
//            {
//                ResponsePacket[0] = IMEI[i]; 
//                P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
//            }
//            ResponsePacket[0] = '#'; 
//            P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)ResponsePacket);
    

    
    if(BTBuff[BTBuffIndex] == '#')
    {
        pToken = MapForward((char*)BTBuff,BTBUFF_SIZE,(char*)"VALETRON",8);
        if(pToken != NULL)
        {
            
            //HAL_Delay(2000);
            memset(bstr,0,sizeof(bstr));
            i=0;
            while(pToken[11+i] != '#')
            {
                bstr[i] = pToken[11+i];
                i++;
                if(i>250)goto DISCARD2;
            }
            bstr[i] = '\0';
            //FieldUpdated = pToken[9];
            
            
            GetEEParams();
            
            
            
            
            
            putcharBT('$');
            
            switch(pToken[9])
            {
                case '0':
                    sprintf((void*)bstr, "Band: %s",Params.Fields.Band);
                    PrintBT(bstr);
                break;
                case '1':
                    sprintf((void*)bstr, "WM: %s",Params.Fields.WorkingMode);
                    PrintBT(bstr);
                break;
                case '2':
                    sprintf((void*)bstr, "MAlert: %s",Params.Fields.MotionAlertMode);
                    PrintBT(bstr);
                break;
                case '3':
                    sprintf((void*)bstr, "MThresh: %hhu",Params.Fields.MotionThreshold);   
                    PrintBT(bstr);
                    
                break;
                case '4':
                    sprintf((void*)bstr, "Contact: %s",Params.Fields.rxNumber);
                    PrintBT(bstr);
                break;
                case '5':
                    sprintf((void*)bstr, "APN: %s",Params.Fields.APNName);
                    PrintBT(bstr);
                break;
                case '6':
                    sprintf((void*)bstr, "APNUser: %s",Params.Fields.APNUsername);
                    PrintBT(bstr);
                break;
                case '7':
                    sprintf((void*)bstr, "APNPass: %s",Params.Fields.APNPassword);
                    PrintBT(bstr);
                break;
                case '8':
                    sprintf((void*)bstr, "HTTPURL: %s",Params.Fields.HTTPURL);
                    PrintBT(bstr);
                break;
                case '9':
                    sprintf((void*)bstr, "HTTPKey: %s",Params.Fields.HTTPKey);
                    PrintBT(bstr);
                break;
                case 'A':
                    sprintf((void*)bstr, "Interval: %u",Params.Fields.PingInterval);
                    PrintBT(bstr);
                break;
                case 'B':
                    sprintf((void*)bstr, "MHost: %s",Params.Fields.MQTTHost);
                    PrintBT(bstr);
                break;
                case 'C':
                    sprintf((void*)bstr, "MPort: %s",Params.Fields.MQTTPort);
                    PrintBT(bstr);
                break;
                case 'D':
                    sprintf((void*)bstr, "MCID: %s",Params.Fields.MQTTClientID);
                    PrintBT(bstr);
                break;
                case 'E':
                    sprintf((void*)bstr, "MTopic: %s",Params.Fields.MQTTTopic);
                    PrintBT(bstr);
                break;
                case 'F':
                    sprintf((void*)bstr, "MProt: %s",Params.Fields.MQTTProtocolName);
                    PrintBT(bstr);
                break;
                case 'G':
                    sprintf((void*)bstr, "MLVL: 0x%hhX",Params.Fields.MQTTLVL);
                    PrintBT(bstr);
                break;
                case 'H':
                    sprintf((void*)bstr, "MFlag: 0x%hhX",Params.Fields.MQTTFlags);
                    PrintBT(bstr);
                break;
                case 'I':
                    sprintf((void*)bstr, "MKAlive: %u",Params.Fields.MQTTKeepAlive);
                    PrintBT(bstr);
                break;
                case 'J':
                    sprintf((void*)bstr, "MUser: %s",Params.Fields.MQTTUsername);
                    PrintBT(bstr);
                break;
                case 'K':
                    sprintf((void*)bstr, "MPass: %s",Params.Fields.MQTTPassword);
                    PrintBT(bstr);
                break;
                case 'L':
                        sprintf((void*)bstr,"Time: 20%02d-%02d-%02d  %02d:%02d:%02d",sDate.Year,sDate.Month,sDate.Date,R.Hours,R.Minutes,R.Seconds);
                        PrintBT(bstr);
                        break;
                    case 'Z':
                    return;
                break;
                
            }
                    
            putcharBT('#');
            putcharBT('\n');
        }
        
        
    }
        
        BTBuffIndex++;
        if(BTBuffIndex >= BTBUFF_SIZE-1)
            BTBuffIndex = 0;
        
        return;
        //DisconnectDevice();
        //UpdateLED3(RED_COLOR);
        //SystemState = State_IdleState;
DISCARD2: ResetBTBuffer();
        
        
    
}
void GetNetworkData(void);

unsigned char FirstBoot = 0;
unsigned char GSMInactiveCount=0;

unsigned char InitGSM(void)
{
    unsigned char i;
    char*pToken;
    //char str[50];
    #ifdef DEBUG_PRINT
        DebugPrint("Entered-InitGSM\r\n"); 
    #endif
    //printf("InitGSM-ENtered\n");
    //while(1){osDelay(10);}
    //int cYear,cMonth,cDate,cHour,cMinute,cSecond;
    // Make PWRKEY High
    UpdateNetwork(0);
    //GSM_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
    
    //osDelay(5000);
    EnableGSM();
    
    if(FirstBoot == 0)
        goto SKIP_RESET;
    RESTART:
    #ifdef DEBUG_PRINT
        DebugPrint("Goto-RESTART-InitGSM\r\n"); 
    #endif
    DisableGSM();
    osDelay(500);
    EnableGSM();
    osDelay(3000);
    //GSM_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
    gpio_set_level(GPIO_PWRKEY,1);
   
    osDelay(1000);
    gpio_set_level(GPIO_PWRKEY,0);
    //GSM_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);

    
    SKIP_RESET:
    FirstBoot = 1;
    
    osDelay(3000);
    //ResetBuffer();    
    LoopTimeout1 = 0;
    
    while(1)
    {
        
        
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"DONE",4) != NULL) || (LoopTimeout1>10))
        {       break; }
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ATREADY",7) != NULL) || (LoopTimeout1>10))
        {       break; }
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"QCRDY",5) != NULL) || (LoopTimeout1>10))
        {       break; }
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"CPIN: READY",11) != NULL) || (LoopTimeout1>10))
        {       break; }
        osDelay(1);
        
    }
    //printf("init wait done\n");
    UpdateNetwork(0);
    osDelay(5000);
     if(DeviceStatus == 0)
        return 3;
    //osDelay(1000);
    LoopTimeout2 = 0;
    while(SendATCommand("AT\r\n","OK","ERROR",3)==3)//Print("AAAT\r\n");
    {
        if(LoopTimeout2 > 20)
        {
            //printf("No response for AT-retrying\n");
            //esp_restart();
            //goto RESTART;
            //MotionTimer = TIME_TO_SLEEP+1;
            return 3; // return due to no response


        }
    }
    //ResetBuffer();
    if(SendATCommand("AT+CPIN?\r\n","OK","ERROR",3) !=1)
    {
        MotionTimer = TIME_TO_SLEEP+1;
        return 3;
    }        
   
    SendATCommand("AT+CMGF=1\r\n","OK","ERROR",3);//    Print("AT+CMGF=1\r\n");
   
    SendATCommand("AT+CMGF=1\r\n","OK","ERROR",3);//Print("AT+CMGF=1\r\n");
 
    
    
    ESP_LOGI(TAG," Buff2Index = %d",Buff2Index);



    if(Buff2Index == 0) 
    {
        #ifdef DEBUG_PRINT
            DebugPrint("BuffIndex=0-InitGSM\r\n"); 
        #endif
        if(++GSMInactiveCount>10)
        {
            #ifdef DEBUG_PRINT
                DebugPrint("GSMInactiveCount>10-InitGSM\r\n"); 
            #endif
            //BackupPackets();
            //WriteSRAM(BUFF2_RESET); //TBD
            ESP_LOGW(TAG,"Rebooting from InitGSM-GSM Inactive count");
            esp_restart();
        }
        printf("Buff2Index=0\n");
        
        
        goto RESTART;
    } 
        
    SendATCommand("AT+CGMR\r\n","OK","ERROR",3);//Print("AT+CGMR\r\n");

    // Shifted up for faster location
    #ifdef EXT_ANT_ENABLED
    
            // #if defined(SIM7600)  
            //     SendATCommand("AT+CGPS=1,1\r\n","OK","ERROR",3);
            // #endif
            #if defined(SIM7070)  
                SendATCommand("AT+CGNSPWR=1\r\n","OK","ERROR",3);     
            #endif       
            #if defined(A7672)
                SendATCommand("AT+CGNSSPWR=1\r\n","READY","ERROR",60);
                SendATCommand("AT+CGNSSPWR?\r\n","OK","ERROR",60);
            #endif
            #if defined(SIM7672)
                
                SendATCommand("AT+CGNSSPWR=1\r\n","OK","ERROR",60);
                SendATCommand("AT+CGNSSPWR?\r\n","OK","ERROR",60);
                SendATCommand("AT+CGNSSPORTSWITCH=1,1\r\n","OK","ERROR",5);
                SendATCommand("AT+CGNSSTST=1\r\n","OK","ERROR",30);
                SendATCommand("AT+CGPSCOLD\r\n","OK","ERROR",30);
                //SendATCommand("AT+CGNSSFTM=1\r\n","OK","ERROR",30);
                //SendATCommand("AT+CGNSSINFO\r\n","OK","ERROR",30);
                SendATCommand("AT+CGNSSIPR?\r\n","OK","ERROR",30);
                SendATCommand("AT+SIMCOMATI\r\n","OK","ERROR",30);
                 osDelay(5000);
                 SendATCommand("AT+BT\r\n","OK","ERROR",30);
                osDelay(5000);
                
                
            #endif
            // SendATCommand("AT+CGPSINFO\r\n","OK","ERROR",3);
        
    #endif


    osDelay(1000);
    // 

    ResetBuffer();
    Print("AT+CSQ\r\n");
    osDelay(500);;
    
    #ifdef SIM800
        SendATCommand("AT+IPR=9600\r\n","OK","ERROR",3);
//        ResetBuffer();
//        Print("AT+IPR=9600\r\n");
//        osDelay(500);
    #endif
    #ifdef SIM7070
        SendATCommand("AT+IPR=115200\r\n","OK","ERROR",3);
//        ResetBuffer();
//        Print("AT+IPREX=115200\r\n");
//        osDelay(500);; 
    #endif
    #ifdef SIM7600
        SendATCommand("AT+IPREX=115200\r\n","OK","ERROR",3);
    #endif

    SendATCommand("AT+CLIP=1\r\n","OK","ERROR",3);
    

//    osDelay(500);
	SendATCommand("AT+CNMI=2,1,0,0,0\r\n","OK","ERROR",3);	

    #ifdef SIM800
        SendATCommand("AT+CLTS=1\r\n","OK","ERROR",3);
        //osDelay(500);
    #else
        SendATCommand("AT+CTZU=1\r\n","OK","ERROR",3);
        //osDelay(500);
    #endif
    
//    ResetBuffer();
    #ifdef SIM800
        SendATCommand("AT+CBAND=\"EGSM_MODE,ALL_BAND\"\r\n","OK","ERROR",3);
    #endif
    #ifdef SIM7070
        #ifdef CNMP_13
        SendATCommand("AT+CNMP=13\r\n","OK","ERROR",3);
        #endif
        #ifdef CNMP_38
        SendATCommand("AT+CNMP=38\r\n","OK","ERROR",3);
        #endif
        #ifdef CNMP_2
        SendATCommand("AT+CNMP=2\r\n","OK","ERROR",3);
        #endif
        
        // NB selection
        #ifdef CMNB_1
        SendATCommand("AT+CMNB=1\r\n","OK","ERROR",3);
        #endif
        
        // NB selection
        #ifdef CMNB_2
        SendATCommand("AT+CMNB=2\r\n","OK","ERROR",3);
        #endif
        
        // NBM1 selection
        #ifdef CMNB_3
        SendATCommand("AT+CMNB=3\r\n","OK","ERROR",3);
        #endif
        
    #endif
    #ifdef SIM7600
        SendATCommand("AT+CNMP=2\r\n","OK","ERROR",3);
    #endif

    sprintf((void*)str,"AT+CGDCONT=1,\"IP\",\"%s\"\r\n",Params.Fields.APNName);
    SendATCommand(str,"OK","ERROR",5);
    
    #ifdef SIM7600
        SendATCommand("ATS0=003\r\n","OK","ERROR",3);
    #endif
    


    #ifdef SIM800
        SendATCommand("AT&W0\r\n","OK","ERROR",3);
    #endif
       // reset all 

    CheckBattery();

  
    SendATCommand("AT+CIMI\r\n","OK","ERROR",5);
    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"\n",1);
    if(pToken != NULL)
    {
        for(i = 0; i < 15; i++)
        {
            IMSI[i] = pToken[i+1];
        }
        IMSI[15] = '\0';
    }
    
    // SendATCommand("AT+COPS=?\r\n","OK","ERROR",500);    
    
    // #ifdef SIM7070
    // osDelay(10000);
    // #endif
    if(CheckNetwork() == 1)
    {
        #ifdef DEBUG_PRINT
            DebugPrint("CheckNetwork returned 1-InitGSM\r\n"); 
        #endif
 
        
        //goto CHECK_NETWORK_AGAIN;//goto RESTART;
        #ifdef NETWORK_FAIL_RESET_ENABLED
            // BackupPackets(); // TBD
            // WriteSRAM(NETWORK_RESET);
            ESP_LOGW(TAG,"Rebooting from InitGSM-Network Fail Reset");
             esp_restart();
        #else
            return 3;
        #endif
        

    }
    UpdateNetwork(1);
    
    
    
    osDelay(1000);
    
    
    SendATCommand("AT+CGSN\r\n","OK","ERROR",5);
    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"\n",1);
    if(pToken != NULL)
    {
        for(i = 0; i < 15; i++)
        {
            IMEI[i] = pToken[i+1];
        }
        IMEI[15] = '\0';
    } 

    SendATCommand("AT+CIMI\r\n","OK","ERROR",5);
    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"\n",1);
    if(pToken != NULL)
    {
        for(i = 0; i < 15; i++)
        {
            IMSI[i] = pToken[i+1];
        }
        IMSI[15] = '\0';
    }
    
    
    if(Params.Fields.WorkingMode[0]=='S')
    {
        DeleteAllSMS();        
        DDelay();
    }
    //GetNetworkData();
    CheckNetworkLocation();
    
    
    // CheckNetworkLocation();
   
    return 0;
}

void InitGPIO(void)
{
    //////////////////////////////////////////////////////////GPIO
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_PWRKEY_GSM_ENABLE_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_LED_SIGNAL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

#ifdef ENABLE_TPS_CONTROL
    //////////////////////////////////////////////////////////GPIO
    //zero-initialize the config structure.
    //gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_TPS_ENABLE_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
#endif
    ///////////////////////////INT1 and SOS
    //no interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

#ifdef ENABLE_TPS_CONTROL
    /* Initialize selected GPIO as RTC IO, enable output, disable pullup and pulldown, enable hold*/
    gpio_hold_dis(GPIO_TPS_ENABLE);
#endif 
    // gpio_hold_en(GPIO_GSM_ENABLE);
    gpio_hold_dis(GPIO_GSM_ENABLE);
    gpio_deep_sleep_hold_dis();

}

void SystemInit(void) //static void echo_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    memcpy(Params.Bytes,DefaultParams.Bytes,sizeof(Params));
   
    InitGPIO();
    EnableMainPower();
 
    configure_led();
 

    InitUART();


    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    

    InitAccelerometer();
    
    
}

void StartTimerTask(void *argument)
{
//    #ifdef DEBUG_PRINT
//        DebugPrint("Entered -StartTimerTask\r\n"); 
//    #endif

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    //vTaskDelay(1);
      if(SleepModeEnabled == 1) 
      {

            //osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);// TBD
      }
      //HandleGPSData();  
      //   
//    unsigned char i;
    //millis++;
    
    if(SystemTimer > 15 && Params.Bytes[0] == 0)
    {
        ESP_LOGI(TAG,"SystemTimer>15");
        while(1);
    }
    
    /* USER CODE BEGIN SysTick_IRQn 1 */
   
    INT1 = (MotionStatusType)gpio_get_level(GPIO_INT1);
    #ifndef VALTRACK_V4_VTS
        ChargingStatus = (ChargingStatusType)gpio_get_level(GPIO_CHARGER_PIN);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
        if(ChargingStatus == CONNECTED)// && pChargingStatus == DISCONNECTED)
        {
            FrontPanelTimer = 0; // Battery LED Always ON during charging
            UpdateNetwork(3);
            UpdateLocation(0);
            WriteLEDStatus();
            //pChargingStatus = ChargingStatus;
        }
    #endif
        //if(LEDInhibit == 0) 
    //    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,!INT1);
    
    SOS = gpio_get_level(GPIO_SOS);

    
    
    if(POWER_BUTTON == 1) 
        SOSActivated = 1; 
        
    #ifdef POWER_BUTTON_SOS_SWAP
        SOS = POWER_BUTTON;
    #endif
    // reboot on press code
    #ifndef VALTRACK_V4_VTS
        if(SOS == 1)
        {
            ButtonPressTimer = 0;
        }
        if(ButtonPressTimer > 4) 
        {
            ESP_LOGI(TAG,"Rebooting from StartTimer - ButtonLongPress");
            MakeAllLED(PURPLE);
            WriteLEDStatus();
            osDelay(2000);
            //esp_restart();
            //PowerButtonSleep = 1;
            //DeepSleep();
        }
    #endif
    // end
    if(SOS != pSOS)
    {
        
        if(SOS == 0)
        {
            FrontPanelTimer = 0;
      
            #ifndef WB_PIN_CONTROLLED_LED
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
            BackupPCAStatus();
            #endif
        }
        if(SOS == 1)
        {
            //ButtonPressTimer = 0;
            #ifndef WB_PIN_CONTROLLED_LED
            if(DeviceStatus == 1)
            {
                RestorePCAStatus();
                //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
            }
            #endif
        }
        
        
        pSOS = SOS;
    }
    if(INT1 == 0)
    {
        #ifndef TIMER_ONLY_WAKEUP
            MotionTimer=0;
        
            #ifdef VALTRACK_V4_VTS
                if(FrontPanelTimer > 120)
                {
                    MakeAllLED(PURPLE);
                }
                FrontPanelTimer=0;
            #endif
        
        #endif
       
    }
    if(xTaskGetTickCount() - lastTickValue >= 100)//++tSeconds>=1000)
    {
        lastTickValue = xTaskGetTickCount();
        SystemTimer++;
        InactivityTimer++;
        RTCTimeout++;
        IntervalTimer++;
        #ifndef TIMER_ONLY_WAKEUP
        MotionTimer++;
        #endif
//        #ifndef MOTION_CONTROLLED_PINGS
//        MotionTimer=0;
//        #endif
        BatteryTimer++;
        //HeartBeatTimer++;
        LoopTimeout1++;
        LoopTimeout2++;
        LoopTimeout3++;
        FrontPanelTimer++;
        DebounceTimer++;
        ButtonPressTimer++;
        //GSMResetTimer++;
        LPUARTTimer++;
        ConnectivityTimer++;
        
        #ifdef TAMPER_DETECT_MODE 
            if(SOSActivated == 1)
            {
                if(SystemTimer%15 == 0)
                {
                    PostSOSPing();
                    SOSActivated = 0;
                }
            }
        
        #endif
        if(BootTimer<10)
            BootTimer++;
        
        if(FrontPanelTimer == 1)
        {
            //VALTRACK_BLE_Advertise(1); //TBD
            
        }
        if(FrontPanelTimer == 180)
        {
           //VALTRACK_BLE_Advertise(0); //TBD
        }
        if(IntervalTimer > Params.Fields.PingInterval && ChargingStatus == DISCONNECTED)
        {
            #ifdef MOTION_CONTROLLED_PINGS
                if(MotionTimer<TIME_TO_SLEEP)
                {
                        
                    PostGPing();
                }
            #else
                PostGPing();
            #endif
           
            IntervalTimer = 0;
        }
        #ifdef LPUART_TIMER_RESET_ENABLED
        // If no events and no cache means nothing to send so timer expire is not valid
        if( ((LPUARTTimer>180) && (SleepModeEnabled!=1)) && ( (HeadIndex != TailIndex) || (GPacketCacheIndex != 0)) )//&& MotionTimer < TIME_TO_SLEEP)
        {
            MotionTimer = TIME_TO_SLEEP+10;
           
            DisableGSM();
            BackupPackets();
            //WriteSRAM(LPUART_TIMER_RESET); // TBD
            ESP_LOGI(TAG,"Rebooting from StartTimer - LPUARTTimer>180");
            esp_restart();
        }
        
        // If no events and no cache means nothing to send so timer expire is not valid
        if( (LPUARTTimer>60 && SleepModeEnabled!=1) && ( (HeadIndex != TailIndex) || (GPacketCacheIndex != 0)))// TRYING TO RESET UART AND SEE IF IT RECOVERS
        {
            // HAL_UART_DeInit(&hlpuart1);
            // MX_LPUART1_UART_Init();
        }
        #endif
        if( (HeadIndex == TailIndex) && (GPacketCacheIndex == 0) )
        {
            LPUARTTimer = 0;// To prevent reset on always on long intervals. bracelet project
        }
        
         if(SystemState  == State_ConnectedState)
         {
             MakeAllLED(BLUE);
         }   
         if(AnswerCall == 1)
         {
             Print("ATA\r\n");
             AnswerCall = 0;
         }

        EEPROMReadTimer++;
        

        
        
        if(DeviceStatus == 1)
        {
            UpdateBattery(ADCBatteryVoltage);
            if(FrontPanelTimer>120)
            {
                MakeAllLED(TURN_OFF);
                WriteLEDStatus();
               
            }
            else
            {
                if(BootTimer == 1)
                    MakeAllLED(BLUE);
                if(BootTimer == 2)
                    MakeAllLED(GREEN);
                if(BootTimer == 3)
                    MakeAllLED(RED);
                if(LEDTouched == 1)
                {
                    WriteLEDStatus();
                    LEDTouched = 0;
                }
            }
            // AverageADCSamples();
            //ADCBatteryVoltage = (((float)BatteryADCCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
            //ADCBatteryVoltage +=0.3;
            //ADCBatteryVoltage = (((float)BatteryADCCount*3.3*(float)DIVIDER_FACTOR)/4096);
            
            
            //WriteLEDStatus();
        }

        if(SMSSent==0)
            NoSignalTimer++;

        
        
        

        
        tSeconds=0; 
        //HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN);
        //HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        ESP_LOGE(TAG," ST = %d, MT= %d, FT = %d, INT1 = %d, SE = %d, V = %0.2f, HT/LT = %d/%d, G = %c, SW = %d, C = %d",
         SystemTimer,MotionTimer,FrontPanelTimer,INT1,SleepModeEnabled,ADCBatteryVoltage,HeadIndex,TailIndex,GPSStatus,SOS,ChargingStatus);
         
         vTaskDelay(1);
        
    }
   
    
   // PG_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
   // CHG_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
		
		
  /* USER CODE END SysTick_IRQn 1 */
    //osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
    //ESP_LOGI(TAG,"End of StartTimerTask %d,SystemTimer = %d",xTaskGetTickCount(),SystemTimer);
  }
  /* USER CODE END 5 */ 
}

uint16_t Addr;
//uint8_t WriteBuffer[256],ReadBuffer[256];
//extern unsigned char Link[];
//extern unsigned char D1,D2,GPSStatus;
//extern double fLat,fLong; 
void SendLocation(void)
{
    
    //unsigned char i;
    WakeUp();
    CheckBattery();
    ResetBuffer();            
    Print((char*)"AT+CMGS=\"");
    Print(rxNumber);
    Print((char*)"\"\r\n");
    DelayProc(850000);
    Print("Location: \r\n");
    Print(Link);
    Print("\r\n");
    Print(BatteryString);
    Print("\r\n -VALTRACK V2 SMS\r\n-www.valetron.com\r\n-www.raviyp.com\r\n");
    putcchar(0x1A);
    DelayProc(850000);            
    
}
void SendLastLocation(void)
{
    
    //unsigned char i;
    WakeUp();
    CheckBattery();
    ResetBuffer();            
    Print((char*)"AT+CMGS=\"");
    Print(Params.Fields.rxNumber);
    Print((char*)"\"\r\n");
    DelayProc(850000);
		#ifdef SOSALERT
	  if(SosAlert==1){
		Print("SOS ALERT:  \r\n");
		}
	  #endif
    Print("Location: \r\n");
    Print("Last Known: \r\n");
    sprintf
    (
        (void*)Link,
        "http://maps.google.com/maps?z=18&q=%lf,%lf",
        pfLat,pfLong
    );
    Print(Link);
    Print("\r\n");
    Print(BatteryString);
    Print("\r\n -VALTRACK V2 SMS\r\n-www.valetron.com\r\n-www.raviyp.com\r\n");
    putcchar(0x1A);
    DelayProc(850000);            
    
}

void SendAlert(void)
{
    
    //unsigned char i;
    WakeUp();
    CheckBattery();
    ResetBuffer();            
    Print((char*)"AT+CMGS=\"");
    Print(Params.Fields.rxNumber);
    Print((char*)"\"\r\n");
    DelayProc(850000);
    Print("Motion Detected: \r\n");
    Print("Location: \r\n");
    
    if(GPSStatus != 'A')
    {
        Print("Last Known: \r\n");
        sprintf
        (
            (void*)Link,
            "http://maps.google.com/maps?z=18&q=%lf,%lf",
            pfLat,pfLong
        );
    }
    else
    {
        sprintf
        (
            (void*)Link,
            "http://maps.google.com/maps?z=18&q=%lf,%lf",
            fLat,fLong
        );
    }
    Print(Link);
    Print("\r\n");
    Print(BatteryString);
    Print("\r\n -VALTRACK V2 SMS\r\n-www.valetron.com\r\n-www.raviyp.com\r\n");
    putcchar(0x1A);
    DelayProc(850000);            
    
}

void GetNetworkLocation(void)
{
    char *pToken;
    if(GPSStatus == 'V')
    {
        ResetBuffer();
        
        SendATCommand("AT+CLBS=1\r\n","+CLBS:","ERROR",15);
        //Print("AT+HTTPREAD=0,50\r\n");
        //DelayProc(850000);
        osDelay(500);
        LoopTimeout1 = 0;
        while(1)
        {
            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"+CLBS:",6);
            if(pToken != NULL)
            {
                sscanf((void*)pToken,"+CLBS: %d",(int*)&NStatus);
                if(NStatus == NL_SUCCESS)
                {
                    //osDelay(1000);//DelayProc(50000);
                    sscanf((void*)Buff2,"+CLBS: %d,%f,%f,%d",(int*)&NStatus,&NLat,&NLong,(int*)&NAccuracy);
                    sprintf((void*)NLPacket,",\"nlat\":\"%f\",\"nlon\":\"%f\",\"ltype\":\"NL\"",NLat,NLong);
                }
            }
            if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
            {       
                break; 
            }
        }
    }
    else
    {
        NLPacket[0] = 0;
    }
                //ResetBuffer2();    
                //Print
}
typedef struct NetworkDataField
{
    unsigned char Bytes[25];
}NetworkDataFieldType;

typedef union CENGTypeStruct
{
    NetworkDataFieldType Params[6];
    struct FieldsStruct
    {
        unsigned char   Index[25];
        unsigned char mcc_mnc[25];
        unsigned char     lac[25];
        unsigned char  cellid[25];
        unsigned char    rsrp[25];
        unsigned char    rsrq[25];
//        unsigned char mnc[6];
//        unsigned char lac[6];
//        unsigned char c1[6];
//        unsigned char c2[6];
    }Fields;
    
    // unsigned char  Index[2];
    // unsigned char bcch[5];
    // unsigned char rxl[5];
    // unsigned char bsic[5];
    // unsigned char cellid[6];
    // unsigned char mcc[5];
    // unsigned char mnc[5];
    // unsigned char lac[6];
    // unsigned char c1[5];
    // unsigned char c2[5];
}NetworkDataType;
#define NETWORK_DATA_SIZE 6
#define NETWORK_PARAMS_COUNT 6

NetworkDataType NetworkData[NETWORK_DATA_SIZE];

void HandleNetworkTowerData(char *pData,NetworkDataType *pNetworkData)
{
    char *pToken;
    unsigned char i;
    char seps[] =": ,\t\n";
    
    pToken = strtok (pData,seps);
    i=0;
    //if(pToken!= NULL) pToken = strtok (NULL, seps);
    while (pToken != NULL)
    {
        
        pToken = strtok (NULL, seps);
        sscanf (pToken, "%s", (char*)&pNetworkData->Params[i]);
        
        pToken = strtok (NULL, seps);
        
        
        i++;
        if(i>=NETWORK_PARAMS_COUNT) break;
    }
}
char cmdstr[200];
unsigned char TowerCount = 0;
void NetworkJSONData(char *pData)
{
    unsigned char i;
    
    sprintf((void*)pData,"[");
    for(i = 0; i < TowerCount; i++)
    {      
        
        if(i>=3) break; // Sending max 3 tower
            
        sprintf(cmdstr,"{%s,%s,%s}",NetworkData[i].Fields.mcc_mnc,NetworkData[i].Fields.lac,NetworkData[i].Fields.cellid);
        strcat(pData,cmdstr);
        
    }
    strcat(pData,"]");
//    sprintf((void*)pData,"[{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"},{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"},{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"}]",
//        NetworkData[0].Fields.cellid,
//        NetworkData[0].Fields.mcc,
//        NetworkData[0].Fields.mnc,
//        NetworkData[0].Fields.lac,
//        NetworkData[1].Fields.cellid,
//        NetworkData[1].Fields.mcc,
//        NetworkData[1].Fields.mnc,
//        NetworkData[1].Fields.lac,
//        NetworkData[2].Fields.cellid,
//        NetworkData[2].Fields.mcc,
//        NetworkData[2].Fields.mnc,
//        NetworkData[2].Fields.lac
//    );
}

void GetNetworkData(void)
{
    char *pToken;
    unsigned char i,j,retries;
    NetworkDataType *pNetworkData;
    retries = 0;
RETRY_NWD:    
    
    ResetBuffer();
    Print( "AT+CNETCI?\r\n"); 
    osDelay(2000);
    
    ResetBuffer();
    Print( "AT+CNETCI=1\r\n"); 
    osDelay(2000);    
    
    ResetBuffer();
    Print( "AT+CNETCI?\r\n");    
    LoopTimeout1 = 0;
    
    while(1)
    {
        pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"+CNETCINONINFO:",15);
        if(pToken != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>10))
        {       return; }
        
    }
    osDelay(5000);
    j=0;
    for(i=0;i<9;i++) // Check for 9 towers
    {
        sprintf(cmdstr,"+CNETCINONINFO: %d",i);
        pToken = MapForward(Buff2,BUFF2_SIZE,(char*)cmdstr,17);
        if(pToken != NULL)
        {
            printf("found = %d\n",i);
            pNetworkData = &NetworkData[j];
            HandleNetworkTowerData(pToken,pNetworkData);
            
            j++; // j is network array length
            if(j>=NETWORK_DATA_SIZE) 
                break;
        }
    }
    TowerCount = j;// For reading later
    if(j == 0 && retries  == 0)
    {
        retries++;
        goto RETRY_NWD; // Try again once more if no tower data arrived
    }
    NetworkJSONData(TowerPacket);
//    pToken = MapForward(Buff2,BUFF2_SIZE,(unsigned char*)"+CNETCINONINFO: 1",17);
//    if(pToken != NULL)
//    {
//        pNetworkData = &NetworkData[1];
//        HandleNetworkTowerData(pToken,pNetworkData);
//    }
//    pToken = MapForward(Buff2,BUFF2_SIZE,(unsigned char*)"+CNETCINONINFO: 2",17);
//    if(pToken != NULL)
//    {
//        pNetworkData = &NetworkData[2];
//        HandleNetworkTowerData(pToken,pNetworkData);
//    }
     
}
// void NetworkJSONData(char *pData)
// {
//     sprintf((void*)pData,"[{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"},{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"},{\"cid\":\"%s\",\"mcc\":\"%s\",\"mnc\":\"%s\",\"lac\":\"%s\"}]",
//         NetworkData[0].Fields.cellid,
//         NetworkData[0].Fields.mcc,
//         NetworkData[0].Fields.mnc,
//         NetworkData[0].Fields.lac,
//         NetworkData[1].Fields.cellid,
//         NetworkData[1].Fields.mcc,
//         NetworkData[1].Fields.mnc,
//         NetworkData[1].Fields.lac,
//         NetworkData[2].Fields.cellid,
//         NetworkData[2].Fields.mcc,
//         NetworkData[2].Fields.mnc,
//         NetworkData[2].Fields.lac
//     );
// }

void ConvertToJSONPacket(HWEventDataType *pPacket,unsigned short *pDataLength, char *pStr)
{
    
query[0] = 0;
//// \"time\":\"20%02d-%02d-%02d %02d:%02d:%02d\",
#ifdef PAYLOAD_NORMAL
    *pDataLength = sprintf((void*)pStr,
    "{\"devid\":\"%s\",\
\"etype\":\"%s\",\
\"lat\":\"%f\",\
\"lon\":\"%f\",\
\"vbat\":\"%f\",\
\"speed\":\"%s\"%s%s}",
        
        IMEI,
        //pPacket->GEvent.Year,pPacket->GEvent.Month,pPacket->GEvent.Date,pPacket->GEvent.Hours,pPacket->GEvent.Minutes,pPacket->GEvent.Seconds,
        (ETypes[pPacket->GEvent.EventType].Bytes),
        pPacket->GEvent.Lat,//tfLat,//pPacket->GEvent.fLat,
        pPacket->GEvent.Long,//tfLong,//pPacket->GEvent.fLong,
        pPacket->GEvent.Voltage,//ChargeVoltage,
        //pPacket->GEvent.Speed,//pPacket->GEvent.Speed//,
        TowerPacket,
        NLPacket,
        query
    );
#endif
    
}
void ConvertToJSON(HWEventDataType *pPacket,unsigned short *pDataLength, char *pStr)
{
    unsigned short tDataLength=0;
    unsigned char topic[50];
query[0] = 0;
#ifdef PAYLOAD_MUKESH_NEPAL

    *pDataLength = sprintf((void*)pStr,
"{"\
"\"payload\": {"\
"\"gps\": {"\
"\"gpslock\": \"1/0\","\
"\"longitude\": \"%lf\","\
"\"latitude\": \"%lf\""\
"},"\
"\"Vbat\": \"%lf\","\
"\"Vin\": \"12.6v\","\
"\"ignition\": \"1/0\","\
"\"fuel\": \"value \","\
"\"RS232\": \"value \","\
"\"speed\": \"%f \","\
"\"in1\": \"0/1\","\
"\"AD2\": \"value \","\
"\"time\":\"20%d-%d-%d %d:%d:%d\",}}",
// GPS lock
pPacket->GEvent.Long,
pPacket->GEvent.Lat,
pPacket->GEvent.Voltage,// Vbat
// 12V IN
// Ignition
//fuel
//RS232
pPacket->GEvent.Speed,
//In1
//AD2,
pPacket->GEvent.Year,pPacket->GEvent.Month,pPacket->GEvent.Date,pPacket->GEvent.Hours,pPacket->GEvent.Minutes,pPacket->GEvent.Seconds
);
#endif

#ifdef PAYLOAD_NORMAL

    memset((void*)topic,0,50);
    #ifdef SIM800
    if(Params.Fields.WorkingMode[0] == 'T')
        sprintf((void*)topic,(void*)Params.Fields.MQTTTopic);
    
    #endif        
    
    *pDataLength = 0;
    tDataLength = sprintf((void*)&pStr[*pDataLength],"%s{\"resource\":[",topic);
    *pDataLength+=tDataLength;
    ConvertToJSONPacket(pPacket,&tDataLength,&pStr[*pDataLength]);
    *pDataLength+=tDataLength;
    for(int i=0;i<PACKET_COUNT-1;i++)
    {
        if(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS)
        {
            tDataLength = sprintf((void*)&pStr[*pDataLength],",");
            *pDataLength+=tDataLength;
            pPacket=&GPacket;
            ConvertToJSONPacket(pPacket,&tDataLength,&pStr[*pDataLength]);
            *pDataLength+=tDataLength;
        }
    }
    tDataLength = sprintf((void*)&pStr[*pDataLength],"]}");
    *pDataLength+=tDataLength;

#endif
    
}
float ADCvalue;
//char cmdstr[200];
unsigned char TCPRetries = 0;
#ifdef SIM7600
char XUDP_Request(char *pFilename, unsigned char pingtype)
{

//    char eventnumber[5];
//    char *pResult;
    //unsigned char retries,i;
//    char *pToken;
//    unsigned char CheckSum;
//    unsigned char cs[2];
    HWEventDataType *pPacket;
    pPacket = &GPacket;
    //retries = 0;
    TCPRetries = 0;
    //unsigned char encodedByte;
    //int X; 
    // unsigned short MQTTProtocolNameLength;
    // unsigned short MQTTClientIDLength;
    // unsigned short MQTTUsernameLength;
    // unsigned short MQTTPasswordLength;
    

    
/*
    #ifndef STANDALONE_DEMO
    CheckSum = GetCheckSum(pPacket->Bytes,34);
    cs[1]=GetAscii(CheckSum&0x0F);  
    cs[0]=GetAscii((CheckSum>>4) & 0x0F);  
    if((pPacket->GEvent.CheckSum[0] != cs[0]) || (pPacket->GEvent.CheckSum[1] != cs[1]))
    {
        retVal=0;
        return 0;
    }
#endif
    */
    
  RESEND_UDP:
    WakeUp();
    SOS = gpio_get_level(GPIO_SOS);
//    if(SOS == 0)
//    {
//        goto SUCCESS;
//    }
    ////Print4("Resending\r\n");
    ////IWDG_ReloadCounter();
    ResetBuffer();
    Print( "AAAAAAAAAAAAAT\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       goto exit; }

    }
    osDelay(1000);
    ResetBuffer();
    Print( "AT+CSCLK=0\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       goto exit; }
        
    }
    osDelay(1000);//DelayProc(250000);
    
//    osDelay(1000);
    ResetBuffer();    
    Print( "AT+NETOPEN\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"+NETOPEN: 0",11) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }
    osDelay(3000);
////  restart:

    ResetBuffer();
    Print( "AT+IPADDR\r\n");
    osDelay(1000);        
    if(MapForward(Buff2,BUFF2_SIZE,(char*)"0.0.0.0",7) != NULL)
    {

        osDelay(1000);
        ResetBuffer();
        Print("AT+CIPCLOSE\r\n");
        LoopTimeout1 = 0;
        while(1)
        {
            if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                    goto exit;
            if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
            {       goto exit; }
            
        }  
        
    }
    
    RECONNECT:
    SendATCommand("AT+CIPOPEN=1,\"UDP\",,,5000\r\n","OK","ERROR",15);
    
    while(1)
    {
        if(DeviceStatus == 0)
            return 0;
//        osDelay(3000);
        
        
        if(RTCTimeout>(Params.Fields.PingInterval+20))
         {
             //SystemInternalClock_Config_LP();
             InitRTCAlarm();
         }
         
        if((MotionTimer > TIME_TO_SLEEP) && (QueEmpty==1))
        {
             goto SUCCESS;
            
        }
        CheckBattery();
        
        // HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN); // TBD
        // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // TBD
        
        
        osDelay(1000);
//       
//        INT1 = (MotionStatusType)gpio_get_level(GPIO_INT1);
        if(INT1 == 0)
        {
            //for(i=0;i<=0x31;i++)
            //{
            //    VALREAD = I2C_RdReg(i);
            //    INT1 = (MotionStatusType)gpio_get_level(GPIO_INT1);
            //}
//            #ifdef LIS3DH_ENABLED
//            ISRstatus = I2C_RdReg(REG_INT1_SRC);
//            InitAccelerometer();
//            #else
//            InitAccelerometer_mma84();
//            #endif
            InitAccelerometer();
            if(MotionTimer > 120)
            {
                PostMotionEvent();                
                #ifndef TIMER_ONLY_WAKEUP
                    MotionTimer=0;
                #endif
            }
        }
        
        if(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS)
        {
            
                
            pPacket=&GPacket;
         
            //if(tfLat == 0)tfLat = 8;
            memset(str,0,500);
            //pPacket->GEvent.Speed[9] = '\0';
            //devid[8] = '\0';
            topiclength = sprintf((void*)topic,(void*)Params.Fields.MQTTTopic);
            

            ConvertToJSON(pPacket,&datalength,str);
            //datalength = sprintf((void*)str,"%s{\"resource\":[{\"devid\":\"%s\"",topic,devid);
            ResetBuffer();
            //Print("AT+CIPSEND\r\n");
            sprintf((void*)cmdstr,"AT+CIPSEND=1,,\"%s\",%s\r\n",Params.Fields.MQTTHost,Params.Fields.MQTTPort);
            
            SendATCommand(cmdstr,">","ERROR",6);
            

//            Print(Params.Fields.MQTTHost);
//            Print("AT+CIPSEND=");
//            Print(Params.Fields.MQTTHost);
//            Print("\r\n");
            LoopTimeout1 = 0;
            while(1)
            {
                if(MapForward(Buff2,BUFF2_SIZE,(char*)">",1) != NULL)
                        break;
                if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
                {
                      
                    goto exit; 
                }
                
            }
            
            Print((void*)str);
            putcchar(0x1A);
            //osDelay(3000);
            osDelay(3000);
            
            
        }
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"CLOSED",6) != NULL)
        {
            goto RECONNECT;
        }
        if(Params.Fields.WorkingMode[0] !='U')
            goto SUCCESS;
        
//        SOS = gpio_get_level(GPIO_SOS);
//        if(SOS == 0)
//        {
//            goto SUCCESS;
//        }
      //  CheckBattery();
    }
    
    
    ////Print4(Buff);   
   
    //DelayProc(850000);
    //DelayProc(850000);
    //DelayProc(850000);
    //DelayProc(850000);
    goto SUCCESS;
    //free(string);
SUCCESS: 
    ClearEventCache();
    ConnectivityTimer = 0;
    ResetBuffer();
    Print("AT+CIPCLOSE=1\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"+CIPCLOSE: 1,0",14) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }          
    osDelay(1000);
    ResetBuffer();
    Print("AT+NETCLOSE\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"+NETCLOSE:",10) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }     
    osDelay(1000);
    
    //Print4("SUCCESS\r\n");
    
    retVal=0;
    return 0;
exit: 
    osDelay(1000);
    ResetBuffer();
    Print("AT+CIPCLOSE=1\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    } 
    osDelay(1000);
    ResetBuffer();
    Print("AT+NETCLOSE\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }
    osDelay(1000);
    //Print4("FAILED\r\n");
    
  
    if(++TCPRetries < 8) 
        goto RESEND_UDP;
    else
    {        
        retVal = 1;
        DisableGSM();
        InitGSM();
    }
    
//    #ifdef EEPROM_FIFO
//    PostEvent( pPacket);//PostEEEvent(pPacket);
//    #endif
    
    RestoreEventCache();
    
    
    return 0;
}
char XHTTP_Request(char *pFilename, unsigned char pingtype)
{

//    char eventnumber[5];
//    char *pResult;
    //unsigned char retries,i;
    HWEventDataType *pPacket;
    char *pToken;
    //unsigned char CheckSum,cs[2];
    pPacket = &GPacket;
    //retries = 0; 
    
    if(ChargingStatus == CONNECTED) return 0;
    
//#ifndef STANDALONE_DEMO
//    CheckSum = GetCheckSum(pPacket->Bytes,34);
//    cs[1]=GetAscii(CheckSum&0x0F);  
//    cs[0]=GetAscii((CheckSum>>4) & 0x0F);  
//    if((pPacket->GEvent.CheckSum[0] != cs[0]) || (pPacket->GEvent.CheckSum[1] != cs[1]))
//    {
//        retVal=0;
//        return 0;
//    }
//#endif
    
//   RESEND_HTTP: // TBD
    if(SystemState == State_ConnectedState) return 0; // Return and idle for proper configuration and prevent EEPROM access
    WakeUp();
    if(DeviceStatus == 0)
        return 0;		
//    SOS = gpio_get_level(GPIO_SOS);
//    if(SOS == 0)
//    {
//        goto SUCCESS;
//    }
    ////Print4("Resending\r\n");
    ////IWDG_ReloadCounter();
    ResetBuffer();
    //Print( "AAAAAAAAAAAAAT\r\n");
    Print( "AT\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       goto exit; }
        
    }
    ResetBuffer();
    Print( "AT+CSCLK=0\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       goto exit; }
        
    }
    osDelay(1000);


    sprintf((void*)str,"AT+CGDCONT=1,\"IP\",\"%s\"\r\n",Params.Fields.APNName);
    SendATCommand(str,"OK","ERROR",5);
    SendATCommand("AT+CGACT=1,1\r\n","OK","ERROR",10);
    if(SendATCommand("AT+CGACT?\r\n","+CGACT: 1,1","ERROR",10) != 1)
    {
        goto exit;
    }
    ResetBuffer();
    Print("AT+HTTPINIT\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {
            ResetBuffer();
          
            break;
        }
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
        {       break; }
        
    }
    
    ResetBuffer();
    Print("AT+HTTPPARA=\"CID\",1\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {      break; }
        
    }
    //IWDG_ReloadCounter();
    ResetBuffer();
    Print( "AT+HTTPPARA=\"URL\",\"");    
    //Print( "robopower.in/api/v2/mysql/_table/log");
    //Print( "alcobrake-lb-406408850.us-east-1.elb.amazonaws.com/api/v2/update");
    
    Print((char*)Params.Fields.HTTPURL);
    Print( "\"\r\n");       
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }
    

    ResetBuffer();
    Print("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
       
    }
        
    ResetBuffer();        
    //Print("AT+HTTPPARA=\"USERDATA\",\"X-DreamFactory-Api-Key: ");
    Print("AT+HTTPPARA=\"USERDATA\",\"");
    Print((char*)Params.Fields.HTTPKey);
    Print("\"\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }

    
//////////////////////////////////////////////////////////
    #ifdef EXT_ANT_ENABLED 
        XCheckGPS();

    #endif
    #ifdef NETWORK_LOCATION_ENABLED
        GetNetworkLocation();
    #endif
///////////////////////////////////////////////////////////    

    memset(str,0,sizeof(str));
    ConvertToJSON(pPacket,&datalength,str);
    sprintf((void*)cmdstr,"AT+HTTPDATA=%d,50000\r\n",datalength);    
    if(SendATCommand(cmdstr,"DOWNLOAD","ERROR",10)!=1) goto exit;
    
    
    DelayProc(250000);
    //putOn4BData=1;
    ResetBuffer();    
    
    Print(str);
    printf(str);
    //putOn4BData=0;
    LoopTimeout1 = 0;
    while(1)
    {
        //Print("\n\n\n\n\n\n\n\n\n\n");
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       
            goto exit; 
        }
        
    }
    
    ////IWDG_ReloadCounter();
    ResetBuffer();
    Print("AT+HTTPACTION=1\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"ACTION:",7) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>60))
        {       
            goto exit; 
        }
       
    }
    if(Version[0]!=0)
    {
        memset(Version,0,sizeof(Version));
    }
    if(query[0]!=0)
    {
        memset(query,0,sizeof(query));
    }
    osDelay(1000);
    ResetBuffer();
    
    SendATCommand("AT+HTTPREAD=0,50\r\n","+HTTPREAD: ","ERROR",10);
    //Print("AT+HTTPREAD=0,50\r\n");
    //DelayProc(850000);
    osDelay(500);
    LoopTimeout1 = 0;
    while(1)
    {
        #ifdef SHEETS_ENABLED
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"Moved",5) != NULL)
        #else
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"logid",5) != NULL)
        #endif
        {
            ConnectivityTimer = 0;
            osDelay(1000);//DelayProc(50000);
            if(MapForward(Buff2,BUFF2_SIZE,(char*)"SET_RELAY",9) != NULL)
            {
                //ResetBuffer2();    
                //Print2((void*)CSetRelay);
            }
            if(MapForward(Buff2,BUFF2_SIZE,(char*)"RESET_RELAY",11) != NULL)
            {
                //ResetBuffer2();     
                //Print2((void*)CResetRelay);
            }
            // Dont process while sending image
            if(ImageSent == 1)
            {
                pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"IMG_",4);
                if(pToken != NULL)
                {
                    DelayProc(850000);
                    DelayProc(850000);
                    //ReadImagePacket(&pToken[4],&EEImagePacket);//"20160112_2016_09_03_17_01_33_11_15",&EEImagePacket,0);                
                }
            }
            if(MapForward(Buff2,BUFF2_SIZE,(char*)"V_RESET",7) != NULL)
            {
                //ResetBuffer2();     
                //Print2((void*)CViolation);
            }
            if(MapForward(Buff2,BUFF2_SIZE,(char*)"GET_VER",7) != NULL)
            {
                sprintf((void*)Version,"\"ver\":\"%s-%s\",",SYS_VERSION,CUSTOMER);
            }
            
            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"SET_INT_",8);
            if(pToken != NULL)
            {
                //ResetBuffer2();     
                switch(pToken[8])
                {
                    case 'A':
                        //Print2((void*)CSetInt005);
                        Params.Fields.PingInterval=5;
                    break;
                    case 'B':
                        //Print2((void*)CSetInt010);
                        Params.Fields.PingInterval=10;
                    break;
                    case 'C':
                        //Print2((void*)CSetInt015);
                        Params.Fields.PingInterval=15;
                    break;
                    case 'D':
                        //Print2((void*)CSetInt030);
                        Params.Fields.PingInterval=30;
                    break;
                    case 'E':
                        //Print2((void*)CSetInt045);
                        Params.Fields.PingInterval=45;
                    break;
                    case 'F':
                        //Print2((void*)CSetInt060);
                        Params.Fields.PingInterval=60;
                    break;
                    case 'G':
                        //Print2((void*)CSetInt120);
                        Params.Fields.PingInterval=120;
                    break;
                    case 'H':
                        //Print2((void*)CSetInt180);
                        Params.Fields.PingInterval=180;
                    break;
                    case 'I':
                        //Print2((void*)CSetInt240);
                        Params.Fields.PingInterval=240;
                    break;
                }
                /*DelayProc(850000);
                DelayProc(850000);
                DelayProc(850000);
                DelayProc(850000);
                DelayProc(850000);
                DelayProc(850000);*/
									
            }                 
           
        
            break;
        }
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       goto exit; }
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL))
        { 
                break;
        }
//        goto exit; }
  
        
    }
    
      
    ////Print4(Buff);   
   
    //DelayProc(850000);
    //DelayProc(850000);
    //DelayProc(850000);
    //DelayProc(850000);
    goto SUCCESS;
    //free(string);
SUCCESS: 
    ClearEventCache();
    if(RFIDDataPresent==1)
    {
        if(RFID[0]!=0)
        {
            memset(RFID,0,20);
        }
    }
    ResetBuffer();
    Print("AT+HTTPTERM\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }       

    //Print4("SUCCESS\r\n");
    retVal=0;
    return 0;
exit: 
    ResetBuffer();
    Print("AT+HTTPTERM\r\n");
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
        {       break; }
        
    }
    
    //Print4("FAILED\r\n");
    
    /*if(PGEvent.EventType == 35 && retries >=1 && Params.Fields.PingInterval <=10)
    {
        retVal=0;
        #ifdef EEPROM_FIFO
        PostEvent( pPacket);//PostEEEvent(pPacket);
        #endif
        return 0;
    }*/
//    if(++retries < 8) goto RESEND_HTTP;
//    else retVal = 1;
//    
    //#ifdef EEPROM_FIFO
    //PostEEEvent(pPacket);
    RestoreEventCache();
    //PostEvent(pPacket);
    //#endif
//    if(Params.Fields.PingInterval > 30)
//    {
//        ResetBuffer();
//        Print( "AT+CSCLK=0\r\n");    
//        LoopTimeout1 = 0;
//        while(1)
//        {
//            if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
//                    break;
//            if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>30))
//            {       break; }
//        
//        }
//        
//    }    
    return 0;
}
#endif // SIM7600
#ifdef SIM7070
void URLDivider(char *pURL,int URLSize, char *pDomain,int DomainSize, char *pPath, int PathSize)
{

    char seps[3] ="/";
    char *pToken;
    unsigned char i=0;
    unsigned short dlength=0,plength=0;
    char URL[200];
    memcpy(URL,pURL,URLSize); // Because strtok destroys original string
    // dlength = sizeof(pDomain);
    memset(pDomain,0,DomainSize);
    //ESP_LOGI(TAG,"%d,%d\r\n",DomainSize,PathSize);
    // plength = sizeof(pPath);
    memset(pPath,0,PathSize);
    
    pToken = strtok (URL,seps);
    if(pToken != NULL)
    {   
        strcat(pDomain,pToken);
        strcat(pDomain,"//");
        pToken = strtok (NULL,seps);
        //ESP_LOGI(TAG,"%s\r\n",pToken);
        if(pToken != NULL)
            strcat(pDomain,pToken);
    }
  
    while (pToken != NULL)
    {
        
        pToken = strtok (NULL, seps);
        
        if(pToken != NULL) 
        {
            strcat(pPath,"/");
            strcat(pPath,(void*)pToken);
        }            
        else
        {
            break;
        }

        i++;
        if(i>10) break;
    }
}

char YHTTP_Request(char *pFilename, unsigned char pingtype)
{
    //char eventnumber[5];
    //char *pResult;
    //unsigned char retries,i;
    char *pToken;//,*pData;
    //char *pToken1;
    //unsigned char CheckSum,cs[2];
    pPacket = &CPacket;
    //retries = 0;
    //TCPRetries = 0;
//    unsigned char encodedByte;
//    int X; 
    int ResponseLength=0;
    //unsigned short MQTTProtocolNameLength;
    unsigned short MQTTClientIDLength;
    //unsigned short MQTTUsernameLength;
    //unsigned short MQTTPasswordLength;
//    unsigned char k=0;
    char Domain[150],Path[150],Header[200],KeyName[50],KeyValue[100];
    #ifdef DEBUG_PRINT
        
        DebugPrint("Entered-TCP_request\r\n"); 
    #endif
    
    
/*
    #ifndef STANDALONE_DEMO
    CheckSum = GetCheckSum(pPacket->Bytes,34);
    cs[1]=GetAscii(CheckSum&0x0F);  
    cs[0]=GetAscii((CheckSum>>4) & 0x0F);  
    if((pPacket->GEvent.CheckSum[0] != cs[0]) || (pPacket->GEvent.CheckSum[1] != cs[1]))
    {
        retVal=0;
        return 0;
    }
#endif
    */
    
//   RESEND_TCP:
    WakeUp();
    SOS = gpio_get_level(GPIO_SOS);
    if(SOS == 0)
    {
        goto SUCCESS;
    }
    ////Print4("Resending\r\n");
    ////IWDG_ReloadCounter();
    ResetBuffer();
    Print( "AAAAAAAAAAAAAT\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>10))
        {       goto EXIT_MQTT; }
        Count++;
    }
   
    // RECONNECT:

    #ifdef DEBUG_PRINT
        
        DebugPrint("CCHSTART OK-TCP_request"); 
    #endif
    // RESTART_MQTT:
    
  /////////////////////////////////////////////////////////////////////////
    MQTTClientIDLength = strlen((void*)IMEI);//strlen(Params.Fields.MQTTClientID);
    topiclength = sprintf((void*)topic,(void*)Params.Fields.MQTTTopic);
    
    // if(SendATCommand("AT+CNACT=0,1\r\n","0,ACTIVE","ERROR",10) != 1) goto EXIT_MQTT;
    SendATCommand("AT+CNACT=0,1\r\n","0,ACTIVE","ERROR",10);
    
//RECHECK_IP:    
    if(SendATCommand("AT+CNACT?\r\n","+CNACT: 0,0,\"0.0.0.0\"","OK",10) == 1)
    //if(SendATCommand("AT+CNACT?\r\n","+CNACT: 0,1","OK",10) != 1)
    {
        sprintf((void*)str,"NO IP ADDRESS");
        //if(SendATCommand("AT+CFUN=0\r\n","OK","ERROR",10) != 1) goto EXIT_MQTT;
        ////
        sprintf(
            (void*)cmdstr,
            "AT+CGDCONT=1,\"IP\",\"%s\"\r\n",
            Params.Fields.APNName
        );
        if(SendATCommand(cmdstr,"OK","ERROR",10) != 1) goto EXIT_MQTT;
        ////
        //if(SendATCommand("AT+CFUN=1\r\n","+CPIN: READY","ERROR",10) != 1) goto EXIT_MQTT;
        if(SendATCommand("AT+CFUN=1\r\n","OK","ERROR",10) != 1) goto EXIT_MQTT;
        ////
        if(SendATCommand("AT+CGATT=1\r\n","OK","ERROR",10) != 1) goto EXIT_MQTT;
        if(SendATCommand("AT+CGATT?\r\n","+CGATT: 1","ERROR",10) != 1) goto EXIT_MQTT;
        if(SendATCommand("AT+CGNAPN\r\n","+CGNAPN: 1,","ERROR",10) != 1) goto EXIT_MQTT;
        sprintf(
            (void*)cmdstr,
            "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\"\r\n",
            Params.Fields.APNName,
            Params.Fields.APNUsername,
            Params.Fields.APNPassword
        );
        if(SendATCommand(cmdstr,"OK","ERROR",10) != 1) goto EXIT_MQTT;
        if(SendATCommand("AT+CNACT=0,1\r\n","+APP PDP: 0,ACTIVE","ERROR",10) != 1) goto EXIT_MQTT;
        if(SendATCommand("AT+CNACT?\r\n","+CNACT: 0,1,\"0.0.0.0\"","OK",10) == 1) goto EXIT_MQTT;
    }
    sprintf((void*)str,"IP ADDRESS OK");
    ///////////////////////////////////////////////////////////////
//    if(Params.Fields.MQTTPort[0] == '8')
//        sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\",1\r\n",IMEI);
//    else
//        sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\"\r\n",IMEI);
//    
//    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
//    {
//        sprintf((void*)str,"CLIEND ID FAILED");
//        goto EXIT_MQTT;
//    }
//    sprintf((void*)str,"CLIENT ID OK");

    ////////////////////////////////////////////////////////////////

// RECONNECT_MQTT:    
    
    URLDivider(Params.Fields.HTTPURL,sizeof((Params.Fields.HTTPURL)),Domain,sizeof(Domain),Path, sizeof(Path));
    ESP_LOGI(TAG,"%s---\r\n",Path);
    if(MapForward(Params.Fields.HTTPURL,sizeof(Params.Fields.HTTPURL),(char*)"https",5) != NULL)
    {
        SendATCommand("AT+CSSLCFG=\"sslversion\",1,3\r\n","OK","ERROR",10);
        SendATCommand("AT+SHSSL=1\r\n","OK","ERROR",10);

    }
    sprintf((void*)cmdstr,"AT+SHCONF=\"URL\",\"%s\"\r\n",Domain);
    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"URL FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)cmdstr,"AT+SHCONF=\"BODYLEN\",1024\r\n");
    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"BODYLEN FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)cmdstr,"AT+SHCONF=\"HEADERLEN\",350\r\n");
    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"HEADER FAILED");
        goto EXIT_MQTT;
    }

//    sprintf(
//        (void*)cmdstr,
//        "AT+CMQTTCONNECT=0,\"tcp://%s:%s\",60,1,\"%s\",\"%s\"\r\n",
//        Params.Fields.MQTTHost,
//        Params.Fields.MQTTPort,
//        Params.Fields.MQTTUsername,
//        Params.Fields.MQTTPassword
//        );
    if(SendATCommand("AT+SHCONN\r\n","OK","ERROR",20) != 1)
    {
        sprintf((void*)str,"CONNECT FAILED");
        goto EXIT_MQTT;
    }
    if(SendATCommand("AT+SHSTATE?\r\n","+SHSTATE: 1","ERROR",20) != 1)
    {
        sprintf((void*)str,"CONNECT CHECK FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)str,"CONNECT OK");
    
    if(SendATCommand("AT+SHCHEAD\r\n","OK","ERROR",20) != 1)
    {
        sprintf((void*)str,"CLEAR HEADER FAILED");
        goto EXIT_MQTT;
    }
    if(SendATCommand("AT+SHAHEAD=\"Content-Type\",\"application/json\"\r\n","OK","ERROR",20) != 1)
    {
        sprintf((void*)str,"JSON HEAD FAILED");
        goto EXIT_MQTT;
    }
    memcpy(Header,Params.Fields.HTTPKey,sizeof(Header)); // Because strtok destroys original string
    pToken = (void*)strtok ((void*)Header,":");
    if(pToken != NULL)
    {
        sscanf((void*)pToken,"%s",KeyName);
    }
    pToken = (void*)strtok (NULL,":");
    if(pToken != NULL)
    {
        sscanf((void*)pToken,"%s",KeyValue);
    }
    GetEEParams(); // Restore params
    //sprintf((void*)cmdstr,"AT+SHAHEAD=\"X-DreamFactory-Api-Key\",\"%s\"\r\n","699cef40368daa8e98d2684830aef4e2fe2d8b9a6c0c1f9da9125cec266c479e");//Params.Fields.HTTPKey);
    sprintf((void*)cmdstr,"AT+SHAHEAD=\"%s\",\"%s\"\r\n",KeyName,KeyValue);//Params.Fields.HTTPKey);
    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"API KEY FAILED");
        goto EXIT_MQTT;
    }
   
    #ifdef DEBUG_PRINT
        
        DebugPrint("ConnectPKT_OK-TCP_request\r\n"); 
    #endif
   while(1)
    {
        #ifdef DEBUG_PRINT
        
            DebugPrint("PUB while 1 Enter-TCP_request\r\n"); 
        #endif
        CheckBattery();
        
        if(CheckNetwork() == 1)
            UpdateNetwork(0);
        else
            UpdateNetwork(1);
        if(MotionTimer > TIME_TO_SLEEP)
        {
            retVal=0;
            return 0;
        }
//////////////////////////////////////////////////////////
    #ifdef EXT_ANT_ENABLED 
        XCheckGPS();

    #endif
    #ifdef NETWORK_LOCATION_ENABLED
        GetNetworkLocation();
    #endif
///////////////////////////////////////////////////////////      
        
        if(SystemState == State_ConnectedState) return 0; // Return and idle for proper configuration and prevent EEPROM access
        
      
            memset(str,0,sizeof(str));
       
            ConvertToJSON(pPacket,&datalength,str);
            //osDelay(2000);
            sprintf((void*)cmdstr,"AT+SHBOD=%d,10000\r\n",datalength);
            // Print(cmdstr);
            //osDelay(1000);
           if(SendATCommand(cmdstr,">","ERROR",20) != 1)
           {
               // Dont use str here
               ESP_LOGI(TAG,"BODY failed");
               goto EXIT_MQTT;
           }
            //osDelay(4000);
            Print(str); 
            LoopTimeout1 = 0;
            while(1)
            {
                if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                {
                    ESP_LOGI(TAG,"PAYLOAD SUCCESS");
                    //TCPRetries=0;
                    ConnectivityTimer = 0;
                    break;
                }
                if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>20))
                {   
                    ESP_LOGI(TAG,"PAYLOAD ERROR");                    
                    goto EXIT_MQTT; 
                        
                }
                
            }
//            sprintf((void*)cmdstr,"AT+SHBOD?\r\n");
//            if(SendATCommand(cmdstr,"+SHBOD:","ERROR",20) != 1)
//            {
//                sprintf((void*)str,"BODY READ FAILED");
//                goto EXIT_MQTT;
//            }
            
            sprintf((void*)cmdstr,"AT+SHREQ=\"%s\",3\r\n",Path);
            //sprintf((void*)cmdstr,"AT+SHREQ=\"/api/v2/update\",3\r\n");
            SendATCommand(cmdstr,"POST","ERROR",200);
            
            
            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"\"POST\",200",10);
            if(pToken!= NULL) 
            {                
                sscanf(pToken+11,"%3d",&ResponseLength);
                ESP_LOGI(TAG,"PUBLISH SUCCESS-200");
            }
            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"\"POST\",302",10);
            if(pToken!= NULL)             
            {
                sscanf(pToken+11,"%3d",&ResponseLength);
                ESP_LOGI(TAG,"PUBLISH SUCCESS-302");
            }
            else
            {
                ESP_LOGI(TAG,"PUBLISH FAILED");
                goto EXIT_MQTT;
            }
            sprintf((void*)cmdstr,"AT+SHREAD=0,%d\r\n",ResponseLength);
            if(SendATCommand(cmdstr,"+SHREAD:","ERROR",20) != 1)
            {
                ESP_LOGI(TAG,"READ FAILED");
                //goto EXIT_MQTT;
            }
            osDelay(2000);
//            if(SendATCommand("AT+SHREQ=\"%s\",3\r\n","\"POST\",200","ERROR",20) != 1)
//            {
//                sprintf((void*)str,"PUBLISH FAILED");
//                goto EXIT_MQTT;
//            }
            ESP_LOGI(TAG,"PUBLISH SUCCESS");   

            osDelay(3000);
            #ifdef DEBUG_PRINT
        
                DebugPrint("PUB complete -TCP_request\r\n"); 
            #endif
            goto SUCCESS;
        


        if(MapForward(Buff2,BUFF2_SIZE,(char*)"CLOSE",5) != NULL)
        {
            #ifdef DEBUG_PRINT
        
                DebugPrint("CLOSED-TCP_request\r\n"); 
            #endif
            goto EXIT_MQTT;//goto RECONNECT;
        }
        if(Params.Fields.WorkingMode[0] !='H')
            goto SUCCESS;
        
        SOS = gpio_get_level(GPIO_SOS);
        if(SOS == 0)
        {
            goto SUCCESS;
        }
        
        //CheckBattery();
     
        
    }
    
    
    goto SUCCESS;
    //free(string);
SUCCESS: 
    ClearEventCache();
    #ifdef DEBUG_PRINT
        
        DebugPrint("TCP_SUCCESS -TCP_request\r\n"); 
    #endif
    //SendATCommand("AT+CMQTTREL=0\r\n","OK","ERROR",10);
    //osDelay(1000);
    SendATCommand("AT+SHDISC\r\n","OK","ERROR",10);
    // SendATCommand("AT+CNACT=0,0\r\n","DEACTIVE","ERROR",10);
//    #ifndef TIMER_ONLY_WAKEUP
//    goto RECONNECT_MQTT;
//    #endif
//    
    //osDelay(1000);
    //SendATCommand("AT+CMQTTSTOP\r\n","+CMQTTSTOP:","ERROR",10);
    //osDelay(1000);
    
    
    //Print4("SUCCESS\r\n");
   
    retVal=0;
    return 0;
//exit: 
EXIT_MQTT:
    #ifdef DEBUG_PRINT
        
        DebugPrint("TCP_EXIT -TCP_request\r\n"); 
    #endif
    //if(Params.Fields.MQTTPort[0] != '1')
    {
        
        SendATCommand("AT+SHDISC\r\n","OK","ERROR",10);
        // SendATCommand("AT+CNACT=0,0\r\n","DEACTIVE","ERROR",10);
        //osDelay(1000);
        
    }
//    ResetBuffer();
//    Print("AT+NETCLOSE\r\n");
//    LoopTimeout1 = 0;
//    while(1)
//    {
//        if(MapForward(Buff2,BUFF2_SIZE,(char*)"NETCLOSE",8) != NULL)
//                break;
//        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>5))
//        {       break; }
//        Count++;
//    }
//    osDelay(1000);
    //Print4("FAILED\r\n");
    
  
//    if(++TCPRetries < 8) 
//        goto RESEND_TCP;
//    else
//    {        
//        #ifdef DEBUG_PRINT
//        
//            DebugPrint("TCP_Retry_exceeded -TCP_request\r\n"); 
//        #endif
//        retVal = 1;
////        HAL_UART_MspDeInit(&hlpuart1);
////        MX_LPUART1_UART_Init();
//        DisableGSM();
//        InitGSM();
//        TCPRetries = 0;
//    }
      
//    #ifdef EEPROM_FIFO
//    PostEEEvent(pPacket);
//    #endif
    
    //PostEvent(pPacket);
    RestoreEventCache();
    
    
    return 0;
}

#endif // SIM7070
#ifdef SIM800
char ZHTTP_Request(char *pFilename, unsigned char pingtype)
{}
#endif // SIM800
const unsigned char ConnectPacket[46]=
{
    0x10,0x2C,0x00,0x06,0x4D,0x51,0x49,0x73,0x64,0x70,0x03,0xC2,0x00,0x3C,0x00,0x06,0x41,0x42,0x43,0x44,0x45,0x46,0x00,
    0x08,0x74,0x63,0x6F,0x71,0x61,0x7A,0x63,0x7A,0x00,0x0C,0x77,0x51,0x30,0x54,0x57,0x6B,0x4C,0x58,0x49,0x51,0x44,0x7A
};

const unsigned char PublishPacket[21]=
{
    0x30,0x13,0x00,0x08,0x76,0x61,0x6C,0x65,0x74,0x72,0x6F,0x6E,0x68,0x65,0x6C,0x6C,0x6F,0x72,0x61,0x76,0x69
};

const unsigned char SubscribePacket[15]=
{
    0x82,0x0D,0x00,0x01,0x00,0x08,0x76,0x61,0x6C,0x65,0x74,0x72,0x6F,0x6E,0x00
};


const unsigned char PingPacket[2]=
{
    0xC0,0x00
};
void XCheckGPS(void)
{
    char *pToken;
    // char *pData; // TBD

    SendATCommand("AT+CGPSINFO\r\n","OK","ERROR",3);
    osDelay(500);



    Count=0;

    // if(MapForward(Buff2,BUFF2_SIZE,(char*)"+CGPSINFO: ,,",13) != NULL)
    //     GPSStatus = 'V';
    // else
    //     GPSStatus = 'A';
    GPSStatus = 'A';
    if(MapForward(Buff2,BUFF2_SIZE,(char*)"+CGPSINFO: ,,",13) != NULL)
        GPSStatus = 'V';
    else if(MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL)
        GPSStatus = 'V';

    UpdateLocation(GPSStatus);

    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"+CGPSINFO:",10);
    if(pToken != NULL)
    {
        unsigned char in = 0;
        f=0;
        //pData = tBuff; // TBD
        memset(Lat,0,sizeof(Lat));
        memset(Long,0,sizeof(Long));
        memset(Speed,0,sizeof(Speed));
        memset(Altitude,0,sizeof(Altitude));
        
        while(1)
        {
            
            HandleGPSINFData(pToken[in]);
            if(pToken[in] == 0x0D)break;
            in++; if(in >200)break;
            
        }
        if(GPSStatus == 'A')
        {
            // ESP_LOGW(TAG,"Date %s",GPSDate);
            ESP_LOGW(TAG,"Time now is %d-%d-20%d,%02d:%02d:%02d",GPSDay,GPSMonth,GPSYear,GPSHours,GPSMinutes,GPSSeconds);
        }
    }
    Speed[9] = '\0';

}

#ifdef SIM7600 //#ifdef SSL_BROKER
char XMQTT_Request(char *pFilename, unsigned char pingtype)
{
    //char eventnumber[5];
    //char *pResult;
    //unsigned char retries,i;
    // char *pToken;
    // char *pData;
    //char *pToken1;
    //unsigned char CheckSum,cs[2];
    pPacket = &CPacket;
    //retries = 0;
    TCPRetries = 0;
    // unsigned char encodedByte;
    // int X; 
    //unsigned short MQTTProtocolNameLength;
    unsigned short MQTTClientIDLength;
    //unsigned short MQTTUsernameLength;
    //unsigned short MQTTPasswordLength;
    // unsigned char k=0;
    #ifdef DEBUG_PRINT
        
        DebugPrint("Entered-TCP_request\r\n"); 
    #endif
    
/*
    #ifndef STANDALONE_DEMO
    CheckSum = GetCheckSum(pPacket->Bytes,34);
    cs[1]=GetAscii(CheckSum&0x0F);  
    cs[0]=GetAscii((CheckSum>>4) & 0x0F);  
    if((pPacket->GEvent.CheckSum[0] != cs[0]) || (pPacket->GEvent.CheckSum[1] != cs[1]))
    {
        retVal=0;
        return 0;
    }
#endif
    */
    
//   RESEND_TCP: // TBD
    WakeUp();
//    SOS = gpio_get_level(GPIO_SOS);
//    if(SOS == 0)
//    {
//        goto SUCCESS;
//    }
    ////Print4("Resending\r\n");
    ////IWDG_ReloadCounter();
    ResetBuffer();
    Print( "AAAAAAAAAAAAAT\r\n");    
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>10))
        {       goto EXIT_MQTT; }
        Count++;
    }
   
    // RECONNECT: // TBD
////  Adding for A7670
//    sprintf((void*)str,"AT+CGDCONT=1,\"IP\",\"%s\"\r\n",Params.Fields.APNName);
//    SendATCommand(str,"OK","ERROR",5);
//    SendATCommand("AT+CGACT=1,1\r\n","OK","ERROR",10);
//    if(SendATCommand("AT+CGACT?\r\n","+CGACT: 1,1","ERROR",10) != 1)
//    {
//        goto exit;
//    }
////  End - Adding for A7670    
    #ifdef DEBUG_PRINT
        
        DebugPrint("CCHSTART OK-TCP_request"); 
    #endif
    // RESTART_MQTT: //TBD
    
  /////////////////////////////////////////////////////////////////////////
    MQTTClientIDLength = strlen((void*)IMEI);//strlen(Params.Fields.MQTTClientID);
    topiclength = sprintf((void*)topic,(void*)Params.Fields.MQTTTopic);

    
    if(SendATCommand("AT+CMQTTSTART\r\n","OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"MQTT START FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)str,"MQTT START OK");
    ///////////////////////////////////////////////////////////////
    if(Params.Fields.MQTTPort[0] == '8')
    {
        #ifdef CUSTOM_MQTT_CLIENT_ID
            sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\",1\r\n",Params.Fields.MQTTClientID);
        #else
            sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\",1\r\n",IMEI);
        #endif
        
    }
    else
    {
        #ifdef CUSTOM_MQTT_CLIENT_ID
            sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\"\r\n",Params.Fields.MQTTClientID);
        #else
            sprintf((void*)cmdstr,"AT+CMQTTACCQ=0,\"%s\"\r\n",IMEI);
        #endif
        
    }
    
    if(SendATCommand(cmdstr,"OK","ERROR",10) != 1)
    {
        sprintf((void*)str,"CLIEND ID FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)str,"CLIENT ID OK");

    ////////////////////////////////////////////////////////////////
    
// RECONNECT_MQTT:    // TBD 
    sprintf(
        (void*)cmdstr,
        "AT+CMQTTCONNECT=0,\"tcp://%s:%s\",60,1,\"%s\",\"%s\"\r\n",
        Params.Fields.MQTTHost,
        Params.Fields.MQTTPort,
        Params.Fields.MQTTUsername,
        Params.Fields.MQTTPassword
        );
    if(SendATCommand(cmdstr,"+CMQTTCONNECT: 0,0","ERROR",20) != 1)
    {
        sprintf((void*)str,"CONNECT FAILED");
        goto EXIT_MQTT;
    }
    sprintf((void*)str,"CONNECT OK");
    
    
   
    #ifdef DEBUG_PRINT
        
        DebugPrint("ConnectPKT_OK-TCP_request\r\n"); 
    #endif
    /////////////////////////////////////////////////////////////////
    sprintf((void*)cmdstr,"AT+CMQTTTOPIC=0,%d\r\n",topiclength);
    if(SendATCommand(cmdstr,">","ERROR",10) != 1)
    {
        sprintf((void*)str,"TOPIC FAILED");
        goto EXIT_MQTT;
    }
    Print(Params.Fields.MQTTTopic); 
    LoopTimeout1 = 0;
    while(1)
    {
        if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                break;
        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>20))
        {       goto EXIT_MQTT; }
        
    }
    sprintf((void*)str,"TOPIC OK");
    /////////////////////////////////////////////////////////////////
    while(1)
    {
        #ifdef DEBUG_PRINT
        
            DebugPrint("PUB while 1 Enter-TCP_request\r\n"); 
        #endif
        CheckBattery();
        
        if(CheckNetwork() == 1)
            UpdateNetwork(0);
        else
            UpdateNetwork(1);
        if( (MotionTimer > TIME_TO_SLEEP) && (IsQueueEmpty(RAMQueue)==0) )
        {
            retVal=0;
            return 0;
        }
//////////////////////////////////////////////////////////
        #ifdef EXT_ANT_ENABLED 
        XCheckGPS();
        
        #endif
        #ifdef NETWORK_LOCATION_ENABLED
            GetNetworkLocation();
        #endif
///////////////////////////////////////////////////////////    
        
        if(SystemState == State_ConnectedState) return 0; // Return and idle for proper configuration and prevent EEPROM access
        
        
     
       
        if(RTCTimeout>(Params.Fields.PingInterval+20))
         {
             
             InitRTCAlarm();
         }
        
       
        
        if(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS)
        {
            pPacket=&GPacket;
            #ifdef DEBUG_PRINT
        
                DebugPrint("PUB Got Packet-TCP_request\r\n"); 
            #endif    
            
 
            //if(tfLat == 0)tfLat = 8;
            memset(str,0,1500);
            //pPacket->GEvent.Speed[9] = '\0';
            //devid[8] = '\0';
            topiclength = sprintf((void*)topic,(void*)Params.Fields.MQTTTopic);

            
            
//            ChargeVoltage = (int)(ChargeVoltageF*1000);
            
//            datalength = sprintf((void*)str,

            ConvertToJSON(pPacket,&datalength,str);
            
            sprintf((void*)cmdstr,"AT+CMQTTPAYLOAD=0,%d\r\n",datalength);
            if(SendATCommand(cmdstr,">","ERROR",10) != 1)
            {
                // Dont use str here
                goto EXIT_MQTT;
            }
            Print(str); 
            LoopTimeout1 = 0;
            while(1)
            {
                if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                {
                    sprintf((void*)str,"PAYLOAD OK");
                    break;
                }
                if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>20))
                {   
                    sprintf((void*)str,"PAYLOAD ERROR");                    
                    goto EXIT_MQTT; 
                        
                }
                
            }
            if(SendATCommand("AT+CMQTTPUB=0,1,60\r\n","OK","ERROR",10) != 1)
            {
                sprintf((void*)str,"PUBLISH NOT OK");
                goto EXIT_MQTT;
            }
            LoopTimeout1 = 0;
            while(1)
            {
                if(MapForward(Buff2,BUFF2_SIZE,(char*)"+CMQTTPUB: 0,0",14) != NULL)
                {
                    sprintf((void*)str,"PUBLISH SUCCESS");
                    TCPRetries=0;
                    ConnectivityTimer = 0;
                    break;
                }
                if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>20))
                {       
                    sprintf((void*)str,"PUBLISH FAILED");
                    goto EXIT_MQTT; 
                }
                
            } 
            osDelay(3000);
            #ifdef DEBUG_PRINT
        
                DebugPrint("PUB complete -TCP_request\r\n"); 
            #endif
            goto SUCCESS;
        }


        if(MapForward(Buff2,BUFF2_SIZE,(char*)"CLOSE",5) != NULL)
        {
            #ifdef DEBUG_PRINT
        
                DebugPrint("CLOSED-TCP_request\r\n"); 
            #endif
            goto EXIT_MQTT;//goto RECONNECT;
        }
        if(Params.Fields.WorkingMode[0] !='T')
            goto SUCCESS;
        
        SOS = gpio_get_level(GPIO_SOS);
        if(SOS == 0)
        {
            goto SUCCESS;
        }
        
        //CheckBattery();
     
        
    }
    
    
    goto SUCCESS;
    //free(string);
SUCCESS: 
    ClearEventCache();
    #ifdef DEBUG_PRINT
        
        DebugPrint("TCP_SUCCESS -TCP_request\r\n"); 
    #endif
    //SendATCommand("AT+CMQTTREL=0\r\n","OK","ERROR",10);
    //osDelay(1000);
    SendATCommand("AT+CMQTTDISC=0,120\r\n","+CMQTTDISC:","ERROR",10);
    SendATCommand("AT+CMQTTREL=0\r\n","OK","ERROR",10);
//    #ifndef TIMER_ONLY_WAKEUP
//    goto RECONNECT_MQTT;
//    #endif
//    
    //osDelay(1000);
    SendATCommand("AT+CMQTTSTOP\r\n","+CMQTTSTOP:","ERROR",10);
    //osDelay(1000);
    
    
    //Print4("SUCCESS\r\n");
    retVal=0;
    return 0;
// exit:  // TBD
EXIT_MQTT:
    #ifdef DEBUG_PRINT
        
        DebugPrint("TCP_EXIT -TCP_request\r\n"); 
    #endif
    //if(Params.Fields.MQTTPort[0] != '1')
    {
        
        //osDelay(1000);
        SendATCommand("AT+CMQTTDISC=0,120\r\n","+CMQTTDISC:","ERROR",10);
        SendATCommand("AT+CMQTTREL=0\r\n","OK","ERROR",10);
        //osDelay(1000);
        SendATCommand("AT+CMQTTSTOP\r\n","+CMQTTSTOP:","ERROR",10);
        //osDelay(1000);
        
    }
//    ResetBuffer();
//    Print("AT+NETCLOSE\r\n");
//    LoopTimeout1 = 0;
//    while(1)
//    {
//        if(MapForward(Buff2,BUFF2_SIZE,(char*)"NETCLOSE",8) != NULL)
//                break;
//        if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (LoopTimeout1>5))
//        {       break; }
//        Count++;
//    }
//    osDelay(1000);
    //Print4("FAILED\r\n");
    
  
//    if(++TCPRetries < 8) 
//        goto RESEND_TCP;
//    else
//    {        
//        #ifdef DEBUG_PRINT
//        
//            DebugPrint("TCP_Retry_exceeded -TCP_request\r\n"); 
//        #endif
//        retVal = 1;
////        HAL_UART_MspDeInit(&hlpuart1);
////        MX_LPUART1_UART_Init();
//        DisableGSM();
//        InitGSM();
//        TCPRetries = 0;
//    }
      
//    #ifdef EEPROM_FIFO
//    PostEEEvent(pPacket);
//    #endif
    
    RestoreEventCache();//PostEvent(pPacket);
    
    return 0;
}
#endif // SIM7600 XMQTT

#ifdef SIM7070 



char YMQTT_Request(char *pFilename, unsigned char pingtype)
{}
#endif // SIM7070



#ifdef SIM800 
char TCP_request(char *pFilename, unsigned char pingtype)
{}


#endif


void SyncRTC (void)
{
    //TBD
//     int cYear,cMonth,cDate,cHour,cMinute,cSecond;
//     char *pToken;
//     unsigned char DecrementCount;
//     ResetBuffer();
// //    Print("AT+CCLK?\r\n");
// //    osDelay(1000);
// //    osDelay(2000);
//     SendATCommand("AT+CCLK?\r\n","OK","ERROR",3);
//     pToken = MapForward(Buff2,BUFF2_SIZE,(unsigned char*)",",1);
//     if(pToken != NULL)
//     {
// 					//		sscanf(Buff2,"20%02d-%02d-%02d  %02d:%02d:%02d",
// 					//																	&Year,&Month,&Date,&Hour,&Minute,&Sec);
// 		DecrementCount = 0;	
//         while(*pToken!='/')
//         {
//             pToken--;
//             if(++DecrementCount>100)return;
//         }
//         pToken-=5;
//         sscanf((void*)pToken,"%02d/%02d/%2d,%02d:%02d:%02d:",&cYear,&cMonth,&cDate,&cHour,&cMinute,&cSecond);
//         sDate.Year=cYear;
//         sDate.Month=cMonth;
//         sDate.Date=cDate;
//         R.Hours=cHour;
//         R.Minutes=cMinute;
//         R.Seconds=cSecond;

//         __HAL_RCC_RTC_ENABLE();
//         /**Initialize RTC Only */
//         hrtc.Instance = RTC;
//         hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//         hrtc.Init.AsynchPrediv = 127;
//         hrtc.Init.SynchPrediv = 255;
//         hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//         hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//         hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//         if (HAL_RTC_Init(&hrtc) != HAL_OK)
//         {
//             Error_Handler();
//         }

            

//         /**Initialize RTC and set the Time and Date 
//         */ 
//         //if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
//         {
//             if (HAL_RTC_SetTime(&hrtc, &R, RTC_FORMAT_BIN) != HAL_OK)
//             {
//                 Error_Handler();
//             }
           
//             if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
//             {
//                 Error_Handler();
//             }

//             HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
//         }


//         HAL_RTCEx_EnableBypassShadow(&hrtc);
                    
//         }

}

void DeepSleep (void)
{
    if((Params.Fields.WorkingMode[0]=='T'  || Params.Fields.WorkingMode[0]=='H') || PowerButtonSleep == 1)
    {
      if(((MotionTimer > TIME_TO_SLEEP) && (IsQueueEmpty(RAMQueue)==0)) || PowerButtonSleep == 1)
        {
//            if(GSMEnabled == 1)
//            {    
//              //  HandleChargingState(); s
//            }
            //Print1("Tada\r\n");
            DisableGSM();
            
            FrontPanelTimer = 500; // Turn off LED and wait
            // osDelay(3000);
  

//        UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_ENABLE);

        //Print1("Hello\r\n");
        //if(SleepModeEnabled == 1) 
        {
            MakeAllLED(TURN_OFF); // Because Disable GSM turns off power to LED

            DisableGSM();           
            
            ESP_LOGI(TAG,"Init Accelerometer : Before Entering Sleep");
            InitAccelerometer();


            RTCSleepModeEnabled = 1;
            
            
            //InitRTCAlarm();
            SleepModeEnabled = 1;
            DisableMainPower();

            esp_wifi_stop();
            esp_wifi_deinit();

            esp_bt_controller_disable();
            esp_bt_controller_deinit();

            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            nimble_port_freertos_deinit();  
            nvs_flash_deinit();
            //nimble_port_stop(); // BLE Causes crash
            
            /////////////////////////////////////
            /* Initialize selected GPIO as RTC IO, enable output, disable pullup and pulldown, enable hold*/
            gpio_hold_en(GPIO_TPS_ENABLE);
            gpio_hold_en(GPIO_GSM_ENABLE);
            gpio_deep_sleep_hold_en();
            /////////////////////////////////
           
            EnterDeepSleep();

            
            // osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever); // TBD
        }

        }
        else
        {
              if(SleepModeEnabled == 1)
              {
                //   BackupPackets(); // TBD
                //   WriteSRAM(DEEP_SLEEP_RESET); // TBD
                ESP_LOGW(TAG,"Rebooting DeepSleep");
                esp_restart();
              }
                  //            EnableGPS();
//           // EnableCharger();
//            if(GSMEnabled == 0)
//            {
//                while(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS);
//                
//               // while(GetEEEvent(&GPacket) == GET_SUCCESS);
//                
//                InitGSM();
//                MotionTimer = 0;
//                PostMotionEvent();
//            }
//            EnableGSM();
            
        }
    
    }
}
unsigned char AlarmString[100];
void InitRTCAlarm(void)
{
    // TBD
//     //unsigned char RTCStr[100];
//     //#ifdef TIMER_ONLY_WAKEUP
//     unsigned short hour=0,minute=0,second=0;
//     //#endif
//     #ifdef DEBUG_PRINT
        
//         DebugPrint("Entered-InitRTCAlarm\r\n"); 
//     #endif
    
//     __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

//     HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN);
//     HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
//     //##-3- Configure the RTC Alarm peripheral #################################
//     // Set Alarm to 02:20:30 
//     //RTC Alarm Generation: Alarm on Hours, Minutes and Seconds 

//     salarmstructure.Alarm = RTC_ALARM_A;
//     salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
//     salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//     salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY ;//| RTC_ALARMMASK_HOURS;
//     //salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
//     salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;

// #ifndef TIMER_ONLY_WAKEUP
//     salarmstructure.AlarmTime.Hours = R.Hours;
//     salarmstructure.AlarmTime.Minutes =  R.Minutes+1;
//     if(salarmstructure.AlarmTime.Minutes >= 60)
//         salarmstructure.AlarmTime.Minutes = 0;
    
//     salarmstructure.AlarmTime.Seconds = 0;
//     //salarmstructure.AlarmTime.SubSeconds = 0x56;
//     RTCTimeout=0;
//     #ifdef DEBUG_PRINT
//         sprintf((void*)RTCStr,"\r\nCTime-%0.2d:%0.2d:%0.2d",R.Hours,R.Minutes,R.Seconds);
//         DebugPrint(RTCStr);
//         sprintf((void*)RTCStr,"\r\nATime%0.2d:%0.2d:%0.2d",salarmstructure.AlarmTime.Hours,salarmstructure.AlarmTime.Minutes,salarmstructure.AlarmTime.Seconds);
//         DebugPrint(RTCStr);
//         DebugPrint("-InitRTCAlarm\r\n"); 
//     #endif
    
    
// #else
//         //Params.Fields.PingInterval = 3;
    
//     hour=R.Hours;
//     minute=R.Minutes;
//     second=R.Seconds;
    
//     hour+=(Params.Fields.PingInterval/60);
//     minute+=(Params.Fields.PingInterval%60);
    
    
//     if(second>=60)
//     {
//         second-=60;
//         minute+=1;
        
        
//     }
    
//     if(minute>=60)
//     {
//         minute-=60;
//         hour+=1;
        
        
//     }
//     if(hour>=24)
//     {
//             hour-=24;
//     }
    
//     salarmstructure.AlarmTime.Hours = hour;//R.Hours;
//     salarmstructure.AlarmTime.Minutes = minute;
//     salarmstructure.AlarmTime.Seconds = second;
    
//     sprintf((void*)AlarmString,"ST: %0.2d:%0.2d:%0.2d|AT: %0.2d:%0.2d:%0.2d",
//                     R.Hours,R.Minutes,R.Seconds,
//                     salarmstructure.AlarmTime.Hours,
//                     salarmstructure.AlarmTime.Minutes,
//                     salarmstructure.AlarmTime.Seconds);
    
//     #ifdef DEBUG_PRINT
//         sprintf((void*)RTCStr,"\r\nCTime-%0.2d:%0.2d:%0.2d",R.Hours,R.Minutes,R.Seconds);
//         DebugPrint(RTCStr);
//         sprintf((void*)RTCStr,"\r\nATime%0.2d:%0.2d:%0.2d",salarmstructure.AlarmTime.Hours,salarmstructure.AlarmTime.Minutes,salarmstructure.AlarmTime.Seconds);
//         DebugPrint(RTCStr);
//         DebugPrint("-InitRTC_IRQHandler\r\n"); 
//     #endif
    
// #endif
//     if(RTCSleepModeEnabled == 1)
//     {      
//         hour=R.Hours;
//         minute=R.Minutes;
//         second=R.Seconds;
        
//         hour+=(1440/60); // 24 hours interval
//         minute+=(1440%60); // 24 hours interval
//         minute-=1;
        
//         if(second>=60)
//         {
//             second-=60;
//             minute+=1;
            
            
//         }
        
//         if(minute>=60)
//         {
//             minute-=60;
//             hour+=1;
            
            
//         }
//         if(hour>=24)
//         {
//                 hour-=24;
//         }
        
//         salarmstructure.AlarmTime.Hours = hour;//R.Hours;
//         salarmstructure.AlarmTime.Minutes = minute;
//         salarmstructure.AlarmTime.Seconds = second;
        
//         sprintf((void*)AlarmString,"ST: %0.2d:%0.2d:%0.2d|AT: %0.2d:%0.2d:%0.2d",
//                         R.Hours,R.Minutes,R.Seconds,
//                         salarmstructure.AlarmTime.Hours,
//                         salarmstructure.AlarmTime.Minutes,
//                         salarmstructure.AlarmTime.Seconds);
//         Print1("HelloRTC\r\n");
//         Print1(AlarmString);
        
//     }
//     if(HAL_RTC_SetAlarm_IT(&hrtc,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK)
//     {
//         /* Initialization Error */
//         Error_Handler(); 
//     }

//     //HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 22, 0);
//     HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);
//     HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

unsigned char FirstRun = 1;
void RTC_Alarm_IRQHandler(void)
{
    // TBD

    
    
    
}
unsigned char VALTRACK_BLE_Status = 0,Prev_VALTRACK_BLE_Status = 0;
// void VALTRACK_BLE_Advertise(unsigned char Status);
unsigned char ForceEraseEEPROM  = 0;

unsigned char ServerRetries = 0;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartMainTask(void *argument)
{
   
   
    char*pToken;
    char*pFilename;   
    unsigned char i,EEPROMReadCount;
    
  /* USER CODE BEGIN 1 */
    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
ESP_LOGI(TAG,"Entered main task");
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */
//  Reset_Device();
//  Config_HSE();
//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();
    // if(BootReason <= 8)
    // {
    //     // Print1("\r\n\r\n");
    //     ESP_LOGI(TAG,"Boot Reason = %s",(char*)(BootReasons[BootReason].Bytes)); // TBD
    //     // Print1("\r\n\r\n");
    // }
//  /* USER CODE BEGIN SysInit */
//  PeriphClock_Config();
//  Init_Exti(); /**< Configure the system Power Mode */
  /* USER CODE END SysInit */

    #ifdef DEBUG_PRINT
        DebugPrint("Entered -StartMainTask\r\n"); 
    #endif
  /* USER CODE BEGIN 2 */
//HAL_ADC_Start_DMA(&hadc, (void*)&BatteryADCCount , 1);
//    InitAccelerometer();
//    MotionTimer=800;
//    DeepSleep();
    #ifndef WB_PIN_CONTROLLED_LED
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); // LED Controller Enable
    #endif
    //DisableGSM();
    EnableGSM(); // For LED
    // UpdateLED1(RED_COLOR);
    // UpdateLED3(RED_COLOR);
    
    // UpdateNetwork(0);
    
    // TBD
    // ADCBatteryVoltage = (((float)BatteryADCCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
    // ADCBatteryVoltage+=ADC_OFFSET;//ADCBatteryVoltage -=0.4;
    //ADCBatteryVoltage = (((float)BatteryADCCount*2*3)/4096);

   
    
    UpdateBattery(ADCBatteryVoltage);
    
    


    //GSM_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
    // ESP_LOGI(TAG,"Reached Enable GSM");
    #ifndef VALTRACK_V4_VTS
    ChargingStatus = (ChargingStatusType)gpio_get_level(GPIO_CHARGER_PIN);
    
    if( (ChargingStatus == CONNECTED) || (ADCBatteryVoltage < 3.0))
    {
        goto CHECK_CHARGER_STATE;
    }
    #endif

    EnableGSM();
    osDelay(1000);// For SIM7672
    gpio_set_level(GPIO_PWRKEY,1);
    osDelay(1000);
    gpio_set_level(GPIO_PWRKEY,0);
    //osDelay(5000);
    //while(1){SendATCommand("AT\r\n","OK","ERROR",5);osDelay(1000);};
    // while(1)
    // {
    //     osDelay(1);
    // }
    // ESP_LOGI(TAG,"After Enable GSM");
    //while(1){LPUARTTimer=0;}
    // STANDBY DELAY
    //osDelay(5000);
    
    // while(1)
    // {
    //     Print("TestText\r\n");
    //     osDelay(500);

    // }
    // TBD
    // ADCBatteryVoltage = (((float)BatteryADCCount*(float)ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);//ADCBatteryVoltage = (((float)BatteryADCCount*2*3)/4096);
    // ADCBatteryVoltage -=0.4;
    UpdateBattery(ADCBatteryVoltage);

    //if(ADCBatteryVoltage < 3.65)
    //    DeviceStatus = 0;
 
    FrontPanelTimer = 0;
    // Clear reset flags in any cases 
    //__HAL_RCC_CLEAR_RESET_FLAGS();
 
    osDelay(300);
    

    ReadEEIndexes();
    GetEEParams();
    // Init Default values
    //if( (Params.Fields.Band[0] == 0xFF) || (LoadDefaultParams == 1) )
    //LoadDefaultParams = 1;
    if( (GetParamString("APNName",cmdstr) != ESP_OK) || (LoadDefaultParams == 1) )
    {
        //memcpy(Params.Bytes,DefaultParams.Bytes,sizeof(Params));
        //StoreEEParams();
        StoreParams((void*)&DefaultParams);
        LoadDefaultParams = 0;
    }
    if(ForceEraseEEPROM == 1)
    {
        EraseEEPROMPackets();
        ForceEraseEEPROM = 0;
    }
    #ifdef DEBUG_PRINT
        DebugPrint("ParamsRead -StartMainTask\r\n"); 
    #endif  
    
//    PostReboot();
    

    
    //EnableGSM();
    
//    while(1)
//    {
//        if(VALTRACK_BLE_Status != Prev_VALTRACK_BLE_Status)
//        {
//            VALTRACK_BLE_Advertise(VALTRACK_BLE_Status);
//            Prev_VALTRACK_BLE_Status = VALTRACK_BLE_Status;
//        }
//    }
    InitAccelerometer();
   
//goto DIE;
    osDelay(5000);
   
    if(InitGSM() == 3)
    {
        MotionTimer = TIME_TO_SLEEP+1; //  Make sure no events are in queue to enter sleep
        ClearPackets();
        printf("entering stop due to init gsm\n");
        goto ENTER_STOP_MODE;
    }
    //InitGPS();
    CheckSignalStrength();
    
    
    //
    //GetNetworkData();
    //
    SyncRTC();
    InitRTCAlarm();
    ADCRunning = 0;
    Count=0;
    SMSNumber = 0;
    //minVal = 0; maxVal = 600;
    rxNumber[10] = '\0';
    AlertTimer = 60000;
    SOSTimer = 0;
    
    //EnterStandyMode();
    LEDInhibit = 1;
    /*for(i=0;i<5;i++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); //LED
        osDelay(250);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); //LED
        osDelay(250);
    }  */  
    LEDInhibit = 0;
    
    MotionTimer = 0;
	
//		i=0;
//		while(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS)
//		{
//			i++;
//			if(i>200)
//				break;
//		}
     
    
    PostReboot();
    
		//
        
    //RestorePackets();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
        ChargingStatus = DISCONNECTED;
        // DIE: // TBD
        //MotionTimer=500;
//        SleepModeEnabled = 1;
	while(1)
	{
//        if(SleepModeEnabled == 1) 
//        {
//            DisableGSM();
//            
//            MakeAllLED(TURN_OFF);
//            //VALTRACK_BLE_Advertise(0);
//            
//            __HAL_RCC_GPIOC_CLK_DISABLE();
//            __HAL_RCC_GPIOH_CLK_DISABLE();
//            __HAL_RCC_GPIOB_CLK_DISABLE();
//            __HAL_RCC_GPIOA_CLK_DISABLE();
//            __HAL_RCC_GPIOE_CLK_DISABLE();
//            __HAL_RCC_ADC_CLK_DISABLE();
//            __HAL_RCC_I2C1_CLK_DISABLE();
//            __HAL_RCC_LPUART1_CLK_DISABLE();
//            __HAL_RCC_USART1_CLK_DISABLE();
//            __HAL_RCC_DMAMUX1_CLK_DISABLE();
//            __HAL_RCC_DMA1_CLK_DISABLE();
//            __HAL_RCC_DMA2_CLK_DISABLE();
//            HAL_UART_DeInit(&hlpuart1);
//            HAL_UART_DeInit(&huart1);
//            HAL_DMA_DeInit(&hdma_adc1);
//            HAL_ADC_DeInit(&hadc1);
//            HAL_I2C_DeInit(&hi2c1);
//            
//            
//            osThreadFlagsWait( 1, osFlagsWaitAny, osWaitForever);;
//        }
            #ifdef DEBUG_PRINT
            DebugPrint("While Loop -StartMainTask\r\n"); 
        #endif
        // HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN); //TBD
        // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //TBD
        
        
        osDelay(1000);
        if(SystemState == State_ConnectedState)
        {
            // VALTRACK_BLE_Advertise(1); // TBD
            #ifndef WB_PIN_CONTROLLED_LED
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
            #endif
            MakeAllLED(BLUE);
            DisableGSM();
            
            while(SystemState == State_ConnectedState)
            {
                FrontPanelTimer=2; // To prevent 1, it will write advertisement repeatedly if it increments to 1. 
                LPUARTTimer=0;
                //Make LED blue here 
                // VTS uses IO so writing here, change to I2C for V4-MF
                // TO BE IMPLEMENTED FOR V4MF
                // TO BE IMPLEMENTED FOR V4MF
                // TO BE IMPLEMENTED FOR V4MF
                // TO BE IMPLEMENTED FOR V4MF
                // TO BE IMPLEMENTED FOR V4MF
                #ifdef VALTRACK_V4_VTS
                    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET); // TBD
                    // osDelay(500);
                    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
                    // osDelay(500);
                #else
                      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //TBD
//                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
//                    osDelay(500);
//                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
//                    osDelay(500);
//                    
                #endif
                // TO BE IMPLEMENTED FOR V4MF
                // TO BE IMPLEMENTED FOR V4MF
                
            }
        }
//        if(StartSending == 1)
//        P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)t);
//		
        
//        if(DeviceStatus == 0)
//        {
//        BACK_TO_SLEEP:
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);
//            DisableGSM();
//            
//            //UpdateNetwork(3);
//            //UpdateLocation(0);
//            
//            // DIIIISSSAAAABLEEE RRRRTTTTCCC HEEERRREEE
//            // DIIIISSSAAAABLEEE RRRRTTTTCCC HEEERRREEE
//            // DIIIISSSAAAABLEEE RRRRTTTTCCC HEEERRREEE
//            // DIIIISSSAAAABLEEE RRRRTTTTCCC HEEERRREEE
//            __disable_irq();
//            LPM_EnterStopMode();
//            //LPM_ExitStopMode();
//            //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//            
//            __enable_irq();
//            // Configures system clock after wake-up from STOP: enable HSE, PLL and select
//            //PLL as system clock source (HSE and PLL are disabled in STOP mode) 
//            //SystemClockConfig_STOP();
//            osDelay(5000);
//            SOS = gpio_get_level(GPIO_SOS);
//            if(SOS == 0)
//            {
//                DeviceStatus = 1;
//                esp_restart();
//            }
//            else 
//                goto BACK_TO_SLEEP;
//            
//            
//        }
//        else //if(FrontPanelTimer < 30)
//        {
//            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); 
//            HAL_ADC_Start_DMA(&hadc1, (void*)&BatteryADCCount , 1);
//            if(CheckNetwork() == 0)            
//                UpdateNetwork(1);            
//            else 
//                UpdateNetwork(0);
//            
//            ADCBatteryVoltage = (((float)BatteryADCCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
//            ADCBatteryVoltage -=0.4;//ADCBatteryVoltage = (((float)BatteryADCCount*2*3)/4096);
//
//            UpdateBattery(ADCBatteryVoltage);            
//            if(ADCBatteryVoltage < 3.65)
//                DeviceStatus = 0;
//        }
        //else
            ////HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); 

        //UpdateBattery(BatteryValue);
        
#ifndef VALTRACK_V4_VTS
            CHECK_CHARGER_STATE:
            if( (ChargingStatus == CONNECTED) || (ADCBatteryVoltage < 3.0))
            {
                //DeviceStatus = 0;
                
                ESP_LOGI(TAG,"Entered Charging Loop");
                DisableGSM();
                osDelay(1000);
                EnableGSM(); // For LED
                UpdateNetwork(3);
                UpdateLocation(0);

                //DisableGSM();
                // printf("before delay\n");
                // osDelay(20000);
                // printf("after delay\n");
                
                /*
                __disable_irq();
                LPM_EnterStopMode();
                //LPM_ExitStopMode();
                //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
                
                __enable_irq();
                // Configures system clock after wake-up from STOP: enable HSE, PLL and select
                //PLL as system clock source (HSE and PLL are disabled in STOP mode) 
                SystemClockConfig_STOP();*/
                while(1)
                {
                    if(ChargingStatus == CONNECTED)
                    {
                        LPUARTTimer = 0;// Feed UART Timer
                        
                        osDelay(5000);
                        // Was disconnected if was here
                        
                    }
                    if(ChargingStatus == DISCONNECTED && ADCBatteryVoltage > 3.5)
                    {
                        // WriteSRAM(CHARGER_RESET); // TBD
                        ESP_LOGI(TAG,"------Rebooting from StartMainTask ChargingStatus=1");
                        MakeAllLED(PURPLE);
                        osDelay(1000);
                        esp_restart();// NVIC_SystemReset();//break;// TBD
                        //osDelay(5000);
                    }
                    //HAL_ADC_Start_DMA(&hadc1, (void*)&BatteryADCCount , 1); //TBD
                    osDelay(500);
                    
                    // TBD
                    // ADCBatteryVoltage = (((float)BatteryADCCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
                    // ADCBatteryVoltage+=ADC_OFFSET;//ADCBatteryVoltage -=0.4;    //ADCBatteryVoltage = (((float)BatteryADCCount*2*3)/4096);
                    UpdateBattery(ADCBatteryVoltage);  
                }
            }
        #endif               
      // CheckBLE();
		#ifdef SOSALERT
            SOS = gpio_get_level(GPIO_SOS);
            if(SOS == 0)
            {
                LEDInhibit=1;
                osDelay(3000);
                SOS = gpio_get_level(GPIO_SOS);
                if(SOS == 0)
                {
                    for(i=0;i<5;i++)
                    {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); //LED
                        osDelay(200);
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); //LED
                        osDelay(200);
                        
                    }
                    LEDInhibit=0;
                 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); //LED
                    SosAlert=1;
                    
                    osDelay(1000);
                    ResetBuffer();
                    Print("AT+CSCLK=0\r\n");
                    
                    osDelay(2000);//DDelay();
        //                DDelay();
        //                DDelay();
                    ResetBuffer1();
                    Print1("$PMTK161,1*29\r\n");
                    osDelay(1000);
                    SendLastLocation();
                    SosAlert=0;
                    osDelay(3000);
                    ResetBuffer();
                    Print("AT+CSCLK=2\r\n");
                    osDelay(2000);
                    
                    
                    DDelay();
                }
            }

        #endif
        
                
         
//        if(MotionTimer > 300)
//        {
//            //DisableGSM();
//            //   

//            osDelay(1000);
//            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //EN
//            //EnterStandbyMode();
//            
//            //SystemClock_ConfigMSI();
//            // Enter Sleep Mode , wake up is done once Key push button is pressed 
//            //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//            DisableGSM();
//            
//    
//            //HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
//            
//            
//            LL_PWR_SetPowerMode(LL_PWR_MODE_STOP2);

//            LL_LPM_EnableDeepSleep();

//            __WFI();

//            //sysClkCfg();
//            
//            
//            /* ... STOP2 mode ... */

//            /* Configure system clock after wake-up from STOP: enable HSE, PLL and select
//            PLL as system clock source (HSE and PLL are disabled in STOP mode) */
//            SystemClock_Config();
//            
//            
//            
//            //EnterSleepMode();
//            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //EN
////            
////            HAL_SuspendTick();
////            // Enter Stop Mode 
////            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

////            // Resume Tick interrupt if disabled prior to sleep mode entry
////            HAL_ResumeTick();
////            // Configures system clock after wake-up from STOP: enable HSE, PLL and select
////            //PLL as system clock source (HSE and PLL are disabled in STOP mode) 
////            SystemClockConfig_STOP();
//            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); //EN
//            //CheckBattery();
//            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); //EN
//            //osDelay(2000);
//            //CheckBattery();
//            //EnableGSM();
//            // 
//            
//        }
        
        /*if(GPSStatus =='A')
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); //GSM ENABLE
            osDelay(200);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); //GSM ENABLE
            osDelay(200);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET); //GSM ENABLE
            osDelay(200);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); //GSM ENABLE
            osDelay(200);
        }*/
        //if(BatteryTimeout == 1)
        //{
        //    CheckBattery();
        //    BatteryTimeout = 0;
        //}
        /* Get the RTC current Time */
//        if(RTCTimeout>(Params.Fields.PingInterval+20))
//         {
//             //SystemInternalClock_Config_LP();
//             InitRTCAlarm();
//             #ifdef DEBUG_PRINT
//                DebugPrint("RTCTimeout+20 -StartMainTask\r\n"); 
//            #endif
//             
//         }
        
        // HAL_RTC_GetTime(&hrtc, &R, RTC_FORMAT_BIN); //TBD
        // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //TBD
        osDelay(1000);
        //GSM_STATUS = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
        INT1 = (MotionStatusType)gpio_get_level(GPIO_INT1);
        
        
        /////////////////////////////////
        
        /////////////////////////////////
        if(INT1 == 0)
        {
            #ifdef DEBUG_PRINT
                DebugPrint("INT1=0 -StartMainTask\r\n"); 
            #endif
            //for(i=0;i<=0x31;i++)
            //{
            //    VALREAD = I2C_RdReg(i);
            //    INT1 = (MotionStatusType)gpio_get_level(GPIO_INT1);
            //}
           
            ISRstatus = I2C_RdReg(REG_INT1_SRC);
            InitAccelerometer();
           
//            InitAccelerometer();
            
            if(MotionTimer > TIME_TO_SLEEP)
            {
                //PostMotionEvent();
                #ifndef TIMER_ONLY_WAKEUP
                    MotionTimer=0;
                #endif
                if(Params.Fields.MotionAlertMode[0]=='S')
                {
                     if(GPSEnabled == 0)
                    {    
                        
                        DDelay();
                        ResetBuffer();
                        
                        Print("AT+CSCLK=0\r\n");
                            
                        
                    }
                   
                    WakeUp();
                    DDelay();
                    ResetBuffer();
                    SendAlert();
                    DDelay();
                    osDelay(3000);
                    ResetBuffer();
                    Print("AT+CSCLK=2\r\n");
                    osDelay(2000);
                    
                    
                    DDelay();
                        
                }
                if(Params.Fields.MotionAlertMode[0]=='C')
                {
                    WakeUp();
                    DDelay();
                    ResetBuffer();
                    //Print("AT+CSCLK=2\r\n");
                    //DDelay();
                    Print("ATD");
                    Print(Params.Fields.rxNumber);
                    Print(";\r\n");
                    for(i=0;i<30;i++)
                    {
                        osDelay(1000);
                        SOS = gpio_get_level(GPIO_SOS);
                        if(SOS == 0) break;
                        
                    }
                    WakeUp();
                    Print("ATH\r\n");
                }
                  
            }
            #ifndef TIMER_ONLY_WAKEUP
                MotionTimer=0;
            #endif
        }

//          UTIL_LPM_SetStopMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_ENABLE);
        ENTER_STOP_MODE:
        #ifdef SLEEP_ENABLED
            DeepSleep();
        #endif
        if(SystemTimer%300==0)
        {
            //CheckSignalStrength();
            CheckNetworkLocation();
        }
        if(Params.Fields.WorkingMode[0]=='S')
        {            
            
            
            if(GPSStatus == 'A' && SMSSent == 0)
            {
							   
                DDelay();
                DDelay();
              
                DDelay();
                ResetBuffer1();
                Print1("$PMTK161,1*29\r\n");
                DDelay();
                
                //ResetBuffer1();
                //Print1("$PMTK225,0*2B\r\n");
                
                DDelay();
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
                GPSStatus = 0;
                D1 = 0;
                D2 = 0;
                SendLocation();
                DDelay();
                ResetBuffer();
				osDelay(3000);
                SMSSent = 1;
                DDelay();
                ResetBuffer();
                Print("AT+CSCLK=2\r\n");
                osDelay(2000);
                
                
                DDelay();
                NoSignalTimer=0;
                
            }
            if(GPSStatus != 'A' && SMSSent == 0 && NoSignalTimer > 120)
            {
							
								
                DDelay();
                DDelay();
                //USART_ITConfig(USART1,USART_IT_RXNE, DISABLE);
                //Print1("$PMTK161,1*29\r\n");
                
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
                ResetBuffer1();
                Print1("$PMTK161,1*29\r\n");
                DDelay();
                
                //ResetBuffer1();
                //Print1("$PMTK225,0*2B\r\n");
                
                //DDelay();
                //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
                GPSStatus = 0;
                D1 = 0;
                D2 = 0;
                SendLastLocation();
                DDelay();
                osDelay(1000);
                ResetBuffer();
                osDelay(3000);
                SMSSent = 1;
                NoSignalTimer = 0;
                DDelay();
                ResetBuffer();
                Print("AT+CSCLK=2\r\n");
                osDelay(2000);
                
                
                DDelay();
                
                
            }
                
            //if( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) != 0 ) 
            //    GPSAwake = 1;
            //else
            //    GPSAwake = 0;
            //if( Buff[i] == '+')i=0;
            
            //if( (Buff[i-21] == 'C') &&(Buff[i-20] == 'L') && (Buff[i-19] == 'I') 
            //        && (Buff[i-18] == 'P'))
            /*if(MapForward(Buff2,BUFF2_SIZE,(char*)"CLIP",4)!= NULL)
            {
                if(++RingCount>=2)
                {
                    //DDelay();
                    RingCount=0;
                    WakeUp();
                    Print("ATA\r\n");
                    //DDelay();
                    if(FirstTime == 0)
                    {
                        pToken = MapForward(Buff2,BUFF2_SIZE,(char*)": \"",3);
                        if(pToken != NULL)
                        {
                            j=3;
                            while(pToken[j] != '"')
                            {
                                rxNumber[j-3] = pToken[j];
                                j++;
                            }
                        }
                        rxNumber[j-3] = '\0';
                        FirstTime = 1;
                        //SendSMS();
                        FirstTime = 2;
                    }

                }
                DDelay();
                ResetBuffer();
            }*/
            //else if( (Buff[i-11] == 'C') &&(Buff[i-10] == 'M') && (Buff[i-9] == 'T') 
            //        && (Buff[i-8] == 'I'))
            pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"CMTI",4);
            if(pToken!= NULL)
            {
                DDelay();
                //pToken = MapForward(pToken,15,(char*)"\",",2);
                //if(pToken!= NULL)
                //{
                SMSNumber = pToken[11];
                //}
                
                
                WakeUp();
                //SendSMS();
                ResetBuffer();
                Print("AT+CMGR=");
                WriteUART2(SMSNumber);
                Print("\r\n");                        
                
                i=0;
                
                Count = 0;
                while(1)
                {
                    if(MapForward(Buff2,BUFF2_SIZE,(char*)"OK",2) != NULL)
                            break;
                    if((MapForward(Buff2,BUFF2_SIZE,(char*)"ERROR",5) != NULL) || (Count>20000))
                    {   break; }
                    Count++;
                }
                
                //
                // Get rx Number
                //
                pToken = MapForward(Buff2+35,70,(char*)"\",",2);
                if(pToken != NULL)
                {
                    //pToken-=10;
                    pToken--;
                    while(*pToken != '"')pToken--;
                    pToken++;
                    
                    // save sender number
                    while(pToken[i] != '"')
                    {
                        rxNumber[i] = pToken[i];
                        i++;
                    }
                    //for(i=0;i<10;i++)
                    //    rxNumber[i] = pToken[i];
                    rxNumber[i] = '\0';           
                }
                
                if( MapForward(Buff2,BUFF2_SIZE,(char*)"00000",5) != NULL)
                {                    
                    //minVal = MIN_NOALERT; maxVal = MAX_NOALERT;
                    Print1("\r\n");//USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
                    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
                    DDelay();
                    DDelay();
                   // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
                    GPSStatus = 0;
                    D1 = 0;
                    D2 = 0;
                    SMSSent = 0;
                    if(GPSEnabled == 0)
                    {    
                        
                        DDelay();
                        ResetBuffer();
                        
                        Print("AT+CSCLK=0\r\n");
                            
                        
                    }
                }
                
               
                //SendSMS();
                
								
				if( MapForward(Buff2,BUFF2_SIZE,(char*)"$VALETRON_2_NUMBER",18) != NULL)
                { 
					


                    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"NUMBER:",7);
                    if(pToken != NULL)
                    {
                        osDelay(2000);
                        memset((void*)bstr,0,sizeof(bstr));
                        i=0;
                        while(pToken[7+i] != '#')
                        {
                            bstr[i] = pToken[7+i];
                            i++;
                            //if(i>250)goto DISCARD;
                        }
                        bstr[i] = '\0';
                    }
                    sscanf((void*)bstr, "%s",Params.Fields.rxNumber);
                    
                    StoreEEParams();
                    
                    Print("AT+CSCLK=0\r\n");
                    osDelay(200);
                    WakeUp();
                    ResetBuffer();            
                    Print((char*)"AT+CMGS=\"");
                    Print(rxNumber);
                    Print((char*)"\"\r\n");
                    DelayProc(850000);
                    Print((void*)bstr);
                    Print("  NUMBER UPDATE OK\r\n");
                    putcchar(0x1A);
                    DelayProc(850000);            
    
                }
                
                if( MapForward(Buff2,BUFF2_SIZE,(char*)"$VALETRON_2_THRESHOLD",21) != NULL)
                { 
					


                    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"THRESHOLD:",10);
                    if(pToken != NULL)
                    {
                        osDelay(2000);
                        memset((void*)bstr,0,sizeof(bstr));
                        i=0;
                        while(pToken[10+i] != '#')
                        {
                            bstr[i] = pToken[10+i];
                            i++;
                            //if(i>250)goto DISCARD;
                        }
                        bstr[i] = '\0';
                    }   
                    sscanf((void*)bstr, "%hhu",(char*)&Params.Fields.MotionThreshold);   
                    StoreEEParams();
                    	
                    Print("AT+CSCLK=0\r\n");
                    osDelay(200);
                    WakeUp();
                    ResetBuffer();            
                    Print((char*)"AT+CMGS=\"");
                    Print(rxNumber);
                    Print((char*)"\"\r\n");
                    DelayProc(850000);
                    Print((void*)bstr);
                    Print("  THRESHOLD SET OK\r\n");
                    putcchar(0x1A);
                    DelayProc(850000);            
    
                }
                
                
                if( MapForward(Buff2,BUFF2_SIZE,(char*)"$VALETRON_2_MOTIONALERT",23) != NULL)
                { 
                 
                    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"MOTIONALERT:",12);
                    if(pToken != NULL)
                    {
                        osDelay(2000);
                        memset((void*)bstr,0,sizeof(bstr));
                        i=0;
                        while(pToken[12+i] != '#')
                        {
                            bstr[i] = pToken[12+i];
                            i++;
                            //if(i>250)goto DISCARD;
                        }
                        bstr[i] = '\0';
                    }
                    sscanf((void*)bstr, "%s",Params.Fields.MotionAlertMode);
                    //sprintf(Params.Fields.MotionAlertMode,"%s","NULL");
                    StoreEEParams();
                    	
                    Print("AT+CSCLK=0\r\n");
                    osDelay(200);
                    WakeUp();
                    ResetBuffer();            
                    Print((char*)"AT+CMGS=\"");
                    Print(rxNumber);
                    Print((char*)"\"\r\n");
                    DelayProc(850000);
                    //Print("  THRESHOLD SET OK\r\n");
                    Print("MotionAlert:");
                    Print((void*)bstr);
                    Print("  UPDATED\r\n");
                    putcchar(0x1A);
                    DelayProc(850000);            
    
                }
                
                if( MapForward(Buff2,BUFF2_SIZE,(char*)"$VALETRON_2_WORKINGMODE",23) != NULL)
                { 
                    
                    pToken = MapForward(Buff2,BUFF2_SIZE,(char*)"WORKINGMODE:",12);
                    if(pToken != NULL)
                    {
                        osDelay(2000);
                        memset((void*)bstr,0,sizeof(bstr));
                        i=0;
                        while(pToken[12+i] != '#')
                        {
                            bstr[i] = pToken[12+i];
                            i++;
                            //if(i>250)goto DISCARD;
                        }
                        bstr[i] = '\0';
                    }
                    sscanf((void*)bstr, "%s",Params.Fields.WorkingMode);
                    //sprintf(Params.Fields.MotionAlertMode,"%s","NULL");
                    StoreEEParams();
                    
                    Print("AT+CSCLK=0\r\n");
                    osDelay(200);
                    WakeUp();
                    ResetBuffer();            
                    Print((char*)"AT+CMGS=\"");
                    Print(rxNumber);
                    Print((char*)"\"\r\n");
                    DelayProc(850000);
                    //Print("  THRESHOLD SET OK\r\n");
                    Print("WORKINGMODE:");
                    Print((void*)bstr);
                    Print("  UPDATED\r\n");
                    putcchar(0x1A);
                    DelayProc(850000);            
    
                }
                
                
                
//                 if( MapForward(Buff2,BUFF2_SIZE,(char*)"$VALETRON_2_ENABLE_M_ALERT#",27) != NULL)
//                { 
//									
//                    sprintf(Params.Fields.MotionAlertMode,"%s","CALL");
//                    StoreEEParams();
//                    
//                    Print("AT+CSCLK=0\r\n");
//                    osDelay(200);
//                    WakeUp();
//                    ResetBuffer();            
//                    Print((char*)"AT+CMGS=\"");
//                    Print(rxNumber);
//                    Print((char*)"\"\r\n");
//                    DelayProc(850000);
//                    Print("MotionAlert:Enabled \r\n");
//                    putcchar(0x1A);
//                    DelayProc(850000);
//                }
                 i=0;
								
                DeleteAllSMS();
                SMSNumber = 0;
                //LoopCount = 0;
                
                //i=0;
                ResetBuffer();
            }

        }
        
        else if(Params.Fields.WorkingMode[0]=='T')
        {
            if(GSMEnabled == 1)
            {
                if((MotionTimer < TIME_TO_SLEEP) || (IsQueueEmpty(RAMQueue)!=0))
                {
                    //if(SysClockConfigFlag!='H')
                    {
                            //SystemClock_Config();
                            //MX_USART2_UART_Init();
                            //MX_USART1_UART_Init();
                    }
                    #ifdef TIMER_ONLY_WAKEUP
                       while(SystemTimer<240){LPUARTTimer=0;};
                    #endif
                    
                    #ifdef DEBUG_PRINT
                        DebugPrint("Entering TCP -StartMainTask\r\n"); 
                    #endif
                    #ifdef SIM7600
                        XMQTT_Request(pFilename,1);
                    #endif
                    #ifdef SIM7070
                        YMQTT_Request(pFilename,1);
                    #endif
                    #ifdef SIM800
                        TCP_request(pFilename,1);
                    #endif
                    #ifdef TIMER_ONLY_WAKEUP
                        MotionTimer = TIME_TO_SLEEP+10;
                    #endif
                }
            }
        }
        else if(Params.Fields.WorkingMode[0]=='U')
        {
            if(GSMEnabled == 1)
            {
                if((MotionTimer < TIME_TO_SLEEP) || (IsQueueEmpty(RAMQueue)!=0))
                {
                    //if(SysClockConfigFlag!='H')
                    {
                            //SystemClock_Config();
                            //MX_USART2_UART_Init();
                            //MX_USART1_UART_Init();
                    }
                    #ifdef TIMER_ONLY_WAKEUP
                       while(SystemTimer<240){LPUARTTimer=0;};
                    #endif
                    
                    #ifdef DEBUG_PRINT
                        DebugPrint("Entering TCP -StartMainTask\r\n"); 
                    #endif
                    #ifdef SIM7600
                        XUDP_Request(pFilename,1);
                    #endif
//                    #ifdef SIM7070
//                        YMQTT_Request(pFilename,1);
//                    #endif
//                    #ifdef SIM800
//                        TCP_request(pFilename,1);
//                    #endif
                    #ifdef TIMER_ONLY_WAKEUP
                        MotionTimer = TIME_TO_SLEEP+10;
                    #endif
                }
            }
        }
        else //if(Params.Fields.WorkingMode[0]=='H')
        {
            if(GSMEnabled == 1)
            {
                // PostReboot();
                if(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS)
                {
									  	
											
//                    #ifdef BATTERY_PRESENT
//                    DisableCharger();
//                    #endif
                    
                    //CheckBattery();
                    
//                    #ifdef BATTERY_PRESENT
//                    EnableCharger();
//                    #endif
                    if((MotionTimer < TIME_TO_SLEEP) || (IsQueueEmpty(RAMQueue)!=0))
                    {   
                        //if(SysClockConfigFlag!='H')
                        {
                            //SystemClock_Config();
                            //MX_USART2_UART_Init();
                            //MX_USART1_UART_Init();
                            
                            
                        }
                        //UpdateNetwork(2);
                        #ifdef TIMER_ONLY_WAKEUP
                            SystemTimer = 0;
                            while(SystemTimer<300 && GPSStatus != 'A'){LPUARTTimer=0;};
                            osDelay(5000);
                            PostReboot();    // Posting here after GPS is available
                            GetEvent(&GPacket,EVENT_QUEUE);
                
                        #endif
                        
                        #ifdef DEBUG_PRINT
                            DebugPrint("Entering HTTP -StartMainTask\r\n"); 
                        #endif
                        
                        #ifdef SIM7600
                            if((XHTTP_Request(pFilename,1)) != 0)
                            {
                                ServerRetries++;
                                if(ServerRetries > 2)
                                {

                                    ServerRetries = 0;
                                    MotionTimer = TIME_TO_SLEEP+1; //  Make sure no events are in queue to enter sleep
                                    ClearEventCache();
                                    while(GetEvent(&GPacket,EVENT_QUEUE) == GET_SUCCESS); // Hopefully doesnt need counter
                                    MotionTimer=TIME_TO_SLEEP+1;
                                    goto ENTER_STOP_MODE;
                                }
                            }
                        
                        #endif
                        #ifdef SIM7070
                            YHTTP_Request(pFilename,1);
                        
                        #endif    
                        #ifdef SIM800
                            ZHTTP_Request(pFilename,1);
                        #endif
                        
                        //UpdateNetwork(1);
                        #ifdef TIMER_ONLY_WAKEUP
                            MotionTimer = TIME_TO_SLEEP+10;
                        #endif
                        
                    }
                    else
                    {
                        ESP_LOGW(TAG,"MotionTimerexpired");
                    }
                }
                
                #ifdef EEPROM_FIFO
                if(EEPROMReadTimer > 900) // Check EEPROM every 15 min
                {
                    while(I2CBusyFlag == 1);
                    I2CBusyFlag = 1;
                    EEPROMReadCount = 0;
                    
                    while( (GetEEEvent(&GPacket) == GET_SUCCESS) && (EEPROMReadCount <5) )
                    {
                        
                        pFilename = filename;
                        //ProcessEventPacket(&GPacket);
                        
    //                    #ifdef BATTERY_PRESENT
    //                    DisableCharger();
    //                    #endif
                        
                        CheckBattery();
                        
    //                    #ifdef BATTERY_PRESENT
    //                    EnableCharger();
    //                    #endif
                        #ifdef SIM7600
                            XHTTP_Request(pFilename,1);                        
                        #endif
                        #ifdef SIM7070
                            YHTTP_Request(pFilename,1);                        
                        #endif    
                        #ifdef SIM800
                            ZHTTP_Request(pFilename,1);
                        #endif
                        EEPROMReadCount++;
                        
                    }
                    
                    I2CBusyFlag = 0;
                    EEPROMReadTimer = 0;
                }
                #endif
            }
        }// //if HTTP MODE end
        
//        if(GSMResetTimer > 300)
//        {
//            SendATCommand("AT+CRESET\r\n","RDY","ERROR",20);
//            osDelay(3000);
//            #ifdef EXT_ANT_ENABLED
//    
//                #ifdef SIM7600
//                    SendATCommand("AT+CGPS=1,1\r\n","OK","ERROR",3);
//                    SendATCommand("AT+CGPSINFO\r\n","OK","ERROR",3);
//                #endif
//            #endif
//            GSMResetTimer = 0;
//        }        
   
        if(CheckNetwork()==1)
        {
            UpdateNetwork(0);
        }
        else
        {
            UpdateNetwork(1);
        }
        #ifdef EXT_ANT_ENABLED
            XCheckGPS();
        #endif
        vTaskDelay(1);
    }
  /* USER CODE END 3 */ 

//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
//  }
  /* USER CODE END 5 */ 
}
void InitFlash(void)
{
    esp_err_t err;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
}
void app_main(void)
{
    
    
 
    //SleepHere();
    // vTaskDelay(5000 / portTICK_PERIOD_MS);    
    SystemInit();
    //esp_sleep_cpu_retention_init(); //  Also return true in sleep_modem.c  modem_domain_pd_allowed function.
    ///
    // EnableGSM();// For LED
    // MakeAllLED(PURPLE);
    // WriteLEDStatus();
    InitFlash();
    // /* name, period/time,  auto reload, timer ID, callback */
    // blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, blehr_tx_hrate);
    //StoreEEParams();
    GetEEParams();

    
    //esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set(TAG); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
    //
    SleepWakeupReason();
    // osDelay(5000);
    // DisableGSM();
    // DisableMainPower();
    // esp_bluedroid_disable();
    // esp_bt_controller_disable();
    // esp_wifi_stop();
    // EnterDeepSleep();
    //ESP_LOGI(TAG,"Before tasks");
    xTaskCreate(ADCTask, "ADCTask", 2048, NULL, 10, NULL);
    xTaskCreate(StartTimerTask, "StartTimerTask", 4096, NULL, 10, NULL);   
    xTaskCreate(StartMainTask, "StartMainTask", 8192, NULL, 10, NULL); //TIMER_TASK_STACK_SIZE
    
    
    
}
