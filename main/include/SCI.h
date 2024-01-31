/***************************************************************************
* File name :   SCI.h                                                      *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/
#ifndef _SCI_H
#define _SCI_H
#include <stdint.h>
#define VALETRON_SYSTEMS


#define VALTRACK_V4_VTS
//#define VALTRACK_V4MF



//#define V4MF_UM_WB
//#define V4MF_UM_L0

#define SIM7600
//#define SIM7070
//#define SIM800

#define A7672
//#define SIM7672


#ifdef SIM7070
    #define CNMP_13  // 2G
    //#define CNMP_38  // 4G NBIOT-CAT-M1
    //#define CNMP_2   // AUTO
    
    //#define CMNB_1 // CAT-M
    //#define CMNB_2 // NB
    #define CMNB_3 // M1 and NB
#endif


extern const char *TAG;

#define DISABLE_CHARGING_LOOP

//#define SE868_ENABLED
#define EXT_ANT_ENABLED

//#define NETWORK_LOCATION_ENABLED

//#define TIMER_ONLY_WAKEUP
#define MOTION_CONTROLLED_PINGS


// LED CONTROLS 

#define WB_PIN_CONTROLLED_LED
//#define IO_EXTENDER_CONTROLLED_LED

// CHARGER PIN CONTROL


#define GPIO_CHARGER_PIN     4
#define GPIO_TPS_ENABLE_PIN  4




//////////////RESETS////////////////////////
#define LPUART_TIMER_RESET_ENABLED
//#define LPUART_TIMER_RESET_DISABLED
#ifndef VALTRACK_V4MF
    #define NETWORK_FAIL_RESET_ENABLED
#else
    #define NETWORK_FAIL_RESET_DISABLED
#endif
/////////////////////////////////////////////

#define SLEEP_ENABLED
//#define SLEEP_DISABLED

#define PARAMS_NORMAL

//#define CUSTOM_MQTT_CLIENT_ID


#define PAYLOAD_NORMAL


#ifndef VALTRACK_V4_VTS

#define RED_RANGE_LOW       0.0
#define RED_RANGE_HIGH      3.7

#define BLUE_RANGE_LOW      3.7
#define BLUE_RANGE_HIGH     3.8

#define GREEN_RANGE_LOW     3.8
#define GREEN_RANGE_HIGH    4

#define GREEN_F_RANGE_LOW   4.0
#define GREEN_F_RANGE_HIGH  4.7

#else

#define RED_RANGE_LOW       0.0
#define RED_RANGE_HIGH      8.0

#define BLUE_RANGE_LOW      8.0
#define BLUE_RANGE_HIGH     11.0

#define GREEN_RANGE_LOW     11.0
#define GREEN_RANGE_HIGH    18.0

#define GREEN_F_RANGE_LOW   11.0
#define GREEN_F_RANGE_HIGH  18.2

#endif

#ifdef SIM7600
    #define PACKET_COUNT 3
#else
    #define PACKET_COUNT 1
#endif



#ifdef SIM800
    #define LPUART_BAUDRATE 9600
#else
    #define LPUART_BAUDRATE 115200
#endif

//ifdef VALTRACK_V4_VTS
    #define LIS3DH_ENABLED
//#else
//    #define MMA854_ENABLED
//#endif

#ifdef MMA854_ENABLED
    #define ACCLEROMETER_I2C_ADDRESS 0x1D  //MMA854
#else 
    #define ACCLEROMETER_I2C_ADDRESS 0x19  //LIS3D
#endif

#define PCA_I2C_ADDRESS 0xC0<<1  //LIS3D

#define ADC_REFERENCE 3.3

// Battery voltage conversion defines START
#ifdef VALTRACK_V4_VTS

    #define R1 100000
    #define R2 3300
    #define ADC_OFFSET    0.4//3.3
    #define DIVIDER_FACTOR (float)(((float)R1+(float)R2)/(float)R2)
        
#else
    #define ADC_OFFSET    0//3.3
    #define DIVIDER_FACTOR 2
    
    
#endif
// Battery voltage conversion defines END

#ifdef PARAMS_MASSIMO
    #define POWER_BUTTON_SOS_SWAP
#endif
    
//#define TAMPER_DETECT_MODE //  Bracelet removal detection for POWER_BUTTON


#define TIME_TO_SLEEP 300
#define HEART_BEAT_INTERVAL 6*3600

#define BLUETOOTH_ENABLED

//#define BATTERY_PRESENT

#define BATTERY_READ_INTERVAL 300

#define SHEETS_ENABLED
/////////////////////////////////////////////////////////////////////////////////////////////

// PWRKEY 7
// GSM ENABLE 10
// LED SIGNAL 8
// TPS ENABLE 4 //OR CHG IN
// INT1 3
// ANALOG IN 2
// IIC DATA 5
// IIC CLOCK 6
#define GPIO_LED_SIGNAL 8
#define GPIO_PWRKEY    7
#define GPIO_GSM_ENABLE    10
#define GPIO_TPS_ENABLE    4
#define GPIO_PWRKEY_GSM_ENABLE_PIN_SEL  ((1ULL<<GPIO_PWRKEY) | (1ULL<<GPIO_GSM_ENABLE))
#define GPIO_TPS_ENABLE_PIN_SEL ((1ULL<<GPIO_TPS_ENABLE))
#define GPIO_INT1     3
#define GPIO_SOS      9
#define GPIO_CHG_IN   4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INT1) | (1ULL<<GPIO_SOS) | (1ULL<<GPIO_CHG_IN))  

//#define ENABLE_TPS_CONTROL 
#define DISABLE_TPS_CONTROL

#define UART_PORT_NUM UART_NUM_1


#define ECHO_TEST_TXD 0//21//(CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD 1//20//(CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      UART_NUM_1//(CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     115200//(CONFIG_EXAMPLE_UART_BAUD_RATE)

#define CONFIG_I2C_MASTER_SCL 6
#define CONFIG_I2C_MASTER_SDA 5



//#define BUF_SIZE (1024)


#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern const char *TAG;

#define BUFF_SIZE 250

extern char Buff[];
extern unsigned short BuffIndex;


#define BUFF2_SIZE 350

extern char Buff2[];
extern unsigned short Buff2Index;

#define TBUFF_SIZE 252

extern char Buff[];
extern unsigned short BuffIndex;

//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
extern unsigned char DebugPrintEnabled;
#endif   
typedef enum 
{
    EVENT_ENGINE_OFF                    ,    // 0   00 
    EVENT_ENGINE_ON                     ,    // 1   01 
    REGULAR_TEST                        ,    // 2   02 
    RANDOM_TEST                         ,    // 3   03 
    EVENT_BYPASS                        ,    // 4   04 
    EVENT_LOG_CLEARED                   ,    // 5   05 
    EVENT_SERVICE_PEROD_SET             ,    // 6   06 
    EVENT_RESET_FOR_VIOLATION           ,    // 7   07 
    EVENT_USE_ONE_TIME_CODE             ,    // 8   08 
    EVENT_RE_DO                         ,    // 9   09 
    EVENT_12V_POWER_REMOVED             ,    // 10  0A
    EVENT_LOG_READ                      ,    // 11  0B
    EVENT_CONFIG_CHANGED                ,    // 12  0C
    EVENT_LOG_FULL                      ,    // 13  0D
    EVENT_SERVICE_ALERT                 ,    // 14  0E
    not_used                            ,    // 15  0F
    EVENT_WEAK_BLOW                     ,    // 16  10
    EVENT_WARNING_FAIL_RR               ,    // 17  11
    EVENT_WARNING_REFUSED               ,    // 18  12
    EVENT_WARNING_SERVICE_PERIOD_END    ,    // 19  13
    EVENT_CAR_BATTERY_ON                ,    // 20  14
    EVENT_CAR_BATTERY_OFF               ,    // 21  15
    EVENT_CALIBATION_DONE               ,    // 22  16
    EVENT_IGNITION_KEYED                ,    // 23  17
    EVENT_WARNING_GIVEN                 ,    // 24  18
    EVENT_STATER_NOT_ACTIVE             ,    // 25  19
    EVENT_INSUFFICIENT_BLOW             ,    // 26  1A
    EVENT_COOL_SAMPLE                   ,    // 27  1B
    EVENT_TAMPERED                      ,    // 28  1C
    EVENT_START_TEST_ATTEMPT            ,    // 29  1D
    EVENT_AB_FC_CONNECTED               ,    // 30  1E
    EVENT_AB_FC_REMOVED	                ,    // 31  1F
    EVENT_CAL_CHK_PASS	                ,    // 32  20
    EVENT_CAL_CHK_FAIL	                ,    // 33  21
    EVENT_ENGINE_NOT_STARTED		    ,    // 34  22
    G_PING      = 35 				    ,    // 35  23
    GPRS_PING   = 36  					,	 // 36  24
    MOTION_PING = 37			        ,	 // 37  25
    REBOOT_PING = 38			        ,	 // 38  26
    SOS_PING    = 39		            ,	 // 39  27
}EventCodeType;
    
typedef enum 
{   
    NL_SUCCESS                                        =0,    
    NL_PARAMETER_ERROR_RETURNED_BY_SERVER             =1,   
    NL_SERVICE_OUT_OF_TIME_RETURNED_BY_SERVER         =2,    
    NL_LOCATION_FAILED_RETURNED_BY_SERVER             =3,    
    NL_QUERY_TIMEOUT_RETURNED_BY_SERVER               =4,    
    NL_CERTIFICATION_FAILED_RETURNED_BY_SERVER        =5,    
    NL_SERVER_LBS_ERROR_SUCCESS                       =6,    
    NL_SERVER_LBS_ERROR_FAILED                        =7,    
    NL_LBS_IS_BUSY                                    =8,    
    NL_OPEN_NETWORK_ERROR                             =9,    
    NL_CLOSE_NETWORK_ERROR                           =10,   
    NL_OPERATION_TIMEOUT                             =11,   
    NL_DNS_ERROR                                     =12,   
    NL_CREATE_SOCKET_ERROR                           =13,   
    NL_CONNECT_SOCKET_ERROR                          =14,   
    NL_CLOSE_SOCKET_ERROR                            =15,   
    NL_GET_CELL_INFO_ERROR                           =16,   
    NL_GET_IMEI_ERROR                                =17,   
    NL_SEND_DATA_ERROR                               =18,   
    NL_RECEIVE_DATA_ERROR                            =19,   
    NL_NONET_ERROR                                   =20,   
    NL_NET_NOT_OPENED                                =21,   
    NL_REPORT_LBS_TO_SERVER_SUCCESS                  =80,   
    NL_REPORT_LBS_TO_SERVER_PARAMETER_ERROR          =81,   
    NL_REPORT_LBS_TO_SERVER_FAILED                   =82,   
    NL_OTHER_ERROR                                   =110 

}NetworkLocationStatusType;
typedef struct
{
    char Bytes[5];
}EngineStatusStringType;

typedef struct
{
    char Bytes[50];
}StringType;
typedef enum
{
    MOTION_DETECTED = 0,
    NO_ACTIVITY = 1

}MotionStatusType;
//
// Overrun event
//
#define SYSTEM_EVENT_FIFO_OVERRUN  0xFF

//
// Message data length (fixed at 16 to fully support STS current 
// message structure)
//             
#define MESSAGE_LENGTH          16
#define MESSAGE_DATA_LENGTH     (MESSAGE_LENGTH-8)

//
// HW data length.  This data is smaller so we can store more
// events in a smaller space using a circular buffer FIFO
//
#define HW_EVENT_LENGTH         24//32+16+10+2

typedef struct
{
    EventCodeType EventType;
    unsigned char Hours;
    unsigned char Minutes;
    unsigned char Seconds;
    unsigned char Date;
    unsigned char Month;
    unsigned char Year;
    float Voltage;
    float Lat;
    float Long;
    float Speed;
    
}GeneralEventType;

typedef union
{
    unsigned char Bytes[HW_EVENT_LENGTH];    
  
    GeneralEventType GEvent;
         
} HWEventDataType;

//extern double fLat,fLong;  
typedef enum 
{
    RTC_RESET,          // 0
    INT1_RESET,         // 1
    BUFF2_RESET,        // 2
    NETWORK_RESET,      // 3
    LPUART_TIMER_RESET, // 4
    MOTION_RESET,       // 5
    HARDFAULT_RESET,    // 6    
    DEEP_SLEEP_RESET,   // 7
    CHARGER_RESET       // 8

}BootReasonType;

#define _countof(a) (sizeof(a)/sizeof(*(a)))

extern char Link[];
extern char D1,D2,GPSStatus;
extern char Lat[10],Long[10];
extern char Altitude[10];
extern unsigned char f;
extern float fLat,fLong,pfLat,pfLong,fSpeed,fAltitude; 

extern char GPSTime[],GPSDate[],GPSSpeed[];
extern int GPSHours,GPSMinutes,GPSSeconds,GPSDay,GPSMonth,GPSYear;

extern unsigned char Byte1,Byte2,Byte3,Byte4,Byte5,Byte6;

unsigned char GetAscii(unsigned char Data);

void  Binary2Ascii(unsigned long HexValue);

//extern unsigned char ProcessPacket;
void ResetBuffer(void);
void ResetBuffer1(void);
void InitSCI(void);
void putcchar(unsigned char Data);
void putcchar1(unsigned char Data);
void Print1(char *pData);
unsigned char ReadUART2(void);
void WriteUART2(unsigned char Data);
void PrintNumber(unsigned short Data);
unsigned char GetCheckSum(char*pData,unsigned char len);
void SCIError_Handler(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void PrintPacket(char*pData,unsigned short Length);
void DebugPrint(char*pData);
/*******************************************************************************
* Funtion name : MapForward                                                    *
*                                                                              *
* Description  : This function performs forward mapping and returns pointer    *
*                to the start of the found data or else returns NULL pointer   *
*                                                                              *
* Arguments    : 1) Poshorter to the data in which mapping is to be made       *
*                2) Length of the data in which mapping is to be made          *
*                3) Poshorter to the data points to be mapped                  *
*                4) Length of the data points to be mapped                     *
*                                                                              *
* Returns      : Pointer in which result is returned                           *
*******************************************************************************/
char * MapForward
(
	char *pMapData,
	unsigned short   MapDataLength,
	char *pMapPoints,
	unsigned short   MapPointsLength	
);
/*****************************************************************************
* Function Name : Print                                                      *
*                                                                            *
* Description   : This function is used to print data onto the serial        *
*                 communications interface of 8051 type MCU                  *
*                                                                            *
* Arguments     : 1) Pointer to the data to be printed                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void Print(char *pData);
/*****************************************************************************
* Function Name : PrintNumber                                                *
*                                                                            *
* Description   : This function is used to print data onto the serial        *
*                 communications interface of 8051 type MCU                  *
*                                                                            *
* Arguments     : 1) Number to be printed                                    *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void PrintNumber(unsigned short Data);

void DelayProc(unsigned long Count);
void HandleGPSData(void);
void HandleGPSINFData(unsigned char Byte);
void HandleGSMData(void);
void RTOSDelay(unsigned int millis);
void InitUART(void);
void osDelay(unsigned int Value);



/** 
  * @brief  RTC Time structure definition  
  */
typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t TimeFormat;       /*!< Specifies the RTC AM/PM Time.
                                 This parameter can be a value of @ref RTC_AM_PM_Definitions */
  
  uint32_t SubSeconds;     /*!< Specifies the RTC_SSR RTC Sub Second register content.
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity */
 
  uint32_t SecondFraction;  /*!< Specifies the range or granularity of Sub Second register content
                                 corresponding to Synchronous pre-scaler factor value (PREDIV_S)
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity.
                                 This field will be used only by HAL_RTC_GetTime function */
  
  uint32_t DayLightSaving;  /*!< Specifies RTC_DayLightSaveOperation: the value of hour adjustment.
                                 This parameter can be a value of @ref RTC_DayLightSaving_Definitions */

  uint32_t StoreOperation;  /*!< Specifies RTC_StoreOperation value to be written in the BCK bit 
                                 in CR register to store the operation.
                                 This parameter can be a value of @ref RTC_StoreOperation_Definitions */
}RTC_TimeTypeDef;

/** 
  * @brief  RTC Date structure definition
  */
typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay.
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

}RTC_DateTypeDef;

/** 
  * @brief  RTC Alarm structure definition
  */
typedef struct
{
  RTC_TimeTypeDef AlarmTime;     /*!< Specifies the RTC Alarm Time members */

  uint32_t AlarmMask;            /*!< Specifies the RTC Alarm Masks.
                                      This parameter can be a value of @ref RTC_AlarmMask_Definitions */
  
  uint32_t AlarmSubSecondMask;   /*!< Specifies the RTC Alarm SubSeconds Masks.
                                      This parameter can be a value of @ref RTC_Alarm_Sub_Seconds_Masks_Definitions */

  uint32_t AlarmDateWeekDaySel;  /*!< Specifies the RTC Alarm is on Date or WeekDay.
                                     This parameter can be a value of @ref RTC_AlarmDateWeekDay_Definitions */

  uint8_t AlarmDateWeekDay;      /*!< Specifies the RTC Alarm Date/WeekDay.
                                      If the Alarm Date is selected, this parameter must be set to a value in the 1-31 range.
                                      If the Alarm WeekDay is selected, this parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint32_t Alarm;                /*!< Specifies the alarm .
                                      This parameter can be a value of @ref RTC_Alarms_Definitions */
}RTC_AlarmTypeDef;

#endif
