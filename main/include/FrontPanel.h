/***************************************************************************
* File name :   FrontPanel.h                                           *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FRONTPANEL_H
#define __FRONTPANEL_H
#include "led_strip.h"

#define LED_BRIGHTNESS 18

#define TURN_OFF  0 ,   0 , 0 
#define RED     100 ,   0 , 0
#define GREEN     0 , 100 , 0
#define BLUE      0 ,   0 , 100
#define YELLOW  100 , 100 , 0
#define ORANGE  100 , 64.7, 0
#define PURPLE  62.7, 12.5, 94.1




typedef union 
{
    unsigned char Bytes[10];
    struct
    {
        unsigned char INPUT0;
        unsigned char INPUT1;
        unsigned char PSC0;
        unsigned char PWM0;
        unsigned char PSC1;
        unsigned char PWM1;
        unsigned char LS0;
        unsigned char LS1;
        unsigned char LS2;
        unsigned char LS3;
        
    }Fields;
}PCARegisterType;

typedef union LEDStruct
{
    unsigned char Bytes[16];
    struct
    {
        unsigned char LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7;
        unsigned char LED8, LED9,LED10,LED11,LED12,LED13,LED14,LED15;
    }Fields;
    
}LEDType;


typedef enum LEDStates
{
    LED_OFF  = 0x00,
    LED_ON   = LED_BRIGHTNESS,    

    
}LEDStatesType;
typedef enum LEDColors
{
    RED_COLOR,
    GREEN_COLOR,
    BLUE_COLOR,
    
}LEDColorType;
typedef struct RGBLEDStatuses
{
    LEDStatesType Red;
    LEDStatesType Green;
    LEDStatesType Blue;
    
}RGBLEDStatusType;

typedef enum IndicatorLEDs
{
    BatteryLEDRed   = 0,
    BatteryLEDGreen = 1,
    BatteryLEDBlue  = 2,
    
    NetworkLEDRed   = 3,
    NetworkLEDGreen = 4,
    NetworkLEDBlue  = 5,
    
    LocationLEDRed   = 6,
    LocationLEDGreen = 7,
    LocationLEDBlue  = 8,
    
    
}IndicatorLEDType;

#ifdef DISABLE_CHARGING_LOOP
typedef enum ChargingStatuses
{
    CONNECTED = 1,
    DISCONNECTED = 0,
    
}ChargingStatusType;
#else
typedef enum ChargingStatuses
{
    CONNECTED = 0,
    DISCONNECTED = 1,
    
}ChargingStatusType;
#endif 

typedef enum SystemStates
{
    State_IdleState = 0,
    State_ConnectedState,
    State_StartedTransactionState,
    State_LeverOperatedState
    
}SystemStateType;


extern SystemStateType SystemState;
extern ChargingStatusType ChargingStatus;
extern ChargingStatusType pChargingStatus;

extern unsigned char I2CBusyFlag;
extern unsigned char LEDTouched;
extern unsigned char BootTimer;

extern led_strip_t *pStrip_a;


void BackupPCAStatus(void);
void RestorePCAStatus(void);
void PCA_I2C_WrReg(unsigned char Reg, unsigned char Data);
unsigned char PCA_I2C_RdReg(unsigned char Reg);
void WriteLEDStatus(void);
void UpdateLED(IndicatorLEDType, LEDStatesType State);
void UpdateBattery(float BatteryValue);    
void UpdateNetwork(unsigned char NetworkValue);
void UpdateLocation(unsigned char CurrentGPSStatus);
void UpdateBluetooth(unsigned char BluetoothStatus);
void UpdateLED1(LEDColorType Color);
void UpdateLED3(LEDColorType Color);
void DisableLED(void);
void MakeAllRed(void);
void MakeAllGreen(void);
void MakeAllBlue(void);
void BlinkAllBlue(void);
void TurnOffLED(void);
void configure_led(void);
void MakeAllLED(uint8_t R, uint8_t G, uint8_t B);
void led_deinit(void);
#endif
