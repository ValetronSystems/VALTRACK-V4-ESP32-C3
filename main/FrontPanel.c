/***************************************************************************
* File name :   FrontPanel.c                                               *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/
#include <SCI.h>
#include "esp_log.h"
#include "FrontPanel.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#define PCA_ADDRESS 0xC0

///////////////////////////////
// #define BLINK_GPIO 8 // LED_SIGNAL
// #define CONFIG_BLINK_LED_RMT_CHANNEL 0
// #define CONFIG_BLINK_PERIOD 1000

// uint8_t s_led_state = 0;
// led_strip_t *pStrip_a;


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      8

#define EXAMPLE_LED_NUMBERS         3
#define EXAMPLE_CHASE_SPEED_MS      10

//static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];


// void blink_led(void)
// {
//     /* If the addressable LED is enabled */
//     //if (s_led_state) {
//         /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
//         pStrip_a->set_pixel(pStrip_a, 0, LED_BRIGHTNESS, 0, 0);
//         pStrip_a->set_pixel(pStrip_a, 1, 0, LED_BRIGHTNESS, 0);
//         pStrip_a->set_pixel(pStrip_a, 2, 0, 0, LED_BRIGHTNESS);
//         /* Refresh the strip to send data */
//         pStrip_a->refresh(pStrip_a, 100);
//     //} else {
//         /* Set all LED off to clear all pixels */
//         //pStrip_a->clear(pStrip_a, 50);
//     //}
// }
rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
void configure_led(void)
{
    // ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    // /* LED strip initialization with the GPIO and pixels number*/
    // pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 3);
    // /* Set all LED off to clear all pixels */
    // pStrip_a->clear(pStrip_a, 50);


    
    ESP_LOGI(TAG, "Create RMT TX channel");
    //rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    //rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    
}

void led_deinit(void)
{
    //led_strip_denit(pStrip_a);

}
LEDType LEDValues = {{0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01}};
LEDStatesType LEDState;
IndicatorLEDType Indicators;
ChargingStatusType ChargingStatus = CONNECTED;
ChargingStatusType pChargingStatus = DISCONNECTED;
unsigned char BootTimer=0;
SystemStateType SystemState = State_IdleState;
unsigned char LEDTouched = 1;

unsigned char I2CBusyFlag = 0;

void WriteLEDStatus(void)
{
    //unsigned char i;
    
    //ESP_LOGI(TAG, "(%d,%d,%d)(%d,%d,%d)(%d,%d,%d)",LEDValues.Bytes[BatteryLEDRed],LEDValues.Bytes[BatteryLEDGreen],LEDValues.Bytes[BatteryLEDBlue],LEDValues.Bytes[NetworkLEDRed],LEDValues.Bytes[NetworkLEDGreen],LEDValues.Bytes[NetworkLEDBlue],LEDValues.Bytes[LocationLEDRed],LEDValues.Bytes[LocationLEDGreen],LEDValues.Bytes[LocationLEDBlue]);
    //if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        // pStrip_a->set_pixel(pStrip_a, 0, LEDValues.Bytes[BatteryLEDRed],LEDValues.Bytes[BatteryLEDGreen],LEDValues.Bytes[BatteryLEDBlue]);
        // pStrip_a->set_pixel(pStrip_a, 1, LEDValues.Bytes[NetworkLEDRed],LEDValues.Bytes[NetworkLEDGreen],LEDValues.Bytes[NetworkLEDBlue]);
        // pStrip_a->set_pixel(pStrip_a, 2, LEDValues.Bytes[LocationLEDRed],LEDValues.Bytes[LocationLEDGreen],LEDValues.Bytes[LocationLEDBlue]);
        // /* Refresh the strip to send data */
        // pStrip_a->refresh(pStrip_a, 100);
   // }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    led_strip_pixels[0] = LEDValues.Bytes[BatteryLEDGreen];//green;
    led_strip_pixels[1] = LEDValues.Bytes[BatteryLEDRed];//red;
    led_strip_pixels[2] = LEDValues.Bytes[BatteryLEDBlue];//blue;

    led_strip_pixels[3] = LEDValues.Bytes[NetworkLEDGreen];//green;
    led_strip_pixels[4] = LEDValues.Bytes[NetworkLEDRed];//red;
    led_strip_pixels[5] = LEDValues.Bytes[NetworkLEDBlue];//blue;

    led_strip_pixels[6] = LEDValues.Bytes[LocationLEDGreen];//green;
    led_strip_pixels[7] = LEDValues.Bytes[LocationLEDRed];//red;
    led_strip_pixels[8] = LEDValues.Bytes[LocationLEDBlue];//blue;

    // }
    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &   tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, 100));

        
   
}


void UpdateLED(IndicatorLEDType LEDnum, LEDStatesType State)
{
    if(BootTimer<4) return; // Let it boot LED blink
    LEDValues.Bytes[LEDnum] = State;
    LEDTouched = 1;//WriteLEDStatus();
}
void UpdateLED1(LEDColorType Color)
{
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    if(BootTimer<4) return; // Let it boot LED blink
    if( Color == RED_COLOR )
         RGBLED.Red   = LED_ON;
    else if( Color == GREEN_COLOR  )
         RGBLED.Green   = LED_ON;
    else if(Color == BLUE_COLOR  )
         RGBLED.Blue   = LED_ON;
    
    
 
    
    //UpdateLED(BatteryLEDRed,   RGBLED.Red);
    //UpdateLED(BatteryLEDGreen, RGBLED.Green);
    //UpdateLED(BatteryLEDBlue,  RGBLED.Blue);
    
    LEDValues.Bytes[BatteryLEDRed] = RGBLED.Red;
    LEDValues.Bytes[BatteryLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[BatteryLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
    
   
}
void UpdateBattery(float BatteryValue)
{
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    
    if(BootTimer<4) return; // Let it boot LED blink
    #ifndef WB_PIN_CONTROLLED_LED   
        if( (BatteryValue >= (float)RED_RANGE_LOW) && (BatteryValue < (float)RED_RANGE_HIGH) )
            {if(ChargingStatus == CONNECTED) {RGBLED.Red   = LED_PWM0;}  else {RGBLED.Red   = LED_ON;}}
        
        if( (BatteryValue >= (float)BLUE_RANGE_LOW) && (BatteryValue < (float)BLUE_RANGE_HIGH) )
            {if(ChargingStatus == CONNECTED) {RGBLED.Blue  = LED_PWM0;}  else {RGBLED.Blue  = LED_ON;}}
        
        if( (BatteryValue >= (float)GREEN_RANGE_LOW) && (BatteryValue < (float)GREEN_RANGE_HIGH) )
            {if(ChargingStatus == CONNECTED) {RGBLED.Green = LED_PWM0;}  else {RGBLED.Green = LED_ON;}}
        
        if( (BatteryValue >= (float)GREEN_F_RANGE_LOW) && (BatteryValue <= (float)GREEN_F_RANGE_HIGH) )
        { RGBLED.Green = LED_ON;}

    #else
      if( (BatteryValue >= (float)RED_RANGE_LOW) && (BatteryValue < (float)RED_RANGE_HIGH) )
            {RGBLED.Red   = LED_ON;}
        
        if( (BatteryValue >= (float)BLUE_RANGE_LOW) && (BatteryValue < (float)BLUE_RANGE_HIGH) )
            { RGBLED.Blue  = LED_ON;}
        
        if( (BatteryValue >= (float)GREEN_RANGE_LOW) && (BatteryValue < (float)GREEN_RANGE_HIGH) )
            {RGBLED.Green = LED_ON;}
        if( (BatteryValue >= (float)GREEN_F_RANGE_LOW) )//&& (BatteryValue <= (float)GREEN_F_RANGE_HIGH) )
        { RGBLED.Green = LED_ON;}
      
        
    #endif 
    
    //UpdateLED(BatteryLEDRed,   RGBLED.Red);
    //UpdateLED(BatteryLEDGreen, RGBLED.Green);
    //UpdateLED(BatteryLEDBlue,  RGBLED.Blue);
    
    LEDValues.Bytes[BatteryLEDRed] = RGBLED.Red;
    LEDValues.Bytes[BatteryLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[BatteryLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
    
   
}

void UpdateNetwork(unsigned char NetworkValue)
{
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    
   if(BootTimer<4) return; // Let it boot LED blink
//    if( NetworkValue == 0 )
//         RGBLED.Green   = LED_PWM0;
//    else if( NetworkValue == 1 )
//         RGBLED.Green   = LED_ON;
//    else if( NetworkValue == 2 )
//         RGBLED.Green   = LED_PWM1;
    if( NetworkValue == 0 )
         RGBLED.Red   = LED_ON;
    else if( NetworkValue == 1 )
         RGBLED.Green   = LED_ON;
    // else if( NetworkValue == 2 )
    //      RGBLED.Green   = LED_PWM1;

 
    
    //UpdateLED(NetworkLEDRed,   RGBLED.Red);
    //UpdateLED(NetworkLEDGreen, RGBLED.Green);
    //UpdateLED(NetworkLEDBlue,  RGBLED.Blue);
    
    LEDValues.Bytes[NetworkLEDRed] = RGBLED.Red;
    LEDValues.Bytes[NetworkLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[NetworkLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
    

   
}
void UpdateLED3(LEDColorType Color)
{
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    if(BootTimer<4) return; // Let it boot LED blink
    if( Color == RED_COLOR )
         RGBLED.Red   = LED_ON;
    else if( Color == GREEN_COLOR  )
         RGBLED.Green   = LED_ON;
    else if(Color == BLUE_COLOR  )
         RGBLED.Blue   = LED_ON;
    
    
 
    
    //UpdateLED(BatteryLEDRed,   RGBLED.Red);
    //UpdateLED(BatteryLEDGreen, RGBLED.Green);
    //UpdateLED(BatteryLEDBlue,  RGBLED.Blue);
    
    LEDValues.Bytes[LocationLEDRed] = RGBLED.Red;
    LEDValues.Bytes[LocationLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[LocationLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
    
   
}
void UpdateLocation(unsigned char CurrentGPSStatus)
{
    
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    if(BootTimer<4) return; // Let it boot LED blink
    if( CurrentGPSStatus == 'V' )
         RGBLED.Red   = LED_ON;
    else if( CurrentGPSStatus == 'A' )
         RGBLED.Green   = LED_ON;
    // Redundant
    //else if( CurrentGPSStatus == 0 )
    //     RGBLED.Green   = LED_OFF;
  
    LEDValues.Bytes[LocationLEDRed] = RGBLED.Red;
    LEDValues.Bytes[LocationLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[LocationLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
   
}
void UpdateBluetooth(unsigned char BluetoothStatus)
{
    RGBLEDStatusType RGBLED = {LED_OFF,LED_OFF,LED_OFF};
    if(BootTimer<4) return; // Let it boot LED blink
    if( BluetoothStatus == 0 )
         RGBLED.Blue   = LED_OFF;
    else if( BluetoothStatus == 1 )
         RGBLED.Blue   = LED_ON;
    // else if( BluetoothStatus == 2 )
    //      RGBLED.Blue   = LED_PWM1;
  
    LEDValues.Bytes[NetworkLEDRed] = RGBLED.Red;
    LEDValues.Bytes[NetworkLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[NetworkLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;//WriteLEDStatus();
   
}


void MakeAllLED(uint8_t R, uint8_t G, uint8_t B)
{
    RGBLEDStatusType RGBLED = {LED_BRIGHTNESS*R/100,LED_BRIGHTNESS*G/100,LED_BRIGHTNESS*B/100};
   
    
    //UpdateLED(NetworkLEDRed,   RGBLED.Red);
    //UpdateLED(NetworkLEDGreen, RGBLED.Green);
    //UpdateLED(NetworkLEDBlue,  RGBLED.Blue);
    
    LEDValues.Bytes[BatteryLEDRed] = RGBLED.Red;
    LEDValues.Bytes[BatteryLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[BatteryLEDBlue] = RGBLED.Blue;
    
    LEDValues.Bytes[NetworkLEDRed] = RGBLED.Red;
    LEDValues.Bytes[NetworkLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[NetworkLEDBlue] = RGBLED.Blue;
    
    LEDValues.Bytes[LocationLEDRed] = RGBLED.Red;
    LEDValues.Bytes[LocationLEDGreen] = RGBLED.Green;
    LEDValues.Bytes[LocationLEDBlue] = RGBLED.Blue;
    LEDTouched = 1;
    //WriteLEDStatus();
    
   
}

