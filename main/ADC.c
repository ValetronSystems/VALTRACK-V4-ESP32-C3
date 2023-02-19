/***************************************************************************
* File name :   ADC.c                                                      *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/

#include <stdio.h>
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
#include "driver/adc.h"
#include <esp_adc_cal.h>
#include "ADC.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

 static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;//ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;





// static void print_char_val_type(esp_adc_cal_value_t val_type)
// {
//     if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
//         printf("Characterized using Two Point Value\n");
//     } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
//         printf("Characterized using eFuse Vref\n");
//     } else {
//         printf("Characterized using Default Vref\n");
//     }
// }
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  64)

//////////////////////////////////////////////////
uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; /* ADC group regular conversion data (array of data) */
float ADCBatteryVoltage;
unsigned short BatteryADCCount = 0;

float VBATCount=0;
int ADCCount=0;
void AverageADCSamples(void)
{
    // ADCCount = 0;
    // for(int i=0;i<ADC_CONVERTED_DATA_BUFFER_SIZE;i++)
    // {
    
    //     ADCCount+=aADCxConvertedData[i];
    // }
    // VBATCount = (float)ADCCount/(float)ADC_CONVERTED_DATA_BUFFER_SIZE;
    // BatteryADCCount = VBATCount;
    // ADCBatteryVoltage = (((float)VBATCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
    // ADCBatteryVoltage+=ADC_OFFSET;

}
int TADCReading=0;
uint32_t voltage;
void ADCTask(void *arg)
{
    //Check if Two Point or Vref are burned into eFuse
    //check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_2, atten);
    } 

    // //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    //print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        // for (int i = 0; i < NO_OF_SAMPLES; i++) {
        //     if (unit == ADC_UNIT_1) {
        //         adc_reading += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_2);
        //     } 
        // }
        // adc_reading /= NO_OF_SAMPLES;


         ADCCount = 0;
        for(int i=0;i<ADC_CONVERTED_DATA_BUFFER_SIZE;i++)
        {
            TADCReading = adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_2);//aADCxConvertedData[i];
            ADCCount+=TADCReading;
            
        }
        VBATCount = (float)ADCCount/(float)ADC_CONVERTED_DATA_BUFFER_SIZE;
        BatteryADCCount = VBATCount;
        ADCBatteryVoltage = (((float)DIVIDER_FACTOR*esp_adc_cal_raw_to_voltage(VBATCount, adc_chars))/1000)+ADC_OFFSET;
        // ADCBatteryVoltage = (((float)VBATCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
        // ADCBatteryVoltage+=ADC_OFFSET;


        // //VBATCount = (float)ADCCount/(float)ADC_CONVERTED_DATA_BUFFER_SIZE;
        // BatteryADCCount = VBATCount;
        // ADCBatteryVoltage = (((float)VBATCount*ADC_REFERENCE*(float)DIVIDER_FACTOR)/4096);
        // ADCBatteryVoltage+=ADC_OFFSET;
        //Convert adc_reading to voltage in mV
        //  voltage = esp_adc_cal_raw_to_voltage(VBATCount, adc_chars);
        //ESP_LOGI(TAG,("Raw: %f\tVoltage: %fmV\n", BatteryADCCount, ADCBatteryVoltage));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
