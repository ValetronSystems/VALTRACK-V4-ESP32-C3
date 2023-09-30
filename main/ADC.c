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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ADC.h"



/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


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
//uint32_t voltage;
void ADCTask(void *arg)
{
    
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
   
    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);


    //Continuously sample ADC1
    while (1) {

        ADCCount = 0;
        for(int i=0;i<ADC_CONVERTED_DATA_BUFFER_SIZE;i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
            // ESP_LOGI(TAG, "\nADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
            
            TADCReading = adc_raw[0][0];//adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_2);//aADCxConvertedData[i];
            ADCCount+=TADCReading;
        }
        VBATCount = (float)ADCCount/(float)ADC_CONVERTED_DATA_BUFFER_SIZE;
        //BatteryADCCount = VBATCount;
        if (do_calibration1_chan0) 
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, VBATCount, &voltage[0][0]));
            
            // ESP_LOGI(TAG, "\nADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, ADCBatteryVoltage);
        }
        ADCBatteryVoltage = (((float)DIVIDER_FACTOR*voltage[0][0])/1000)+ADC_OFFSET;
      
        //float ADCBatteryVoltage = (((float)2*voltage[0][0])/1000)+0;
        vTaskDelay(pdMS_TO_TICKS(1000));
       

    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}



/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}