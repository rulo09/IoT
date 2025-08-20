#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

#define ADC1_CHAN0 ADC_CHANNEL_4 
#define ADC_ATTEN ADC_ATTEN_DB_12

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw;
static float voltage;

esp_err_t config_ADC();
esp_err_t get_ADC_value();

void app_main(void){
    config_ADC();

    while(true){
        get_ADC_value();

        vTaskDelay(250/ portTICK_PERIOD_MS);
    }
}

esp_err_t config_ADC(){
    /* ADC1 Init */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    /* ADC1 Config */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);

    return ESP_OK;
}

esp_err_t get_ADC_value(){
    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);

    printf("Raw data: %d \n", adc_raw);

    voltage = (adc_raw * 3.3 / 4095.0);
    printf("Voltage: %2.2f V\n", voltage);
    
    return ESP_OK;
}
