#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

// Definimos el sensor de temperatura
#define ADC1_CHAN0 ADC_CHANNEL_4 
#define ADC_ATTEN ADC_ATTEN_DB_12

// Definimos Buzzer
#define BUZZER 18

// Definimos los leds
#define LEDG 2   
#define LEDY 4
#define LEDR 16

//Definimos estados de prendido y apagado
#define OFF 0
#define ON 1


adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw;
static float temp;

// Obtenemos el valor en volts del LM35, prototipos de función
esp_err_t config_ADC();
esp_err_t get_ADC_value(float *temp);
void display_temperature_interface(float temperature);

void configure_pin(){
    /* Reestablecemos los pin GPIO al estado predeterminado */
    gpio_reset_pin(LEDG); 
    gpio_reset_pin(LEDY);
    gpio_reset_pin(LEDR); 
    // Hacemos lo mismo para el buzzer
    gpio_reset_pin(BUZZER);  


    /* Configuramos como salida */
    gpio_set_direction(LEDG , GPIO_MODE_OUTPUT); 
    gpio_set_direction(LEDY , GPIO_MODE_OUTPUT); 
    gpio_set_direction(LEDR , GPIO_MODE_OUTPUT); 
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT); 
   

    /* Establecemos como nivel inicial apagado */
    gpio_set_level(LEDG, OFF); 
    gpio_set_level(LEDY, OFF); 
    gpio_set_level(LEDR, OFF);
}

void app_main(void){
    config_ADC();
    configure_pin(); 
    float temperature = 0; 

    while(true){
        get_ADC_value(&temperature);
        display_temperature_interface(temperature);

        // Si la temperatura es muy baja solo se prende el led verde
        if(temperature >= 9 && temperature <= 14){
            gpio_set_level(LEDG, ON);
            gpio_set_level(LEDY, OFF);
            gpio_set_level(LEDR, OFF);
            gpio_set_level(BUZZER, OFF);
        }
        // Si la temperatura es media se prende el led amarillo
        if(temperature > 14 && temperature <= 16){
            gpio_set_level(LEDG, OFF);
            gpio_set_level(LEDY, ON);
            gpio_set_level(LEDR, OFF);
            gpio_set_level(BUZZER, OFF);
        }
        // Si la temperatura es alta se prende el led rojo y se activa el buzzer
        if(temperature >= 16){
            gpio_set_level(LEDG, OFF);
            gpio_set_level(LEDY, OFF);
            gpio_set_level(LEDR, ON);
            gpio_set_level(BUZZER, ON);
        }
        
       // Perro guardian
        vTaskDelay(2000/ portTICK_PERIOD_MS);

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

esp_err_t get_ADC_value(float *temp) {
    // Read the raw ADC value
    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);

    // Convert the raw ADC value to voltage
    float voltage = (adc_raw * 3.3) / 4095.0;

    // Convert the voltage to temperature in Celsius
    *temp = voltage * 100.0;  // LM35 outputs 10 mV per degree Celsius

    printf("Temperatura: %2.2f °C\n", *temp);

    return ESP_OK;
}

// Función para mostrar la interfaz visual en la terminal
void display_temperature_interface(float temperature) {
    printf("\033[2J\033[H");  // Limpiar la terminal y mover el cursor al inicio

    printf("Temperatura: %2.2f °C\n", temperature);

    if(temperature >= 9 && temperature <= 14){
        printf("=====================================\n");
        printf("|         TERMÓMETRO LM35          |\n");
        printf("=====================================\n");
        printf("| Temperatura actual: %2.2f °C   Todo parece estable :)   |\n", temperature);
        printf("|-----------------------------------|\n");

    }

    if(temperature > 14 && temperature <= 16){
        printf("=====================================\n");
        printf("|         TERMÓMETRO LM35          |\n");
        printf("=====================================\n");
        printf("| Temperatura actual: %2.2f °C   PELIGRO!!! La temperatura está subiendo  |\n", temperature);
        printf("|-----------------------------------|\n");

    }

    if(temperature > 16){
        printf("=====================================\n");
        printf("|         TERMÓMETRO LM35          |\n");
        printf("=====================================\n");
        printf("| Temperatura actual: %2.2f °C   CUIDADO, RIESGO DE EXPLOSIÓN INMINENTE!!!! |\n", temperature);
        printf("|-----------------------------------|\n");

    }
    

    // Barra de progreso visual (acotada al intervalo 15-24°C)
    int bar_length;
    if (temperature < 9.0) {
        bar_length = 0;  // Si la temperatura es menor a 15°C, la barra no se muestra
    } else if (temperature > 18.0) {
        bar_length = 30;  // Si la temperatura es mayor a 24°C, la barra se llena completamente
    } else {
        bar_length = (int)((temperature - 9.0) / (18.0 - 9.0) * 30);  // Escalar la temperatura al rango 15-24°C
    }

    printf("| [");
    for (int i = 0; i < 30; i++) {
        if (i < bar_length) {
            printf("#");  // Carácter de la barra de progreso
        } else {
            printf(" ");
        }
    }
    printf("] |\n");

    printf("=====================================\n");
}