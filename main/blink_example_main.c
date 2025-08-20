/*
// PRACTICA QUE MUEVE UN MOTOR EN UNA DIRECCION Y LUEGO EN OTRA Y SOLO AVANZA EN PRESENCIA DE LUZ USANDO FOTORESISTENCIA


#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

#define IN1 17
#define IN2 16
#define FTD 14
#define BUTTON 18

#define LOW 0
#define HIGH 1

bool state = false;
uint16_t counter = 0;

const char *TAG = "Sistema QROBUS"; 

void isr_handler(void *args) {
    counter++; 
    state = !state;

esp_err_t init_isr() {
    gpio_config_t GPIO_config = {
        .pin_bit_mask = (1ULL << BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // ← Activamos pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE        // ← Interrupción al presionar
    };

    gpio_config(&GPIO_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, isr_handler, NULL);

    return ESP_OK;
}

esp_err_t forward() {
    gpio_set_level(IN1, HIGH);
    gpio_set_level(IN2, LOW);
    return ESP_OK;
}

esp_err_t reward() {
    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, HIGH);
    return ESP_OK;
}

esp_err_t stop() {
    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, LOW);
    return ESP_OK;
}

esp_err_t initialize_pin() {
    gpio_reset_pin(IN1);
    gpio_reset_pin(IN2);
    gpio_reset_pin(FTD);
    gpio_reset_pin(BUTTON);

    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(FTD, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);

    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, LOW);

    return ESP_OK;
}

void app_main() {
    ESP_ERROR_CHECK(initialize_pin());
    ESP_ERROR_CHECK(init_isr());

    while (1) {
        if (gpio_get_level(FTD)) {
            if (state) {
                ESP_ERROR_CHECK(forward());
            } else {
                ESP_ERROR_CHECK(reward());
            }
        } else {
            ESP_ERROR_CHECK(stop());
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // pequeña pausa para evitar rebotes
    }
}
*/


/*
// PRACTICA PARA MOVER SERVOMOTOR


*/

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/mcpwm_prelude.h"
// #include "esp_adc/adc_oneshot.h"

// /* Set the parameters according to your servo */
// #define SERVO_MIN_PULSEWIDTH_US 500 /* Minimum pulse width in microsecond */
// #define SERVO_MAX_PULSEWIDTH_US 2400 /* Maximum pulse width in microsecond */
// #define SERVO_MIN_DEGREE 0 /* Minimum angle */
// #define SERVO_MAX_DEGREE 180 /* Maximum angle */
// #define SERVO_PULSE_GPIO 14 /* GPIO connects to the PWM signal line */
// #define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* 1MHz, 1us per tick */
// #define SERVO_TIMEBASE_PERIOD 20000 /* 20000 ticks, 20ms */

// #define POTENCIOMETRO_ADC_CHANNEL ADC_CHANNEL_6  // GPIO del potenciometro

// adc_oneshot_unit_handle_t adc1_handle;
// // Prototipos de funciones
// esp_err_t config_ADC();
// int get_ADC_value(adc_channel_t channel);

// static const char *TAG = "PWM servo";

// mcpwm_timer_handle_t timer = NULL;
// mcpwm_oper_handle_t oper = NULL;
// mcpwm_cmpr_handle_t comparator = NULL;
// mcpwm_gen_handle_t generator = NULL;

// esp_err_t mcpwm_config();
// static inline uint32_t angle_to_compare(int angle);

// long map(long x, long in_min, long in_max, long out_min, long out_max) {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//   }

// void app_main(void){
//     mcpwm_config();
//     ESP_ERROR_CHECK(config_ADC());

//     int new_value = 0;

//     while (1) {
//         int new_value = map(get_ADC_value(POTENCIOMETRO_ADC_CHANNEL), 0, 4095, 0, 180); // Mapear el valor a un rango de 0 a 180 grados

//         ESP_LOGI(TAG, "New Angle of rotation: %d", new_value);
//         ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(new_value)));

//         /* Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply */
//         vTaskDelay(pdMS_TO_TICKS(100));
        
    
//     }
// }

// esp_err_t config_ADC() {
//     // Configuración inicial del ADC
//     adc_oneshot_unit_init_cfg_t init_config1 = {
//         .unit_id = ADC_UNIT_1,
//         .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
//     };

//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
//     // Configuración de los canales
//     adc_oneshot_chan_cfg_t config = {
//         .bitwidth = ADC_BITWIDTH_12,
//         .atten = ADC_ATTEN_DB_11,
//     };

//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POTENCIOMETRO_ADC_CHANNEL, &config));
//     return ESP_OK;
// }

// int get_ADC_value(adc_channel_t channel) {
//     int adc_raw;
//     // Leer valor ADC del canal especificado
//     adc_oneshot_read(adc1_handle, channel, &adc_raw);
//     return adc_raw;
// }

// esp_err_t mcpwm_config(){
//     ESP_LOGI(TAG, "Create timer and operator");
    
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // operator must be in the same group to the timer
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Connect timer and operator");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    
//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = SERVO_PULSE_GPIO,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator));

//     // set the initial compare value, so that the servo will spin to the center position
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

//     ESP_LOGI(TAG, "Set generator action on timer and compare event");
//     // go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Enable and start timer");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

//     return ESP_OK;
// }

// static inline uint32_t angle_to_compare(int angle){
//     return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
// }


/*
PRACTICA DE SENSORES DHT TEMPERATURA Y HUMEDAD

*/

/*
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

#define DHTPIN GPIO_NUM_26 // Pin GPIO donde está conectado el sensor DHT
#define BUZZER 25 // Pin GPIO para el buzzer

const char *TAG = "DHT22 SENSOR"; // Etiqueta para el registro de logs
gpio_num_t dht_gpio = DHTPIN; // Pin GPIO donde está conectado el sensor DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; // Tipo de sensor DHT (DHT11 o DHT22)

void app_main(){
    int16_t humidity = 0; // Variable para almacenar la humedad
    int16_t temperature = 0; // Variable para almacenar la temperatura

    while (true)
    {
        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK) {
            ESP_LOGI(TAG, "Humidity: %d%%, Temperature: %d°C", humidity / 10, temperature / 10); // Imprimir los valores de humedad y temperatura
            
        } 
        
        else {
            ESP_LOGE(TAG, "Failed to read data from DHT sensor"); // Imprimir error si no se puede leer el sensor
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 2 segundos antes de la siguiente lectura
    }

    

}
*/

/*
La siguiente práctica usa un sensor de humedad y temperatura DHT22, una ESP32, un motor DC, un servomotor y una fotoresistencia.
Al sobrepasar cierta temperatura con el sensor DHT22, el motor DC se activa y el servomotor gira 90 grados, simulando abrir una compuerta 
y activar una ventila que también activa una alarma.
A su vez la fotoresistencia nos permite aproximar el nivel de luz en el ambiente, pero no influye en el funcionamiento del resto del sistema.
*/

/*----------------- Librerías necesarias ------------------*/
// Librerías estándar de ESP32 para control de GPIO, FreeRTOS y logueo
//#include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// // Librería para el sensor DHT22
// #include "dht.h"
// // Librerías para control de ADC
// #include "driver/adc.h"
// #include "esp_adc_cal.h"
// // Librería para control de servomotores mediante PWM
// #include "driver/mcpwm_prelude.h"


// /* Configuración de los parámetros del servomotor */
// #define SERVO_MIN_PULSEWIDTH_US 500 /* Ancho de pulso mínimo en microsegundos */
// #define SERVO_MAX_PULSEWIDTH_US 2400 /* Ancho de pulso máximo en microsegundos */
// #define SERVO_MIN_DEGREE 0 /* Ángulo mínimo en grados */
// #define SERVO_MAX_DEGREE 180 /* Ángulo máximo en grados */
// #define SERVO_PULSE_GPIO 21 /* GPIO conectado a la señal PWM del servomotor */
// #define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* Resolución del temporizador en Hz (1us por tick) */
// #define SERVO_TIMEBASE_PERIOD 20000 /* Periodo del temporizador en ticks (20ms) */

// /* Configuración de pines */
// #define DHTPIN GPIO_NUM_26      // GPIO para el sensor DHT22
// #define BUZZER GPIO_NUM_25      // GPIO para el buzzer
// #define PHOTO_RESISTOR_ADC_CHANNEL ADC1_CHANNEL_7  // Canal ADC para la fotoresistencia (GPIO35 en ESP32)
// #define PHOTO_RESISTOR_GPIO GPIO_NUM_35  // GPIO conectado a la fotoresistencia
// #define IN1 17 // Pin GPIO para el motor DC
// #define IN2 16  // Pin GPIO para el motor DC

// #define LOW 0
// #define HIGH 1

// #define TEMP_THRESHOLD 30 // Umbral de temperatura en grados Celsius (multiplicado por 10 para evitar decimales)

// /* Definición de la etiqueta de log */
// const char *TAG = "SENSOR_TEST";
// gpio_num_t dht_gpio = DHTPIN;
// dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;  // Tipo de sensor DHT22/AM2301

// // Características del ADC
// static esp_adc_cal_characteristics_t *adc_chars;
// static const adc_atten_t atten = ADC_ATTEN_DB_11;
//static const adc_unit_t unit = ADC_UNIT_1;


/*----------------- Funciones ------------------*/

/* Función de mapeo de valores para la fotoresistencia (escala de 0 a 100) */
//long map(long x, long in_min, long in_max, long out_min, long out_max) {
//    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

/* Control del motor DC hacia adelante */
// esp_err_t forward() {
//     gpio_set_level(IN1, HIGH);
//     gpio_set_level(IN2, LOW);
//     return ESP_OK;
// }

// /* Detener el motor DC */
// esp_err_t stop() {
//     gpio_set_level(IN1, LOW);
//     gpio_set_level(IN2, LOW);
//     return ESP_OK;
// }

// /* Inicialización de pines para el motor DC */
// esp_err_t initialize_pin() {
//     gpio_reset_pin(IN1);
//     gpio_reset_pin(IN2);

//     gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
//     gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

//     gpio_set_level(IN1, LOW);
//     gpio_set_level(IN2, LOW);
//     return ESP_OK;
// }

// /* Activación del buzzer por un tiempo determinado */
// void beep_buzzer(int duration_ms) {
//     gpio_set_level(BUZZER, 1);     // Encender el buzzer
//     vTaskDelay(pdMS_TO_TICKS(duration_ms));  // Esperar el tiempo especificado
//     gpio_set_level(BUZZER, 0);     // Apagar el buzzer
// }

// /* Configuración del ADC para la lectura de la fotoresistencia */
// void init_adc() {
//     // Configurar ADC para lectura de 12 bits y canal adecuado
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     adc1_config_channel_atten(PHOTO_RESISTOR_ADC_CHANNEL, atten);
    
//     // Caracterización del ADC
//     adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//     esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, 1100, adc_chars);
// }

// /* Función para leer el valor del ADC de la fotoresistencia */
// uint32_t read_adc() {
//     return adc1_get_raw(PHOTO_RESISTOR_ADC_CHANNEL);
// }

// /*----------------- Configuración del Servomotor ------------------*/
// static const char *PWM_TAG = "PWM servo";

// // Definición de variables para el control de PWM
// mcpwm_timer_handle_t timer = NULL;
// mcpwm_oper_handle_t oper = NULL;
// mcpwm_cmpr_handle_t comparator = NULL;
// mcpwm_gen_handle_t generator = NULL;

// /* Configuración del servomotor PWM */
// esp_err_t mcpwm_config();
// static inline uint32_t angle_to_compare(int angle);  // Función para convertir ángulo a valor de comparación

// /*----------------- Función principal ------------------*/
// void app_main() {
//     ESP_ERROR_CHECK(initialize_pin());  // Inicializar pines del motor DC
//     mcpwm_config();  // Configuración del servomotor PWM
//     init_adc(); // Inicialización del ADC

//     gpio_reset_pin(BUZZER);  // Configuración del buzzer
//     gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
//     gpio_set_level(BUZZER, LOW);  // Apagar buzzer al inicio

//     float new_value_light = 0;
//     int16_t humidity = 0;
//     int16_t temperature = 0;

//     while (1) {
//         // Leer nivel de luz desde la fotoresistencia
//         new_value_light = map(read_adc(), 0, 4095, 0, 100);

//         // Leer datos del sensor DHT22 (humedad y temperatura)
//         if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) != ESP_OK) {
//             ESP_LOGE(TAG, "¡Error al leer el sensor DHT22!");  // Error si no se puede leer
//             continue;
//         }

//         if (temperature <= TEMP_THRESHOLD * 10) { // Comparar temperatura (multiplicada por 10)
//             // Si la temperatura es alta
//             ESP_LOGI(TAG, " TEMPERATURA NORMAL \n Humedad: %d%%, Temperatura: %d°C, Luz: %.1f%%, Extractor: Apagado, Ventila: Apgado",
//                 humidity / 10, temperature / 10, new_value_light);

//             ESP_ERROR_CHECK(stop());  // Detener motor
//             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(90)));  // Girar servomotor 90 grados
//             gpio_set_level(BUZZER, LOW);  // Activar alarma
//         } else {
//             // Si la temperatura es normal
//             ESP_LOGE(TAG, "RIESGO DE TEMPERATURA ALTA CUIDADO \n Humedad: %d%%, Temperatura: %d°C, Luz: %.1f%%, Extractor: Encendida, Ventila: Encendida",
//                 humidity / 10, temperature / 10, new_value_light);

//             ESP_ERROR_CHECK(forward());  // Activar motor
//             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));  // Girar servomotor a 0 grados
//             gpio_set_level(BUZZER, HIGH);  // Desactivar alarma
//         }

//         vTaskDelay(pdMS_TO_TICKS(1000));  // Esperar 1 segundo antes de la siguiente lectura
//     }
// }

// /*----------------- Configuración del Servomotor ------------------*/

// /* Función para configurar el servomotor con PWM */
// esp_err_t mcpwm_config(){
//     ESP_LOGI(PWM_TAG, "Crear temporizador y operador");
    
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // El operador debe estar en el mismo grupo que el temporizador
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Conectar temporizador y operador");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Crear comparador y generador desde el operador");
    
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = SERVO_PULSE_GPIO,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

//     // Establecer el valor de comparación inicial para que el servomotor se coloque en el centro
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

//     ESP_LOGI(TAG, "Establecer acción del generador sobre el evento del temporizador y del comparador");
//     // Acción de alta en evento vacío del temporizador
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // Acción de baja en evento de comparación
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Habilitar y empezar el temporizador");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

//     return ESP_OK;
// }

// /* Función que convierte el ángulo en valor de comparación para el PWM */
// static inline uint32_t angle_to_compare(int angle){
//     return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
//}

/////////////////////////////////////////////////////////////////////////////////////////


// Ejemplo de código para la práctica de comunicación UART en ESP32 por puerto serial
/*

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "freertos/queue.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

esp_err_t uart_initialize();
static void rx_task(void *arg);

const char *TAG = "RX UART event";
static QueueHandle_t uart_queue;

void app_main(){
    ESP_ERROR_CHECK(uart_initialize());

    xTaskCreate(rx_task, "uart_rx_task", TASK_MEMORY, NULL, configMAX_PRIORITIES-1, NULL);
}

esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static void rx_task(void *arg){
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uart_event_t rx_event;

    while (true) {
        if(xQueueReceive(uart_queue, &rx_event, portMAX_DELAY)){

            bzero(data, BUF_SIZE);

            switch(rx_event.type){
                case UART_DATA:
                    
                    uart_read_bytes(UART_PORT, data, rx_event.size, pdMS_TO_TICKS(100));
                    
                    uart_write_bytes(UART_PORT, (const char *) data, rx_event.size);
                    
                    uart_flush(UART_PORT);

                    ESP_LOGI(TAG, "Data received: %s", data);

                    break;

                default:
            }


        }
    }
}

*/


// Ejemplo de código para la práctica de sensor ultrasónico en ESP32

/*
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>

#define MAX_DISTANCE_CM 500 // 5m max

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define TRIGGER_GPIO 4
#define ECHO_GPIO 5
#else
#define TRIGGER_GPIO 17
#define ECHO_GPIO 16
#endif

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        uint32_t distance;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %ld cm\n", distance);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

*/

/*
// Ejemplo de código para la conexión serial entre dos ESP32 mediante UART
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2048

#define TXD_PIN GPIO_NUM_17  // TX del ESP
#define RXD_PIN GPIO_NUM_16  // RX del ESP

const char *TAG = "UART_COMM";
static QueueHandle_t uart_queue;

esp_err_t uart_initialize();
static void rx_task(void *arg);
static void tx_task(void *arg);

void app_main(){
    ESP_ERROR_CHECK(uart_initialize());

    // Crea la tarea de transmisión (envío cada segundo)
    xTaskCreate(tx_task, "uart_tx_task", TASK_MEMORY, NULL, 1, NULL);

    // Crea la tarea de recepción (escucha y muestra)
    xTaskCreate(rx_task, "uart_rx_task", TASK_MEMORY, NULL, configMAX_PRIORITIES - 1, NULL);
}

esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Instalar el driver con buffers y cola de eventos
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static void tx_task(void *arg){
    const char true_char = 'T';
    const char false_char = 'F';

    srand((unsigned int) time(NULL));  // Semilla para números aleatorios

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1 segundo

        int random_value = rand();
        char to_send = (random_value % 2 == 0) ? true_char : false_char;

        uart_write_bytes(UART_PORT, &to_send, 1);

        ESP_LOGI(TAG, "Enviado: %c", to_send);
    }
}

static void rx_task(void *arg){
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uart_event_t rx_event;

    while (true) {
        if (xQueueReceive(uart_queue, &rx_event, portMAX_DELAY)) {
            bzero(data, BUF_SIZE);

            switch(rx_event.type){
                case UART_DATA:
                    uart_read_bytes(UART_PORT, data, rx_event.size, pdMS_TO_TICKS(100));
                    data[rx_event.size] = '\0'; // aseguramos terminación

                    ESP_LOGI(TAG, "Recibido: %s", data);
                    break;

                default:
                    break;
            }
        }
    }
}
*/


// Ejemplo de código para mandar las señales de un sensor ultrasónico por UART

/*
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "driver/uart.h"
#include "string.h"

#define MAX_DISTANCE_CM 500 // 5m max   
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define TRIGGER_GPIO 4
#define ECHO_GPIO 5
#else
#define TRIGGER_GPIO 17
#define ECHO_GPIO 16
#endif

esp_err_t uart_initialize();
static void rx_task(void *arg);

const char *TAG = "RX UART event";
static QueueHandle_t uart_queue;

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);
    char buff_temp[6]; // Buffer para almacenar el valor de distancia como string

    while (true)
    {
        uint32_t distance;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else{
            sprintf(buff_temp, "%ld", distance); // Convertir el valor a string

            uart_write_bytes(UART_PORT_NUM, "", 2); // Enviar el valor por UART
            uart_write_bytes(UART_PORT_NUM, buff_temp, 6); // Enviar el valor de distancia
            uart_write_bytes(UART_PORT_NUM, "", 2); // Enviar el valor por UART
            uart_write_bytes(UART_PORT_NUM, "\n", 2); // Enviar el valor por UART
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }


        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}


static void rx_task(void *arg){
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uart_event_t rx_event;

    while (true) {
        if(xQueueReceive(uart_queue, &rx_event, portMAX_DELAY)){

            bzero(data, BUF_SIZE);

            switch(rx_event.type){
                case UART_DATA:
                    
                    uart_read_bytes(UART_PORT_NUM, data, rx_event.size, pdMS_TO_TICKS(100));
                    
                    uart_write_bytes(UART_PORT_NUM, (const char *) data, rx_event.size);
                    
                    uart_flush(UART_PORT_NUM);

                    break;

                default:
            }


        }
    }
}

void app_main()
{
    // Inicializar UART
    ESP_ERROR_CHECK(uart_initialize());

    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}

*/

// Segundo intento para la comunicación de el sensor ultrasónico por UART
/*
#include <stdio.h>
#include <string.h>  // ¡Añade esta línea para strlen()!
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "driver/uart.h"

#define MAX_DISTANCE_CM 400
#define TRIGGER_GPIO 17
#define ECHO_GPIO 16
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

void ultrasonic_task(void *pvParameters) {
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };
    ultrasonic_init(&sensor);

    char buffer[16];

    while (true) {
        uint32_t distance;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);

        if (res == ESP_OK) {
            snprintf(buffer, sizeof(buffer), "%ld", distance); // Anadir  los delimitadores (diagonal,asterisco)-(asterisco-diagonal)  al principio y al final, esta parte no los incluye pues está comentada
            uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));  // Ahora strlen() está definido
            uart_flush(UART_PORT_NUM);
        } else {
            printf("Error: %s\n", esp_err_to_name(res));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0);

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);
}
    */


/*
// El siguiente código es para la comunicación entre un sensor de temperatura y humedad DHT22 y un ESP32 mediante UART.
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"
#include "driver/uart.h"
#include "string.h"
#include "freertos/queue.h"

#define DHTPIN GPIO_NUM_26
#define BUZZER 25
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2048
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

static const char *TAG = "DHT22 SENSOR";

void dht_task(void *pvParameters){
    int16_t humidity = 0;
    int16_t temperature = 0;
    char buffer[16];
    
    gpio_num_t dht_gpio = DHTPIN;
    dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;
    
    while (true) {
        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK) {
            // Correctly format the data with proper decimal handling
            float hum = humidity / 10.0f;
            float temp = temperature / 10.0f;
            
            snprintf(buffer, sizeof(buffer), "%.1f,%.1f", hum, temp);
            ESP_LOGI(TAG, "Humidity: %.1f%%, Temperature: %.1fC", hum, temp);
            
            // Send formatted data via UART
           
            uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
            
            // Add newline for better readability
            uart_write_bytes(UART_PORT_NUM, "\n", 1);
        } else {
            ESP_LOGE(TAG, "Error reading DHT sensor");
            uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0);

    xTaskCreate(dht_task, "dht_task", TASK_MEMORY, NULL, 5, NULL);
}
*/

// Código para conectar la ESP32 por WiFi a thingSpeak 
/*
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_http_client.h"
#include "connect_wifi.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

#define DHTPIN GPIO_NUM_26 // Pin GPIO donde está conectado el sensor DHT


gpio_num_t dht_gpio = DHTPIN; // Pin GPIO donde está conectado el sensor DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; // Tipo de sensor DHT (DHT11 o DHT22)

static const char *TAG = "ThingSpeak"; // Etiqueta para el registro de logs

char *api_key = "3WW22217MFHYOMZN";

void send_data_to_thingspeak(void *pvParameters){
    char *thingspeak_base_url = "https://api.thingspeak.com/update";
    char query_params[100];
    char full_url[200];  // Suficiente espacio para URL base + parámetros
    esp_err_t err;

    int16_t humidity = 0;
    int16_t temperature = 0;

    esp_http_client_config_t config = {
        .url = thingspeak_base_url,  // Configura la URL base aquí
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

    while (true){
        if(dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK){
            ESP_LOGI(TAG, "Humidity: %d%%, Temperature: %d°C", humidity / 10, temperature / 10);
            
            // Construye los parámetros de consulta
            snprintf(query_params, sizeof(query_params), "?api_key=%s&field1=%d&field2=%d", api_key, humidity / 10, temperature / 10);
            
            // Construye la URL completa
            snprintf(full_url, sizeof(full_url), "%s%s", thingspeak_base_url, query_params);
            
            ESP_LOGI(TAG, "Full URL: %s", full_url);
            
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Configura la URL completa
            esp_http_client_set_url(client, full_url);

            err = esp_http_client_perform(client);

            if (err == ESP_OK){
                int status_code = esp_http_client_get_status_code(client);
                if (status_code == 200){
                    ESP_LOGI(TAG, "Message sent Successfully");
                }
                else{
                    ESP_LOGE(TAG, "Message failed, status code: %d", status_code);
                }
            }
            else{
                ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
            }
        }
        else {
            ESP_LOGE(TAG, "Failed to read DHT sensor");
        }
        
        vTaskDelay(pdMS_TO_TICKS(30000));  // Espera 30 segundos entre lecturas
    }
    
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}


void app_main(void){

    
	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	connect_wifi();

	if (wifi_connect_status){
		xTaskCreate(&send_data_to_thingspeak,  
                    "send_data_to_thingspeak",
                    1024 * 3,                 
                    NULL,                     
                    6,                         
                    NULL);                     
	}
}
*/

/*
// Codigo para leer campos en un canal de thingspeak y mover un servomotor o iniciar uun motor 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_http_client.h"
#include "connect_wifi.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
 // Librería para control de servomotores mediante PWM
#include "driver/mcpwm_prelude.h"

#include "cJSON.h"

// / Configuración de los parámetros del servomotor /
#define SERVO_MIN_PULSEWIDTH_US 500 / Ancho de pulso mínimo en microsegundos /
#define SERVO_MAX_PULSEWIDTH_US 2400 / Ancho de pulso máximo en microsegundos /
#define SERVO_MIN_DEGREE 0 / Ángulo mínimo en grados /
#define SERVO_MAX_DEGREE 180 / Ángulo máximo en grados /
#define SERVO_PULSE_GPIO 21 / GPIO conectado a la señal PWM del servomotor /
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 / Resolución del temporizador en Hz (1us por tick) /
#define SERVO_TIMEBASE_PERIOD 20000 / Periodo del temporizador en ticks (20ms) /


 / Configuración de pines /
#define IN1 17 // Pin GPIO para el motor DC
#define IN2 16  // Pin GPIO para el motor DC

#define HIGH 1
#define LOW 0

#define TEMP_THRESHOLD 30
#define HUMIDITY_THRESHOLD 30

static const char *TAG = "ThingSpeak"; // Etiqueta para el registro de logs

char *api_key = "YC8JQRFKU2IFTX37"; // API KEY de lectura


/ Control del motor DC hacia adelante /
esp_err_t forward() {
    gpio_set_level(IN1, HIGH);
    gpio_set_level(IN2, LOW);
    return ESP_OK;
}

 / Detener el motor DC /
esp_err_t stop() {
    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, LOW);
    return ESP_OK;
}

// / Inicialización de pines para el motor DC /
esp_err_t initialize_pin() {
    gpio_reset_pin(IN1);
    gpio_reset_pin(IN2);
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, LOW);
    return ESP_OK;
}

// /----------------- Configuración del Servomotor ------------------/
static const char *PWM_TAG = "PWM servo";

// Definición de variables para el control de PWM
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

// / Configuración del servomotor PWM /
esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle);


void receive_data_fron_thingspeak(void *pvParameters) {
    ESP_LOGI(TAG, "Starting ThingSpeak fetch task");

    char buffer[1024]; // Buffer for reading response
    char url[256];     // Buffer for dynamic URL

    // Build the URL with your channel ID and API key
    snprintf(url, sizeof(url),"https://api.thingspeak.com/channels/2941223/feeds.json?api_key=%s&results=2", api_key);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
        .skip_cert_common_name_check = true,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .buffer_size = 1024,  // Increased buffer size
        .user_data = NULL,
    };

    // Initialize the HTTP client
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI(TAG, "Fetching data from ThingSpeak...");
        ESP_LOGI(TAG, "Using URL: %s", url);
        
        // Open the HTTP connection
        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        // Fetch headers
        int content_length = esp_http_client_fetch_headers(client);
        ESP_LOGI(TAG, "Content length: %d", content_length);

        if (content_length > 0 || esp_http_client_is_chunked_response(client)) {
            int total_read = 0;
            int read_len;

            // Read response in chunks
            while ((read_len = esp_http_client_read(client, 
                buffer + total_read, 
                sizeof(buffer) - total_read - 1)) > 0 ){
                total_read += read_len;
                if (total_read >= sizeof(buffer) - 1) {
                    ESP_LOGW(TAG, "Buffer full, truncating response");
                    break;
                }
            }

            if (total_read > 0) {
                buffer[total_read] = '\0'; // Null-terminate
                ESP_LOGI(TAG, "Received %d bytes", total_read);
                ESP_LOGD(TAG, "Raw response: %s", buffer);

                // Parse JSON response
                cJSON *root = cJSON_Parse(buffer);
                if (root) {
                    // Check channel info
                    cJSON *channel = cJSON_GetObjectItem(root, "channel");
                    if (channel) {
                        ESP_LOGI(TAG, "Channel: %s", 
                                cJSON_GetObjectItem(channel, "name")->valuestring);
                    }

                    // Process feeds array
                    cJSON *feeds = cJSON_GetObjectItem(root, "feeds");
                    if (feeds && cJSON_IsArray(feeds)) {
                        int feed_count = cJSON_GetArraySize(feeds);
                        ESP_LOGI(TAG, "Found %d feed entries", feed_count);

                        // Get the latest entry (first in array)
                        if (feed_count > 0) {
                            cJSON *latest_feed = cJSON_GetArrayItem(feeds, 0);
                            cJSON *field1 = cJSON_GetObjectItem(latest_feed, "field1");
                            cJSON *field2 = cJSON_GetObjectItem(latest_feed, "field2");
                            
                            if (field1 && cJSON_IsString(field1)) {
                                float value_f1 = atof(field1->valuestring);
                                ESP_LOGI(TAG, "Ultima lectura de temperatura: %.2f", value_f1);
                            
                                // Add your control logic here
                                if (value_f1 > TEMP_THRESHOLD) {
                                    ESP_LOGE(TAG, "La temperatura es muy alta!");
                                        ESP_ERROR_CHECK(forward());
                                        // Si el valor de la humedad pasa cierto rango, se mueve el servo
                                } else {
                                    ESP_LOGI(TAG, "La temperatura normal");
                                    ESP_ERROR_CHECK(stop());
                                    // Si la humedad regresa a un nivel normal se vuelve a mover
                                }
                            } else {
                                ESP_LOGE(TAG, "Invalid field1 in feed");
                            }

                            if(field1 && cJSON_IsString(field2)){
                                float value_f2 = atof(field2->valuestring);
                                ESP_LOGI(TAG, "Ultima lectura de humedad: %.2f", value_f2);

                                if(value_f2 > HUMIDITY_THRESHOLD){
                                    ESP_LOGE(TAG, "Humedad alta auxilio");
                                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(90)));
                                }

                                else{
                                    ESP_LOGI(TAG, "Humedad normal :)");
                                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));
                                }
                                
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "No feeds found in response");
                    }
                    cJSON_Delete(root);
                } else {
                    const char *error_ptr = cJSON_GetErrorPtr();
                    if (error_ptr) {
                        ESP_LOGE(TAG, "JSON parse error before: %s", error_ptr);
                    }
                    ESP_LOGE(TAG, "Failed to parse JSON response");
                }
            } else {
                ESP_LOGE(TAG, "No data received");
            }
        } else {
            ESP_LOGE(TAG, "Empty response from server");
        }

        // Close connection and wait before next request
        esp_http_client_close(client);
        vTaskDelay(pdMS_TO_TICKS(10000)); // 30 seconds delay
    }

    // Cleanup (unreachable in this case)
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}



void app_main(void){
    ESP_ERROR_CHECK(initialize_pin()); 
    mcpwm_config();
    

	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	connect_wifi();

	if (wifi_connect_status){
		xTaskCreate(&receive_data_fron_thingspeak,  / Function that implements the task. /
                    "recieve_data_from_thingspeak", / Text name for the task. /
                    8192,                  / Stack size in words, not bytes. /
                    NULL,                      / Parameter passed into the task. /
                    6,                         / Priority at which the task is created. /
                    NULL);                     / Used to pass out the created task's handle. /
	}
}


// /----------------- Configuración del Servomotor ------------------/

// / Función para configurar el servomotor con PWM /
esp_err_t mcpwm_config(){
    ESP_LOGI(PWM_TAG, "Crear temporizador y operador");
    
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // El operador debe estar en el mismo grupo que el temporizador
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Conectar temporizador y operador");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Crear comparador y generador desde el operador");
    
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Establecer el valor de comparación inicial para que el servomotor se coloque en el centro
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Establecer acción del generador sobre el evento del temporizador y del comparador");
//     // Acción de alta en evento vacío del temporizador
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // Acción de baja en evento de comparación
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Habilitar y empezar el temporizador");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

// / Función que convierte el ángulo en valor de comparación para el PWM /
static inline uint32_t angle_to_compare(int angle){
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
    */


    // El siguiente código lee presencia usando un sensor infrarojo y ultra sónico, al detectar presencia dos servomotores se mueven haciendo el primero
    // un barrido de 180 grados y el segundo un barrido de 40 a 135 grados, al finalizar el barrido se apagan los motores

    /*
    #include <stdio.h>
#include <string.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include <ultrasonic.h>

// Configuración del servo
#define SERVO_HORIZONTAL_GPIO 16
#define SERVO_VERTICAL_GPIO 17
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

// Rango de movimiento
#define HORIZONTAL_INITIAL 45
#define HORIZONTAL_FINAL 135
#define VERTICAL_INITIAL 0  
#define VERTICAL_FINAL 180
#define MOVEMENT_STEP 5
#define MOVEMENT_DELAY_MS 1000

// Sensor ultrasónico
#define TRIGGER_GPIO 18
#define ECHO_GPIO 19
#define MAX_DISTANCE_CM 400
#define DiSTANCE_THRESHOLD_MAX 10
#define DiSTANCE_THRESHOLD_MIN 5

// LED's
#define Led_Rojo 27
#define Led_Verde 26
#define OFF 0
#define ON 1

// Sensor IR (fototransistor)
#define FOTOTRANSISTOR_ADC_CHANNEL ADC_CHANNEL_3   // ADC1_3
#define UMBRAL_FOTOTRANSISTOR 3500

static adc_oneshot_unit_handle_t adc_handle;

// MCPWM handles
mcpwm_timer_handle_t timer_horizontal = NULL;
mcpwm_oper_handle_t oper_horizontal = NULL;
mcpwm_cmpr_handle_t comparator_horizontal = NULL;
mcpwm_gen_handle_t generator_horizontal = NULL;

mcpwm_timer_handle_t timer_vertical = NULL;
mcpwm_oper_handle_t oper_vertical = NULL;
mcpwm_cmpr_handle_t comparator_vertical = NULL;
mcpwm_gen_handle_t generator_vertical = NULL;

// Funciones
esp_err_t configure_servo(mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, 
                         mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator, 
                         int gpio_num);

static inline uint32_t angle_to_compare(int angle);
void move_servo(mcpwm_cmpr_handle_t comparator, int angle);
void scanning_movement();
void configure_pin();
void configure_adc();
int read_adc(adc_channel_t channel);

void app_main(void) {
    // Configurar pines y ADC
    configure_pin();
    configure_adc();

    // Configurar servos
    configure_servo(&timer_horizontal, &oper_horizontal, 
                    &comparator_horizontal, &generator_horizontal, 
                    SERVO_HORIZONTAL_GPIO);

    configure_servo(&timer_vertical, &oper_vertical, 
                    &comparator_vertical, &generator_vertical, 
                    SERVO_VERTICAL_GPIO);

    // Inicializar sensor ultrasónico
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };
    ultrasonic_init(&sensor);

    // Posición inicial
    move_servo(comparator_horizontal, HORIZONTAL_INITIAL);
    move_servo(comparator_vertical, VERTICAL_INITIAL);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (true) {
        int fototransistor_value = read_adc(FOTOTRANSISTOR_ADC_CHANNEL);
        int fototransistor_bin = (fototransistor_value > UMBRAL_FOTOTRANSISTOR) ? 1 : 0;
        printf("Fototransistor: %d\n", fototransistor_bin);

        uint32_t distance = 0;
        bool distance_ok = false;
        
        if (fototransistor_bin == 1) {
            esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
            
            if (res == ESP_OK) {
                printf("Distancia: %ld cm\n", distance);
                distance_ok = (distance == DiSTANCE_THRESHOLD_MAX);
                
                if (distance_ok) {
                    gpio_set_level(Led_Verde, ON);
                    gpio_set_level(Led_Rojo, OFF);
                } else {
                    gpio_set_level(Led_Rojo, ON);
                    gpio_set_level(Led_Verde, OFF);
                }
            } else {
                printf("Error midiendo distancia: %s\n", esp_err_to_name(res));
                gpio_set_level(Led_Rojo, ON);
                gpio_set_level(Led_Verde, OFF);
            }
        }

        // Verificar ambas condiciones antes de iniciar el barrido
        if (fototransistor_bin == 1) {
            scanning_movement();  // Solo se ejecuta cuando ambas condiciones son verdaderas
        } else {
            // No se cumplen las condiciones: volver a posición inicial
            move_servo(comparator_horizontal, HORIZONTAL_INITIAL);
            move_servo(comparator_vertical, VERTICAL_INITIAL);
            gpio_set_level(Led_Rojo, ON);
            gpio_set_level(Led_Verde, OFF);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t configure_servo(mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, 
                         mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator, 
                         int gpio_num) {
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, *timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(*oper, &comparator_config, comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(*oper, &generator_config, generator));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        *generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        *generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(*timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

static inline uint32_t angle_to_compare(int angle) {
    return angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 180 + SERVO_MIN_PULSEWIDTH_US;
}

void move_servo(mcpwm_cmpr_handle_t comparator, int angle) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
}


void scanning_movement() {
    static int current_horizontal = HORIZONTAL_INITIAL;
    static int current_vertical = VERTICAL_INITIAL;
    static bool moving_forward = true;

    // Movimiento horizontal
    if (moving_forward) {
        current_horizontal += MOVEMENT_STEP;
        if (current_horizontal >= HORIZONTAL_FINAL) {
            current_horizontal = HORIZONTAL_FINAL;
            moving_forward = false;
        }
    } else {
        current_horizontal -= MOVEMENT_STEP;
        if (current_horizontal <= HORIZONTAL_INITIAL) {
            current_horizontal = HORIZONTAL_INITIAL;
            moving_forward = true;

            if (current_vertical < VERTICAL_FINAL) {
                current_vertical += MOVEMENT_STEP;
                if (current_vertical > VERTICAL_FINAL) {
                    current_vertical = VERTICAL_FINAL;
                }
                move_servo(comparator_vertical, current_vertical);
            }
        }
    }

    move_servo(comparator_horizontal, current_horizontal);
    printf("Posición actual - Horizontal: %d°, Vertical: %d°\n", current_horizontal, current_vertical);
    vTaskDelay(pdMS_TO_TICKS(MOVEMENT_DELAY_MS));
}

void configure_pin() {
    gpio_reset_pin(Led_Rojo);
    gpio_reset_pin(Led_Verde);
    gpio_set_direction(Led_Rojo, GPIO_MODE_OUTPUT);
    gpio_set_direction(Led_Verde, GPIO_MODE_OUTPUT);
    gpio_set_level(Led_Rojo, OFF);
    gpio_set_level(Led_Verde, OFF);
}

void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,  
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,  
        .atten = ADC_ATTEN_DB_11,    
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, FOTOTRANSISTOR_ADC_CHANNEL, &config));
}

int read_adc(adc_channel_t channel) {
    int adc_value = 0;
    if (adc_oneshot_read(adc_handle, channel, &adc_value) != ESP_OK) {
        printf("Error leyendo ADC en canal %d\n", channel);
    }
    return adc_value;
}

*/

// El siguiente codigo es para mandar datso desde un joystick a un broker mqtt, en este caso a un broker local, el cual se ejecuta en una Raspberry Pi
/*
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ESP_WIFI_SSID "INVITADOS"
#define ESP_WIFI_PASS "invitados00--"

#define MQTT_TOPIC_X "esp32/joystick/x"
#define MQTT_TOPIC_Y "esp32/joystick/y"

#define JOYSTICK_X ADC_CHANNEL_0 // GPIO36
#define JOYSTICK_Y ADC_CHANNEL_3 // GPIO39
#define BUTTON GPIO_NUM_34

static const char *TAG = "MQTT_JOYSTICK";

esp_mqtt_client_handle_t client = NULL;
uint32_t MQTT_CONNECTED = 0;

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            MQTT_CONNECTED = 1;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            MQTT_CONNECTED = 0;
            break;
        default:
            break;
    }
}

// MQTT start
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://172.31.99.55:1883",
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// Wi-Fi connection
static void wifi_init(void) {
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "Wi-Fi init finished.");
}

// Joystick publishing task
void pub_joystick_task(void *params) {
    int raw_x = 0, raw_y = 0;
    int voltage_x = 0, voltage_y = 0;
    int button_state;

    while (true) {
        adc_oneshot_read(adc_handle, JOYSTICK_X, &raw_x);
        adc_oneshot_read(adc_handle, JOYSTICK_Y, &raw_y);

        adc_cali_raw_to_voltage(adc_cali_handle, raw_x, &voltage_x);
        adc_cali_raw_to_voltage(adc_cali_handle, raw_y, &voltage_y);

        button_state = gpio_get_level(BUTTON);

        if (MQTT_CONNECTED) {
            char msg[20];

            sprintf(msg, "%d", voltage_x);
            esp_mqtt_client_publish(client, MQTT_TOPIC_X, msg, 0, 0, 0);

            sprintf(msg, "%d", voltage_y);
            esp_mqtt_client_publish(client, MQTT_TOPIC_Y, msg, 0, 0, 0);

            
        }

        ESP_LOGI(TAG, "X: %d mV, Y: %d mV, Button: %s", voltage_x, voltage_y, button_state ? "Not Pressed" : "Pressed");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();  // ← aquí se conecta al WiFi

    // Inicializar botón
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON);

    // Configuración ADC
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_X, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_Y, &chan_cfg));

    // Calibración ADC
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle));

    // Iniciar MQTT y tarea
    mqtt_app_start();
    xTaskCreate(pub_joystick_task, "pub_joystick_task", 4096, NULL, 5, NULL);
}
    */

// El siguiente codigo es para leer datos desde un broker mqtt para poder mover dos motores DC.
// Al recibir un mensaje con el valor del voltaje de un joystick se mueve en el sentido indicado por el joystick/*
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "ultrasonic.h"

#define MOTOR_IN1 16
#define MOTOR_IN2 17
#define MOTOR_IN3 18
#define MOTOR_IN4 19
#
#define TRIGGER_GPIO 33
#define ECHO_GPIO 32
#define DISTANCIA_MAX 400
#define DISTANCIA_PARO 5

#define WIFI_SSID "INFINITUM4EAF"
#define WIFI_PASS "HNn5RHDFV2"
#define MQTT_BROKER_URI "mqtt://192.168.1.86:1883"

#define MQTT_SUB_X "esp32_2/joystick/x"
#define MQTT_SUB_Y "esp32_2/joystick/y"
#define MQTT_SUB_BUTTON "esp32_2/joystick/button"

static const char *TAG = "CARRO_ROSA";

esp_mqtt_client_handle_t client;
uint32_t MQTT_CONNECTED = 0;
int joystick_x = 0;
int joystick_y = 0;
int joystick_button = 0;

ultrasonic_sensor_t sensor = {
    .trigger_pin = TRIGGER_GPIO,
    .echo_pin = ECHO_GPIO
};

// Inicializar motores
void motores_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2) | (1ULL << MOTOR_IN3) | (1ULL << MOTOR_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    gpio_set_level(MOTOR_IN3, 0);
    gpio_set_level(MOTOR_IN4, 0);
}

// Medir distancia con el sensor ultrasónico
float medir_distancia_cm() {
    uint32_t distance_cm = 0;
    esp_err_t res = ultrasonic_measure_cm(&sensor, DISTANCIA_MAX, &distance_cm);
    if (res == ESP_OK) {
        return (float)distance_cm;
    }
    return -1.0;
}

// Controlar motores con joystick y distancia
void control_motores(int x, int y) {
    if (!MQTT_CONNECTED) {
        ESP_LOGW(TAG, "MQTT no conectado, motores detenidos.");
        goto detener;
    }

    float distancia = medir_distancia_cm();
    if (distancia >= 0 && distancia <= DISTANCIA_PARO) {
        ESP_LOGW(TAG, "Objeto a %.2f cm. Deteniendo.", distancia);
        goto detener;
    }

    if (x > 2000) {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 0);
        ESP_LOGI(TAG, "Derecha");
    } else if (x < 1000) {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 1);
        gpio_set_level(MOTOR_IN4, 0);
        ESP_LOGI(TAG, "Izquierda");
    } else if (y > 2000) {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 1);
        gpio_set_level(MOTOR_IN4, 0);
        ESP_LOGI(TAG, "Adelante");
    } else if (y < 1000) {
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 1);
        ESP_LOGI(TAG, "Atrás");
    } else {
detener:
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0);
        gpio_set_level(MOTOR_IN4, 0);
        ESP_LOGI(TAG, "Detener");
    }
}

// Manejo de eventos MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            MQTT_CONNECTED = 1;
            esp_mqtt_client_subscribe(client, MQTT_SUB_X, 0);
            esp_mqtt_client_subscribe(client, MQTT_SUB_Y, 0);
            esp_mqtt_client_subscribe(client, MQTT_SUB_BUTTON, 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT desconectado");
            MQTT_CONNECTED = 0;
            break;

        case MQTT_EVENT_DATA: {
            char topic[event->topic_len + 1];
            char data[event->data_len + 1];

            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            memcpy(data, event->data, event->data_len);
            data[event->data_len] = '\0';

            if (strcmp(topic, MQTT_SUB_X) == 0) {
                joystick_x = atoi(data);
                ESP_LOGI(TAG, "X = %d", joystick_x);
            } else if (strcmp(topic, MQTT_SUB_Y) == 0) {
                joystick_y = atoi(data);
                ESP_LOGI(TAG, "Y = %d", joystick_y);
            } else if (strcmp(topic, MQTT_SUB_BUTTON) == 0) {
                joystick_button = atoi(data);
                ESP_LOGI(TAG, "Botón = %d", joystick_button);
            }

            control_motores(joystick_x, joystick_y);
            break;
        }

        default:
            break;
    }
}

// Conexión Wi-Fi
void wifi_init(void) {
    ESP_LOGI(TAG, "Conectando a Wi-Fi...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// Inicio de MQTT
void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motores_init();
    ultrasonic_init(&sensor);
    wifi_init();
    mqtt_app_start();

  

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}