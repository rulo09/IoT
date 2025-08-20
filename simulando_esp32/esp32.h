#ifndef ESP32_H
#define ESP32_H

#include <string.h> // Para usar memset

#define N 5 // Número de ESP32
#define DATOS_POR_ESP 102 // Número de datos que cada ESP32 almacena (100 datos + 2 posiciones extra)
#define size 100

// Definición de la estructura ESP32
typedef struct {
    int ID; // Identificador único del ESP32
    float presion[DATOS_POR_ESP]; // Datos de presión
    float temp[DATOS_POR_ESP]; // Datos de temperatura
    float hum[DATOS_POR_ESP]; // Datos de humedad
    float lux[DATOS_POR_ESP]; // Datos de luminosidad
} ESP32;

// Declaraciones de funciones
float aleatorios_flotantes(float min, float max); // Genera números flotantes aleatorios
int generarESP(ESP32 ESPS[]); // Genera datos para los ESP32
void inicializarESP(ESP32 *esp); // Declaración (prototipo) de la función

#endif // ESP32_H