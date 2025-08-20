#ifndef DESVEST_H
#define DESVEST_H

#include "esp32.h" // Incluir el archivo de cabecera que define ESP32 y N

/**
 * Calcula la desviación estándar de un conjunto de datos.
 * @param datos Arreglo de valores flotantes.
 * @return La desviación estándar de los datos.
 */
float desvest(float datos[]);

/**
 * Calcula la media (promedio) de un conjunto de datos.
 * @param datos Arreglo de valores flotantes.
 * @return La media de los datos.
 */
float calcular_media(float datos[]);

/**
 * Calcula la desviación media de los datos de presión, temperatura, humedad y luminosidad
 * para cada ESP32 en el arreglo ESPS.
 * @param ESPS Arreglo de estructuras ESP32.
 */
void desviacion_media_ESPS(ESP32 ESPS[]);

#endif // DESVEST_H