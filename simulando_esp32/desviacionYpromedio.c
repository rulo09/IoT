#include <stdio.h>
#include <math.h> // Para usar sqrt

#include "desviacionYpromedio.h"
#include "esp32.h"

// Función para calcular la media de un arreglo de floats
float calcular_media(float datos[]) {
    float suma = 0;

    for (int i = 0; i < size; i++) {
        suma += datos[i];
    }

    return suma / size; // Retorna la media
}

// Función para calcular la desviación estándar de un arreglo de floats
float desvest(float datos[]) {
    float media = calcular_media(datos); // Calcula la media
    float suma = 0;

    for (int i = 0; i < size; i++) {
        suma += (datos[i] - media) * (datos[i] - media); // Suma de cuadrados de las diferencias
    }

    return sqrt(suma / size); // Retorna la desviación estándar
}

// Función para calcular y guardar la media y desviación estándar en las posiciones 101 y 102
void desviacion_media_ESPS(ESP32 ESPS[]) {
    int media_index = 101; // Posición para guardar la media
    int desviacion_index = 102; // Posición para guardar la desviación estándar

    for (int i = 0; i < N; i++) {
        // Calcular y guardar para presión
        ESPS[i].presion[media_index] = calcular_media(ESPS[i].presion);
        ESPS[i].presion[desviacion_index] = desvest(ESPS[i].presion);

        // Calcular y guardar para humedad
        ESPS[i].hum[media_index] = calcular_media(ESPS[i].hum);
        ESPS[i].hum[desviacion_index] = desvest(ESPS[i].hum);

        // Calcular y guardar para luminosidad
        ESPS[i].lux[media_index] = calcular_media(ESPS[i].lux);
        ESPS[i].lux[desviacion_index] = desvest(ESPS[i].lux);

        // Calcular y guardar para temperatura
        ESPS[i].temp[media_index] = calcular_media(ESPS[i].temp);
        ESPS[i].temp[desviacion_index] = desvest(ESPS[i].temp);
    }
}