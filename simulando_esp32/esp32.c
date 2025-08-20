#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h> // Para usar memset

#include "esp32.h"

// Función para generar números flotantes aleatorios
float aleatorios_flotantes(float min, float max) {
    float escala = rand() / (float)RAND_MAX;
    return min + escala * (max - min);
}

// Función para inicializar la estructura ESP32 con ceros
void inicializarESP(ESP32 *esp) {
    memset(esp, 0, sizeof(ESP32));
}

// Función para generar los datos de los ESP32
int generarESP(ESP32 ESPS[]) {
    srand(time(NULL)); // Inicializar la semilla de números aleatorios (solo una vez)

    for (int i = 0; i < N; i++) {
        inicializarESP(&ESPS[i]); // Inicializar la estructura para evitar datos basura
        ESPS[i].ID = 1 + i; // Asignar un ID único a cada ESP32 (1, 2, 3, ...)

        for (int j = 0; j < DATOS_POR_ESP; j++) { // Usar DATOS_POR_ESP en lugar de size
            // Generar datos aleatorios para cada sensor
            ESPS[i].presion[j] = aleatorios_flotantes(88000, 91000); // Presión
            ESPS[i].temp[j] = aleatorios_flotantes(10, 30); // Temperatura
            ESPS[i].hum[j] = aleatorios_flotantes(30, 60); // Humedad
            ESPS[i].lux[j] = aleatorios_flotantes(6000, 120000); // Luminosidad
        }
    }

    return 1; // Retornar un valor indicando que la generación fue exitosa
}