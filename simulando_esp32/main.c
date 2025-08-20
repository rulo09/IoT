/*
Raúl Martínez
28-02-2025

Este codigo simula cinco ESP32 los cuale toman los datos de presión (PA), temperatura (°C), 
humedad (%) y luz (lumens) cada diez minutos, retornando su promedio y su desviación estándar. 

El código busca usar y consolidar los fundamentos de programación en lenguaje C, sobre todo de
arreglos y estructuras, pero también ciclos for, funciones, dividir el codigo en archivos, generar numeros aleatorios, entre otras cosas. 

*/


#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "esp32.h"
#include "desviacionYpromedio.h"

int main() {
    ESP32 ESPS[N]; // Crear un arreglo de 5 ESP32
    clock_t inicio, fin;
    double tiempo_transcurrido = 0;

    inicio = clock(); // Tiempo inicial

    while (1) { // Bucle infinito para actualizar datos cada minuto
        fin = clock(); // Tiempo actual
        tiempo_transcurrido = (double)(fin - inicio) / CLOCKS_PER_SEC; // Tiempo en segundos

        if (tiempo_transcurrido >= 60.0) { // Si ha pasado 1 minuto
            inicio = clock(); // Reiniciar el tiempo inicial

            // Generar datos para los ESP32
            if (generarESP(ESPS)) {
                // Calcular y almacenar la media y desviación estándar
                desviacion_media_ESPS(ESPS);

                // Imprimir los datos de cada ESP32
                for (int i = 0; i < N; i++) {
                    printf("ESP32 ID: %d\n", ESPS[i].ID);

                    // Imprimir los primeros 5 valores de cada sensor
                    printf("Presión: ");
                    for (int j = 0; j < 5; j++) {
                        printf("%.2f ", ESPS[i].presion[j]);
                    }
                    printf("\n");

                    printf("Temperatura: ");
                    for (int j = 0; j < 5; j++) {
                        printf("%.2f ", ESPS[i].temp[j]);
                    }
                    printf("\n");

                    printf("Humedad: ");
                    for (int j = 0; j < 5; j++) {
                        printf("%.2f ", ESPS[i].hum[j]);
                    }
                    printf("\n");

                    printf("Luminosidad: ");
                    for (int j = 0; j < 5; j++) {
                        printf("%.2f ", ESPS[i].lux[j]);
                    }
                    printf("\n");

                    // Imprimir la media y desviación estándar
                    printf("Media de Presión: %.2f\n", ESPS[i].presion[100]);
                    printf("Desviación Estándar de Presión: %.2f\n", ESPS[i].presion[101]);

                    printf("Media de Temperatura: %.2f\n", ESPS[i].temp[100]);
                    printf("Desviación Estándar de Temperatura: %.2f\n", ESPS[i].temp[101]);

                    printf("Media de Humedad: %.2f\n", ESPS[i].hum[100]);
                    printf("Desviación Estándar de Humedad: %.2f\n", ESPS[i].hum[101]);

                    printf("Media de Luminosidad: %.2f\n", ESPS[i].lux[100]);
                    printf("Desviación Estándar de Luminosidad: %.2f\n", ESPS[i].lux[101]);

                    printf("\n"); // Separador entre ESP32
                }
            } else {
                fprintf(stderr, "Error al generar datos para los ESP32.\n");
                return 1;
            }
        }
    }

    return 0;
}