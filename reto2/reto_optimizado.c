#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>

// Constantes
#define MAX_ATTEMPTS 3 // Número máximo de intentos para ingresar datos válidos

// Prototipos de funciones
int criba();
bool es_primo(int n);
int generar_aleatorios();
int menu();
void limpiar_buffer();

// Función principal
int main() {
    srand(time(NULL)); // Inicializar la semilla para números aleatorios
    while (true) {
        if (menu() == 4) {
            printf("Adios! :(\n");
            break;
        }
    }
    return 0;
}

// Función para encontrar los primeros N números primos
int criba() {
    int N;
    int intentos = 0;

    // Solicitar al usuario un número válido
    while (intentos < MAX_ATTEMPTS) {
        printf("Ingresa un numero positivo para encontrar los primeros N numeros primos: ");
        if (scanf("%d", &N) == 1 && N > 0) {
            break;
        } else {
            printf("Error: Debes ingresar un numero positivo.\n");
            limpiar_buffer();
            intentos++;
        }
    }

    if (intentos == MAX_ATTEMPTS) {
        printf("Demasiados intentos fallidos. Volviendo al menu.\n");
        return -1;
    }

    int contador = 0; // Contador de números primos encontrados
    int numero = 2;   // Empezamos a verificar desde el primer número primo

    printf("Los primeros %d numeros primos son:\n", N);
    while (contador < N) {
        if (es_primo(numero)) {
            printf("%d ", numero);
            contador++;
        }
        numero++;
    }
    printf("\n");

    return 0;
}

// Función para verificar si un número es primo
bool es_primo(int n) {
    if (n < 2) return false;
    for (int i = 2; i * i <= n; i++) {
        if (n % i == 0) {
            return false;
        }
    }
    return true;
}

// Función para generar números aleatorios
int generar_aleatorios() {
    int N, min, max;
    int intentos = 0;

    // Solicitar la cantidad de números aleatorios
    while (intentos < MAX_ATTEMPTS) {
        printf("Cuantos numeros aleatorios quieres generar? ");
        if (scanf("%d", &N) == 1 && N > 0) {
            break;
        } else {
            printf("Error: Debes ingresar un numero positivo.\n");
            limpiar_buffer();
            intentos++;
        }
    }

    if (intentos == MAX_ATTEMPTS) {
        printf("Demasiados intentos fallidos. Volviendo al menu.\n");
        return -1;
    }

    // Solicitar el rango
    intentos = 0;
    while (intentos < MAX_ATTEMPTS) {
        printf("Ingresa el rango (minimo y maximo): ");
        if (scanf("%d %d", &min, &max) == 2 && min <= max) {
            break;
        } else {
            printf("Error: El minimo debe ser menor o igual al maximo.\n");
            limpiar_buffer();
            intentos++;
        }
    }

    if (intentos == MAX_ATTEMPTS) {
        printf("Demasiados intentos fallidos. Volviendo al menu.\n");
        return -1;
    }

    // Generar y mostrar los números aleatorios
    printf("Numeros aleatorios generados:\n");
    for (int i = 0; i < N; i++) {
        int num = rand() % (max - min + 1) + min;
        printf("%d ", num);
    }
    printf("\n");

    return 0;
}

// Función para mostrar el menú y procesar la opción del usuario
int menu() {
    int opcion;
    while (true) {
        printf("\n--- Menu ---\n"
               "1) Encontrar numeros primos\n"
               "2) Verificar si un numero es primo\n"
               "3) Generar numeros aleatorios\n"
               "4) Salir\n"
               "Ingresa tu opcion: ");

        if (scanf("%d", &opcion) == 1 && opcion >= 1 && opcion <= 4) {
            limpiar_buffer();
            break;
        } else {
            printf("Error: Opcion no valida. Intenta de nuevo.\n");
            limpiar_buffer();
        }
    }

    switch (opcion) {
        case 1:
            criba();
            break;
        case 2: {
            int num;
            printf("Ingresa un numero para verificar si es primo: ");
            if (scanf("%d", &num) == 1) {
                if (es_primo(num)) {
                    printf("%d es un numero primo.\n", num);
                } else {
                    printf("%d no es un numero primo.\n", num);
                }
            } else {
                printf("Error: Entrada no valida.\n");
            }
            limpiar_buffer();
            break;
        }
        case 3:
            generar_aleatorios();
            break;
        case 4:
            return 4; // Salir
        default:
            printf("Opcion no valida.\n");
            break;
    }
    return 0;
}

// Función para limpiar el buffer de entrada
void limpiar_buffer() {
    while (getchar() != '\n'); // Limpiar caracteres no leídos
}