#include <stdio.h>
#include <time.h> // Incluir para medir el tiempo
#include "menu.h"
#include "alg_sort.h"
#include "media_desv.h"

void print_menu() {
    printf("***** Welcome *****\n");
    printf("1. Imprimir arreglo aleatorio\n");
    printf("2. Ordenar via Burbuja\n");
    printf("3. Ordenar via Quicksort\n");
    printf("4. Ordenar via Shellsort\n");
    printf("5. Conocer la desviación estandar y el promedio de cada algoritmo");
    printf("6. Salir\n");
    printf("¿Que deseas hacer?: ");
}

int menu_ord(int *arreglito, int n) {
    print_menu();
    int opt;
    scanf("%d", &opt);

    clock_t inicio, fin; // Variables para medir el tiempo
    double tiempo_ejecucion;

    switch (opt) {
        case 1:
            printf_array(arreglito, n);
            break;
        case 2:
            inicio = clock(); // Iniciar medición de tiempo
            bubble_sort(arreglito, n);
            fin = clock(); // Finalizar medición de tiempo
            tiempo_ejecucion = ((double)(fin - inicio)) / CLOCKS_PER_SEC; // Calcular tiempo en segundos
           
            printf_array(arreglito, n);
            printf("Tiempo de ejecucion (Burbuja): %.6f segundos\n", tiempo_ejecucion);
            break;
        case 3:
            inicio = clock(); // Iniciar medición de tiempo
            quick_sort(arreglito, 0, n - 1);
            fin = clock(); // Finalizar medición de tiempo
            tiempo_ejecucion = ((double)(fin - inicio)) / CLOCKS_PER_SEC; // Calcular tiempo en segundos
            
            printf_array(arreglito, n);
            printf("Tiempo de ejecucion (Quicksort): %.6f segundos\n", tiempo_ejecucion);
            break;
        case 4:
            inicio = clock(); // Iniciar medición de tiempo
            shell_sort(arreglito, n);
            fin = clock(); // Finalizar medición de tiempo
            tiempo_ejecucion = ((double)(fin - inicio)) / CLOCKS_PER_SEC; // Calcular tiempo en segundos
    
            printf_array(arreglito, n);
            printf("Tiempo de ejecucion (Shellsort): %.6f segundos\n", tiempo_ejecucion);
            break;
        case 5:

            break;  
        case 6: 
            return 6; // Salir del menú
        default:
            printf("Opcion no valida.\n");
            break;
    }
    return 0; // Retorno por defecto
}