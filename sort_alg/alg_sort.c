#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "alg_sort.h"


int *bubble_sort(int array[], int n){
    int i;
    int j;
    int temp;

    for(i=0; i<n-1; i++)
        for(j=0; j<n-1-i; j++)
            if(array[j] > array[j + 1]){
                temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }

    return array;
}

// Función principal de Quick Sort
int* quick_sort(int array[], int izquierda, int derecha) {
    if (izquierda < derecha) {
        // Obtener el índice del pivote
        int pivote = partition(array, izquierda, derecha);
        
        // Ordenar recursivamente los subarreglos
        quick_sort(array, izquierda, pivote - 1);
        quick_sort(array, pivote + 1, derecha);
    }

    return array;
}

// Función de apoyo para intercambiar elementos
void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

// Función para partir el arreglo por mitades
int partition(int array[], int izquierda, int derecha) {
    int pivote = array[derecha]; // Elegir el último elemento como pivote
    int i = (izquierda - 1); // Índice del elemento más pequeño

    for (int j = izquierda; j <= derecha - 1; j++) {
        // Si el elemento actual es menor que el pivote
        if (array[j] < pivote) {
            i++; // Incrementar el índice del elemento más pequeño
            swap(&array[i], &array[j]); // Intercambiar elementos
        }
    }
    // Intercambiar el pivote con el elemento en la posición correcta
    swap(&array[i + 1], &array[derecha]);
    
    // Devolver la posición del pivote
    return (i + 1);
}


int *shell_sort(int array[], int n){
    int i = 0; 
    int j = 0; 
    int temp = 0;
    int interval = floor(n / 2); 

    while(interval > 0){
        for(i = 0; i < n; i++){
            temp = array[i];
            j = i; 
            
            while(j >= interval + 1 && array[j - interval] > temp){
                array[j] = array[j - interval]; 
                j = j - interval; 
            }
            array[j] = temp; 
        }
        interval = floor(interval / 2); 
    }
    return array; 
}

void printf_array(int *arr, int tamanio){
    int k;

    for(k=0; k<tamanio; k++)
        printf("%d\n", arr[k]);
}