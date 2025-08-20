/* Libreria del sistema */
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

/* Constantes */
#define PI 3.14159265
#define N 10

/* Variables globales */

/* Prototipos de Funciones */



float deg2rad(float deg) {
    return (PI / 180) * deg;
}

int get_randi(int min,int max) {    
        return ( rand() % (max - min + 1)) + min ;
}

int linear_search(int arreglo[], int num){
     int c; 
     for(c = 0; c < N; c++){
        if(arreglo[c] == num)
        return c; 
     }
     return -1;
}

int main() {
    unsigned int seed = time(NULL);
    srand(seed);
    
    int array[N];
    int k;

    for (k = 0; k < N; k++) {
        array[k] = (get_randi(1,N));
        printf("%d\n", array[k]);
    }

    // Algoritmo de busqueda lineal
    int n; 
    printf("ingrese el numero a busacar: "); 
    scanf("%d", &n); 
    printf("Numero %d encontrado en la posicion %d", n, linear_search(array, n));

    return 0;
}