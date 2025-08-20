#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "menu.h"
#include "alg_sort.h"
#include "randi.h"
#include "media_desv.h"

#define MAX_RAND 10


int main(){
    int tamanio;
    printf("Cual es el tamanio de tu arreglo?: ");
    scanf("%d", &tamanio);

    int *arreglito = get_random_array(tamanio);

    while (menu_ord(arreglito, tamanio) != 6) {
        // El bucle continúa hasta que el usuario elija salir
    }

    printf("Adios! :(\n");



    return 0;
}
