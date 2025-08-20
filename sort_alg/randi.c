#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "randi.h"

int get_rand_i(int min, int max){
    return (rand() % (max - min + 1)) + min;
}

int *get_random_array(int n){
    /* Plantamos la semilla con base al tiempo */
    srand(time(NULL));

    int *array = (int *)calloc(n, sizeof(int));

    int c;
    for(c=0; c<n; c++)
        array[c] = get_rand_i(0, n);

    return array;
}