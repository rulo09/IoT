#include <stdio.h>
#include <time.h>
#include <math.h>

#include "media_desv.h"



float bubble10[1] = {0}; 
float bubble100[6] = {0.00004,0, 000041, 0, 0.000021}; 
float bubble1k[6] = {0.003, 0.001, 0.002, 0.002, 0.0032, 0.0015}; 
float bubble10k[6] = {0.177, 0.275, 0.160, 0.129, 0.143, 0.1416}; 
float bubble100k[6] = {24.99, 21.14, 44.71, 29.07, 16.19, 25.36}; 
float bubble1M[4] = {2414, 1800, 2690, 946}; 

float quicksort10[1] = {0}; 
float quicksort100[3] = {0, 0.00008, 0.000007}; 
float quicksort1k[3] = {0, 0.00095, 0.000088}; 
float quicksort10k[4] = {0.002, 0.00095, 0.0012, 0.0023}; 
float quicksort100k[5] = {0.012, 0.013, 0.011, 0.01, 0.067}; 
float quicksort1M[6] = {5.05, 0.523, 0.14, 0.12, 0.2, 0.15}; 


float shellsort10[1] = {0}; 
float shellsort100[2] = {0, 0.00004}; 
float shellsort1k[2] = {0, 0.00051}; 
float shellsort10k[4] = {0.001, 0.0015, 0.0005}; 
float shellsort100k[4] = {0.006, 0.00052, 0.022, 0.02}; 
float shellsort1M[4] = {0.05, 0.055, 0.301, 0.26}; 


float calcular_media(float arr[], int size){
    float suma = 0;

    for(int i = 0; i < size; i++){
        suma += arr[i]; 
    }
    
    return suma / size; 

    return 0; 
}

float desvest(float arr[], int size){
    float media = calcular_media(arr, size); 
    float suma = 0; 

    for(int i = 0; i < size; i++){
        suma += (arr[i] - media) * (arr[i] - media); 

    }

    return sqrt(suma/size); 

}

