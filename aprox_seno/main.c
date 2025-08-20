#include <stdlib.h>
#include <stdio.h>
#include <math.h>

long int factorial(int n){
    if(n == 0){
        return 1;
    }
    return n * factorial(n-1);
}

float aprox_seno(float x){
    return  (x -
            (pow(x,3)/factorial(3)) +
            (pow(x,5)/factorial(5)) -
            (pow(x,7)/factorial(7)) +
            (pow(x,9)/factorial(9)) -
            (pow(x,11)/factorial(11)) +
            (pow(x,13)/factorial(13)))
            ;
}

float sum_sin(int m, float x){
    float seno = 0; 
    
    for(int n = 1; n <= m; n++){
        seno +=
        (pow(-1, n +1) * pow(x, 2 * n - 1)) 
        /
        (factorial(2 * n - 1)); 
        
    }

    return seno;
}


float aprox_exponencial(float x, int m){
    float expo = 1; 

    for(int n = 1; n < m; n++){
        expo += 
        (pow(x, n) 
        /
         factorial(n));

    }

    return expo;

}



int main(){
    printf(" Aproximando el seno con factorial %f\n\n", aprox_seno(3.1415));
    printf("El valor real de seno%f\n\n", sin(3.1415));
    printf("Aproximando el seno con sumatoria %f\n\n", sum_sin(7, 3.1415));
    printf("Aproximando la func exponencial con factorial %f\n", aprox_exponencial(1, 13));
    printf("El valor real de la funcion exponencial %f", exp(1)); 

    return 0; 
}