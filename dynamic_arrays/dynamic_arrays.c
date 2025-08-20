#include <stdio.h>
#include <stdlib.h>

#define clear_buf() int d; while((d = getchar()) != '\n' && d != EOF) {}

int main(){
    int aux;

    /* Reservamos un espacio en memoria de tamanio n */
    // int *ip = (int *)malloc(n * sizeof(int));
    // int *ip = (int *)calloc(n, sizeof(int));
    int *ip = (int *)malloc(sizeof(int));

    /* Ingresamos n datos */
    int c = 0;
    int flag;
    char ans;

    do{
        printf("Ingresa el dato %d: ", c+1);
        scanf("%d", &aux);
        ip[c] = aux;

        clear_buf()

        printf("Deseas ingresar otro dato? [s/n]: ");
        scanf("%c", &ans);

        if(ans == 'n' || ans == 'N')
            flag = 0;
        else{
            flag = 1;
            c++;
        }

        if(flag)
            ip = (int *)realloc(ip, (c+1) * sizeof(int));

    }
    while(flag);

    /* Imprimimos los n datos */
    int k;
    for(k=0; k<=c; k++)
        printf("Dato %d es: %d \n", k + 1, ip[k]);

    /* Liberamos el espacio de memoria reservado */
    free(ip);

    return 0;
}


/*
 * Tarea.
 * Realizar un programa que le permita calcular la media aritmetica de n muestras de tipo numerico con punto decimal. La intencion es
 *  ayudar a Benjamin de Ciencias de la Tierra a analizar los datos que adquirio en su practica de campo.
 * Considere que n es variable y por lo tanto, el arreglo debe ser dinamico y redimensionado en tiempo de ejecucion.
 *
 * Utilice funciones para resolver este problema.
 */
