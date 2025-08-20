#include <stdio.h>
#include <stdlib.h>

// Macro para limpiar el buffer de entrada
#define clear_buf() int d; while((d = getchar()) != '\n' && d != EOF) {}

float* pedir_datos(int *num_datos) {
    float aux;

    // Reservamos un espacio en memoria de tamaño inicial 1 para guardar decimales
    float *ip = (float*)malloc(sizeof(float));
    if (ip == NULL) {
        printf("Error: No se pudo reservar memoria \n");
        return NULL; // Código de error
    }

    /* Ingresamos n datos */
    int c = 0;
    int flag;
    char ans;

    do {
        printf("Ingresa el dato %d: ", c + 1);
        if (scanf("%f", &aux) != 1) {  // Corregido para leer un float
            printf("Error: Entrada no válida. Debes ingresar un número.\n");
            clear_buf();
            continue;
        }
        ip[c] = aux;

        clear_buf();        

        printf("Deseas ingresar otro dato? [s/n]: ");
        scanf("%c", &ans);

        if (ans == 'n' || ans == 'N')
            flag = 0;
        else {
            flag = 1;
            c++;
            ip = (float *)realloc(ip, (c + 1) * sizeof(float));
            if (ip == NULL) {
                printf("Error: No se pudo reservar memoria adicional\n");
                return NULL;
            }
        }
    } while (flag);

    *num_datos = c + 1;

    return ip;  // Retornamos el arreglo dinámico
}

float calcular_media(float *c, int N) {
    float suma = 0;
    for (int i = 0; i < N; i++) {
        suma += c[i];
    }

    return suma / N;
}

int main() {
    int num_datos;
    float *datos = pedir_datos(&num_datos); // Pedimos los datos al usuario

    if (datos == NULL) {
        return 1; // Salir con código de error si no se pudo reservar memoria
    }

    float media = calcular_media(datos, num_datos);
    printf("La media aritmetica es: %.2f\n", media);  // Corregido el formato de impresión

    free(datos); // Liberar la memoria asignada dinámicamente

    return 0;
}