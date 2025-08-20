/* Libreria del sistema */
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define True 1
#define False 0

// variables globales
int flag = True; 

int criba(){
    int N; // Declaramos N como una variable local

    // Solicitamos al usuario que ingrese un número
    printf("Completa la frase con el numero deseado:\n"
           "Me gustaria conocer cuales son los numeros primos presentes en los primeros ____ \n"
           "numeros naturales: ");
    scanf("%d", &N);

    // Validamos que N sea un número positivo
    if (N <= 0) {
        printf("Error: Debes ingresar un numero positivo.\n");
        return -1; // Retornamos un valor de error
    }

    // Declaramos el arreglo con tamaño N
    // Asignamos memoria dinámica para el arreglo
    int *Numeros = malloc((N + 1) * sizeof(int));
    if (Numeros == NULL) {
        printf("Error: No se pudo asignar memoria.\n");
        return -1;
    }

    // Inicializamos el arreglo Numeros con numeros de 0 a N 
    for (int i = 0; i <= N; i++){
        Numeros[i] = i;
    }

    // Implementación del algoritmo de la Criba de Eratóstenes
    for (int i = 2; i * i <= N; i++) {
        if (Numeros[i] != 0) {  // Si el número no ha sido marcado como no primo
            for (int j = i * i; j <= N; j += i) {
                Numeros[j] = 0;  // Marcamos los múltiplos de i como no primos
            }
        }
    }

    printf("Los numeros primos presentes en los primeros %d numeros son:\n", N); 
    for(int i = 2; i <= N; i++){
        if(Numeros[i] != 0){
            printf("%d ", i); 
        }
    } 
    printf("\n");

    free(Numeros);
    return 0;
}

// Función para determinar si un número es primo
int esprimo() {
    int N;
    printf("Que numero te gustaria saber si es primo? \n"
           "Ingresa un numero: ");
    scanf("%d", &N);

    // Manejar casos especiales
    if (N < 2) {
        printf("El numero no es primo.\n");
        return 0; // No es primo
    }

    // Evaluar si N es primo
    for (int i = 2; i <= N / 2; i++) {
        if (N % i == 0) {
            printf("El numero no es primo.\n\n");
            return 0; // No es primo
        }
    }

    // Si no se encontraron divisores, es primo
    printf("El numero si es primo.\n\n");
    return 1; // Es primo
}

// Función para generar un número aleatorio en un rango [min, max]
int getrand(int min, int max) {
    return rand() % (max - min + 1) + min;
}

// Función para solicitar datos al usuario, generar números aleatorios y mostrarlos
int randi() {
    int N, min, max;

    // Solicitar al usuario la cantidad de números aleatorios y el rango
    printf("Cuantos numeros aleatorios enteros quieres?\n");
    scanf("%d", &N);

    printf("En que rango los quieres?\n");
    printf("Ingresa el minimo: ");
    scanf("%d", &min);
    printf("Ingresa el maximo: ");
    scanf("%d", &max);

    // Validar que el rango sea válido
    if (min > max) {
        printf("Error: El minimo debe ser menor o igual al maximo.\n");
        return -1;
    }

    // Asignar memoria dinámica para el arreglo
    int *array = malloc(N * sizeof(int));
    if (array == NULL) {
        printf("Error: No se pudo asignar memoria.\n");
        return -1;
    }

    // Inicializar la semilla para números aleatorios
    srand(time(NULL));

    // Generar y mostrar los números aleatorios
    for (int i = 0; i < N; i++) {
        array[i] = getrand(min, max);
        printf("El %d numero es %d\n", i + 1, array[i]);
    }

    // Liberar la memoria asignada
    free(array);

    return 0;
}

int menu(){
    int n; 
    printf("Hola bienvenidx al programa primos y aleatorios\n"
    "\n"
    "Que opcion te gustaria\n"
    "Ingresa un numero \n"
    "1) Obtener N numeros primos\n"
    "2) Determinar si un numero es primo\n"
    "3) Generar N cantidad de numeros aleatorios\n"
    "4) Salir :( \n"); 

    scanf("%d", &n);

    switch (n)
    {
    case 1:
        criba();
        break;
    case 2: 
        esprimo(); 
        break; 
    case 3: 
        randi(); 
        break;
    case 4:
        printf("adios unu\n");
        flag = False; 
        break;
    default:
        printf("Opcion no valida. Intenta de nuevo.\n");
        break;
    }
    return 0;
}

int main(){
    
    while (flag)
    {
        menu(); 
    }
    
    return 0; 
}