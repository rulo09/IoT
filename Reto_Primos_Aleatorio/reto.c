/* Libreria del sistema */
#include <stdio.h>
#include <time.h>
#include <stdlib.h>


# define True 1


int criba(){
    printf("Estas en la opcion 1"); 

}

int esprimo(){
    printf("Estas en la opcion 2");
}

int randi(){
    printf("Estas en la opcion 3");
}

int menu(){
    int n; 
    printf("Hola bienvenidx al programa primos y aleatorios\n"
    "\n"
    "Que opcion te gustaria\n"
    "Ingres un numero \n"
    "1) Obtener N numeros primos\n"
    "2) Determinar si un numero es primo\n"
    "Generar N cantidad de numeros aleatorios\n"
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
    case 4:
        printf("adios unu");
        break;
    }
}

int main(){


    while (True)
    {
        menu(); 
    }
    

    return 0; 
}