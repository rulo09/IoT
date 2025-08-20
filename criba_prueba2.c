/*
Disene un programa que le premita desplegar nÃºmeros primos utilizando el mÃ©todo conocido como criba de aretÃ³stenes
* Autor: RaÃºl MartÃ­nez CastrejÃ³n
* Correo: ramanez2003@gmail.com
* Fecha de creaciÃ³n 06/02/2025 
* Ãšltima modificaciÃ³n  --/--/----
*/

#include <stdio.h>

/* constantes */
#define N 1001

/* variables globales */
/* prototipos de funciones */

int main(){
    int Numeros[N]; 
    int not_a_prime[N];
    int Primos[N/10];
    int i; 
    int k; 
    int no_primo; 
    int index = 0; 
    
    // Iniciamos los arreglos en 0
    for (int i = 1; i < N; i++){
        Numeros[i] = 0;
        not_a_prime[i] = 0;
    }
    
    // Inicializamos el arreglo Numeros con números de 1 a N 
        for (int i = 1; i < N; i++){
        Numeros[i] = i;
        
        // Verificamos que se llene correctamente el arreglo 
        //printf("El areglo en la posicion %d tiene el valor %d \n", i, primos[i]);
    }
 
    
    // Multiplicamos de 2 hasta N/2 para encontrar todos los números no primos
        for (int i = 2; i < N / 2; i++) {  
        for (int k = N / 2; k >= 2; k--) {  
            no_primo = Numeros[i] * Numeros[k]; 
            
            if (no_primo > 1000) { // Si el resultado es mayor a 1000 la operación se omite
                continue; 
            } 
            
             /* Vereficar los resultados de la multiplicacion
            printf("La multiplicación de %d por %d da como resultado %d, el cual no es número primo\n",
                   primos[i], primos[k], no_primo);
                   */
                   
            // Guardamos el resultado de cada iteración en el arreglo not a prime
            not_a_prime[index] = no_primo;
            // Confirmamos como se guardan los datos
            printf("El arreglos not a prime en la posición %d guarda el valor de %d \n",index, not_a_prime[index]);
            index++; // Pasamos al siguiente indice
        }
    }

// Comparamos ambos arreglos y guardamos el resultado en un nuevo arreglo
// NO FUNCIONA ESTA PARTE (TODAVIA NO )
    /* for(int i = 0; i < N; i++){
        if(Numeros[i] != not_a_prime[i]){
            primos[i] = Numeros[i]
            printf("El numero %d es Primo :D", primos[i])
        }
    }
    
    */
    
    return 0;
}

