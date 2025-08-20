/*
Disene un programa que le premita calcular la media aritméticade las 
calificaciones de los estudiantes del área de profundización de matemáticas y 
computación de la materia de dinámica de sistemas físicos
* Autor: Raúl Martínez Castrejón
* Correo: ramanez2003@gmail.com
* Fecha de creación 06/02/2025 
* Última modificación  --/--/----
*/

#include <stdio.h>

/* constantes */
#define N 5

/* variables globales */
/* prototipos de funciones */

int main(){
    float xg = 0; 
    // float calificacion; 
    
    // Forma 1 arreglos
    //float calificacion[N] = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // forma 2 arreglos
    int k;
    
    for(k=0; k<N; k++)
        calificacion[k] = 0.0
    
    
    for(int k = 0; k<N; k++){
        /* pedimos la calificacion */
        printf("Ingresa la %d calificación: ", k + 1);
        scanf("%f", &calificacion[k]);
        xg += calificacion; 
    }
    
    xg /= N; 
    
    printf("La media de calificacion es: %.2f", xg);
    
    printf("Dirección de xg %hx", &xg);
    
    return 0;
    
}

