/*
 * Tarea.
 * Realizar un programa que le permita calcular la media aritmetica de n muestras de tipo numerico con punto decimal. La intencion es
 *  ayudar a Benjamin de Ciencias de la Tierra a analizar los datos que adquirio en su practica de campo.
 * Considere que n es variable y por lo tanto, el arreglo debe ser dinamico y redimensionado en tiempo de ejecucion.
 *
 * Utilice funciones para resolver este problema.
 */

 // Incluimos las liberías necesarias
 #include <stdio.h>
 #include <stdlib.h>

 // Macros para no guardar basura
 #define clear_buf() int d; while((d = getchar()) != '\n' && d != EOF) {}

int pedir_datos(){
    float aux;

    // Reservamos un espacio en memoria de tamano n para guardar decimales
    float *ip = (float*)malloc(sizeof(float));

    // Verificamos que se pueda reservar la memoria en cada iteracion
    if(ip == NULL){
        printf("Error: No se pudo reservar memoria \n"); 
        return -1; // Codigo de error
    }

    
    /* Ingresamos n datos */
    int c = 0;
    int flag;
    char ans;
        
    
        do{
            printf("Ingresa el dato %d: ", c + 1);
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
                ip = (float *)realloc(ip, (c+1) * sizeof(float));
    
        }
        while (flag); 
    
        return c; 


}

/*
int calcular_media(pedir_datos){
    // Una vez ha terminado de pedir los datos deberíamos ir a esta funcion
    float media_aritmetica = 0; 
    float suma = 0; 
    int N = sizeof(c); 

    for(int i = 0; i < N; i++){
        suma += c[i]; 

    }

   return media_aritmetica = suma / N; 
}
*/



 int main(){
    pedir_datos();
    /*
     calcular_media(pedir_datos);    
    
    printf("La media aritmetica de todas tus operaciones es %.2f", calcular_media);
    */
   
    return 0; 
 }