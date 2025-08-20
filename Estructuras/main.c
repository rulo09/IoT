#include <stdio.h>
#include <time.h>
#include <stdlib.h>

// Definición de la estructura Estudiante
typedef struct {
    int edad;
    float peso;
    float estatura;
    char *nombre;
    char genero;
    unsigned long long int num_matricula;
    char *carrera;
    int semestre;
    float promedio;
} Estudiante;

// Arreglos de datos predefinidos
char *nombres[6] = {"Raul", "Maria", "Areli", "Diego", "Enrique", "Ana"};
char generos[3] = {'H', 'M', 'X'};
char *carreras[9] = {"Tecnologia", "C. de Tierra", "Negocios I.", "Ortesis y P", "Aeroespacial", 
                     "Genomicas", "Mates p Desarrollo", "Neurociencias", "Ing Renovables"};

#define N 100 // Número de estudiantes

// Prototipos de funciones
int aleatorios_enteros(int min, int max);
float aleatorios_flotantes(float min, float max);
void generarEstudiantes(Estudiante estudiantes[]);
void calcularPromedios(Estudiante estudiantes[], float *promedioProm, float *promedioEdad, 
                       float *promedioPeso, float *promedioEstatura);
void contarDatos(Estudiante estudiantes[], int contadorGenero[], int contadorCarreras[], 
                 int contadorSemestres[], int contadorNombres[]);
void imprimirResultados(float promedioProm, float promedioEdad, float promedioPeso, 
                        float promedioEstatura, int contadorGenero[], int contadorCarreras[], 
                        int contadorSemestres[], int contadorNombres[]);

// Función principal
int main() {
    srand(time(NULL)); // Semilla para números aleatorios

    Estudiante estudiantes[N];
    int contadorGenero[3] = {0};       // Contador para géneros (H, M, X)
    int contadorCarreras[9] = {0};     // Contador para carreras
    int contadorSemestres[16] = {0};   // Contador para semestres (1-16)
    int contadorNombres[6] = {0};      // Contador para nombres

    float promedioProm, promedioEdad, promedioPeso, promedioEstatura;

    // Generar estudiantes aleatorios
    generarEstudiantes(estudiantes);

    // Calcular promedios
    calcularPromedios(estudiantes, &promedioProm, &promedioEdad, &promedioPeso, &promedioEstatura);

    // Contar datos
    contarDatos(estudiantes, contadorGenero, contadorCarreras, contadorSemestres, contadorNombres);

    // Imprimir resultados
    imprimirResultados(promedioProm, promedioEdad, promedioPeso, promedioEstatura, 
                       contadorGenero, contadorCarreras, contadorSemestres, contadorNombres);

    return 0;
}

// Función para generar números enteros aleatorios
int aleatorios_enteros(int min, int max) {
    return (rand() % (max - min + 1)) + min;
}

// Función para generar números flotantes aleatorios
float aleatorios_flotantes(float min, float max) {
    float escala = rand() / (float)RAND_MAX;
    return min + escala * (max - min);
}

// Función para generar estudiantes aleatorios
void generarEstudiantes(Estudiante estudiantes[]) {
    for (int k = 0; k < N; k++) {
        estudiantes[k].edad = aleatorios_enteros(17, 30);
        estudiantes[k].peso = aleatorios_flotantes(45, 100);
        estudiantes[k].estatura = aleatorios_flotantes(1.45, 1.90);
        estudiantes[k].nombre = nombres[aleatorios_enteros(0, 5)];
        estudiantes[k].genero = generos[aleatorios_enteros(0, 2)];
        estudiantes[k].num_matricula = aleatorios_enteros(1e8, 1e9);
        estudiantes[k].carrera = carreras[aleatorios_enteros(0, 8)];
        estudiantes[k].semestre = aleatorios_enteros(1, 16);
        estudiantes[k].promedio = aleatorios_flotantes(6, 10);
    }
}

// Función para calcular promedios
void calcularPromedios(Estudiante estudiantes[], float *promedioProm, float *promedioEdad, 
                       float *promedioPeso, float *promedioEstatura) {
    float sumaProm = 0, sumaEdad = 0, sumaPeso = 0, sumaEstatura = 0;

    for (int k = 0; k < N; k++) {
        sumaProm += estudiantes[k].promedio;
        sumaEdad += estudiantes[k].edad;
        sumaPeso += estudiantes[k].peso;
        sumaEstatura += estudiantes[k].estatura;
    }

    *promedioProm = sumaProm / N;
    *promedioEdad = sumaEdad / N;
    *promedioPeso = sumaPeso / N;
    *promedioEstatura = sumaEstatura / N;
}

// Función para contar datos
void contarDatos(Estudiante estudiantes[], int contadorGenero[], int contadorCarreras[], 
                 int contadorSemestres[], int contadorNombres[]) {
    for (int k = 0; k < N; k++) {
        // Contar géneros
        for (int i = 0; i < 3; i++) {
            if (estudiantes[k].genero == generos[i]) {
                contadorGenero[i]++;
                break;
            }
        }

        // Contar carreras
        for (int i = 0; i < 9; i++) {
            if (estudiantes[k].carrera == carreras[i]) {
                contadorCarreras[i]++;
                break;
            }
        }

        // Contar semestres
        contadorSemestres[estudiantes[k].semestre - 1]++;

        // Contar nombres
        for (int i = 0; i < 6; i++) {
            if (estudiantes[k].nombre == nombres[i]) {
                contadorNombres[i]++;
                break;
            }
        }
    }
}

// Función para imprimir resultados
void imprimirResultados(float promedioProm, float promedioEdad, float promedioPeso, 
                        float promedioEstatura, int contadorGenero[], int contadorCarreras[], 
                        int contadorSemestres[], int contadorNombres[]) {
    printf("Promedio de promedios: %.2f\n", promedioProm);
    printf("Promedio de edades: %.2f\n", promedioEdad);
    printf("Promedio de pesos: %.2f\n", promedioPeso);
    printf("Promedio de estaturas: %.2f\n", promedioEstatura);

    printf("\nCantidad de estudiantes por género:\n");
    for (int i = 0; i < 3; i++) {
        printf("%c: %d\n", generos[i], contadorGenero[i]);
    }

    printf("\nCantidad de estudiantes por carrera:\n");
    for (int i = 0; i < 9; i++) {
        printf("%s: %d\n", carreras[i], contadorCarreras[i]);
    }

    printf("\nCantidad de estudiantes por semestre:\n");
    for (int i = 0; i < 16; i++) {
        printf("Semestre %d: %d\n", i + 1, contadorSemestres[i]);
    }

    printf("\nCantidad de estudiantes por nombre:\n");
    for (int i = 0; i < 6; i++) {
        printf("%s: %d\n", nombres[i], contadorNombres[i]);
    }
}