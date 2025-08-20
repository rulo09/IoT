#ifndef ALG_SORT_H
#define ALG_SORT_H

int *bubble_sort(int array[], int n); 
int *quick_sort(int array[], int izquierda, int derecha);
void swap(int* a, int* b);
int partition(int array[], int izquierda, int derecha);
int *shell_sort(int array[], int n);
void printf_array(int *arr, int tamanio);

#endif