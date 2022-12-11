#include "pwmwrite.h"

static void swap(channe_t *a, channe_t *b)
{
    channe_t t = *a;
    *a = *b;
    *b = t;
}

static int partition (channe_t *arr, int low, int high)
{
    int pivot = arr[high].val;
    int left = low;
    int right = high - 1;
    while(1){
        while(left <= right && arr[left].val < pivot) left++;
        while(right >= left && arr[right].val > pivot) right--;
        if (left >= right) break;
        swap(&arr[left],&arr[right]);
        left++;
        right--;
    }
    swap(&arr[left],&arr[high]);
    return left;
}

static void quickSort(channe_t *arr, int low, int high)
{
    if (low < high)
    {
        int index = partition(arr, low, high);
        quickSort(arr, low, index - 1);
        quickSort(arr, index + 1, high);
    }
}


