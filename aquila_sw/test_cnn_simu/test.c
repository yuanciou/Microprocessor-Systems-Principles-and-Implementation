#include<stdio.h>
#include <stdlib.h>
int main()
{
    

    
    float a[10] ={10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
    float b[10] = {1,2,3,4,5,6,7,8,9,10};

    

    for (int i = 0; i<2; i++){
        int dymax = 2;
        int dxmax = 5;
        float factor = 3;
        *((int volatile *)0xC4000024) = dymax*dxmax;
        for (int dy = 0; dy < dymax; dy++)
            for (int dx = 0; dx < dxmax; dx++)
            {
                *((float volatile *)0xC4000028) = (float) (a[(dy*5)+dx] + (i*2.0));
            }

        //*((float volatile *)0xC4400008) = factor;

        *((int volatile *)0xC400002C) = 1;
        while(*((int volatile *)0xC400002C));

        b[i] = *((float volatile *)0xC4000030);
    }
  
    for (int j = 0; j < 2 ; j++)
    {
        printf("\nAns %f\n", b[j]);
    }

    return 0;
}

