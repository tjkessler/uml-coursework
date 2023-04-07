#include <stdio.h>
#include <stdlib.h>

float rand_float();


int main()
{
    // set variables
    int num_elem = 10000;

    // generate random floats between 0 and 1
    float rv_x[num_elem];
    for (int i = 0; i < num_elem; i++)
    {
        rv_x[i] = rand_float();
    }
    float rv_y[num_elem];
    for (int i = 0; i < num_elem; i++)
    {
        rv_y[i] = rand_float();
    }

    // given f(x) = x, count the number of occurrences where y < f(x)
    int count = 0;
    for (int i = 0; i < num_elem; i++)
    {
        if (rv_y[i] < rv_x[i])
        {
            count += 1;
        }
    }

    printf("Number of occurrences: %d \n", count);
    printf("Proportion of y < f(x): %f \n", (float)count / (float)num_elem);

    return 0;
}


float rand_float()
{
    float r = (float)rand() / (float)RAND_MAX;
    return r;
}
