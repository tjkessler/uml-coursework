#include <stdio.h>
#include <stdlib.h>
#include <math.h>


int main()
{
    // set up variables
    int N = 5;
    float dx = 1 / (float)N;

    // create input data, length N, (0, 1) in steps of 1 / dx
    float x[N - 1];
    for (int i = 0; i < N; i++)
    {
        x[i] = dx * (float)i;
    }

    // set up matrix with boundary conditions
    float f[N - 1];
    f[0] = 1;
    f[N - 1] = 2;
    for (int i = 1; i < N - 1; i++)
    {
        f[i] = 0;
    }

    // create "copy" matrix with above matrix's data
    float f_old[N - 1];
    for (int i = 0; i < N; i++)
    {
        f_old[i] = f[i];
    }

    // run Jacobi for 200 iterations, stopping if error falls below 0.0001
    int ITER = 200;
    float TOL = 0.0001;
    int stop_iter;
    // these are from isolating y w.r.t. y' and y'' given dx = 1 / N
    float s1 = (2.0 - 3.0 * dx) / (4 - 4 * dx);
    float s2 = (2.0 + 3.0 * dx) / (4 - 4 * dx);
    float err[ITER];
    for (int i = 0; i < ITER; i++)
    {
        stop_iter = i + 1;
        err[i] = 0.0;
        for (int j = 1; j < N - 1; j++)
        {
            f[j] = s1 * f_old[j - 1] + s2 * f_old[j + 1];
            err[i] += pow((f[j] - f_old[j]), 2);
        }
        err[i] = sqrt(err[i] / pow(N, 2));
        if (err[i] <= TOL)
        {
            break;
        }
        for (int j = 0; j < N; j++)
        {
            f_old[j] = f[j];
        }
    }

    // export error over time to file
    FILE *jacobi_res = fopen("jacobi_res.txt", "w");
    for (int i = 0; i < stop_iter; i++)
    {
        fprintf(jacobi_res, "%d %f \n", i, err[i]);
    }

    return 0;
}
