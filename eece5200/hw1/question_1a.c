#include <stdio.h>
#include <stdlib.h>

float rand_float();
void plot_freq(int *freq, int n_elem, float bin_width);


int main()
{
    // set variables
    int num_elem = 10000;
    float mbin = 10.0;
    float bin_width = 1 / mbin;

    // generate random floats between 0 and 1
    float rand_vals[num_elem];
    for (int i = 0; i < num_elem; i++)
    {
        rand_vals[i] = rand_float();
    }

    // set up array to count frequencies
    int pt_freq[(int)mbin];
    for (int i = 0; i < mbin; i++)
    {
        pt_freq[i] = 0;
    }

    // count frequencies
    for (int i = 0; i < num_elem; i++)
    {
        for (int j = 1; j < (int)mbin + 1; j++)
        {
            if (rand_vals[i] > (j - 1) * bin_width &&
                rand_vals[i] <= j * bin_width)
            {
                pt_freq[j - 1] += 1;
                break;
            }
            else
            {
                continue;
            }    
        }  
    }

    // plot frequency using gnuplot
    plot_freq(pt_freq, (int)mbin, bin_width);
    
    return 0;
}


float rand_float()
{
    float r = (float)rand() / (float)RAND_MAX;
    return r;
}


void plot_freq(int *freq, int n_elem, float bin_width)
{
    // write data to temporary file
    FILE *temp = fopen("data.temp", "w");
    for (int i = 0; i < n_elem; i++)
    {
        fprintf(temp, "%f %d \n", (float)(i + 1) * bin_width, freq[i]);
    }

    // plot data
    char *commands[] = {
        "set title \"Frequency of 10000 Random Values, Between 0 and 1\"",
        "set xlabel \"Value Range\"",
        "set ylabel \"Frequency\"",
        "set xrange [0:1.1]",
        "set yrange [0:1500]",
        "set style data histogram",
        "set style fill solid border -1",
        "plot 'data.temp' using 1:2:xtic(1) with boxes"
    };
    FILE *gnuplotpipe = popen("gnuplot -persistent", "w");
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
    {
        fprintf(gnuplotpipe, "%s \n", commands[i]);
    }

    return;
}
