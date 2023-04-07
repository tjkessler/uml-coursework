#include <stdio.h>
#include <stdlib.h>

float rand_float();
void plot_pmf_pdf(float *probs, int n_elem, float bin_width);


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

    // calculate probabilities
    float probabilities[(int)mbin];
    for (int i = 0; i < (int)mbin; i++)
    {
        probabilities[i] = (float)pt_freq[i] / (float)num_elem;
    }
    
    // plot PMF and PDF
    plot_pmf_pdf(probabilities, (int)mbin, bin_width);

    return 0;
}


float rand_float()
{
    float r = (float)rand() / (float)RAND_MAX;
    return r;
}


void plot_pmf_pdf(float *probs, int n_elem, float bin_width)
{
    // write probability data to temporary file
    FILE *temp_pmf = fopen("data_pmf.temp", "w");
    for (int i = 0; i < n_elem; i++)
    {
        fprintf(temp_pmf, "%f %f \n", (float)(i + 1) * bin_width, probs[i]);
    }

    // calculate pdf, save to temporary file
    float pdf[n_elem];
    float sum;
    for (int i = 0; i < n_elem; i++)
    {
        sum = 0;
        for (int j = 0; j < i; j++)
        {
            sum += probs[j];
        }
        pdf[i] = sum;
    }
    FILE *temp_pdf = fopen("data_pdf.temp", "w");
    for (int i = 0; i < n_elem; i++)
    {
        fprintf(temp_pdf, "%f %f \n", (float)(i + 1) * bin_width, pdf[i]);
    }
    

    // plot data
    char *commands[] = {
        "set title \"PMF and PDF of 10000 Random Values, Between 0 and 1\"",
        "set xlabel \"Value Range\"",
        "set ylabel \"Probability of Occurrence\"",
        "set xrange [0:1.1]",
        "set yrange [0:1.0]",
        "plot 'data_pmf.temp', 'data_pdf.temp'"
    };
    FILE *gnuplotpipe = popen("gnuplot -persistent", "w");
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
    {
        fprintf(gnuplotpipe, "%s \n", commands[i]);
    }

    return;
}
