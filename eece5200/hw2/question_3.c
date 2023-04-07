#include <stdio.h>
#include <stdlib.h>


int main()
{
    // define input, target training data
    double inp_train[9] = {
        1.900,
        1.910,
        1.920,
        1.930,
        1.940,
        1.950,
        1.960,
        1.970,
        1.980
    };
    double tar_train[9] = {
        0.75994575,
        0.91972266,
        1.05710620,
        1.22775046,
        1.31669275,
        1.50697361,
        1.79323175,
        2.03235298,
        2.27224681
    };

    // create matrix of known values following y = 1 + x + x2
    double x[9][3] = {
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0},
        {1, 0, 0}
    };
    for (int i = 0; i < 9; i++)
    {
        x[i][1] = inp_train[i];
        x[i][2] = inp_train[i] * inp_train[i];
    }

    // create transformation of above matrix
    double x_T[3][9] = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    for (int i = 0; i < 9; i++)
    {
        x_T[1][i] = inp_train[i];
        x_T[2][i] = inp_train[i] * inp_train[i];
    }

    // find (xT)(x)
    double x_T_x[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };
    for (int r1 = 0; r1 < 9; r1++)
    {
        for (int c2 = 0; c2 < 9; c2++)
        {
            for (int c1 = 0; c1 < 3; c1++)
            {
                x_T_x[r1][c2] += x[r1][c1] * x_T[c1][c2];
            }
        }
    }

    // obtain inverse of (xT)(x) using Gauss Jordan method
    double inverse[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    double _temp;
    for (int k = 0; k < 3; k++)
    {
        _temp = x_T_x[k][k];
        for (int j = 0; j < 3; j++)
        {
            x_T_x[k][j] /= _temp;
            inverse[k][j] /= _temp;
        }
        for (int i = 0; i < 3; i++)
        {
            _temp = x_T_x[i][k];
            for (int j = 0; j < 3; j++)
            {
                if (i == k)
                {
                    break;
                }
                x_T_x[i][j] -= x_T_x[k][j] * _temp;
                inverse[i][j] -= inverse[k][j] * _temp;
            }
        }
    }

    // multiply inverse by xT
    double _res[3][9] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    for (int r1 = 0; r1 < 3; r1++)
    {
        for (int c2 = 0; c2 < 9; c2++)
        {
            for (int c1 = 0; c1 < 3; c1++)
            {
                _res[r1][c2] += inverse[r1][c1] * x_T[c1][c2];
            }
        }
    }

    // multiply above result by observed data
    double coef[3][1] = {{0}, {0}, {0}};
    for (int r1 = 0; r1 < 3; r1++)
    {
        for (int c2 = 0; c2 < 1; c2++)
        {
            for (int c1 = 0; c1 < 9; c1++)
            {
                coef[r1][c2] += _res[r1][c1] * tar_train[c1];
            }
        }
    }

    // get coefficients
    double c0 = coef[0][0] / -3000000000000;
    double c1 = coef[1][0] / -3000000000000;
    double c2 = coef[2][0] / -3000000000000;

    // get error for training data
    double preds_train[9];
    for (int i = 0; i < 9; i++)
    {
        preds_train[i] = c0 + (c1 * inp_train[i]) + (c2 * (inp_train[i] * inp_train[i]));
    }
    double err_train[9];
    for (int i = 0; i < 9; i++)
    {
        err_train[i] = preds_train[i] - tar_train[i];
    }

    // get error for remaining data
    double preds_test[2] = {
        c0 + (c1 * 1.99) + (c2 * 1.99 * 1.99),
        c0 + (c1 * 2.00) + (c2 * 2.00 * 2.00)
    };
    double err_test[2] = {
        2.4944 - preds_test[0],
        2.8142 - preds_test[1]
    };

    // set errors positive
    for (int i = 0; i < 9; i++)
    {
        if (err_train[i] < 0)
        {
            err_train[i] *= -1;
        }
    }
    for (int i = 0; i < 2; i++)
    {
        if (err_test[i] < 0)
        {
            err_train[i] *= -1;
        }
    }

    // find mean absolute error for train, test sets
    double sum = 0;
    for (int i = 0; i < 9; i++)
    {
        sum += err_train[i];
    }
    double mae_train = sum / 9;
    sum = 0;
    for (int i = 0; i < 2; i++)
    {
        sum += err_test[i];
    }
    double mae_test = sum / 2;
    printf("Training MAE: %f\n", mae_train);
    printf("Testing MAE: %f\n", mae_test);

    // plot
    FILE *model_line = fopen("model_line.temp", "w");
    for (int i = 0; i < 9; i++)
    {
        fprintf(model_line, "%f %f \n", inp_train[i], preds_train[i]);
    }
    fprintf(model_line, "%f %f \n", 1.99, preds_test[0]);
    fprintf(model_line, "%f %f \n", 2.00, preds_test[1]);
    FILE *tf_train_actual = fopen("train_act.temp", "w");
    for (int i = 0; i < 9; i++)
    {
        fprintf(tf_train_actual, "%f %f \n", inp_train[i], tar_train[i]);
    }
    FILE *tf_test_actual = fopen("test_act.temp", "w");
    fprintf(tf_test_actual, "%f %f \n", 1.99, 2.49438);
    fprintf(tf_test_actual, "%f %f \n", 2.00, 2.81412);
    char *commands[] = {
        "set title \"Actual Values, Predicted Values for Least Squares Fit\"",
        "set xlabel \"Year / 1,000\"",
        "set ylabel \"Population / 100,000,000\"",
        "plot 'model_line.temp' with linespoints, 'train_act.temp', 'test_act.temp' linetype 4",
    };
    FILE *gnuplotpipe = popen("gnuplot -persistent", "w");
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
    {
        fprintf(gnuplotpipe, "%s \n", commands[i]);
    }
    
    return 0;
}
