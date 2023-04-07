from matplotlib import pyplot as plt
from re import compile
import numpy as np

float_re = compile(r'[0-9]{1,}\.[0-9]{1,}')


def main():

    with open('fort.11', 'r') as val_file:
        vals = val_file.readlines()
    val_file.close()
    vals = [v.split() for v in vals]
    x_vals = [float(v[0]) for v in vals]
    y_vals = [float(v[1]) for v in vals]

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.xlabel('Value of $e^z$')
    plt.ylabel('Chebishev-Derived Value')
    plt.scatter(x_vals, y_vals, color='blue')
    plt.title('Mean absolute error: $2.0266E-06$')
    plt.savefig('results_partb.png')
    plt.clf()

    with open('fort.12', 'r') as val_file:
        vals = val_file.readlines()
    val_file.close()
    vals = [v.split() for v in vals]
    x_vals = [float(v[0]) for v in vals]
    y_vals = [float(v[1]) for v in vals]

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.xlabel('Value of $d/dz$')
    plt.ylabel('Chebishev-Derived Value')
    plt.scatter(x_vals, y_vals, color='green')
    plt.title('Mean absolute error: $1.1176E-07$')
    plt.savefig('results_partc.png')
    plt.clf()


if __name__ == '__main__':

    main()
