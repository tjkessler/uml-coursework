from matplotlib import pyplot as plt


def main():

    with open('jacobi_res.txt', 'r') as txt_file:
        lines = txt_file.readlines()
        jacobi_errs = []
        for line in lines:
            jacobi_errs.append(float(line.split(' ')[1]))
    txt_file.close()

    with open('gauss_siedel_res.txt', 'r') as txt_file:
        lines = txt_file.readlines()
        gs_errs = []
        for line in lines:
            gs_errs.append(float(line.split(' ') [1]))
    txt_file.close()

    x_jac = [i for i in range(len(jacobi_errs))]
    x_gs = [i for i in range(len(gs_errs))]

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.xlabel('Iteration')
    plt.ylabel('Convergence Error')
    plt.plot(x_jac, jacobi_errs, color='red', label='Jacobi Iteration')
    plt.plot(x_gs, gs_errs, color='blue', label='Gauss-Siedel Iteration')
    plt.legend(loc='upper right')
    plt.savefig('convergence_curve.png')


if __name__ == '__main__':

    main()
