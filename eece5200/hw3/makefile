
all: q1j q1gs q2 q3 q4

q1j: question_1_jacobi.c
	gcc -o q1j.o question_1_jacobi.c

q1gs: question_1_gauss_siedel.c
	gcc -o q1gs.o question_1_gauss_siedel.c

q2: question_2.f
	gfortran -o q2.o question_2.f libport3.a

q3: question_3.f
	gfortran -o q3.o question_3.f libport3.a

q4: question_4.f
	gfortran -o q4.o question_4.f svdc.f
