c-------------------------------------------------
c homework4, question 1: Travis Kessler
c-------------------------------------------------

	parameter(N = 16, PI = 4.D0*DATAN(1.D0))

	real x(N)
	complex x_i(N)
	complex x_k(N)
	complex c_wf(N)
	complex x_k_new(N)
	real x_i_new(N)

	real temp
	complex temp_c

	real delta_x
	real x_i_d(N)

c determine x, x_i (a.k.a. u(x))
c x_i = i * 2 * pi / N
c u(x_i) = cos(3x_i)
	do i=0,N-1
		x(i + 1) = i * 2 * pi / N
		x_i(i + 1) = cmplx(COS(3 * x(i + 1)), 0)
	enddo

c calculate X_k (a.k.a. U(k))
	do k=0,N-1
		x_k(k + 1) = cmplx(0, 0)
		do m=0,N-1
			temp = 2 * pi * k * m / N
			temp_c = cmplx(COS(temp), -1 * SIN(temp))
			x_k(k + 1) = x_k(k + 1) + (x_i(m + 1) * temp_c)
		enddo
	enddo

c compute weight factor
	do k=0,N/2
		c_wf(k + 1) = cmplx(0, k)
	enddo
	do k=(N/2)+1,N-1
		c_wf(k + 1) = cmplx(0, N - k)
	enddo

c calculate X_k_new (a.k.a. U_new(k) = C(k) * U(k))
	do i=1,N
		x_k_new(i) = x_k(i) * c_wf(i)
	enddo

c inverse DFT, X_k_new -> x_i_new == df/dx
	do k=0,N-1
		x_i_new(k + 1) = 0
		do m=0,N-1
			temp = 2 * pi * k * m / N
			temp_c = cmplx(COS(temp), SIN(temp))
			x_i_new(k + 1) = x_i_new(k + 1) + (x_k_new(k + 1) * temp_c)
		enddo
	enddo

c print results
	write(*,*) 'du/dx for x_i, i = (0, N)'
	write(*,*) 'N = ', N
	do i=1,N
		write(*,*) i - 1, x_i_new(i)
	enddo

c-------------------------------------------------
c PART 2
c-------------------------------------------------

	delta_x = 2 * pi / N

c evaluate u(x_i + dx/2)
	do i=0,N-1
		temp = (i * 2 * pi / N) + (2 * pi / N)
		x_i_d(i + 1) = cos(3 * temp)
	enddo

	write(*,*) ''
	write(*,*) 'u(x + dx/2) where dx/2 = 2 * pi / N'
	write(*,*) 'N = ', N
	do i=1,N
		write(*,*) i - 1, x_i_d(i)
	enddo

	end
