c-------------------------------------------------
c homework5, question 1: Travis Kessler
c-------------------------------------------------

c Compute Chebyshev sum at point z given supplied coefficients and order
	real function chebyshev_sum(z, coef, N)
		real coef(0:N)
		real T(0:N)
		x_ = 2 * z - 1
		T(0) = 1.0
		T(1) = x_
		do i=2,N
			T(i) = 2 * T(i-1) * x_ - T(i-2)
		enddo
		chebyshev_sum = 0.0
		do i=0,N
			chebyshev_sum = chebyshev_sum + coef(i) * T(i)
		enddo
		return
	end

c Set up variables
	parameter(N=3, PI=4.D0*DATAN(1.D0))
	real T(0:N,0:N), coef(0:N)
	real E_real(0:16), E_guess(0:16)

c Compute f(x) (stored in coef), T(i,j)
	do i=0,N
		theta_ = i * pi / n
		x_ = COS(theta_)
		coef(i) = exp((x_ + 1) / 2)
		do j=0,N
			T(i,j) = COS(j * theta_)
		enddo
	enddo

c Given f(x) = [T]a (form of AX = B), compute unknowns a (i.e. X)
	call gess(N+1, T, N+1, coef, N+1, 1, cond)
	write(*,*) "Coefficients a:"
	do i=0,N
		write(*,*) "a", i, " = ", coef(i)
	enddo
	write(*,*) ""

c Given coefficients a, find Chebyshev sum at each interval (0, 1) di = 1/16;
c then, compute the mean absolute error of all samples
	write(*,*) "Interval=16; actual e^z compared to Cheb. derived:"
	write(*,*) "Actual ", "Chebyshev-Derived"
	error_ = 0.0
	do i=0,16
		z_samp_ = i * 1 / 16.0
		E_real(i) = exp(z_samp_)
		E_guess(i) = chebyshev_sum(z_samp_, coef, N)
		write(*,*) E_real(i), E_guess(i)
		write(11,*) E_real(i), E_guess(i)
		error_ = error_ + ABS(E_real(i) - E_guess(i))
	enddo
	write(*,*) "Given 16 samples of z, mean absolute error = ", error_ / 16.0
	write(*,*) ""

c Calculate derivative of Chebyshev sum w.r.t. z with dz = 16, compare to actual
	write(*,*) "dz=16; deriv. of actual compared to Cheb. derived:"
	write(*,*) "Actual ", "Chebyshev-Derived"
	error_ = 0.0
	do i=0,15
		der_ = E_real(i + 1) - E_real(i)
		deg_ = E_guess(i + 1) - E_guess(i)
		write(*,*) der_, deg_
		write(12,*) der_, deg_
		error_ = error_ + ABS(der_ - deg_)
	enddo
	write(*,*) "Deriv. mean absolute error: ", error_ / 16.0

	end
