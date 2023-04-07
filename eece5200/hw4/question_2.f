c-------------------------------------------------
c homework4, question 2: Travis Kessler
c-------------------------------------------------

	parameter(N_ELEM = 30, PI = 4.D0*DATAN(1.D0))

	real w
	real c
	real x_i(N_ELEM)
	complex u_x_i(N_ELEM)
	complex v_x_i(N_ELEM)
	complex U_k_i(N_ELEM)
	complex V_k_i(N_ELEM)
	complex A_k_i(N_ELEM)
	complex A_k_hat(N_ELEM)
	complex W_k_tilde(N_ELEM)
	real W_n(N_ELEM)
	real W_x(N_ELEM)

	real temp
	complex temp_c

	w = 2 * pi / N_ELEM
	c = w / 2

c compute x_i, u_x_i, v_x_i given N = 30, freq. index = 30/2 - 1 = 14
	do i=0,N_ELEM-1
		x_i(i + 1) = i * w
		u_x_i(i + 1) = cmplx(COS(7 * x_i(i + 1)), 0)
		v_x_i(i + 1) = cmplx(COS(7 * x_i(i + 1)), 0)
	enddo

c calculate fourier transforms U_k_i, V_k_i
	do k=0,N_ELEM-1
		U_k_i(k + 1) = cmplx(0, 0)
		V_k_i(k + 1) = cmplx(0, 0)
		do m=0,N_ELEM-1
			temp = w * k * m
			temp_c = cmplx(COS(temp), -1 * SIN(temp))
			U_k_i(k + 1) = U_k_i(k + 1) + (u_x_i(m + 1) * temp_c)
			V_k_i(k + 1) = V_k_i(k + 1) + (v_x_i(m + 1) * temp_c)
		enddo
	enddo

c fourier transform of u_i * v_i = A_k_i
	do k=0,N_ELEM-1
		A_k_i(k + 1) = cmplx(0, 0)
		do l=0,N_ELEM-1
			do m=0,N_ELEM-1
				temp_c = cmplx(0, 0)
				do n=0,N_ELEM-1
					temp = w * n * (l + m - k)
					if (temp.lt.0) then
						temp_c = temp_c + cmplx(COS(temp), -1 * SIN(temp))
					else
						temp_c = temp_c + cmplx(COS(temp), SIN(temp))
					endif
				enddo
				temp_c = temp_c / cmplx(N_ELEM * N_ELEM, 0)
				A_k_i = A_k_i + temp_c
			enddo
		enddo
	enddo

c 1/2 step time series, aliasing removal (compute A_k_hat)
	do k=0,N_ELEM-1
		A_k_hat(k + 1) = cmplx(0, 0)
		do l=0,N_ELEM-1
			do m=0,N_ELEM-1
				temp_c = cmplx(0, 0)
				do n=0,N_ELEM-1
					temp = w * n * (l + m - k)
					if (temp.lt.0) then
						temp_c = temp_c + cmplx(COS(temp), -1 * SIN(temp))
					else
						temp_c = temp_c + cmplx(COS(temp), SIN(temp))
					endif
				enddo
				temp = w * (l + m - k) / 2
				if (temp.lt.0) then
					temp_c = temp_c * cmplx(COS(temp), -1 * SIN(temp))
					temp_c = temp_c * 1 / N_ELEM / N_ELEM
				else
					temp_c = temp_c * cmplx(COS(temp), SIN(temp))
					temp_c = temp_c * 1 / N_ELEM / N_ELEM
				endif
				temp_c = U_k_i(l + 1) * V_k_i(m + 1) * temp_c
				A_k_hat(k + 1) = A_k_hat(k + 1) + temp_c
			enddo
		enddo
	enddo

c compute W_k_tilde
	do k=0,N_ELEM-1
		temp = w * k / 2
		temp_c = A_k_hat(k + 1) * cmplx(COS(temp), -1 * SIN(temp))
		temp_c = A_k_i(k + 1) + temp_c
		W_k_tilde(k + 1) = 0.5 * temp_c
	enddo

c inverse transform W_k_tilde -> w(n)
	do k=0,N_ELEM-1
		W_n(k + 1) = 0
		do m=0,N_ELEM-1
			temp = w * k * m
			temp_c = cmplx(COS(temp), SIN(temp))
			W_n(k + 1) = W_n(k + 1) + (W_k_tilde(k + 1) * temp_c)
		enddo
	enddo

c print x, w(n)

	write(*,*) ' x          ', '       w(n)'
	do i=1,N_ELEM
		write(*,*) x_i(i), W_n(i)
	enddo

	end
