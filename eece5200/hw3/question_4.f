c-------------------------------------------------
c homework3, question 4: Travis Kessler
c-------------------------------------------------

	parameter(LDA=9, M=LDA, N=3)

	real YEAR(9), POP(9)
	real YEAR_TEST(2), POP_TEST(2)
	real PRED(9), PRED_TEST(2)
	real ERR2(9), ERR2_TEST(2)
	real UERR, UERR_TEST

	real A(LDA, N), w(M), c(N)
	real V(LDA, N), U(LDA, M), SIGMA(N), SCRATCH(N)
	real bhat(N), z(N)

c set up data
	do i=1,9
		YEAR(i) = 190 + i - 1
	enddo
	do j=10,11
		YEAR_TEST(j - 9) = 190 + j - 1
	enddo
	POP(1) = 75
	POP(2) = 91
	POP(3) = 105
	POP(4) = 122
	POP(5) = 131
	POP(6) = 150
	POP(7) = 179
	POP(8) = 203
	POP(9) = 227
	POP_TEST(1) = 249
	POP_TEST(2) = 281

c export data to file
	open(10, file='_actual_vals.temp')
		do i=1,9
			write(10,*) YEAR(i), POP(i)
		enddo
		do i=1,2
			write(10,*) YEAR_TEST(i), POP_TEST(i)
		enddo
	close(10)

c create A matrix, function of c1 + c2*year + c3*year^2, w/ pop data
	do i=1,M
		A(i,1) = 1
		A(i,2) = YEAR(i)
		A(i,3) = YEAR(i)**2
		w(i) = POP(i)
	enddo

c call SVD, obtain U, Sigma, V matrices
	call SVD(LDA,M,N,A,SIGMA,1,U,1,V,IERR,SCRATCH)

c ===============================================
c PART 1: NO DIMENSIONALITY REDUCTION
c ===============================================

	write(*,*) '=== NO DIMENSIONALITY REDUCTION ==='

c find coefficients c1, c2, c3
	do i=1,N
		bhat(i) = 0
		do k=1,M
			bhat(i) = bhat(i) + U(k,i) * w(k)
		enddo
		z(i) = bhat(i) / sigma(i)
	enddo
	do i=1,N
		c(i) = 0
		do j=1,N
			c(i) = V(i,j) * z(j) + c(i)
		enddo
	enddo
	write(*,*) 'COEFFICIENTS:'
	do i=1,3
		write(*,*) i, c(i)
	enddo

c use coefficients to predict year
	do i=1,LDA
		PRED(i) = c(1) + c(2)*YEAR(i) + c(3)*(YEAR(i)**2)
	enddo
	do i=1,2
		PRED_TEST(i) = c(1) + c(2)*YEAR_TEST(i) + c(3)*(YEAR_TEST(i)**2)
	enddo
	write(*,*) 'PREDICTIONS:'
	do i=1,9
		write(*,*) i, PRED(i), POP(i), YEAR(i)
	enddo
	do i=1,2
		write(*,*) i + 9, PRED_TEST(i), POP_TEST(i), YEAR_TEST(i)
	enddo

c export results to file
	open(10, file='_svd_full.temp')
		do i=1,9
			write(10,*) YEAR(i), PRED(i)
		enddo
		do i=1,2
			write(10,*) YEAR_TEST(i), PRED_TEST(i)
		enddo
	close(10)

c calculate error for fitting, testing sets
	UERR = 0
	do i=1,LDA
		UERR = UERR + (PRED(i) - POP(i))**2
	enddo
	UERR = SQRT(UERR / LDA)
	UERR_TEST = 0
	do i=1,2
		UERR_TEST = UERR_TEST + (PRED_TEST(i) - POP(i))**2
	enddo
	UERR_TEST = SQRT(UERR_TEST / 2)
	write(*,*) 'RMSE, fitting set: ', UERR
	write(*,*) 'RMSE, testing set: ', UERR_TEST

c ===============================================
c PART 2: REMOVE SMALLEST SINGULAR VALUE
c ===============================================

	write(*,*)
	write(*,*) '=== SMALLEST SINGULAR VALUE REMOVED ==='

c find coefficients c1, c2, c3
	argmin = 1e6
	do i=1,N
	 	if (sigma(i).lt.argmin) then
			argmin = sigma(i)
			smallest=i
	 	endif
	enddo
	do i=1,N
		bhat(i) = 0
		do k=1,M
			bhat(i) = bhat(i) + U(k,i) * w(k)
		enddo
		if (i.ne.smallest) then
			z(i) = bhat(i) / sigma(i)
		else
			z(i) = 0
		endif
	enddo
	do i=1,N
		c(i) = 0
		do j=1,N
			c(i) = V(i,j) * z(j) + c(i)
		enddo
	enddo
	write(*,*) 'COEFFICIENTS:'
	do i=1,3
		write(*,*) i, c(i)
	enddo

c use coefficients to predict year
	do i=1,LDA
		PRED(i) = c(1) + c(2)*YEAR(i) + c(3)*(YEAR(i)**2)
	enddo
	do i=1,2
		PRED_TEST(i) = c(1) + c(2)*YEAR_TEST(i) + c(3)*(YEAR_TEST(i)**2)
	enddo
	write(*,*) 'PREDICTIONS:'
	do i=1,9
		write(*,*) i, PRED(i), POP(i), YEAR(i)
	enddo
	do i=1,2
		write(*,*) i + 9, PRED_TEST(i), POP_TEST(i), YEAR_TEST(i)
	enddo

c export results to file
	open(10, file='_svd_reduced.temp')
		do i=1,9
			write(10,*) YEAR(i), PRED(i)
		enddo
		do i=1,2
			write(10,*) YEAR_TEST(i), PRED_TEST(i)
		enddo
	close(10)

c calculate error for fitting, testing sets
	UERR = 0
	do i=1,LDA
		UERR = UERR + (PRED(i) - POP(i))**2
	enddo
	UERR = SQRT(UERR / LDA)
	UERR_TEST = 0
	do i=1,2
		UERR_TEST = UERR_TEST + (PRED_TEST(i) - POP(i))**2
	enddo
	UERR_TEST = SQRT(UERR_TEST / 2)
	write(*,*) 'RMSE, fitting set: ', UERR
	write(*,*) 'RMSE, testing set: ', UERR_TEST

	end
