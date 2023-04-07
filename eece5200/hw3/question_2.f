c-------------------------------------------------
c homework3, question 2: Travis Kessler
c-------------------------------------------------

	parameter (LDA=10, N=3)
	real A(LDA,LDA), B(LDA), C(LDA)
	real WR(LDA), WI(LDA), Z(LDA,LDA)
	complex E(LDA,LDA)
	complex CC(LDA), DD(LDA)
	data (A(1,j), j=1,3) / 0.0, 1.0, 0.0/
	data (A(2,j), j=1,3) / 0.0, 0.0, 1.0/
	data (A(3,j), j=1,3) / -6.0, -11.0, -6.0/
	data (B(j), j=1,3)   / 0.0, 0.0, 20.0/
	data (C(j), j=1,3)   / 60.0, 16.0, 1.0/

c find eigenvalues and vectors
	call EIGEN(LDA, N, A, WR, WI, Z)
	write(*,*) 'Eigenvalues:'
	do i=1,N
		write(*,*) i, wr(i), wi(i)
	enddo
	write(*,*) ''
	write(*,*) 'Eigenvectors:'
	write(*,*) '1                ', '2                 ', '3'
	do i=1,N
		write(*,*) z(i,1), z(i,2), z(i,3)
	enddo
	write(*,*) ''

c state transition matrix = e^(lambda t)

	write(*,*) 'State Transition Matrix:'
	do i=1,N
		write(*,*) i, '   e^(', wr(i), '*t)'
	enddo
	write(*,*) ''

c calculate complex E matrix
	j=1
	do while(j.le.N)
		if (wi(j).ne.0.0) then
			do i=1,N
				E(i,j) = cmplx(Z(i,j), Z(i,j+1))
				E(i,j+1) = cmplx(Z(i,j), -Z(i,j+1))
			enddo
			j = j+2
		else
			do i=1,N
				E(i,j) = cmplx(Z(i,j), 0.0)
			enddo
			j = j+1
		endif
	enddo

c C(T)*E -> CC
	do j=1,N
		CC(j) = cmplx(0.0, 0.0)
		do i=1,N
			CC(j) = CC(j) + C(i) * E(i,j)
		enddo
	enddo

c E(-1)*B
	do i=1,N
		B(i) = B(i)
		B(i+N) = 0.0
		do j=1,N
			A(i,j) = real(E(i,j))
			A(i,j+N) = -aimag(E(i,j))
			A(i+N,j) = aimag(E(i,j))
			A(i+N,j+N) = real(E(i,j))
		enddo
	enddo

	call gess (2*N,A,LDA,B,LDA,1,cond)

c solution -> complex
	do i=1,N
		DD(i) = cmplx(B(i), B(i+N)) * CC(i)
	enddo

c print solution
	write(*,*) 'y(t) = '
	do i=1,N
		write(*,*) '+ ', DD(I), ' * e^(', cmplx(wr(i),wi(i)), '*t)'
	enddo  

	END
