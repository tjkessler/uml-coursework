c-------------------------------------------------
c homework3, question 3: Travis Kessler
c-------------------------------------------------

c set up params, vars
	parameter (LDA=3, N=3)
	real A(N,N), B(N,N), AINV(N,N), AINVB(N,N)
	real E(N,N), EINV(N,N), LAMBDA(N,N), ST(N,N), STTEMP(N,1)
	real WR(LDA), WI(LDA), Z(LDA,LDA)

c set up data, specified by problem
	data (A(1,i), i=1,3) / -2.0, 1.0, 2.0/
	data (A(2,i), i=1,3) / 2.0, 3.0, -2.0/
	data (A(3,i), i=1,3) / 1.0, -2.0, 3.0/

	data (B(1,i), i=1,3) / -2.0, 2.0, 4.0/
	data (B(2,i), i=1,3) / 3.0, 1.0, -1.0/
	data (B(3,i), i=1,3) / 0.0, 0.0, 0.0001/

c B inverse not allowed, find A inverse
	call matrix_inverse(A, AINV, N)

c calculate A(inv) * B
	AINVB = MATMUL(AINV, B)

c find eigenvectors, eigenvalues of A(inv) * B
	call EIGEN(LDA, N, AINVB, WR, WI, Z)

c invert eigenvalues and eigenvectors, prop. of A(-1)B as opposed to AB(-1)
	do i=1,N
		WR(i) = 1 / WR(i)
		do j=1,N
			Z(i,j) = 1 / Z(i,j)
		enddo
	enddo

c create E matrix, Lambda matrix
	do i=1,N
		do j=1,N
			E(i,j) = Z(i,j)
		enddo
		LAMBDA(i,i) = WR(i)
	enddo

c find E inverse
	call matrix_inverse(E, EINV, N)

c calculate state transition matrix, E * lambda * E inverse
	ST = MATMUL(E, LAMBDA)
	ST = MATMUL(ST, EINV)

c print eigenvalues, eigenvectors
	write(*,*) 'Eigenvalues:'
	do i=1,N
		write(*,*) i, WR(i)
	enddo
	write(*,*) ''
	write(*,*) 'Eigenvectors:'
	write(*,*) '1                ', '2                 ', '3'
	do i=1,N
		write(*,*) z(i,1), z(i,2), z(i,3)
	enddo
	write(*,*) ''

c print state transition matrix
	write(*,*) 'State transition matrix:'
	do i=1,N
		write(*,*) i, ST(i,1), ST(i,2), ST(i,3)
	enddo

	end
c === END ===


c invert matrix a of rank n, return to b
	subroutine matrix_inverse(a, b, n)

		real large, a(n,n), b(n,n), temp
		integer irow

		do i = 1,n
			do j = 1,n
				b(i,j) = 0.0
			end do
			b(i,i) = 1.0
		end do

		do i = 1,n

			large = a(i,i)
			do j = i,n
				if (a(j,i).gt.large) then
					large = a(j,i)
					irow = j
				end if
			end do

			if (large.gt.a(i,i)) then
				do k = 1,n
					temp = a(i,k)
					a(i,k) = a(irow,k)
					a(irow,k) = temp
					temp = b(i,k)
					b(i,k) = b(irow,k)
					b(irow,k) = temp
				end do
			end if

			temp = a(i,i)
			do j = 1,n
				a(i,j) = a(i,j)/temp
				b(i,j) = b(i,j)/temp
			end do

			do j = i+1,n
				temp = a(j,i)
				do k = 1,n
					a(j,k) = a(j,k) - temp*a(i,k)
					b(j,k) = b(j,k) - temp*b(i,k)
				end do
			end do

		end do

		do i = 1,n-1
			do j = i+1,n
				temp = a(i,j)
				do l = 1,n
					a(i,l) = a(i,l)-temp*a(j,l)
					b(i,l) = b(i,l)-temp*b(j,l)
				end do
			end do
		end do

	end
