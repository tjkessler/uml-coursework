void sfht(float *,int ,int );
/*
** Procedure: realfft      PROGRAMMER: Ron Mayer  DATE: 04/10/93
**    Function: Computes FFT of a real valued data set
**    Inputs: n    = number of points
**            real = the array of data to be transformed.
**    Output: The first half of the array "real", (from 0 to n/2)
**            is filled with the real components of an FFT.
**            The second half of the array (from n/2+1 to n-1) is
**            filled with the imaginary components of an FFT (data at
**            n-1 is the lowest freq imag component).  
**    Note: Unlike alot of real valued ffts I do not generate the DC and
**          Nyquist imaginary component because for real valued data
**          these are always zero; allowing the entire transform to
**          fit in the original array.
**          The routine realifft is the inverse of this routine.
*/
void srealfft(int n, float *real)
{
 double a,b;
 int i,j;
 int IFLAG;
 IFLAG =1;
 sfht(real,n,IFLAG);
 for (i=1,j=n-1;i<j;i++,j--) {
  a = real[i];
  b = real[j];
  real[i] = (a+b)*0.5;
  real[j] = real[i]-b;
 }
}

//-----
//-----
void srealifft(int n, float *real)
{
 double a,b;
 int i,j;
 double scale;
 scale = 1.0/n;
 for (i=1,j=n-1;i<j;i++,j--) {
  a = real[i];
  b = real[j];
  real[j] = (a-b)*scale;
  real[i] = (a+b)*scale;
 }
 real[0]*=scale;
 real[i]*=scale;
 sfht(real,n,1);
}
