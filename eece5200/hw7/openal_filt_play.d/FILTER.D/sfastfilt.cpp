void srealfft (int,float *);//dft
void srealifft(int,float *);//idft
/*------------------------------
/* USAGE OF srealfft(x,2*N)
	input: real x[0:2*N-1]
	output: complex
	format output
	RE(x)  IM(x)
	x[0]	0
	x[1]	x[2*N-1]
	.
	.
	.
	
	x[N-1]	x[N+1]
	x[N]	0	
---------------------------------*/
void sfastfilt (int N, float *h,float *x, float *s)
{
float rex,imx,reh,imh;
int i,N2;

N2=N*2;

for (i=N;i<N2;i++)x[i]=0;
srealfft (N2,x);
//i=0:
		rex = x[0]; imx=0;
		reh = h[0]; imh=0;
	x[0]= rex*reh-imx*imh;
for (i=1;i<N;i++)
{ 
		rex = x[i]; imx=x[N2-i];
		reh = h[i]; imh=h[N2-i];
		x[i]   = rex*reh-imx*imh;
		x[N2-i]= rex*imh+imx*reh;
}

//i=N:
		rex = x[N]; imx=0;
		reh = h[N]; imh=0;
		x[N]= rex*reh-imx*imh;
//

srealifft (N2,x);
for (i=0;i<N;i++)x[i]=x[i]+s[i]; 
for (i=0;i<N;i++)s[i]=x[i+N];
}
//-------------
