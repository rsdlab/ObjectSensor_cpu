
// g++ -o mptest0 mp_test0.cpp CMinpack.cpp

#include <stdio.h>
#include <math.h>
#include "CMinpack.hpp"

/* N = number of parameters in model, M = number of measured points to match. */
#define N       3
#define M       7

/* Array of 'measured' points. */
static double lorz[M] = {0.2,
                       0.4,
                       1.0,
                       2.0,
                       1.0,
                       0.4,
                       0.2};
/* Parameter list with inital estimates. */
static double x[N] = { 1.0,
                        1.0,
                        2.0};
/* msk[] allows selective activation of specific parameters. '1'
 * means to enable modifications, '0' means to disable modifications.
 */
static int msk[N] = { 1, 1, 1};

/* Function to calculate residuals: r = (model estimate - measured value).
 * Note that residuals are contained in fvec[i].
 *
 * This test model is a simple Lorentzian pulse with x[0] 
 * controlling the amplitude, x[1] controlling horizontal
 * scale, and x[2] controlling the horizontal offset.
 */
double fvec[M] = {0.0};
void fcn(int m,int n,double x[],double fv[],int *iflag)
{
    int i;
    double tmp;
    
    for (i = 0; i < m; i++) {
        tmp = x[2] - (double)i;
        fv[i] = (x[0] / (1.0 + x[1] * tmp * tmp))-lorz[i];
    }
}

int main()
{
  int m,n,info,i,ecode,nfev;
  double tol,fnorm,tmp;

  m = M;
  n = N;

  /* set convergence tolerances */
  tol = sqrt(DPMPAR0);
  fprintf(stderr,"Model: A/(1+B*(x-c)^2)\n");
  fprintf(stderr,"Target: 2/(1+(x-3)^2) -> A=2, B=1, C=3\n");
    
  /* solve system */
  CMinpack minpack;
  ecode = minpack.LM(fcn,m,n,x,msk,fvec,tol,&info,&nfev);
  //ecode=lmdif0(fcn,m,n,x,msk,fvec,tol,&info,&nfev);
    
  printf("Error code: %d\n",ecode);
  fnorm = minpack.enorm(m,fvec);
  printf("Final L2 norm, %lg \n",fnorm);
  printf("Exit parameter, %d \n",info);
  printf("Final approximate solution, A=%lf, B=%lf, C=%lf \n",
	 x[0],x[1],x[2]);
  printf("Residuals:\n");
  for (i=0;i<m;i++)
    printf("fvec[%d] = %lf\n",i,fvec[i]);
  
  return 0;
}

