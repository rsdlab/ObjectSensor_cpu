
//
// CMinpack class for non-linear optimization
//
// Jaeil Choi
// Last modified in Dec, 2005
//
// This code is a C version of MINPACK which can be obtained at
//   Netlib or http://www.crbond.com/scientific.htm (It's in public domain.)
// LMDIF   ( Levenberg-Marquardt, no Jacobian )
//   minimize the sum of the squares of M nonlinear functions in N
//   variables by a modification of the Levenberg-Marquardt algorithm
//

#include <iostream>
#include <cstdio>
#include <cmath>
#include "CMinpack.hpp"
#include "lm.h"
using namespace std;

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))
// extern double dpmpar[];
// double dpmpar[] = { 2.220446049250313e-16,
// 		    2.225073858507201e-308,
//		    1.797693134862316e308 };

// ===================================================================
// wrapper functions
// ===================================================================

int CMinpack::LM(void f(double *x, double *fvec, int nx, int nf, int *iflag),
		 double x[], double fvec[], int nx, int nf, int maxiter, double tol)
{
  // Levenberg-Marquardt method : minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  // f     : user-supplied subroutine which calculates the objective functions
  // nf    : number of functions
  // nx    : number of variables    nf >= nx
  // x     : O [N] solution vector (initial estimate => final solution)
  // fvec  : O [M] output function values evaluated at the output solution 'x'
  // tol   : terminate if the relative error in the sum of squares is at most 'tol'
  //         or the relative error between 'x' and the solution is at most 'tol'
  // info  : O  termination code (0...7)
  // nfev  : O  the number of evaluation of the function f

  int info = 0, nfev = 0;

  if (verbose) {
    printf("Levenberg-Marquardt optimization (LM - simplified)\n");
    printf("  with %d parameters for %d measurements\n", nx, nf);
    printf("  tol = %g   \n", tol);
  }

  int ret = lmdif0( f, x, fvec, nx, nf, maxiter, tol, &info, &nfev );

  switch (info) {
  case 0:  sprintf(termination, "improper_input_parameters");  break;
  case 1:  sprintf(termination, "F_error_<_ftol");  break;
  case 2:  sprintf(termination, "delta_<_xtol*xnorm");  break;
  case 3:  sprintf(termination, "both_error_less_than_tol");  break;
  case 4:  sprintf(termination, "fvec_orthogonal_to_Jacobian");  break;
  case 5:  sprintf(termination, "nfev_>_200*(N+1)");  break;
  case 6:  sprintf(termination, "too_small_tol_for_F_error");  break;
  case 7:  sprintf(termination, "too_small_tol_for_x_error");  break;
  case 99:  sprintf(termination, "max_iterations");  break;
  default: sprintf(termination, "unknown"); break;
  }
  if (verbose) {
    printf("  Result: %d evaluations   TerminatedBy: %s\n", nfev, termination);
  }

  return ret;
}

void CMinpack::LM(void f(double *x, double *fvec, int nx, int nf, int *iflag),
		  double x[], double fvec[], int nx, int nf, int msk[], int maxit,
		  double ftol, double xtol, double gtol, int maxfev, double epsfcn,
		  double diag[], int mode, double factor, int *info, int *nfev,
		  double **fjac, int ipvt[], double qtf[],
		  double wa1[], double wa2[], double wa3[], double wa4[])
{
  // Levenberg-Marquardt method : minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  // f     : user-supplied subroutine which calculates the objective functions
  // nf    : number of functions
  // nx    : number of variables    nf >= nx
  // x     : IO [N] solution vector (initial estimate => final solution)
  // msk   :    [N] selective (de)activation of specific variables (1:active 0:fixed). If NULL, all variables are active
  // fvec  : O  [M] output function values evaluated at the output solution 'x'
  // ftol  : terminate if relative reductions are at most 'ftol'
  // xtol  : terminate if relative error between two consecutive iterates is at most 'xtol'
  // gtol  : terminate if cosine of angle between 'fvec' and any column of Jacobian is at most 'gtol'
  // maxfev: terminate if the number of calls to 'f' is at least 'maxfev' by the end of an iteration
  // epsfcn: forward-difference approximation assumes that the relative errors in the functions are of the order of 'epsfcn'
  // diag  : [N] (mode==1) internally set, (mode==2) multiplicative scale factors for the variables
  // mode  : (mode==1) scaled internally, (mode==2) scale is specified by 'diag'
  // factor: (0.1, 100.0) 100.0 is generally recommended. initial step bound = 'factor' * 'norm of diag*x'
  // nprint: if positive, special calls of 'f' with iflag=0 at the first iteration and every 'nprint' iterations thereafter.
  // info  : O  termination code (0...8)
  // nfev  : O  the number of iterations
  // fjac  : O [M][N] upper triangular matrix R with nonincreasing diagonal entries,
  //         s.t. Pt * (Jt * J) * P = Rt * R, where P is permutation matrix and J is Jacobian
  // ipvt  : [N] working array of int
  // qtf   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [M] working array of double

  if (verbose) {
    printf("Levenberg-Marquardt optimization (LM - detailed)\n");
    printf("  with %d parameters for %d measurements\n", nx, nf);
    printf("  ftol = %g  xtol = %g  gtol = %g   maxfev = %d \n", ftol, xtol, gtol, maxfev);
  }

  lmdif( f, x, fvec, nx, nf, msk, maxit,
	 ftol, xtol, gtol, maxfev, epsfcn,
	 diag, mode, factor, info, nfev,
	 fjac, ipvt, qtf, wa1, wa2, wa3, wa4 );

  switch (*info) {
  case 0:  sprintf(termination, "improper_input_parameters");  break;
  case 1:  sprintf(termination, "F_error_<_ftol");  break;
  case 2:  sprintf(termination, "delta_<_xtol*xnorm");  break;
  case 3:  sprintf(termination, "both_error_<_tol");  break;
  case 4:  sprintf(termination, "cos(angle_fvec_Jacobian)_<_gtol");  break;
  case 5:  sprintf(termination, "n_ftn_evaluation_>_maxfev");  break;
  case 6:  sprintf(termination, "too_small_ftol_for_F_error");  break;
  case 7:  sprintf(termination, "too_small_xtol_for_x_error");  break;
  case 8:  sprintf(termination, "too_small_gtol_for_cos(angle)");  break;
  case 99:  sprintf(termination, "max_iterations");  break;
  default: sprintf(termination, "unknown"); break;
  }
  if (verbose) {
    printf("  Result: %d evaluations   terminatedBy: %s\n", *nfev, termination);
  }
}

// ===================================================================
// Levenberg-Marquardt method  (an implementation by Manolis Lourakis)
// ===================================================================

int CMinpack::LM2(void f(double *x, double *me, int nx, int nm, void *adata),
		  double *x, double *me, int nx, int nm, int itmax, double *opts, void *adata)
{
  // x     : [nx] unknown parameters (initial estimate -> solution)
  // me    : [nm] measurement error vector (uninitialized allocted buffer)
  // nx    : number of unknown parameters
  // nm    : number of measurments (size of the measurement error vector)
  // itmax : maximum number of iterations
  // opts  : [5] opts[0-4] = minimization options (NULL for defaults)
  //            \tau      : scale factor for initial \mu
  //            \epsilon1 : stopping thresholds for ||J^T e||_inf
  //            \epsilon2 : stopping thresholds for ||Dp||_2
  //            \epsilon3 : stopping thresholds for ||e||_2
  //            \delta    : step used in difference approximation to the jacobian.
  //                        If \delta<0, the jacobian is approximated with central differences which are more accurate (but slower!)
  //                        compared to the forward differences employed by default.
  // adata : pointer to possibly needed additional data for 'f'. set to NULL if not needed
  if (nm < nx) {
    fprintf(stderr, "Error (CMinpack::LM2): less measurements (%d) than unknowns (%d)\n", nm, nx);
    return 0;
  }
  if (logging) openLogFile();
  FILE *fp = (verbose ? stdout : (logging ? logfp : NULL));
  if (fp) {
    fprintf(fp, "Levenberg-Marquardt optimization (LM2)\n");
    fprintf(fp, "  with %d parameters for %d measurements\n", nx, nm);
    if (opts) {
      fprintf(fp, "  options : tau=%g eps1=%g eps2=%g eps3=%g delta=%g\n",
	      opts[0], opts[1], opts[2], opts[3], opts[4]);
    } else {
      fprintf(fp, "  options : (default) tau=%g eps1=%g eps2=%g eps3=%g delta=%g\n",
	      LM_INIT_MU, LM_STOP_THRESH, LM_STOP_THRESH, LM_STOP_THRESH, LM_DIFF_DELTA);
    }
  }

  // additional parameters are
  //   info  : O [LM_INFO_SZ] information regarding the minimization. set to NULL if don't care
  //           info[0]= ||e||_2 at initial x.
  //           info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, \mu/max[J^T J]_ii ], all computed at estimated x.
  //           info[5]= # iterations, info[6]=reason for terminating
  //           info[7]= # function evaluations,  info[8]= # jacobian evaluations
  //   work  : I [] working memory, (NULL or at least LM_DIF_WORKSZ(nx, nm)*sizeof(double))
  //   covar : O [mxm] Covariance matrix corresponding to LS solution. set to NULL if not needed.
  // double info[LM_INFO_SZ];  // moved to CMinpack member variable

  // LM method ( double precision, unconstrained, without jacobian )
  //    actual function body is defined as LEVMAR_DIF in 'lm_core.c'.
  int niter = dlevmar_dif( f, x, me, nx, nm, itmax, opts, info, NULL, NULL, adata, fp );

  switch ((int)info[6]) {   // reason for termination
  case 1:  sprintf(termination, "small_gradient_JtTe"); break;  // decrease eps1
  case 2:  sprintf(termination, "small_Dp");            break;	// decrease eps2 (for relative change in x)
  case 3:  sprintf(termination, "iteration_>_itmax");   break;	// increase itmax
  case 4:  sprintf(termination, "singular_matrix");     break;
  case 5:  sprintf(termination, "no_error_reduction");  break;
  case 6:  sprintf(termination, "small_||e||_2");       break;	// decrease eps3
  default: sprintf(termination, "unknown");             break;
  }
  if (fp) {
    fprintf(fp, "  Result: %d iterations  %d evaluations   terminatedBy: %s\n",
	   (int)info[5], (int)info[7], termination);
    fprintf(fp, "  Result: (%s)  SUM(|e|2) :  %g  =>  %g \n",
	    (niter>0 ? "successful" : "failed"), info[0], info[1]);
  }
  if (logging) closeLogFile();
  return niter;
}


// ===================================================================
// MINPACK's original functions
// ===================================================================

// -------------------------------------------------------------------
// MINPACK -- lmdif.c  (original lmdif function)
// -------------------------------------------------------------------

void CMinpack::lmdif(void f(double *x, double *fvec, int nf, int nx, int *iflag),
		     double x[], double fvec[], int nx, int nf,
		     int msk[], int maxit, double ftol,
		     double xtol, double gtol, int maxfev, double epsfcn, double diag[],
		     int mode, double factor, int *info, int *nfev, double **fjac,
		     int ipvt[], double qtf[],
		     double wa1[], double wa2[], double wa3[], double wa4[])
{
  // Levenberg-Marquardt method : minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  // f     : user-supplied subroutine which calculates the objective functions
  // nf    : number of functions
  // nx    : number of variables    nf >= nx
  // x     : O [N] solution vector (initial estimate => final solution)
  // msk   :   [N] selective (de)activation of specific variables (1:active 0:fixed). If NULL, all variables are active
  // maxit : maximum number of iterations
  // fvec  : O [M] output function values evaluated at the output solution 'x'
  // ftol  : terminate if relative reductions are at most 'ftol'
  // xtol  : terminate if relative error between two consecutive iterates is at most 'xtol'
  // gtol  : terminate if cosine of angle between 'fvec' and any column of Jacobian is at most 'gtol'
  // maxfev: terminate if the number of calls to 'f' is at least 'maxfev' by the end of an iteration
  // epsfcn: forward-difference approximation assumes that the relative errors in the functions are of the order of 'epsfcn'
  // diag  : [N] (mode==1) internally set, (mode==2) multiplicative scale factors for the variables
  // mode  : (mode==1) scaled internally, (mode==2) scale is specified by 'diag'
  // factor: (0.1, 100.0) 100.0 is generally recommended. initial step bound = 'factor' * 'norm of diag*x'
  // nprint: if positive, special calls of 'f' with iflag=0 at the first iteration and every 'nprint' iterations thereafter.
  // info  : O  0:improper_input_parameters, 1:ftol, 2:xtol, 3:ftol_and_xtol,
  //         4:gtol, 5:maxfev, 6:too_small_ftol, 7:too_small_xtol, 8:too_small_gtol
  // nfev  : O  the number of iterations
  // fjac  : O [M][N] upper triangular matrix R with nonincreasing diagonal entries,
  //         s.t. Pt * (Jt * J) * P = Rt * R, where P is permutation matrix and J is Jacobian
  // ipvt  : [N] working array of int
  // qtf   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [N] working array of double
  // wa1   : [M] working array of double
  int i,iflag,iter,j,l, it;
  double actred,delta=0,dirder,epsmch,fnorm,fnorm1,gnorm;
  double par,pnorm,prered,ratio,sum,temp=0,temp1,temp2,xnorm=0;

  /* initialize */
  epsmch = DPMPAR0;

  *info = 0;
  iflag = 0;
  *nfev = 0;

  /* check for input parameter errors */
  if ((nx <= 0) || (nf < nx) || (maxfev <= 0) || (factor <= 0)) {
    printf("Error (LM): invalid input nx=%d nf=%d maxfev=%d factor=%g\n", nx, nf, maxfev, factor);
    return;
  }
  if (mode == 2) {
    for (j = 0; j < nx; j++)
      if (diag[j] <= 0) return;
  }

  /* evaluate the function at the starting point and calculate its norm */
  iflag = 1;
  f(x,fvec,nx,nf,&iflag);
  *nfev = 1;
  if (iflag < 0) {
    *info = iflag;
    return;
  }
  fnorm = enorm(nf,fvec);

  /* initialize levenberg-marquardt counters */
  par = 0;
  iter = 1;

  /* outer loop */
  for (it = 0; it < maxit; it++) {
    /* calculate jacobian matrix */
    iflag = 2;
    fdjac2(f,x,fvec,nx,nf,fjac,&iflag,epsfcn,wa4);
    *nfev += nx;
    if (iflag < 0) {
      *info = iflag;
      return;
    }
    f(x,fvec,nx,nf,&iflag);

    if (verbose) printf("  fnorm %.15e\n",enorm(nf,fvec));

    /* compute the qr factorization of the jacobian */
    qrfac(nf,nx,fjac,1,ipvt,wa1,wa2,wa3);
    if (iter == 1) {
      if (mode != 2) {
	for (j = 0;j < nx; j++) {
	  diag[j] = wa2[j];
	  if (wa2[j] == 0.0) diag[j] = 1.0;
	}
      }
      for (j = 0;j < nx; j++)
	wa3[j] = diag[j] * x[j];
      xnorm = enorm(nx,wa3);
      delta = factor * xnorm;
      if (delta == 0) delta = factor;
    }
    for (i = 0; i < nf; i++)
      wa4[i] = fvec[i];
    for (j = 0;j < nx; j++) {
      if (fjac[j][j] != 0.0) {
	sum = 0.0;
	for (i = j;i < nf; i++)
	  sum += fjac[j][i] * wa4[i];
	temp = -sum / fjac[j][j];
	for (i = j; i < nf; i++)
	  wa4[i] += fjac[j][i] * temp;
      }
      fjac[j][j] = wa1[j];
      qtf[j] = wa4[j];
    }
    /* compute the norm of the scaled gradient */
    gnorm = 0.0;
    if (fnorm != 0.0) {
      for (j = 0; j < nx; j++) {
	l = ipvt[j];
	if (wa2[l] == 0.0) continue;
	sum = 0.0;
	for (i = l; i <= j; i++)
	  sum += fjac[j][i] * qtf[i] / fnorm;
	gnorm = max(gnorm,fabs(sum/wa2[l]));
      }
    }
    /* test for convergence of the gradient norm */
    if (gnorm <= gtol) *info = 4;
    if (*info != 0) {
      *info = iflag;
      return;
    }
    /* rescale if necessary */
    if (mode != 2) {
      for (j = 0; j < nx; j++)
	diag[j] = max(diag[j],wa2[j]);
    }
    /* beginning of inner loop */
    do {
      /* determine the levenberg-marquardt parameter */
      lmpar(nx,fjac,ipvt,diag,qtf,delta,&par,wa1,wa2,wa3,wa4);
      for (j = 0;j < nx; j++) {
	wa1[j] = -wa1[j];
	wa2[j] = x[j] + wa1[j];
	wa3[j] = diag[j] * wa1[j];
      }
      pnorm = enorm(nx,wa3);
      if (iter == 1) delta = min(delta,pnorm);
      iflag = 1;
      f(wa2,wa4,nx,nf,&iflag);
      (*nfev)++;
      if (iflag < 0) {
	*info = iflag;
	return;
      }
      fnorm1 = enorm(nf,wa4);
      actred = -1.0;
      if (0.1 * fnorm1 < fnorm)
	actred = 1.0 - (fnorm1*fnorm1/(fnorm*fnorm));
      for (j = 0;j < nx; j++) {
	wa3[j] = 0.0;
	l = ipvt[j];
	temp = wa1[l];
	for (i = 0; i <= j; i++)
	  wa3[i] += fjac[j][i] * temp;
      }
      temp1 = enorm(nx,wa3) / fnorm;
      temp2 = sqrt(par) * pnorm / fnorm;
      prered = temp1*temp1 + temp2*temp2 / 0.5;
      dirder = -(temp1*temp1 + temp2*temp2);
      ratio = 0.0;
      if (prered != 0.0) ratio = actred/prered;
      if (ratio <= 0.25) {
	if (actred > 0.0) temp = 0.5;
	if (actred < 0.0) temp = 0.5*dirder/(dirder+0.5*actred);
	delta = temp * min(delta,pnorm/0.1);
	par /= temp;
      }
      else {
	if ((par == 0.0) || (ratio >= 0.75)) {
	  delta = pnorm / 0.5;
	  par *= 0.5;
	}
      }
      if (ratio >= 0.0001) {
	for (j = 0; j < nx; j++) {
	  if (!msk || msk[j])  x[j] = wa2[j];  /* handle masked variables */
	  wa2[j] = diag[j] * x[j];
	}
	for (i = 0; i < nf; i++)
	  fvec[i] = wa4[i];
	xnorm = enorm(nx,wa2);
	fnorm = fnorm1;
	iter++;
      }
      if ((fabs(actred) <= ftol) && (prered <= ftol) &&
	  (0.5*ratio <= 1.0)) *info = 1;
      if (delta <= xtol*xnorm) *info = 2;
      if ((fabs(actred) <= ftol) && (prered <= ftol) &&
	  (0.5*ratio <= 1.0) && (*info == 2)) *info = 3;
      if (*nfev >= maxfev) *info = 5;
      if ((fabs(actred) <= epsmch) && (prered <= epsmch) &&
	  (0.5*ratio <= 1.0)) *info = 6;
      if (delta <= epsmch*xnorm) *info = 7;
      if (gnorm <= epsmch) *info = 8;
      //printf("info=%d iflag=%d\n", *info, iflag);
      if (*info != 0) return;
    } while (ratio <= 0.0001);
  }
  if (*info == 0 && it >= maxit) *info = 99;
}


// -------------------------------------------------------------------
// MINPACK -- lmdif0.c  (modified lmdif with simpler interface and mask)
// -------------------------------------------------------------------

int CMinpack::lmdif0(void f(double *x, double *fvec, int nx, int nf, int *iflag),
		     double x[], double fvec[], int nx, int nf,
		     int maxit, double tol, int *info, int *nfev)
{
  // Levenberg-Marquardt method : minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  // f     : user-supplied subroutine which calculates the objective functions
  // nf    : number of functions
  // nx    : number of variables    nf >= nx
  // x     : O [N] solution vector (initial estimate => final solution)
  // fvec  : O [M] output function values evaluated at the output solution 'x'
  // maxit : maximum number of iterations
  // tol   : terminate if the relative error in the sum of squares is at most 'tol'
  //         or the relative error between 'x' and the solution is at most 'tol'
  // info  : O  0:improper_input_parameters, 1:tol_1st_case, 2:tol_2nd_case,
  //         3:tol_both, 4:fvec_orthogonal_to_Jacobian, 5:nfev>200*(N+1),
  //         6:too_small_tol, 7:too_small_tol
  // nfev  : O  the number of iterations
  int    j, maxfev, mode;
  int    *ipvt;
  double ftol, xtol, gtol, epsfcn, factor;
  double *diag, **fjac, *qtf, *wa1, *wa2, *wa3, *wa4;

  /* Check input parameters */
  if (nx <= 0 || nf < nx || tol < 0.0) {
    *info = 0;
    return(1);
  }
  /* Allocate memory for working arrays. */
  ipvt = (int *)calloc(nx, sizeof(int));
  diag = (double *)calloc(nx, sizeof(double));
  qtf = (double *)calloc(nx, sizeof(double));
  wa1 = (double *)calloc(nx, sizeof(double));
  wa2 = (double *)calloc(nx, sizeof(double));
  wa3 = (double *)calloc(nx, sizeof(double));
  wa4 = (double *)calloc(nf, sizeof(double));

  /* create 2d matrix for Jacobian */
  fjac = (double **)calloc(nx, sizeof(double *));
  for (j=0;j<nx;j++) fjac[j] = (double *)calloc(nf, sizeof(double));

  /* set convergence tolerances */
  ftol = tol;
  xtol = tol;
  gtol = 0.0;

  maxfev = 800;
  epsfcn = 0.0;
  mode = 1;
  factor = 100;
  *nfev = 0;

  lmdif(f, x, fvec, nx, nf, NULL, maxit, ftol, xtol, gtol, maxfev, epsfcn, diag, mode,
        factor, info, nfev, fjac, ipvt, qtf, wa1, wa2, wa3, wa4);

  if (*info == 8) *info = 4;
  for (j=0;j<nx;j++) free(fjac[j]);
  free(fjac);
  free(wa4);
  free(wa3);
  free(wa2);
  free(wa1);
  free(qtf);
  free(diag);
  free(ipvt);
  return(0);
}


// -------------------------------------------------------------------
// MINPACK -- enorm.c
// -------------------------------------------------------------------

double CMinpack::enorm(int n, double x[])
{
  int i;
  double sum;
  sum = x[0] * x[0];
  for (i=1;i<n;i++)  sum += x[i]*x[i];
  return sqrt(sum);
}


double CMinpack::rownorm(int m, int r, int c, double **x)
{
  /* compute row norm for column c from row r to row m */
  int i;
  double sum;
  sum = x[r][c] * x[r][c];
  for (i = r+1; i < m; i++)  sum += x[i][c] * x[i][c];
  return sqrt(sum);
}

double CMinpack::colnorm(int m, int r, int c, double **x)
{
  int i;
  double sum;
  sum = x[r][c] * x[r][c];
  for (i = c+1;i < m; i++)  sum += x[r][i] * x[r][i];
  return sqrt(sum);
}

// -------------------------------------------------------------------
// MINPACK -- qrfac.c
// -------------------------------------------------------------------

void CMinpack::qrfac(int m,int n,double **a,int pivot,int ipvt[],
		     double rdiag[],double acnorm[],double wa[])
{
  int i,j,jp1,k,kmax,minmn;
  double ajnorm,epsmch,sum,temp;

  /* get machine precision */
  epsmch = DPMPAR0;
  /* compute the initial column norms and initialize several arrays */
  for (j = 0;j < n; j++) {
    acnorm[j] = colnorm(m,j,0,a);
    rdiag[j] = acnorm[j];
    wa[j] = rdiag[j];
    if (pivot) ipvt[j] = j;
  }
  /* reduce a to r with householder transformations */
  minmn = (m < n) ? m : n;
  for (j = 0;j < minmn; j++) {
    if (pivot) {
      /* bring column with largest norm into the pivot position */
      kmax = j;
      for  (k = j;k < n; k++)
	if (rdiag[k] > rdiag[kmax]) kmax = k;
      if (kmax != j) {
	for (i = 0;i < m; i++) {
	  temp = a[j][i];
	  a[j][i] = a[kmax][i];
	  a[kmax][i] = temp;
	}
	rdiag[kmax] = rdiag[j];
	wa[kmax] = wa[j];
	k = ipvt[j];
	ipvt[j] = ipvt[kmax];
	ipvt[kmax] = k;
      }
    }
    /* compute the householder transformation */
    ajnorm = colnorm(m,j,j,a);
    if (ajnorm != 0.0) {
      if (a[j][j] < 0.0) ajnorm = -ajnorm;
      for (i = j;i < m; i++)  a[j][i] /= ajnorm;
      a[j][j] += 1.0;
      jp1 = j + 1;
      if (n > jp1) {
	for (k = jp1;k < n; k++) {
	  sum = 0.0;
	  for (i = j;i < m; i++)  sum += a[j][i]*a[k][i];
	  temp = sum / a[j][j];
	  for (i = j; i < m; i++)  a[k][i] -=temp*a[j][i];
	  if (!pivot || !rdiag[k]) continue;
	  temp = a[k][j] / rdiag[k];
	  rdiag[k] *= sqrt(max(0.0,1.0-temp*temp));
	  if (0.5 * (rdiag[k] * rdiag[k] / (wa[k] * wa[k])) > epsmch) continue;
	  rdiag[k] = colnorm(m,k,jp1,a);
	  wa[k] = rdiag[k];
	}
      }
    }
    rdiag[j] = -ajnorm;
  }
}

// -------------------------------------------------------------------
// MINPACK -- qrsolv.c
// -------------------------------------------------------------------

void CMinpack::qrsolv(int n,double **r,int ipvt[],double diag[],
		      double qtb[],double x[],double sdiag[],double wa[])
{
  int i,j,jp1,k,kp1,l,nsing;
  double qtbpj,sum,temp,dsin,dcos,dtan,dcotan;

  for (j = 0; j < n; j++) {
    for (i = j; i < n; i++)  r[j][i] = r[i][j];
    x[j] = r[j][j];
    wa[j] = qtb[j];
  }
  for (j = 0; j < n; j++) {
    l = ipvt[j];
    if (diag[l] != 0.0) {
      for (k = j; k < n; k++)  sdiag[k] = 0.0;
      sdiag[j] = diag[l];
      qtbpj = 0.0;
      for (k = j; k < n; k++) {
	if (sdiag[k] == 0.0) continue;
	if (fabs(r[k][k]) < fabs(sdiag[k])) {
	  dcotan = r[k][k] / sdiag[k];
	  dsin = 1.0 / sqrt(1.0 + dcotan * dcotan);
	  dcos = dsin * dcotan;
	}
	else {
	  dtan = sdiag[k] / r[k][k];
	  dcos = 1.0 / sqrt(1.0 + dtan * dtan);
	  dsin = dcos * dtan;
	}
	r[k][k] = dcos * r[k][k] + dsin * sdiag[k];
	temp = dcos * wa[k] + dsin * qtbpj;
	qtbpj = -dsin * wa[k] + dcos * qtbpj;
	wa[k] = temp;
	kp1 = k + 1;
	if (n <= kp1) continue;
	for (i = kp1; i < n; i++) {
	  temp = dcos * r[k][i] + dsin * sdiag[i];
	  sdiag[i] = -dsin * r[k][i] + dcos * sdiag[i];
	  r[k][i] = temp;
	}
      }
    }
    sdiag[j] = r[j][j];
    r[j][j] = x[j];
  }
  nsing = n;
  for (j = 0; j < n; j++) {
    if ((sdiag[j] == 0.0) && (nsing == n))
      nsing = j;
    if (nsing < n)
      wa[j] = 0.0;
  }
  if (nsing >= 1) {
    for (k = 0; k < nsing; k++) {
      j = nsing - k - 1;
      sum = 0.0;
      jp1 = j + 1;
      if (nsing > jp1) {
	for (i = jp1; i < nsing; i++)
	  sum += r[j][i] * wa[i];
      }
      wa[j] = (wa[j] - sum) / sdiag[j];
    }
  }
  for (j = 0; j < n; j++) {
    l = ipvt[j];
    x[l] = wa[j];
  }
}

// -------------------------------------------------------------------
// MINPACK -- fdjac2.c
// -------------------------------------------------------------------

void CMinpack::fdjac2(void f(double *,double *,int,int,int *),
		      double x[], double fvec[], int nx, int nf, double **fjac,
		      int *iflag, double epsfcn, double wa[])
{
  int i,j;
  double eps,epsmch,h,temp;

  epsmch = (epsfcn > DPMPAR0) ? epsfcn : DPMPAR0;
  eps = sqrt(epsmch);

  for (j = 0;j < nx; j++) {
    temp = x[j];
    if (temp == 0.0) h = eps;
    else h = eps * fabs(temp);
    x[j] = temp + h;
    f(x,wa,nx,nf,iflag);
    if (*iflag < 0) break;
    x[j] = temp;
    for (i = 0;i < nf; i++)
      fjac[j][i] = (wa[i] - fvec[i]) / h;
  }
}

// -------------------------------------------------------------------
// MINPACK -- lmpar.c
// -------------------------------------------------------------------

void CMinpack::lmpar(int n, double **r, int ipvt[], double diag[], double qtb[],
		     double delta, double *par, double x[], double sdiag[],
		     double wa1[], double wa2[])
{
  int i,iter,j,jp1,k,l,nsing;
  double dxnorm,dwarf,fp,gnorm,parc,parl,paru;
  double sum,temp;

  dwarf = DPMPAR1;
  nsing = n;
  for (j = 0; j < n; j++) {
    wa1[j] = qtb[j];
    if ((r[j][j] == 0.0) && (nsing == n))  nsing = j;
    if (nsing < n) wa1[j] = 0.0;
  }
  if (nsing >= 1) {
    for (k = 0;k < nsing; k++) {
      j = nsing - k - 1;
      wa1[j] /= r[j][j];
      temp = wa1[j];
      if (j < 1) continue;
      for (i = 0; i < j; i++)  wa1[i] -= r[j][i] * temp;
    }
  }
  for (j = 0; j < n; j++) {
    l = ipvt[j];
    x[l] = wa1[j];
  }
  iter = 0;
  for (j = 0; j < n; j++)
    wa2[j] = diag[j] * x[j];
  dxnorm = enorm(n,wa2);
  fp = dxnorm - delta;
  if (fp <= 0.1*delta) {
    if (iter == 0) *par = 0.0;
    return;
  }
  parl = 0.0;
  if (nsing >= n) {
    for (j = 0; j < n; j++) {
      l = ipvt[j];
      wa1[j] = diag[l] * wa2[l] / dxnorm;
    }
    for (j = 0; j < n; j++) {
      sum = 0.0;
      if (j >= 1) {
	for (i = 0; i < j; i++)
	  sum += r[j][i] * wa1[i];
      }
      wa1[j] = (wa1[j] - sum) / r[j][j];
    }
    temp = enorm(n,wa1);
    parl = ((fp / delta) / temp) / temp;
  }
  for (j = 0;j < n; j++) {
    sum = 0.0;
    for (i = 0; i <= j; i++)  sum += r[j][i] * qtb[i];
    l = ipvt[j];
    wa1[j] = sum / diag[l];
  }
  gnorm = enorm(n,wa1);
  paru = gnorm / delta;
  if (paru == 0.0)  paru = dwarf / min(delta,0.1);
  *par = max(*par,parl);
  *par = min(*par,paru);
  if (*par == 0.0)  *par = gnorm / dxnorm;
  while (1) {
    iter++;
    if (*par == 0.0)  *par = max(dwarf,0.001 * paru);
    temp = sqrt(*par);
    for (j = 0; j < n; j++)
      wa1[j] = temp * diag[j];
    qrsolv(n,r,ipvt,wa1,qtb,x,sdiag,wa2);
    for (j = 0; j < n; j++)
      wa2[j] = diag[j] * x[j];
    dxnorm = enorm(n,wa2);
    temp = fp;
    fp = dxnorm - delta;

    if ((fabs(fp) <= 0.1*delta) || ((parl == 0.0) && (fp <= temp) && (temp > 0.0)) || iter == 10) {
      if (iter == 0) *par = 0.0;
      return;
    }
    for (j = 0; j < n; j++) {
      l = ipvt[j];
      wa1[j] = diag[l] * wa2[l] / dxnorm;
    }
    for (j = 0; j < n; j++) {
      wa1[j] /= sdiag[j];
      temp = wa1[j];
      jp1 = j + 1;
      if (jp1 < n)
	for (i = jp1; i < n; i++)
	  wa1[i] -= r[j][i] * temp;
    }
    temp = enorm(n,wa1);
    parc = ((fp/delta) / temp) / temp;
    if (fp > 0.0)  parl = max(parl,*par);
    if (fp < 0.0)  paru = min(paru,*par);
    *par = max(parl,*par+parc);
  }
}
