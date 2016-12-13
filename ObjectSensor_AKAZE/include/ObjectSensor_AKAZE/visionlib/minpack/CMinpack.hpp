
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

#ifndef CMINPACK_H
#define CMINPACK_H

// IEEE double precision machine parameters.
#define DPMPAR0  2.220446049250313e-16
#define DPMPAR1  2.225073858507201e-308
#define DPMPAR2  1.797693134862316e308

#include <iostream>
#include <stdlib.h>
#include <string.h>

class CMinpack {
public:
  // READ-ONLY ! None of the followings shouldn't be changed by user apps.
  char termination[80];
  bool verbose;
  bool logging;
  char logfilename[80];
  FILE *logfp;
  
  double info[20];	// to save optimization result
  
public:
  CMinpack() : verbose(false), logging(false), logfp(NULL) {}
  ~CMinpack() {}
  
  // -----------------------------------------------------------------
  // Levenberg-Marquardt method : 
  //   Minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  //   for the explanation of arguments, read comments in CMinpack.cpp
  // -----------------------------------------------------------------
public:
  int  LM(void f(double *x, double *fvec, int nx, int nf, int *iflag), 
	  double x[], double fvec[], int nx, int nf, int maxiter, double tol);
  
  void LM(void f(double *x, double *fvec, int nx, int nf, int *iflag),
	  double x[], double fvec[], int nx, int nf, int msk[], int maxit,
	  double ftol, double xtol, double gtol, int maxfev, double epsfcn, 
	  double diag[], int mode, double factor, int *info, int *nfev, 
	  double **fjac, int ipvt[], double qtf[], 
	  double wa1[], double wa2[], double wa3[], double wa4[]);
  
  // -----------------------------------------------------------------
  // Levenberg-Marquardt method (implementation by Manolis Lourakis)
  //   Minimize a sum of squares of non-linear functions,
  //   F(x) = 1/2 SUMi( f_i(x)^2 ), where i = 1...M
  //   for the explanation of arguments, read comments in CMinpack.cpp
  // -----------------------------------------------------------------
public:
  int  LM2(void f(double *p, double *fx, int np, int nf, void *adata),
	   double *p, double *fx, int np, int nf, int itmax, double *opts=NULL, void *adata=NULL);
  
  // -----------------------------------------------------------------
  // functions for Debugging 
  // -----------------------------------------------------------------
public:
  void setLogOn(char *fname) { strcpy( logfilename, fname ); logging = true; }
  void setLogOff(void)       { logfilename[0] = '\0'; logging = false; }
  void addLogMessage(char *msg) { if (logfp) fprintf(logfp, "%s\n", msg); }
private:
  bool openLogFile(void)  { logfp = fopen(logfilename, "a+"); return (logfp != NULL); }
  void closeLogFile(void) { if (logfp) fclose(logfp); logfp = NULL; }
  
  // -----------------------------------------------------------------
  // MINPACK's original functions
  // -----------------------------------------------------------------
public:
  double enorm(int n, double a[]);
  void   lmdif(void f(double *x, double *fvec, int nx, int nf, int *iflag),
	       double x[], double fvec[], int nx, int nf, int msk[], int maxit,
	       double ftol, double xtol, double gtol, int maxfev, double epsfcn,
	       double diag[], int mode, double factor, int *info, int *nfev,
	       double **fjac, int ipvt[], double qtf[], 
	       double wa1[], double wa2[], double wa3[], double wa4[]);
  int    lmdif0(void f(double *x, double *fvec, int nx, int nf, int *iflag),
		double x[], double fvec[], int nx, int nf, 
		int maxit, double tol, int *info, int *nfev);
  
  // -----------------------------------------------------------------
private:
  double rownorm(int m, int r, int c, double **a);
  double colnorm(int m, int r, int c, double **a);
  void   fdjac2(void f(double *,double *,int,int,int *),
		double x[], double fvec[], int nx, int nf, double **fjac,
		int *iflag, double epsfcn, double wa[]);
  void   lmpar(int n,double **fjac,int ipvt[],double diag[],
	       double qtf[],double delta,double *par,double wa1[],
	       double wa2[],double wa3[],double wa4[]);
  void   qrfac(int m,int n,double **a,int pivot,int ipvt[],
	       double rdiag[],double acnorm[],double wa[]);
  void   qrsolv(int n,double **r,int ipvt[],double diag[],
		double qtb[],double x[],double sdiag[],double wa[]);
};


#endif // CMINPACK_H
