//
// MTX::MatrixSolver<> class template
//
// Jaeil Choi
// Last modified in Oct, 2008
//
// This code include various matrix operations in following 4 categories
//   inversion	           : by Gauss-Jordan, Determinant, etc
//   solving linear system : by LU, QR, Gauss-Jordan, etc
//   determinant           :
//   decomposition         : LU, QR, SVD, Cholesky, Eigenvalue
//
// Matrix decomposition (LU, QR, SVD, Cholesky, Eigenvalue) codes
// use some functions from TNT (Template Numerical Toolkit) and JAMA.
// TNT and JAMA were created by NIST, and they are in public domain.
// For more information, visit 'http://math.nist.gov/tnt/'.
//
// Determinant -------------------------------------------------------
//   T  getDeterminant(Matrix<T> &A);
//   T  getDeterminant(int N, T A[]);
//   T  getDeterminantByRecursion(int N, T A[]);
// Inverse -----------------------------------------------------------
//   bool inverseByDeterminant(int N, T A[], T Ai[], T det=0);
//   bool inverseByGaussJordan(Matrix<T> &A);
//   bool inverseByGaussJordan(Matrix<T> &A, Matrix<T> &Ainv);
//   bool inverseByGaussJordan(int N, T A[], T Ainv[]=NULL);
//   bool inverseByGaussJordan(TNT::Array2D<T> &A, TNT::Array2D<T> &AInv);
//   bool inverseByCholesky(Matrix<T> &A, Matrix<T> &Ainv);		// for spd A
//   bool inverseByCholesky(int N, T A[], T Ainv[]=NULL);		// for spd A
//   void InverseTriangular(int N, T A[], char UpperLower);
// Solving "A x = b" -------------------------------------------------
//   bool solveByGaussJordan(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);
//   bool solveByGaussJordan(int N, T A[], T x[], T b[]);
//   bool solveByGaussJordan(TNT::Array2D<T> &A, TNT::Array1D<T> &b);
//   int  solveByCramersRule(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b, T det = 0);
//   bool solveByLU(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);
//   bool solveByLU(int M, int N, T A[], T x[], T b[]);
//   bool isSPD(int N, T A[]);
//   bool solveByCholesky(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);	// for spd A
//   bool solveByCholesky(int N, T A[], T x[], T b[]);			// for spd A
//   bool solveByQR(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);	// least sq.
//   bool solveByQR(int M, int N, T A[], T x[], T b[]);		// least sq.
//   bool solveBySVD(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);	// lease sq. for singular A
//   bool solveBySVD(int M, int N, T A[], T x[], T b[]);	// lease sq. for singular A
//   bool solveByLeastSquares(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b);  // obsolete
// Solving "A x = 0" -------------------------------------------------
//   bool solveNullBySVD(Matrix<T> &A, T xr[], T xl[]=NULL);
//   bool solveNullBySVD(int M, int N, T A[], T xr[], T xl[]=NULL);
// LU decomposition --------------------------------------------------
//   ( A(piv,:) = L*U  for any matrix A(mxn), where m >= n )
//   void LU(Matrix<T> &A, Matrix<T> &L, Matrix<T> &U);
//   void LU(Matrix<T> &A, Matrix<T> &L, Matrix<T> &U, Matrix<T> &piv);
//   void LU(int M, int N, T A[], T L[], T U[], T piv[]=NULL);
// QR decomposition --------------------------------------------------
//   ( A = Q(mxn)*R(nxn)  for any matrix A )
//   void QR(Matrix<T> &A, Matrix<T> &Q, Matrix<T> &R);
//   void QR(Matrix<T> &A, Matrix<T> &Q, Matrix<T> &R, Matrix<T> &H);
//   void QR(int M, int N, T A[], T Q[], T R[], T H[]=NULL);
// RQ decomposition --------------------------------------------------
//   ( to extract intrinsic/extrinsic parameters from projection matrix )
//   void RQ_3x3(T A[9],  T R[9], T Q[9]);
//   void RQ_3x4(T A[12], T R[9], T Q[9]);
// SVD decomposition -------------------------------------------------
//   ( A = U(mxn)*S(nxn)*V(nxn)' for any matrix A )
//   void SVD(Matrix<T> &A, Matrix<T> &U, Matrix<T> &S, Matrix<T> &V, bool matrix_s = false);
//   void SVD(int M, int N, T A[], T U[], T S[], T V[], bool matrix_s = false);
//   double SVD_norm2(int N, T S[]);
//   double SVD_cond(int N, T S[]);
//   int    SVD_rank(int N, T S[]);
// Cholesky decomposition --------------------------------------------
//   ( A = L * L'  for symmetric positive definite matrix A )
//   bool Cholesky(Matrix<T> &A, Matrix<T> &L);
//   bool Cholesky(int size, T A[], T L[]);
// Eigenvalue decomposition ------------------------------------------
//   ( A = V*D*V'  for any matrix A. But for non-symmetric matrix A, )
//   ( D will be block-diagonal with complex eigen values )
//   void Eig(Matrix<T> &A, Matrix<T> &V, Matrix<T> &D, bool matrix_d = false);
//   void Eig(int N, T A[], T V[], T D[], bool matrix_d = false);
// Transformation ----------------------------------------------------
//   bool getTransformationBySimplices( T xf[], int dim, ... );
//   void getTransformationByPoints( Matrix<T> M, int dim, int n, T *pa, T*pb );
//   void getRotationFromMatrix2D(T R[], T M[], bool for_frames=false);
//   void getRotationFromMatrix3D(T R[], T M[], bool for_frames=false);
//   void getRotationByPoints( Matrix<double> M, int dim, int n, bool local, ... );
//


#ifndef MTX_MATRIX_SOLVER_HPP
#define MTX_MATRIX_SOLVER_HPP
#define USE_MTX_MATRIX_SOLVER


#include <iostream>
#include <float.h>
#include "mtx_matrix.hpp"
#include "mtx_jama_lu.h"
#include "mtx_jama_qr.h"
#include "mtx_jama_svd.h"
#include "mtx_jama_cholesky.h"
#include "mtx_jama_eig.h"


#define V2D_COPY(d,s)           do { (d)[0] = (s)[0];  (d)[1] = (s)[1]; } while(0)
#define V2D_LENGTH(a)           (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1]))
#define V2D_DIV_VALUE(a,v)      do { (a)[0] /= (v);  (a)[1] /= (v); } while(0)
#define V2D_NORMALIZE(a)        do { float len = V2D_LENGTH(a);  if (len > 0) V2D_DIV_VALUE( a, len ); } while(0)
#define V3D_COPY(d,s)           do { (d)[0] = (s)[0];  (d)[1] = (s)[1];  (d)[2] = (s)[2]; } while(0)
#define V3D_LENGTH(a)           (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]))
#define V3D_DIV_VALUE(a,v)      do { (a)[0] /= (v);  (a)[1] /= (v);  (a)[2] /= (v); } while(0)
#define V3D_NORMALIZE(a)        do { float len = V3D_LENGTH(a);  if (len > 0) V3D_DIV_VALUE( a, len ); } while(0)

#define VALID_EQUATION3(A, x, b) \
  (A.nRow == A.nCol && A.nCol == x.nRow && x.nCol == 1 && x.nRow == b.nRow && b.nCol == 1)
#define VALID_EQUATION2(A, b) \
  (A.nRow == A.nCol && A.nRow == b.nRow && b.nCol == 1)

namespace MTX {


template <class T>
class MatrixSolver {

 public:
  MatrixSolver()  { }
  ~MatrixSolver() { }

  // ================================================================
  // Determinant
  //
  // The first approach:  (decomposition into permutations of I)
  // | a b | = a*d*| 1 0 | + b*c*| 0 1 |
  // | c d |       | 0 1 |       | 1 0 |
  // det(A) = SUM ( (product of coefficients) * det(Permutation of I) )
  //
  // The second approach:  (extension in cofactors)
  // | a11 a12 a13 | = | a11   0   0 |   |  0  a12   0 |   |   0   0 a13 |
  // | a21 a22 a23 | = |   0 a22 a23 | + | a21   0 a23 | + | a21 a22   0 |
  // | a31 a32 a33 | = |   0 a32 a33 |   | a22   0 a33 |   | a31 a32   0 |
  // det(A) = ai1Ai1 + ai2Ai2 + ... + ainAin
  // The cofactor Aij is the determinant of Mij with the correct sign.
  // Aij = ((-1)^(i+j)) det(Mij)
  // The minor Mij is formed by deleting row i and column j of A.
  //
  // Another approach would be:
  // det(A) = det(P^)*det(L)*det(D)*det(U) = +/- product of pivots
  // In fact, the pivots are the result of systemically condensing
  // the information that was originally spread over all n^2 entries
  // of the matrix. Therefore we prefer explicit formular.
  // ================================================================
public:

  inline T getDeterminant(Matrix<T> &A) {
    // calculate the determinant of NxN matrix 'A'.
    if      (A.nRow == 1) return A.data[0];
    else if (A.nRow == 2)     // ad - bc
      return ( A.data[0] * A.data[3] - A.data[1] * A.data[2] );
    else if (A.nRow == 3)
      // +a11a22a33+a12a23a31+a13a21a32-a11a23a32-a12a21a33-a13a22a31
      return ( + A.data[0] * A.data[4] * A.data[8]
	       + A.data[1] * A.data[5] * A.data[6]
	       + A.data[2] * A.data[3] * A.data[7]
	       - A.data[0] * A.data[5] * A.data[7]
	       - A.data[1] * A.data[3] * A.data[8]
	       - A.data[2] * A.data[4] * A.data[6] );
    else return getDeterminantByRecursion(A.nRow, A.data);
  }

  inline T getDeterminant(int N, T A[]) {
    // calculate the determinant of NxN matrix 'A'.
    if      (N == 1) return A[0];
    else if (N == 2)     // ad - bc
      return ( A[0] * A[3] - A[1] * A[2] );
    else if (N == 3)
      // +a11a22a33+a12a23a31+a13a21a32-a11a23a32-a12a21a33-a13a22a31
      return ( + A[0] * A[4] * A[8]
	       + A[1] * A[5] * A[6]
	       + A[2] * A[3] * A[7]
	       - A[0] * A[5] * A[7]
	       - A[1] * A[3] * A[8]
	       - A[2] * A[4] * A[6] );
    else return getDeterminantByRecursion(N, A);
  }

  T getDeterminantByRecursion(int N, T A[]) {
    // calculate the determinant of NxN matrix 'A'.
    if (N < 4) return getDeterminant(N, A);
    else {
      // Expansions in cofactors:
      // det(A) = ai1Ai1 + ai2Ai2 + ... + ainAin
      // Aij = ((-1)^(i+j)) det(Mij)
      // The minor Mij is formed by deleting row i and column j of A.
      int k, i, j, sign;
      T   sum = 0;
      Matrix<T> m(N-1, N-1);
      for (k = 0; k < N; k++) {
	if (A[k] == 0) continue;
	sign = ((k % 2) == 0 ? +1 : -1);
	for (i = 0; i < m.nRow; i++)
	  for (j = 0; j < m.nCol; j++)
	    m(i,j) = A[ (i+1) * N + j + ((j>=k) ? 1 : 0) ];
	sum += A[k] * sign * getDeterminantByRecursion(N-1, m.data);
      }
      return sum;
    }
  }

  // ================================================================
  // Matrix Inversioin
  // ================================================================

  bool inverseByDeterminant(int N, T A[], T Ai[], T det=0) {
    // Adjugate matrix:
    // [ a11 a12 .. a1n ] [ A11 A21 .. An1 ]   [ det(A)   0    ..   0    ]
    // [ a21 a22 .. a2n ] [ A12 A22 .. An2 ] = [   0    det(A) ..   0    ]
    // [  :   :      :  ] [  :   :      :  ]   [   :      :         :    ]
    // [ an1 an2 .. ann ] [ A1n A2n .. Ann ]   [   0      0    .. det(A) ]
    // A * adj(A) = det(A) * I     (adj(A) is transpose of cofactor matrix)
    // A * (adj(A)/det(A)) = I,  or A^ = adj(A)/det(A)
    int i, j, i2, j2, inew, jnew, sign;
    if        (N == 2) {
      if (det == 0) det = ( A[0] * A[3] - A[1] * A[2] );
      if (det == 0) return false;
      Ai[0] = +A[3]/det;  Ai[2] = -A[2]/det;
      Ai[1] = -A[1]/det;  Ai[3] = +A[0]/det;
    } else if (N == 3) {
      // +a11a22a33+a12a23a31+a13a21a32-a11a23a32-a12a21a33-a13a22a31
      if (det == 0) det = ( + A[0] * A[4] * A[8] + A[1] * A[5] * A[6]
			    + A[2] * A[3] * A[7] - A[0] * A[5] * A[7]
			    - A[1] * A[3] * A[8] - A[2] * A[4] * A[6] );
      if (det == 0) return false;
      Ai[0] = (+A[4]*A[8]-A[5]*A[7])/det;  Ai[3] = (-A[3]*A[8]+A[5]*A[6])/det;  Ai[6] = (+A[3]*A[7]-A[4]*A[6])/det;
      Ai[1] = (-A[1]*A[8]+A[2]*A[7])/det;  Ai[4] = (+A[0]*A[8]-A[2]*A[6])/det;  Ai[7] = (-A[0]*A[7]+A[1]*A[6])/det;
      Ai[2] = (+A[1]*A[5]-A[2]*A[4])/det;  Ai[5] = (-A[0]*A[5]+A[2]*A[3])/det;  Ai[8] = (+A[0]*A[4]-A[1]*A[3])/det;
    } else {
      if (det == 0) det = getDeterminantByRecursion(N, A);
      if (det == 0) return false;
      Matrix<T> m(N-1, N-1);
      for (i = 0; i < N; i++) {
	for (j = 0; j < N; j++) {
	  sign = ((i + j) % 2 == 0) ? 1 : -1;
	  for (i2 = 0; i2 < m.nRow; i2++) {
	    for (j2 = 0; j2 < m.nCol; j2++) {
	      inew = (j2 >= j) ? j2 + 1 : j2;
	      jnew = (i2 >= i) ? i2 + 1 : i2;
	      m(i2,j2) = A[ inew * N + jnew ];
	    }
	  }
	  // set each element
	  Ai[i * N + j] = getDeterminantByRecursion( N-1, m.data ) / det * sign;
	}
      }
    }
    return true;
  }

  bool inverseByGaussJordan(Matrix<T> &A) {
    assert(A.nRow == A.nCol);
    TNT::Array2D<T> TA(A.nRow, A.nCol, A.data);
    TNT::Array1D<T> Tb(A.nRow, 1.0);
    return solveByGaussJordan( TA, Tb );
  }
  bool inverseByGaussJordan(Matrix<T> &A, Matrix<T> &Ainv) {
    assert(A.nRow == A.nCol);
    Ainv.set( A.nRow, A.nCol ); // reallocate only if it's necessary
    return inverseByGaussJordan( A.nRow, A.data, Ainv.data );
  }
  bool inverseByGaussJordan(int N, T A[], T Ainv[]=NULL) {
    if (Ainv) memcpy( Ainv, A, sizeof(T) * N * N );
    else      Ainv = A;
    TNT::Array2D<T> TA(N, N, Ainv);
    TNT::Array1D<T> Tb(N, 1.0);
    return solveByGaussJordan( TA, Tb );
  }
  bool inverseByGaussJordan(TNT::Array2D<T> &A, TNT::Array2D<T> &AInv) {
    TNT::Array1D<T> b( A.dim2(), 1.0 );
    AInv = A.copy();
    return solveByGaussJordan( AInv, b );
  }

  bool inverseByCholesky(Matrix<T> &A, Matrix<T> &Ainv) {
    // A[] must be symmetric positive definite
    Ainv.set( A.nRow, A.nRow );
    return inverseByCholesky(A.nRow, A.data, Ainv.data);
  }
  bool inverseByCholesky(int N, T A[], T Ainv[]=NULL) {
    // A[] must be symmetric positive definite
    if (Ainv == NULL) Ainv = A;
    TNT::Array2D<T> TA(N, N, A);
    JAMA::Cholesky<T> jCholesky( TA );	// Cholesky decomposition A = L*L'
    if (jCholesky.isspd != 1) return false;
    // A^ = (L*L')^ = L^' * L^
    int i, j, k, pos;
    T   *L = &(jCholesky.L_[0][0]), sum, *c1, *c2, *ainv;
    InverseTriangular( N, L, 'L' );
    for (i = 0; i < N; i++) {
      c1 = L + i;
      ainv = Ainv + i * N;
      for (j = 0; j < i; j++) ainv[j] = Ainv[j*N+i];
      for (j = i; j < N; j++) {
	c2 = L + j;
	for (sum = 0, k = j; k < N; k++) {
	  pos = k * N;
	  sum += c1[pos] * c2[pos];
	}
	ainv[j] = sum;
      }
    }
    return false;
  }

  void InverseTriangular(int N, T A[], char UpperLower) {
    // A[] must be (N x N), and will be over-written
    int i, j, k;
    T   *a = A, *ar, sum;
    if (UpperLower == 'U' || UpperLower == 'u') {
      for (j = 0; j < N; j++, a += N+1) {
	*a = 1.0 / *a;			// A(j,j)
	for (i = 0; i < j; i++) {	// A(0:j-1, j) = A(0:j-1, 0:j-1) * A(0:j-1, j)
	  ar = A + i * N;		// beginning of the row
	  for (sum=0, k=i; k < j; k++) sum += ar[k] * A[k*N+j];
	  ar[j] = sum;
	}
	for (i = 0; i < j; i++) A[i*N+j] *= -(*a);  // A(0:j-1, j) *= -A(j,j)
      }
    } else {
      for (i = 0; i < N; i++, a += N+1) {
	*a = 1.0 / *a;			// A(i,i)
	ar = A + i * N;			// beginning of the row
	for (j = 0; j < i; j++) {	// A(i, 0:i-1) = A(i, 0:i-1) * A(0:i-1, 0:i-1)
	  for (sum=0, k=j; k < i; k++) sum += ar[k] * A[k*N+j];
	  ar[j] = sum;
	}
	for (j = 0; j < i; j++) ar[j] *= -(*a);  // A(0:j-1, j) *= -A(j,j)
      }
    }
  }

  // ================================================================
  // Solving Linear System  "A x = b"
  // ================================================================
public:
  // solveByGaussJordan ---------------------------------------------

  bool solveByGaussJordan(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b) {
    assert(A.nRow == A.nCol && A.nRow == b.nRow);
    x.set( A.nRow, 1 );	// reallocate only if it's necessary
    return solveByGaussJordan( A.nRow, A.data, x.data, b.data );
  }
  bool solveByGaussJordan(int N, T A[], T x[], T b[]) {
    T *B = (T*)malloc( sizeof(T) * N * N );
    memcpy( B, A, sizeof(T) * N * N );
    memcpy( x, b, sizeof(T) * N );
    TNT::Array2D<T> TA(N, N, B);
    TNT::Array1D<T> Tb(N, x);
    bool result = solveByGaussJordan( TA, Tb );
    free(B);  return result;
  }
  bool solveByGaussJordan(TNT::Array2D<T> &A, TNT::Array1D<T> &b) {
    // Upon return, 'A' hold its inverse, 'b' holds the solution x = A^ b.
    assert (A.dim1() == A.dim2() && A.dim1() == b.dim());
    int    n = b.dim();
    TNT::Array1D<int> indxc(n);
    TNT::Array1D<int> indxr(n);
    TNT::Array1D<int> ipiv(n, 0);
    int    i, icol=0, irow=0, j, k, l, ll;
    double big, dum, pivinv;

    for (i = 0; i < n; i++) {
      big=0.0;
      for (j = 0; j < n; j++)
	if (ipiv[j] != 1)
	  for (k = 0; k < n; k++)  /* find biggest in row j */
	    if (ipiv[k] == 0) {
	      if (fabs(A[j][k]) >= big) {
		big = fabs(A[j][k]);  irow=j;  icol=k;
	      }
	    } else if (ipiv[k] > 1) return false;
      ipiv[icol]++;
      if (irow != icol) {
	for (l = 0; l < n; l++)  std::swap(A[irow][l], A[icol][l]);
	std::swap(b[irow],b[icol]);
      }
      if (fabs(A[icol][icol]) < 0.00001) return false;
      indxr[i] = irow;  indxc[i] = icol;
      pivinv = 1.0/A[icol][icol];
      A[icol][icol] = 1.0;
      for (l = 0; l < n; l++)	A[icol][l] *= pivinv;
      b[icol] *= pivinv;
      for (ll = 0; ll < n; ll++)
	if (ll != icol) {
	  dum = A[ll][icol];
	  A[ll][icol] = 0.0;
	  for (l = 0; l < n; l++) A[ll][l] -= A[icol][l] * dum;
	  b[ll] -= b[icol] * dum;
	}
    }
    for (l = n-1; l >= 0; l--)
      if (indxr[l] != indxc[l])
	for (k = 0; k < n; k++) std::swap(A[k][indxr[l]], A[k][indxc[l]]);
    return true;
  }

  // solveByCramersRule ---------------------------------------------

  int solveByCramersRule(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b, T det = 0) {
    // Solve linear system  " A x = b " using Cramer's rule
    // x = A^ * b = (adj(A) / det(A)) * b = (adj(A) * b) / det(A)
    // xj = det(Bj) / det(A),  where
    // Bj = [ a_1 a_2 .. b_ .. a_n ] (vector b replaces jth column of A)
    // proof: det(Bj) = b1A1j + b2A2j + ... + bnAnj
    //        And this is the jth component of ( adj(A) * b ).
    // Thus each component of x is a ratio of two determinant,
    // a polynomial of degree n divided by another polynomial of degree n.

    if (!VALID_EQUATION2(A, b)) return -1;
    int i, j;
    T   detBj, detA;

    if (det == 0) {
      detA = getDeterminantByRecursion(A.nRow, A.data);
      if (detA == 0) { return -1; }
    }

    x.set(A.nCol, 1);
    Matrix<T> Bj(A.nRow, A.nRow);
    for (j = 0; j < A.nCol; j++) {
      Bj = A;
      for (i = 0; i < A.nRow; i++) Bj[i][j] = b.data[i];
      detBj = getDeterminantByRecursion( Bj.nRow, Bj.data );
      x.data[j] = detBj / detA;
    }
  }

  // solveByLU ------------------------------------------------------

  bool solveByLU(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b) {
    // 'A' is MxN, 'x' is Mx1, and 'b' is Mx1.
    int M = A.nRow, N = A.nCol;
    x.set( M, 1 );  // reallocate only if it's necessary
    return solveByLU( M, N, A.data, x.data, b.data );
  }
  bool solveByLU(int M, int N, T A[], T x[], T b[]) {
    // 'A' is MxN, 'x' is Mx1, and 'b' is Mx1.
    TNT::Array2D<T> TA(M,N, A);
    int i, k;
    // LU decomposition
    JAMA::LU<T> jLU( TA );
    if (!jLU.isNonsingular()) return false;
    // permute b[]  ( x = jLU.permute_copy(tb, jLU.piv); )
    for (i = 0; i < M; i++) x[i] = b[ jLU.piv[i] ];
    // Solve L*Y = B(piv)
    for (k = 0; k < N; k++)
      for (i = k+1; i < N; i++)
	x[i] -= x[k] * jLU.LU_[i][k];
    // Solve U*X = Y;
    for (k = N-1; k >= 0; k--) {
      x[k] /= jLU.LU_[k][k];
      for (i = 0; i < k; i++)
	x[i] -= x[k] * jLU.LU_[i][k];
    }
    return true;
  }

  // solveByCholesky -------------------------------------------------

  bool isSPD(int N, T A[]) {
    TNT::Array2D<T> TA(N,N, A);
    JAMA::Cholesky<T> jCholesky( TA );
    return (jCholesky.isspd == 1);
  }
  bool solveByCholesky(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b) {
    // linear system solver of symmetric positive-definite matrix
    // 'A' is NxN, 'x' is Nx1, and 'b' is Nx1.
    if (A.nRow != A.nCol) return false;
    int N = A.nRow;
    x.set( N, 1 );  // reallocate only if it's necessary
    return solveByCholesky( N, A.data, x.data, b.data );
  }
  bool solveByCholesky(int N, T A[], T x[], T b[]) {
    // linear system solver for symmetric positive-definite matrix A
    // 'A' is NxN, 'x' is Nx1, and 'b' is Nx1.
    TNT::Array2D<T> TA(N,N, A);
    // Cholesky decomposition
    JAMA::Cholesky<T> jCholesky( TA );
    if (jCholesky.isspd != 1) return false;
    // Solve L*y = b
    memcpy( x, b, sizeof(T)*N );
    int i, k;
    // Solve L*y = b;
    for (k = 0; k < N; k++) {
      for (i = 0; i < k; i++) x[k] -= x[i] * jCholesky.L_[k][i];
      x[k] /= jCholesky.L_[k][k];
    }
    // Solve L'*X = Y;
    for (k = N-1; k >= 0; k--) {
      for (i = k+1; i < N; i++) x[k] -= x[i] * jCholesky.L_[i][k];
      x[k] /= jCholesky.L_[k][k];
    }
    return true;
  }

  // solveByQR ------------------------------------------------------

  bool solveByQR(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b) {
    // the least squares solution of nonsquare systems of linear equations.
    int M = A.nRow, N = A.nCol;
    x.set( N, 1 );  // reallocate only if it's necessary
    return solveByQR( M, N, A.data, x.data, b.data );
  }
  bool solveByQR(int M, int N, T A[], T x[], T b[]) {
    // the least squares solution of nonsquare systems of linear equations.
    // 'A' is MxN, 'x' is Nx1, and 'b' is Mx1.
    TNT::Array2D<T> TA(M,N, A);
    // QR decomposition
    JAMA::QR<T> jQR( TA );		// Q : MxN   R : NxN (upper triangular)
    if (!jQR.isFullRank()) return false;
    // Compute Y = transpose(Q)*b
    TNT::Array1D<T> x2(M);		// note x2 is Mx1, while x is Nx1
    memcpy( &(x2[0]), b, sizeof(T)*M );
    int i, k;
    // Compute Y = Qt * b
    for (k = 0; k < N; k++) {
      T s = 0.0;
      for (i = k; i < M; i++) s += jQR.QR_[i][k] * x2[i];
      s = -s / jQR.QR_[k][k];
      for (i = k; i < M; i++) x2[i] += s * jQR.QR_[i][k];
    }
    // Solve R*X = Y;
    for (k = N-1; k >= 0; k--) {
      x2[k] /= jQR.Rdiag[k];
      for (i = 0; i < k; i++)  x2[i] -= x2[k] * jQR.QR_[i][k];
    }
    for (i = 0; i < N; i++) x[i] = x2[i];
    return true;
  }

  // solveBySVD ------------------------------------------------------

  bool solveBySVD(Matrix<T> &A, Matrix<T> &x, Matrix<T> &b) {
    // the least squares solution of nonsquare systems of linear equations.
    // Note that SVD is numerically stable (good for singular or close-to-singular matrices)
    if (sizeof(T) < sizeof(double)) printf("Warning (solveBySVD): use 'double' for this function\n");
    int M = A.nRow, N = A.nCol, i;
    x.set( N, 1 );  // reallocate only if it's necessary
    // A = U S Vt   and   Ut U = I,  Vt V = V Vt = I
    // x = A^ b = (U S Vt)^ b = V S^ Ut b
    Matrix<T> U(M, N), S(N, 1), V(N, N), Utb;
    SVD( A, U, S, V, false );	// get S as a vector
    // We want to calculate  x = V * Si * Ut * b    (N x N) * (N x N) * (N x M) * (M x 1)
    double threshold=0, s;
    for (i=0; i<N; i++) threshold += S(i);
    threshold *= 2*DBL_EPSILON;
    Utb.multAtB( U, b );	// (M x N)T * (M x 1) = (N x 1)
    for (i=0; i<N; i++) {
      s = S(i);
      Utb(i) *= (s <= threshold ? 0.0 : 1.0 / s);
    }
    x.mult( V, Utb );		// (N x N) * (N x 1) = (N x 1)
    return true;
  }
  bool solveBySVD(int M, int N, T A[], T x[], T b[]) {
    // the least squares solution of nonsquare systems of linear equations.
    // Note that SVD is numerically stable (good for singular or close-to-singular matrices)
    // 'A' is MxN, 'x' is Nx1, and 'b' is Mx1.
    Matrix<T> AA(M, N, A), X(N, 1, x), B(M, 1, b);
    return solveBySVD( AA, X, B );
  }

  // =================================================================
  // Solving A * x = 0  (null vector of A)
  // =================================================================

  bool solveNullBySVD(Matrix<T> &A, T xr[], T xl[]=NULL, T *ratio=NULL) {
    return solveNullBySVD( A.nRow, A.nCol, A.data, xr, xl, ratio );
  }
  bool solveNullBySVD(int M, int N, T A[], T xr[]=NULL, T xl[]=NULL, T *ratio=NULL) {
    // Solve A * x = 0 for x, that is the null vector of A.
    //   'xr' : right null vectors of A (eigenvector of A*At associated with the smallest eigenvalue)
    //   'xl' : left  null vectors of A (eigenvector of At*A associated with the smallest eigenvalue)
    //   'ratio' : ratio of smallest singular value to the largest one
    TNT::Array2D<T> TA( M, N,  A );
    JAMA::SVD<T> jSVD( TA );	// A = U*S*V' for an mxn matrix A with m >= n, where
    //				// U is mxn orthogonal, S is nxn diagonal, and V is nxn orthogonal matrix.
    int i;
    //cout << "SVD  ev : "; for (i = 0; i < N; i++) cout << jSVD.s[i] << " ";  cout << endl;
    if (xr) for (i = 0; i < N; i++)  xr[i] = jSVD.V[i][N-1];  // the last column of V
    if (xl) for (i = 0; i < M; i++)  xl[i] = jSVD.U[i][N-1];  // the last column of U
    if (ratio) *ratio = jSVD.s[N-1] / jSVD.s[0];
    return true;
  }

  // ================================================================
  // LU decomposition -----------------------------------------------
  //   Calculate A(piv,:) = L*U for an mxn matrix A with m >= n, where
  // L is mxn unit lower triangular, U is nxn upper triangular matrix,
  // and piv is length m permutation vector.
  // the LU decomposition is an mxn unit lower triangular matrix L,
  // an nxn upper triangular matrix U, and a permutation vector piv of length m
  // so that A(piv,:) = L*U. If m < n, then L is mxm and U is mxn.
  //   The LU decompostion with pivoting ALWAYS EXISTS,
  // even if the matrix is singular, so the constructor will never fail.
  // The primary use of the LU decomposition is in the solution of
  // square systems of simultaneous linear equations.
  // This will fail if TNT::LU::isNonsingular() returns false.
  // NOTE: YOU MUST MAKE PUBLIC 'LU::LU_' and 'LU::piv' in 'jama_lu.h'.
  // ================================================================

public:
  void LU(Matrix<T> &A, Matrix<T> &L, Matrix<T> &U) {
    int M = A.nRow, N = A.nCol;
    L.set( M, N );  U.set( N, N );  // reallocate only if it's necessary
    return LU( M, N, A.data, L.data, U.data );
  }
  void LU(Matrix<T> &A, Matrix<T> &L, Matrix<T> &U, Matrix<T> &piv) {
    int M = A.nRow, N = A.nCol;
    L.set( M, N );  U.set( N, N );  piv.set( M, 1 );  // reallocate only if it's necessary
    return LU( M, N, A.data, L.data, U.data, piv.data );
  }
  void LU(int M, int N, T A[], T L[], T U[], T piv[]=NULL) {
    // 'A' is MxN, 'L' is MxN, 'U' is NxN, and 'piv' is 'Mx1'
    TNT::Array2D<T> TA(M,N, A);
    JAMA::LU<T> jLU( TA );		// LU decomposition
    int i, j, pos;
    for (i = 0; i < M; i++)		// get 'L' and 'U'
      for (j = 0; j < N; j++) {
	pos = i*N + j;
	if      (i >  j) L[pos] = jLU.LU_[i][j];
	else if (i <  j) L[pos] = 0.0;
	else if (i == j) L[pos] = 1.0;
	if      (i <= j) U[pos] = jLU.LU_[i][j];
	else if (i <  N) U[pos] = 0.0;
      }
    if (piv) for (i = 0; i < M; i++) piv[i] = jLU.piv[i]; // get 'piv'
  }

  // ================================================================
  // QR decomposition -----------------------------------------------
  //   Calculate A = Q*R for an mxn matrix A, where
  // Q is mxn orthogonal, and R is nxn upper triangular matrix.
  //   The QR decompostion ALWAYS EXISTS, even if the matrix does not
  // have full rank, so the constructor will never fail.
  // The primary use of the QR decomposition is in the least squares
  // solution of nonsquare systems of simultaneous linear equations.
  // This will fail if isFullRank() returns 0 (false).
  //   The Q and R factors can be retrived via the getQ() and getR()
  // methods. Furthermore, a solve() method is provided to find
  // the least squares solution of Ax=b using the QR factors.
  // NOTE: YOU MUST MAKE PUBLIC 'QR::QR_' and 'QR::Rdiag' in 'jama_qr.h'.
  // ================================================================

public:
  void QR(Matrix<T> &A, Matrix<T> &Q, Matrix<T> &R) {
    int M = A.nRow, N = A.nCol;
    Q.set( M, N );  R.set( N, N );  // reallocate only if it's necessary
    QR( M, N, A.data, Q.data, R.data );
  }
  void QR(Matrix<T> &A, Matrix<T> &Q, Matrix<T> &R, Matrix<T> &H) {
    int M = A.nRow, N = A.nCol;
    Q.set( M, N );  R.set( N, N );  H.set( M, N );  // reallocate only if it's necessary
    QR( M, N, A.data, Q.data, R.data, H.data );
  }
  void QR(int M, int N, T A[], T Q[], T R[], T H[]=NULL) {
    // 'A' is MxN, 'Q' is MxN, 'R' is NxN, and 'H' is 'MxN'
    TNT::Array2D<T> TA(M,N, A);
    JAMA::QR<T> jQR( TA );		// QR decomposition
    int i, j, k, pos;
    for (k = N-1; k >= 0; k--) {	// get 'Q'
      for (i = 0; i < M; i++) Q[i*N+k] = 0.0;
      Q[k*N+k] = 1.0;
      for (j = k; j < N; j++)
	if (jQR.QR_[k][k] != 0) {
	  T s = 0.0;
	  for (i = k; i < M; i++) s += jQR.QR_[i][k]*Q[i*N+j];
	  s = -s / jQR.QR_[k][k];
	  for (i = k; i < M; i++) Q[i*N+j] += s * jQR.QR_[i][k];
	}
    }
    for (i = 0; i < N; i++)		// get 'R'
      for (j = 0; j < N; j++) {
	if      (i <  j) R[i*N + j] = jQR.QR_[i][j];
	else if (i == j) R[i*N + j] = jQR.Rdiag[i];
	else             R[i*N + j] = 0.0;
      }
    if (H) {				// householder vectors
      for (int i = 0; i < M; i++)
	for (int j = 0; j < N; j++)
	  if (i >= j) H[i*N + j] = jQR.QR_[i][j];
	  else H[i*N + j] = 0.0;
    }
  }

public:
  void RQ_3x4(T A[12], T R[9], T Q[9]) {
    // separate projection matrix into intrinsic/extrinsic parameters
    T *A2[3], *R2[3], *Q2[3];
    A2[0] = A + 0;  A2[1] = A + 4;  A2[2] = A + 8;
    R2[0] = R + 0;  R2[1] = R + 3;  R2[2] = R + 6;
    Q2[0] = Q + 0;  Q2[1] = Q + 3;  Q2[2] = Q + 6;
    RQ_(A2, R2, Q2);
  }
private:
  void RQ_(T *A[3], T *R[3], T *Q[3]) {
    T W[2][3], *v, x[3], tmp[3], tmp2;
    memset(&W[0][0], 0, 6*sizeof(T));
    for (int i = 0; i < 3; i++) {
      memcpy(R[i], A[i], 3*sizeof(T));
      memset(Q[i],    0, 3*sizeof(T));  Q[i][i] = 1.0;
    }
    // the third row of R and Q
    v = W[1];
    V3D_COPY(x, R[2]);
    V3D_COPY(v, x);
    v[2] = (x[2] >= 0 ? +1 : -1) * V3D_LENGTH(x);
    R[2][0] = 0;  R[2][1] = 0;  R[2][2] = -v[2];
    v[2] = v[2] + x[2];
    V3D_NORMALIZE(v);
    tmp[0] = 2 * (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2]);
    tmp[1] = 2 * (R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2]);
    R[0][0] -= tmp[0] * v[0];  R[0][1] -= tmp[0] * v[1];  R[0][2] -= tmp[0] * v[2];
    R[1][0] -= tmp[1] * v[0];  R[1][1] -= tmp[1] * v[1];  R[1][2] -= tmp[1] * v[2];
    tmp2 = Q[2][0] * v[0] + Q[2][1] * v[1] + Q[2][2] * v[2];
    tmp[0] = tmp2 * v[0];  tmp[1] = tmp2 * v[1];  tmp[2] = tmp2 * v[2];
    Q[2][0] -= 2 * tmp[0];  Q[2][1] -= 2 * tmp[1];  Q[2][2] -= 2 * tmp[2];
    // the second row of R and Q
    v = W[0];
    V2D_COPY(x, R[1]);
    V2D_COPY(v, x);
    v[1] = (x[1] >= 0 ? +1 : -1) * V2D_LENGTH(x);
    R[1][0] = 0;  R[1][1] = -v[1];
    v[1] = v[1] + x[1];
    V2D_NORMALIZE(v);
    tmp[0] = 2 * (R[0][0] * v[0] + R[0][1] * v[1]);
    R[0][0] -= tmp[0] * v[0];  R[0][1] -= tmp[0] * v[1];
    v = W[0];
    tmp2 = Q[1][0] * v[0] + Q[1][1] * v[1];
    tmp[0] = tmp2 * v[0];  tmp[1] = tmp2 * v[1];
    Q[1][0] -= 2 * tmp[0];  Q[1][1] -= 2 * tmp[1];
    v = W[1];
    tmp2 = Q[1][0] * v[0] + Q[1][1] * v[1] + Q[1][2] * v[2];
    tmp[0] = tmp2 * v[0];  tmp[1] = tmp2 * v[1];  tmp[2] = tmp2 * v[2];
    Q[1][0] -= 2 * tmp[0];  Q[1][1] -= 2 * tmp[1];  Q[1][2] -= 2 * tmp[2];
    // the first row of Q
    v = W[0];
    tmp2 = Q[0][0] * v[0] + Q[0][1] * v[1];
    tmp[0] = tmp2 * v[0];  tmp[1] = tmp2 * v[1];
    Q[0][0] -= 2 * tmp[0];  Q[0][1] -= 2 * tmp[1];
    v = W[1];
    tmp2 = Q[0][0] * v[0] + Q[0][1] * v[1] + Q[0][2] * v[2];
    tmp[0] = tmp2 * v[0];  tmp[1] = tmp2 * v[1];  tmp[2] = tmp2 * v[2];
    Q[0][0] -= 2 * tmp[0];  Q[0][1] -= 2 * tmp[1];  Q[0][2] -= 2 * tmp[2];
    if (R[2][2] < 0) {
      R[0][2] *= -1;  R[1][2] *= -1;  R[2][2] *= -1;	// negate 3rd col vector
      Q[2][0] *= -1;  Q[2][1] *= -1;  Q[2][2] *= -1;	// negate 3rd row vector
    }
    if (R[1][1] < 0) {
      R[0][1] *= -1;  R[1][1] *= -1;  R[2][1] *= -1;	// negate 2nd col vector
      Q[1][0] *= -1;  Q[1][1] *= -1;  Q[1][2] *= -1;	// negate 2nd row vector
    }
    if (R[0][0] < 0) {
      R[0][0] *= -1;  R[1][0] *= -1;  R[2][0] *= -1;	// negate 1st col vector
      Q[0][0] *= -1;  Q[0][1] *= -1;  Q[0][2] *= -1;	// negate 1st row vector
    }
  }

  // ================================================================
  // SVD decomposition ----------------------------------------------
  //   Calculate A = U*S*V' for an mxn matrix A with m >= n, where
  // U is mxn orthogonal, S is nxn diagonal, and V is nxn orthogonal matrix.
  //   The singular values, sigma[k] = S[k][k], are ordered
  // so that sigma[0] >= sigma[1] >= ... >= sigma[n-1].
  //   The singular value decompostion ALWAYS EXISTS, so the constructor
  // will never fail. The matrix condition number and the effective
  // numerical rank can be computed from this decomposition.
  // NOTE: YOU MUST MAKE PUBLIC 'SVD::U', 'SVD::V', and 'SVD::s' in 'jama_svd.h'.
  // ================================================================

public:
  void SVD(Matrix<T> &A, Matrix<T> &U, Matrix<T> &S, Matrix<T> &V, bool matrix_s = false) {
    // Note that singular values are in descending order.
    // Note that this function returns U and V, not Vt, where A = U * S * Vt.
    int M = A.nRow, N = A.nCol;
    U.set(M,N);  S.set(N, (matrix_s ? N : 1));  V.set(N,N);
    SVD( M, N,  A.data, U.data, S.data, V.data, matrix_s );
  }
  void SVD(int M, int N, T A[], T U[], T S[], T V[], bool matrix_s = false) {
    // 'A' is MxN, 'U' is MxN, 'S' is NxN, and 'V' is 'NxN'
    // Note that singular values are in descending order.
    // Note that this function returns U and V, not Vt, where A = U * S * Vt.
    TNT::Array2D<T> TA(M,N, A);
    JAMA::SVD<T> jSVD( TA );		// SVD decomposition
    int i, mxn = M*N, nxn = N*N;
    memcpy( U, &(jSVD.U[0][0]), mxn * sizeof(T) );	// get 'U'
    if (matrix_s) {
      memset( S, 0, nxn * sizeof(T) );			// get 'S' as matrix
      for (i = 0; i < N; i++) S[i*N+i] = jSVD.s[i];
    } else {
      memcpy( S, &(jSVD.s[0]), N * sizeof(T) );		// get 'S' as vector
    }
    memcpy( V, &(jSVD.V[0][0]), nxn * sizeof(T) );	// get 'V'
  }

  // ================================================================
  // Cholesky decomposition -----------------------------------------
  //   For a symmetric, positive definite matrix A,
  // this function computes the Cholesky factorization, i.e.
  // it computes a lower triangular matrix L such that A = L*L'.
  // If the matrix is not symmetric or positive definite,
  // the function computes only a partial decomposition.
  // This can be tested with the is_spd() flag.
  // ================================================================

public:
  bool Cholesky(Matrix<T> &A, Matrix<T> &L) {
    int size = A.nRow;
    if (A.nRow != A.nCol) return false;
    return Cholesky( size, A.data, L.data );
  }
  bool Cholesky(int size, T A[], T L[]) {
    // 'A' and 'L' is  sizexsize
    TNT::Array2D<T> TA(size, size, A);
    JAMA::Cholesky<T> jCholesky( TA );	// Cholesky decomposition
    int i, j, pos;
    for (i = 0; i < size; i++)		// get 'L' and 'U'
      for (j = 0; j < size; j++) {
	pos = i * size + j;
	L[pos] = ( i >= j ? jCholesky.L_[i][j] : 0.0 );
      }
    return (jCholesky.isspd == 1);
  }

  // ================================================================
  // Eigenvalue decomposition ---------------------------------------
  // Computes eigenvalues and eigenvectors of a real (non-complex) matrix.
  //   If A is symmetric, then A = V*D*V' where the eigenvalue matrix D
  // is diagonal and the eigenvector matrix V is orthogonal. That is,
  // the diagonal values of D are the eigenvalues, and V*V' = I, where
  // I is the identity matrix. The columns of V represent the eigenvectors
  // in the sense that A*V = V*D.
  //   If A is not symmetric, then the eigenvalue matrix D is block diagonal
  // with the real eigenvalues in 1x1 blocks and any complex eigenvalues,
  // a + i*b, in 2x2 blocks, [a, b; -b, a]. This keeps V a real matrix
  // in both symmetric and non-symmetric cases, and A*V = V*D.
  // NOTE: YOU MUST MAKE PUBLIC 'Eigenvalue::V' and 'Eigenvalue::d' in 'jama_eig.h'.
  // ================================================================

public:
  void Eig(Matrix<T> &A, Matrix<T> &V, Matrix<T> &D, bool matrix_d = false) {
    // Note that eigenvalues are not sorted.
    if (A.nRow != A.nCol) return;
    int N = A.nRow;
    V.set(N,N);  D.set(N, (matrix_d ? N : 1));
    Eig( N,  A.data, V.data, D.data, matrix_d );
  }
  void Eig(int N, T A[], T V[], T D[], bool matrix_d = false) {
    // 'A' is MxN, 'U' is MxN, 'S' is NxN, and 'V' is 'NxN'
    // Note that eigenvalues are not sorted.
    TNT::Array2D<T> TA(N,N, A);
    JAMA::Eigenvalue<T> jEig( TA );		// Eigenvalue decomposition
    int i, j, pos, nxn = N*N;
    memcpy( V, &(jEig.V[0][0]), nxn * sizeof(T) );	// get 'V' (eigenvectors)
    if (matrix_d) {
      memset( D, 0, nxn * sizeof(T) );			// get 'D' as matrix
      for (i = 0; i < N; i++) D[i*N+i] = jEig.d[i];
    } else {
      memcpy( D, &(jEig.d[0]), N * sizeof(T) );		// get 'D' (eigenvalues)
    }
  }

};

}	// end of namespace MTX


#endif  // MTX_MATRIX_SOLVER_HPP


// ===================================================================
#if 0	// start of example code
// ===================================================================
#include <iostream>
#include "mtx_matrix_solver.hpp"
using namespace std;
int main(void)
{
  MTX::Matrix<double> A(2,2), Ai(2,2);
  MTX::MatrixSolver<double> solver;
  A.assign(2, 2,  2.0, 1.0, 4.0, 3.0 );
  solver.inverseByDeterminant(2, A.data, Ai.data);
  MTX::PrintMatrices(A, Ai, "A  and  Ainv  by determinant");
  return EXIT_SUCCESS;
}
// ===================================================================
#endif		// end of example code
// ===================================================================
