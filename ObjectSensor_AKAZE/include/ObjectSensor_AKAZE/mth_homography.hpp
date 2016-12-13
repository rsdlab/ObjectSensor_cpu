
//
// MTH::Homography<> class template
//
// Jaeil Choi
// last modified in Dec, 2005
//

#ifndef MTH_HOMOGRAPHY_HPP
#define MTH_HOMOGRAPHY_HPP

#include <iostream>
#include "vm_macros.h"
#include "mtx_matrix.hpp"
#include "mtx_matrix_solver.hpp"

namespace MTH {

template <class T>
class Homography
{
public:
  T			*h;		// 3x3 homography (DOF 8)
  T			dumh[9];	// 3x3 homography (DOF 8)
  MTX::Matrix<T>	H;		// MTX::Matrix wrapper of 'h[9]'

public:
  Homography() : h(dumh) { H.set(3,3,h); };
  Homography(T *data) : h(data) { H.set(3,3,h); };
  Homography(MTX::Matrix<T> &A, MTX::Matrix<T> &B) : h(dumh) { H.set(3,3,h); calcaulteHomography(A, B); };
  Homography(T c0[3], T c1[3], T c2[3]) : h(dumh) { setHomography(c0, c1, c2); };
  ~Homography() {};

  // -----------------------------------------------------------------
  // Creating homography from 4 points
  // USE <double> FOR THIS TEMPLATE, TO CALCULATE THE HOMOGRAPHY CORRECTLY.
  // -----------------------------------------------------------------
public:
  bool calculateHomography(T p0[2], T p1[2], T p2[2], T p3[2],
	      T q0[2], T q1[2], T q2[2], T q3[2]) {
    //  [ wx' ] = [ a b c ] * [ x ]  =>  (ax + by + c) / (gx + hy + 1) = x'
    //  [ wy' ]   [ d e f ]   [ y ]      (dx + ey + f) / (gx + hy + 1) = y'
    //  [  w  ]   [ g h 1 ]   [ 1 ]
    MTX::Matrix<T> A, B, X(8,1, h);
    A.assign( 8, 8,
	      p0[0], p0[1], 1.0,  0.0,   0.0,   0.0,  -q0[0]*p0[0], -q0[0]*p0[1],
	      0.0,   0.0,   0.0,  p0[0], p0[1], 1.0,  -q0[1]*p0[0], -q0[1]*p0[1],
	      p1[0], p1[1], 1.0,  0.0,   0.0,   0.0,  -q1[0]*p1[0], -q1[0]*p1[1],
	      0.0,   0.0,   0.0,  p1[0], p1[1], 1.0,  -q1[1]*p1[0], -q1[1]*p1[1],
	      p2[0], p2[1], 1.0,  0.0,   0.0,   0.0,  -q2[0]*p2[0], -q2[0]*p2[1],
	      0.0,   0.0,   0.0,  p2[0], p2[1], 1.0,  -q2[1]*p2[0], -q2[1]*p2[1],
	      p3[0], p3[1], 1.0,  0.0,   0.0,   0.0,  -q3[0]*p3[0], -q3[0]*p3[1],
	      0.0,   0.0,   0.0,  p3[0], p3[1], 1.0,  -q3[1]*p3[0], -q3[1]*p3[1] );
    B.assign( 8, 1,  q0[0], q0[1],  q1[0], q1[1],  q2[0], q2[1],  q3[0], q3[1] );
    MTX::MatrixSolver<T> solver;
    bool ret = solver.solveBySVD( A, X, B );
    h[8] = 1.0;
    return ret;
  }

  // -----------------------------------------------------------------
  // Creating homography from multiple points
  // USE <double> FOR THIS TEMPLATE, TO CALCULATE THE HOMOGRAPHY CORRECTLY.
  // -----------------------------------------------------------------
public:
  bool calculateHomography(MTX::Matrix<T> &A, MTX::Matrix<T> &B) {
    // B = H A, where
    //   A : 2xN or 3xN 2D coordinate matrix of source points      (3rd row == 1.0)
    //   B : 2xN or 3xN 2D coordinate matrix of destination points (3rd row == 1.0)
    if (A.nRow < 2 || B.nRow < 2 || A.nCol != B.nCol) return false;
    int i, N = A.nCol;

    // normalize point coordinates (uv image coordinates)
    MTX::Matrix<T>  Ta,    Tbi;
    T           ta[3]={0,0,0}, tb[3]={0,0,0};
    normalizeUV(A, ta, Ta, false);  // ta[3]=(cx, cy, scale)  and  An = Ta A
    normalizeUV(B, tb, Tbi, true);  // tb[3]=(cx, cy, scale)  and  B = Tbi Bn

    // set the equation for H
    // s [ bu ] = [   h1t  ] [ au ]  =>  bu = (h1t a) / (h3t a)
    //   [ bv ]   [   h2t  ] [ av ]      bv = (h2t a) / (h3t a)
    //   [ 1  ]   [   h3t  ] [ 1  ]
    // => (h1t a)         - (h3t a) bu = e1 =>  [   at  0 0 0  -bu*(at) ] [ h1 ] = [ 0 ]
    //            (h2t a) - (h3t a) bv = e2     [ 0 0 0   at   -bv*(at) ] [ h2 ]   [ 0 ]
    //                                                                    [ h3 ]   [ 0 ]
    // since the last element of H (h3[2]) is 1, move the last column, and make it 'Ax = b'.
    MTX::Matrix<T> M(2*N, 8), b(2*N, 1), X(8,1,h);
    T  *mp, *bp, au, av, bu, bv;
    M.assignZero();
    for (i = 0, mp = M.data, bp = b.data; i < N; i++) {
      au = (A.data[i+0] - ta[0]) * ta[2];   av = (A.data[i+N] - ta[1]) * ta[2];
      bu = (B.data[i+0] - tb[0]) * tb[2];   bv = (B.data[i+N] - tb[1]) * tb[2];
      G8V_SET( mp,   au,  av, 1.0,  0.0, 0.0, 0.0,  -bu*au, -bu*av );  mp += 8;  *bp++ = bu;
      G8V_SET( mp,  0.0, 0.0, 0.0,   au,  av, 1.0,  -bv*au, -bv*av );  mp += 8;  *bp++ = bv;
    }
    // solve M * x = b using SVD  (numerically stable solution)
    MTX::MatrixSolver<T> solver;
    solver.solveBySVD( M, X, b );
    h[8] = 1.0f;

    // inverse the normalization trasformation of uv coordinates
    // ( H = Tb^ Hn Ta, since Bn = Hn An => (Tb B) = Hn (Ta A) )
    H.mult( Tbi, H, Ta );
    if (h[8] != 1.0) H.multValue( 1.0 / H.data[8] );
    h[8] = 1.0f;

    return true;
  }

  bool checkHomography(MTX::Matrix<T> &A, MTX::Matrix<T> &B, MTX::Matrix<T> &H,
		       float fraction=0.1, bool verbose=false) {
    MTX::Matrix<T> Tmp, Rst, Ans;
    int i, N = A.nCol;
    T   len=0, len_avg=0, dist=0, dist_avg=0, dist_max=0, dist_min=0;
    if (fraction <= 0 || fraction > 1.0) fraction = 0.1;
    if (verbose) printf("Checking homography for %d points .. ", N);
    for (i = 0; i < N; i++) {
      Tmp.assign(3,1,  A(0,i), A(1,i), 1.0);
      Ans.assign(3,1,  B(0,i), B(1,i), 1.0);
      Rst.mult( H, Tmp );
      Rst.multValue( 1/Rst(2) );
      len  = G2V_LENGTH( Rst.data );
      dist = G2V_DISTANCE( Rst.data, Ans.data );
      len_avg  += len;
      dist_avg += dist;
      if (i==0 || dist > dist_max) dist_max = dist;
      if (i==0 || dist < dist_min) dist_min = dist;
    }
    len_avg /= N;
    dist_avg /= N;
    bool good = (dist_avg < len_avg * fraction);
    if (verbose) {
      if (good) printf(" GOOD   (dist avg=%6.2f  min=%5.2f  max=%6.2f)\n", dist_avg, dist_min, dist_max);
      else      printf(" BAD !  (dist avg=%6.2f  min=%5.2f  max=%6.2f)\n", dist_avg, dist_min, dist_max);
    }
    return good;
  }

  bool checkConvexity(bool same_orientation=true) {
    // Check if it transforms a rectangle to a convex shape.
    T p[4][2]={ {0,0}, {100,0}, {100,100}, {0,100} }, q[4][2];
    int  i;
    for (i=0; i<4; i++) transform( p[i], q[i] );
    T v1p[2], v12[2], side, side_last=0;
    for (i=0; i<4; i++) {
      T *vp=q[i], *v1=q[(i+1)%4], *v2=q[(i+2)%4];
      G2V_SUB( v1p, vp, v1 );
      G2V_SUB( v12, v2, v1 );
      side = (v12[0] * v1p[1] - v12[1] * v1p[0]);
      //printf("p(%.2f %.2f) v1(%.2f %.2f) v2(%.2f %.2f)  side=%.2f\n", vp[0], vp[1], v1[0], v1[1], v2[0], v2[1], side);
      if (same_orientation) { if (side <= 0) return false; }
      else { if (i>0 && side_last*side <= 0) return false; }
      side_last = side;
    }
    return true;
  }

  void setHomography(T c0[3], T c1[3], T c2[3]) {
    H.set(3, 3, h);
    G3M_SET( h,  c0[0], c1[0], c2[0], c0[1], c1[1], c2[1], c0[2], c1[2], c2[2] );
  }

private:
  void normalizeUV(MTX::Matrix<T> &M, T tr[3], MTX::Matrix<T> &Tr, bool inverse=false) {
    // find a transformation of image coordinates in M (2xN or 3xN), so that their
    // centroid is at the origin and average distance is equal to sqrt(2).
    //   tr[3]   : The center and scale  ( cx, cy, scale )
    //   Tr(3,3) : The same transformation as a (inverse) matrix
    if (M.nRow < 2 || M.nCol <= 0) return;
    int  pidx, N = M.nCol;
    int  N2 = (N > 20 ? 20 : N);    // There might be TOO many points,
    //			and the normalization doesn't have to be precise.
    T    cx, cy, x, y, dist, scale;
    // translation of the centroid
    for (pidx = 0, cx = cy = 0; pidx < N2; pidx++) {
      cx += M.data[pidx+0];
      cy += M.data[pidx+N];
    }
    cx = cx / N2;   cy = cy / N2;
    // isotropic scaling
    for (pidx = 0, dist = 0; pidx < N2; pidx++) {
      x = M.data[pidx+0] - cx;
      y = M.data[pidx+N] - cy;
      dist += sqrt(x * x + y * y);
    }
    dist = (dist / N2);
    scale = 1.414213 / dist;
    G3V_SET( tr, cx, cy, scale );
    // save the transformation in Tr
    if (!inverse) {	// M -> Mn
      Tr.assign( 3, 3,  scale, 0.0, -cx*scale,  0.0, scale, -cy*scale,  0.0, 0.0, 1.0 );
    } else {		// Mn -> M
      Tr.assign( 3, 3,  1/scale, 0.0, cx,   0.0, 1/scale, cy,   0.0, 0.0, 1.0 );
    }
  }

  // -----------------------------------------------------------------
  // Using homography
  // -----------------------------------------------------------------
public:
  void transform(T p[2], T q[2]) {
    T  w =  h[6] * p[0] + h[7] * p[1] + h[8];
    q[0] = (h[0] * p[0] + h[1] * p[1] + h[2]) / w;
    q[1] = (h[3] * p[0] + h[4] * p[1] + h[5]) / w;
  }
  void transform(T px, T py, T q[2]) {
    T  w =  h[6] * px + h[7] * py + h[8];
    q[0] = (h[0] * px + h[1] * py + h[2]) / w;
    q[1] = (h[3] * px + h[4] * py + h[5]) / w;
  }

  // -----------------------------------------------------------------
  // Etc.
  // -----------------------------------------------------------------
public:
  void printInfo(const char *title=NULL, T p0[2]=NULL, T p1[2]=NULL, T p2[2]=NULL) {
    title = (title ? title : "Homography");
    int  i, len = strlen(title);  char blank[81];
    for (i=0; i<len && i<80; i++) blank[i]=' '; blank[i] = '\0';
    T q0[2];  if (p0) transform( p0, q0 );
    T q1[2];  if (p1) transform( p1, q1 );
    T q2[2];  if (p2) transform( p2, q2 );
    printf("%s [ %10.4f %10.4f %10.4f ]", title, h[0], h[1], h[2]);  if (p0) printf("   xy(%4.1f %4.1f) -> uv(%5.1f %5.1f) \n", p0[0], p0[1], q0[0], q0[1]);
    printf("%s [ %10.4f %10.4f %10.4f ]", blank, h[3], h[4], h[5]);  if (p1) printf("   xy(%4.1f %4.1f) -> uv(%5.1f %5.1f) \n", p1[0], p1[1], q1[0], q1[1]);
    printf("%s [ %10.4f %10.4f %10.4f ]", blank, h[6], h[7], h[8]);  if (p2) printf("   xy(%4.1f %4.1f) -> uv(%5.1f %5.1f) \n", p2[0], p2[1], q2[0], q2[1]);
  }

};

}	// end of namespace MTH

#endif	// MTH_HOMOGRAPHY_HPP


// ===================================================================
#if 0	// beginning of the example code
// ===================================================================
#include <iostream>
#include "src/common/mth_homography.hpp"
using namespace std;
int main(void)
{
  MTH::Homography<double> H;
  double p0[2]={0,0}, p1[2]={0,1}, p2[2]={1,0}, p3[2]={1,1};
  double q0[2]={4,4}, q1[2]={4,6}, q2[2]={6,4}, q3[2]={6,6};
  H.calculateHomography( p0, p1, p2, p3, q0, q1, q2, q3 );
  H.printInfo(NULL, p0, p1, p2);
  double p[] = { 0, 0, 1, 1,  0, 1, 0, 1 };
  double q[] = { 4, 4, 6, 6,  4, 6, 4, 6 };
  MTX::Matrix<double> P(2,4,p), Q(2,4,q);
  H.calculateHomography( P, Q );
  H.printInfo(NULL, p0, p1, p2);
  return EXIT_SUCCESS;
}
// ===================================================================
#endif		// end of the example code
// ===================================================================
