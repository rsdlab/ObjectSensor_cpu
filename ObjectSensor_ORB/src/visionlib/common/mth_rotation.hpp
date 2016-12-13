
//
// MTH::Rotation<> class template
//
// Jaeil Choi
// Last modified in Dec, 2005
//

#ifndef MTH_ROTATION_HPP
#define MTH_ROTATION_HPP
#define USE_MTH_ROTATION

#include <iostream>
#include <cmath>
#include "vm_macros.h"

namespace MTH {

template <class T>
class Rotation {
 public:
  char  info[256];
 public:
  Rotation()  { }
  ~Rotation() { }

  // ================================================================
  // Rotation vector and Rodrigues rotation formula
  // ================================================================
public:
  // O. Faugeras. "Three-dimensional Computer Vision: A Geometric Viewpoint", MIT 1993

  void R2Rv(T R[9], T r[3], T *theta=NULL) {
    // convert rotation matrix to rotation axis and angle (radian)
    T q[4], angle;
    R2Q( R, q );           // rotation matrix -> normalized quaternion
    Q2Rv( q, r, &angle, false );  // quaternion -> rotation axis and angle
    if (theta) *theta = angle;
    else G3V_MUL_VALUE( r, angle );
  }

  void Rv2R(T r[3], T R[9]) {
    // convert rotation axis (|r|=angle) to rotation matrix
    T    len,  ra[3];
    if ((len = G3V_LENGTH(r)) == 0) {
      G3M_SET( R,  1, 0, 0,  0, 1, 0,  0, 0, 1 );
    } else {
      G3V_SET( ra, r[0]/len, r[1]/len, r[2]/len );
      Rv2R( ra, len, R, false );
    }
  }

  void Rv2R(T r[3], T theta, T R[9], bool normalize=true) {
    // convert rotation axis and angle to rotation matrix
    if (normalize) G3V_NORMALIZE_WITH_TYPE( r, T );
    T  c, s, u;
    c = cos(theta);  s = sin(theta);  u = 1 - c;
    G3M_SET( R,
	     r[0]*r[0]*u + c,      r[0]*r[1]*u - r[2]*s,  r[0]*r[2]*u + r[1]*s,
	     r[0]*r[1]*u + r[2]*s, r[1]*r[1]*u + c,       r[1]*r[2]*u - r[0]*s,
	     r[0]*r[2]*u - r[1]*s, r[1]*r[2]*u + r[0]*s,  r[2]*r[2]*u + c );
  }

  // ================================================================
  // Quaternion   q[] = (w, x, y, z)
  // ================================================================
public:
  inline T QNorm(T q[4]) {
    return sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
  }
  void QNormalize(T q[4]) {
    T len = QNorm( q );
    if (len < 0.000001) return;
    q[0] /= len;  q[1] /= len;  q[2] /= len;  q[3] /= len;
  }

  void R2Q(T R[9], T q[4]) {
    // convert rotation matrix to normalized quaternion
    T  s, tr = 1 + R[0] + R[4] + R[8];
    if (tr > 0.00000001) {
      s = sqrt(tr) * 2;
      q[0] = 0.25 * s;
      q[1] = ( R[7] - R[5] ) / s;
      q[2] = ( R[2] - R[6] ) / s;
      q[3] = ( R[3] - R[1] ) / s;
    } else {
      if (R[0] > R[4] && R[0] > R[8]) {
	s = sqrt( 1.0 + R[0] - R[4] - R[8] ) * 2;
	q[0] = ( R[7] - R[5] ) / s;
	q[1] = 0.25 * s;
	q[2] = ( R[3] + R[1] ) / s;
	q[3] = ( R[2] + R[6] ) / s;
      } else if (R[4] > R[8]) {
	s = sqrt( 1.0 + R[4] - R[0] - R[8] ) * 2;
	q[0] = ( R[2] - R[6] ) / s;
	q[1] = ( R[3] + R[1] ) / s;
	q[2] = 0.25 * s;
	q[3] = ( R[7] + R[5] ) / s;
      } else {
	s = sqrt( 1.0 + R[8] - R[0] - R[4] ) * 2;
	q[0] = ( R[3] - R[1] ) / s;
	q[1] = ( R[2] + R[6] ) / s;
	q[2] = ( R[7] + R[5] ) / s;
	q[3] = 0.25 * s;
      }
    }
    QNormalize(q);
  }
  void Q2R(T q[4], T R[9], bool normalize=true) {
    // convert quaternion to rotation matrix
    if (normalize) QNormalize( q );
    T xx = q[1]*q[1];  T xy = q[1]*q[2];  T xz = q[1]*q[3];
    T xw = q[1]*q[0];  T yy = q[2]*q[2];  T yz = q[2]*q[3];
    T yw = q[2]*q[0];  T zz = q[3]*q[3];  T zw = q[3]*q[0];
    R[0] = 1 - 2 * ( yy + zz );
    R[1] =     2 * ( xy - zw );
    R[2] =     2 * ( xz + yw );
    R[3] =     2 * ( xy + zw );
    R[4] = 1 - 2 * ( xx + zz );
    R[5] =     2 * ( yz - xw );
    R[6] =     2 * ( xz - yw );
    R[7] =     2 * ( yz + xw );
    R[8] = 1 - 2 * ( xx + yy );
  }
  void Q2Rv(T q[4], T axis[3], T *angle, bool normalize=true) {
    // convert quaternion to rotation axis and angle(radian)
    if (normalize) QNormalize( q );
    T cos = q[0];
    T sin = sqrt( 1 - cos * cos );
    if ( fabs(sin) < 0.0005 ) sin = 1;
    *angle = acos( cos ) * 2;
    axis[0] = q[1] / sin;
    axis[1] = q[2] / sin;
    axis[2] = q[3] / sin;
  }
};


}	// end of namespace MTH

#endif // CROTATION_HPP
