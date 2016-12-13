
//
// Geometry3D<T> class template
//
// Jaeil Choi
// last modified in June, 2003
//
// Note that functions of template classes are included
// in the compiled codes only if they are called.
//

#ifndef CGEOMETRY3D_HPP
#define CGEOMETRY3D_HPP

#include <iostream>
#include <iomanip>
#include <cfloat>
#include <cmath>
#include "vm_macros.h"
using namespace std;


template <class T>
class Geometry3D {

 public:

  // ================================================================
  // Line / Line Segment
  //
  // T     getRandomSampleOnLineSegment(T v0[3], T v1[3], T p[3]);
  // T     getDistanceLinesegPoint( T v0[3], T v1[3], T p[3], T *projection = NULL);
  // void  getGradientLineLength(T v0[3], T v1[3], T result[3]);
  // ================================================================

  bool getPointByIntersectingTwoLines(T ap[3], T adir[3], T bp[3], T bdir[3], T xp[3], int mode=0) {
    // find the plane of 'ap[3]', 'bp[3]', and 'xyz[3]',
    //   and calculate the rotation matrix for the plane.
    T nx[3], ny[3], nz[3], Rno[9];
    G3V_SUB  ( nx, bp, ap );  G3V_NORMALIZE( nx );
    if      (mode < 0)  G3V_CROSS( nz, nx, adir );	// intersection on line 'a'
    else if (mode > 0)  G3V_CROSS( nz, nx, bdir );	// intersection on line 'b'
    else                G3V_CROSS( nz, adir, bdir );	// intersection in between
    G3V_NORMALIZE( nz );
    G3V_CROSS( ny, nz, nx );
    G3M_SET( Rno, nx[0], nx[1], nx[2], ny[0], ny[1], ny[2], nz[0], nz[1], nz[2] );
    // get 2D lines on the new plane using the rotation 'Rno[9]'
    T apn[3], adirn[3], bpn[3], bdirn[3], lineA[3], lineB[3];
    G3M_MUL_MV( apn, Rno, ap );  G3M_MUL_MV( adirn, Rno, adir );
    G3M_MUL_MV( bpn, Rno, bp );  G3M_MUL_MV( bdirn, Rno, bdir );
    G2V_LINE_EQ_NP( lineA, -adirn[1], +adirn[0], apn[0], apn[1] );
    G2V_LINE_EQ_NP( lineB, -bdirn[1], +bdirn[0], bpn[0], bpn[1] );
    // intersect the two lines
    T xyz[3] = {0,0,0};
    T det = ( lineA[0] * lineB[1] - lineA[1] * lineB[0] );
    if (fabs(det) < 0.0001) return false;
    xyz[0] = - (+ lineB[1] * lineA[2] - lineA[1] * lineB[2]) / det;
    xyz[1] = - (- lineB[0] * lineA[2] + lineA[0] * lineB[2]) / det;
    // get the intersection point back to 3D using the rotation 'Rno[9]'
    G3M_MUL_VM( xp, xyz, Rno );
    return true;
  }

  // ================================================================
  // Plane
  //
  // bool getPlane(T plane[4], T p0[3], T p1[3], T p2[3]);
  // int  intersectPlaneAndLineDir( T plane[4], T from[3], T dir[3], T result[3] = NULL);
  // ================================================================

  bool getPlane(T plane[4], T p0[3], T p1[3], T p2[3]) {
    // calculate the equation of the plane containing given three points
    // assuming the points are CCW order
    T v1[3], v2[3];
    G3V_SUB( v1, p1, p0 );  G3V_NORMALIZE( v1 );
    G3V_SUB( v2, p2, p0 );  G3V_NORMALIZE( v2 );
    G3V_CROSS( plane, v1, v2 );  G3V_NORMALIZE( plane );
    plane[3] = - G3V_DOT( plane, p0 );
    return true;
  }
  bool getPlaneFromTwoLines(T plane[4], T p[3], T dir1[3], T dir2[3]) {
    T p1[3];  G3V_ADD( p1, p, dir1 );
    T p2[3];  G3V_ADD( p2, p, dir2 );
    return getPlane( p, p1, p2 );
  }
  T DistanceFromPointToPlane(T point[3], T p0[3], T p1[3], T p2[3]) {
    T plane[4];
    getPlane( plane, p0, p1, p2 );
    return ( G3V_DOT( plane, point ) + plane[3] );
  }

  int intersectPlaneAndLineDir( T plane[4], T from[3], T dir[3], T result[3] = NULL) {
    int intersect;
    // check whether they intersect or not
    if      ( G3V_DOT( dir, plane ) < -1.0e-8 ) intersect = -1;  // from front to back
    else if ( G3V_DOT( dir, plane ) > +1.0e-8 ) intersect = +1;  // from back to front
    else intersect = 0;                                     // does not intersect
    // find the coordinates of the intersection point
    if (intersect != 0 && result != NULL) {
      // N . (from + t * dir) + plane[3] = 0  , where N = (plane[0] plane[1] plane[2])
      // t = - ( N . from + plane[3] ) / ( N . dir )
      // result = from + t * dir
      T t = -( G3V_PLANE_POINT(plane, from) / G3V_DOT(plane, dir) );
      G3V_SCALED_ADD( result, from, t, dir );
    }
    return intersect;
  }

  bool getLineByIntersectingTwoPlanes(T pa[4], T pb[4], T xyz[3], T dir[3]) {
    // calculate the normal of the intersecting line using cross product
    if (fabs(G3V_DOT(pa, pb)) > 0.999) return false;
    G3V_CROSS( dir, pa, pb );  G3V_NORMALIZE( dir );
    // find a point that lies on both 'pa' and 'pb'
    T  A[4], x[2], b[2], Ai[4], det;
    if (fabs(dir[0]) > fabs(dir[1])) {	// search it on YZ plane (when X=0)
      G2M_SET( A, pa[1], pa[2], pb[1], pb[2] );
      G2V_SET( b, -pa[3], -pb[3] );	// A * x = b
      det = ( A[0] * A[3] - A[1] * A[2] );
      G2M_SET( Ai, +A[3]/det, -A[1]/det, -A[2]/det, +A[0]/det );
      G2M_MUL_MV( x, Ai, b );		// x = Ainv * b
      G3V_SET( xyz, 0, x[0], x[1] );	// the point on the intersecting line
    } else {				// search it on XZ plane (when Y=0)
      // solve A*x = b   =>   x = Ainv * b
      G2M_SET( A, pa[0], pa[2], pb[0], pb[2] );
      G2V_SET( b, -pa[3], -pb[3] );	// A * x = b
      det = ( A[0] * A[3] - A[1] * A[2] );
      G2M_SET( Ai, +A[3]/det, -A[1]/det, -A[2]/det, +A[0]/det );
      G2M_MUL_MV( x, Ai, b );		// x = Ainv * b
      G3V_SET( xyz, x[0], 0, x[1] );	// the point on the intersecting line
    }
    return true;
  }

  // ================================================================
  // Tetrahedron
  //
  // bool getTetrahedronCircumsphere( T v0[3], T v1[3], T v2[3], T v3[3], T center[3], T &radius ) ;
  // bool getTetrahedronCircumsphere( T v0[3], T v1[3], T v2[3], T v3[3], T center[3], T &radius ) ;
  // bool getTetrahedronIncenter( T v0[3], T v1[3], T v2[3], T v3[3], T center[3], T &radius ) ;
  // T    getTetrahedronAspectRatio(T v0[3], T v1[3], T v2[3], T v3[3]) ;
  // ================================================================

  bool getTetrahedronCircumsphere( T v0[3], T v1[3], T v2[3], T v3[3], T center[3], T *radius ) {
    // | C - Vi | = R     after squaring and subtracting the equation for i = 0,
    // 2 (Vi - Vo).C = Vi.Vi - Vo.Vo	or, equivalently,
    // (Vi - Vo).(C - Vo) = 1/2 (Vi - Vo).(Vi - Vo) = 1/2 |Vi - Vo|^2
    // This is 3 equations for 3 unknowns.
    T v10[3], v20[3], v30[3], det;
    G3V_SUB( v10, v1, v0 );	//                    ( v10x v10y v10z )
    G3V_SUB( v20, v2, v0 );	//  volume = 1/6 * det( v20x v20y v20z )
    G3V_SUB( v30, v3, v0 );	//                    ( v30x v30y v30z )
    det = ( + v10[0] * ( v20[1] * v30[2] - v20[2] * v30[1] )
	    - v10[1] * ( v20[0] * v30[2] - v20[2] * v30[0] )
	    + v10[2] * ( v20[0] * v30[1] - v20[1] * v30[0] ) );
    if (det == 0) return false;
    T L10 = G3V_LENGTH2( v10 );
    T L20 = G3V_LENGTH2( v20 );
    T L30 = G3V_LENGTH2( v30 );
    center[0] = v0[0] + ( + (v20[1]*v30[2] - v20[2]*v30[1]) * L10
			  - (v10[1]*v30[2] - v10[2]*v30[1]) * L20
			  + (v10[1]*v20[2] - v10[2]*v20[1]) * L30 ) / (2 * det);
    center[1] = v0[1] + ( - (v20[0]*v30[2] - v20[2]*v30[0]) * L10
			  + (v10[0]*v30[2] - v10[2]*v30[0]) * L20
			  - (v10[0]*v20[2] - v10[2]*v20[0]) * L30 ) / (2 * det);
    center[2] = v0[2] + ( + (v20[0]*v30[1] - v20[1]*v30[0]) * L10
			  - (v10[0]*v30[1] - v10[1]*v30[0]) * L20
			  + (v10[0]*v20[1] - v10[1]*v20[0]) * L30 ) / (2 * det);
    *radius = G3V_DISTANCE( center, v0 );
    return true;
  }

  bool getTetrahedronInsphere( T v0[3], T v1[3], T v2[3], T v3[3], T center[3], T *radius ) {
    // Ni . (C - Vi) = R	where Ni are inward normals.  Equivalently,
    // (Ni,-1).(C,R) = Ni.Vi
    // This is 4 equations for 4 unknowns.

    // T M[4][4], b[4];
    // if (getTriangleNormal( v0, v2, v1, M[0] ) == false) return false;
    // if (getTriangleNormal( v1, v2, v3, M[1] ) == false) return false;
    // if (getTriangleNormal( v2, v0, v3, M[2] ) == false) return false;
    // if (getTriangleNormal( v3, v0, v1, M[3] ) == false) return false;
    // M[0][3] = M[1][3] = M[2][3] = M[3][3] = -1;
    // b[0] = G3V_DOT( M[0], v0 );
    // b[1] = G3V_DOT( M[1], v1 );
    // b[2] = G3V_DOT( M[2], v2 );
    // b[3] = G3V_DOT( M[3], v3 );
    T va[3], vb[3], N[3], L0, L1, L2, L3, L;
    G3V_SUB(va,v2,v0);  G3V_SUB(vb,v1,v0);  G3V_CROSS(N,va,vb);  L0 = G3V_LENGTH(N);
    G3V_SUB(va,v2,v1);  G3V_SUB(vb,v3,v1);  G3V_CROSS(N,va,vb);  L1 = G3V_LENGTH(N);
    G3V_SUB(va,v0,v2);  G3V_SUB(vb,v3,v2);  G3V_CROSS(N,va,vb);  L2 = G3V_LENGTH(N);
    G3V_SUB(va,v0,v3);  G3V_SUB(vb,v1,v3);  G3V_CROSS(N,va,vb);  L3 = G3V_LENGTH(N);
    if (L0 == 0 || L1 == 0 || L2 == 0 || L3 == 0) return false;
    L = L0 + L1 + L2 + L3;
    center[0] = ( v0[0]*L3 + v1[0]*L2 + v2[0]*L1 + v3[0]*L0 ) / L ;
    center[1] = ( v0[1]*L3 + v1[1]*L2 + v2[1]*L1 + v3[1]*L0 ) / L ;
    center[2] = ( v0[2]*L3 + v1[2]*L2 + v2[2]*L1 + v3[2]*L0 ) / L ;
    *radius   = abs ( + ( + v1[0] * ( v2[1]*v3[2] - v3[1]*v2[2] )
			  - v2[0] * ( v1[1]*v3[2] - v3[1]*v1[2] )
			  + v3[0] * ( v1[1]*v2[2] - v2[1]*v1[2] ) ) +
		      - ( + v0[0] * ( v2[1]*v3[2] - v3[1]*v2[2] )
			  - v2[0] * ( v0[1]*v3[2] - v3[1]*v0[2] )
			  + v3[0] * ( v0[1]*v2[2] - v2[1]*v0[2] ) ) +
		      + ( + v0[0] * ( v1[1]*v3[2] - v3[1]*v1[2] )
			  - v1[0] * ( v0[1]*v3[2] - v3[1]*v0[2] )
			  + v3[0] * ( v0[1]*v1[2] - v1[1]*v0[2] ) ) +
		      - ( + v0[0] * ( v1[1]*v2[2] - v2[1]*v1[2] )
			  - v1[0] * ( v0[1]*v2[2] - v2[1]*v0[2] )
			  + v2[0] * ( v0[1]*v1[2] - v1[1]*v0[2] ) ) ) / L ;
    return true;
  }

  // =================================================================
  // Measurement of mesh quality
  // =================================================================

  T getTetrahedronEdgeLength(T v0[3], T v1[3], T v2[3], T v3[3], bool max) {
    T d[6], ext=0;
    d[0] = G3V_DISTANCE2( v0, v1 );  d[1] = G3V_DISTANCE2( v0, v2 );
    d[2] = G3V_DISTANCE2( v0, v3 );  d[3] = G3V_DISTANCE2( v1, v2 );
    d[4] = G3V_DISTANCE2( v1, v3 );  d[5] = G3V_DISTANCE2( v2, v3 );
    if (max) {
      for (int i=0; i<6; i++) if (i == 0 || d[i] > ext) ext = d[i];
    } else {
      for (int i=0; i<6; i++) if (i == 0 || d[i] < ext) ext = d[i];
    }
    return (T)(sqrt(ext));
  }

  T getTetrahedronAltitude(T v0[3], T v1[3], T v2[3], T v3[3], bool max) {
    T d[4], ext=0;
    d[0] = DistanceFromPointToPlane( v0, v1, v2, v3 );
    d[1] = DistanceFromPointToPlane( v1, v0, v3, v2 );
    d[2] = DistanceFromPointToPlane( v2, v0, v1, v3 );
    d[3] = DistanceFromPointToPlane( v3, v0, v2, v1 );
    if (max) {
      for (int i=0; i<4; i++) if (i == 0 || d[i] > ext) ext = d[i];
    } else {
      for (int i=0; i<4; i++) if (i == 0 || d[i] < ext) ext = d[i];
    }
    return ext;
  }

  T getTetrahedronAngle(T v0[3], T v1[3], T v2[3], T v3[3], bool max) {
    T p[4][4], angle, ext;
    getPlane(p[0], v1, v3, v2);
    getPlane(p[1], v0, v2, v3);
    getPlane(p[2], v0, v3, v1);
    getPlane(p[3], v0, v1, v2);
    ext = (max ? M_PI : 0);	// calculate angle between face normals
    for (int i = 0; i < 4; i++)
      for (int j = i+1; j < 4; j++) {
	angle = (T)acos(G3V_DOT(p[i], p[j]));
	if (max) { if (angle < ext) ext = angle; }
	else     { if (angle > ext) ext = angle; }
      }
    return (M_PI - ext);	// return angle between faces
  }

  T getTetrahedronAspectRatio(T v0[3], T v1[3], T v2[3], T v3[3], int type) {
    // Note :
    //   negative types : the bigger,  the better
    //   positive types : the smaller, the better
    float numerator, denominator, tmp[3], max_angle, ratio = 0;
    switch (type) {
    case +1:	// min_height / max_edge [ 0 => 0.8165 ]
      numerator   = getTetrahedronAltitude(v0, v1, v2, v3, false);
      denominator = getTetrahedronEdgeLength(v0, v1, v2, v3, true);
      ratio = (denominator==0 ? 0 : (numerator/denominator));
      break;
    case +2:	// min_height / max_edge + 1/4 * max_facenormal_angle
      numerator   = getTetrahedronAltitude(v0, v1, v2, v3, false);
      denominator = getTetrahedronEdgeLength(v0, v1, v2, v3, true);
      max_angle   = getTetrahedronAngle(v0, v1, v2, v3, true);
      if (denominator == 0) return 0;
      ratio = (numerator / denominator) + 0.25 * cos(max_angle);
      break;
    case -1:	// circumradius / min_edge [ 0.61237 <= $$ ]
      if (!getTetrahedronCircumsphere(v0, v1, v2, v3, tmp, &numerator)) return 0;
      denominator = getTetrahedronEdgeLength(v0, v1, v2, v3, false);
      ratio = (denominator==0 ? 0 : (numerator/denominator));
      break;
    case -2:	// circumradius / inradius  [ 3 <= $$ ]
      if (!getTetrahedronCircumsphere(v0, v1, v2, v3, tmp, &numerator)) return 0;
      if (!getTetrahedronInsphere(v0, v1, v2, v3, tmp, &denominator)) return 0;
      ratio = (denominator==0 ? 0 : (numerator/denominator));
      break;
    default:
      cerr << "Warning: Invalid type for getTetrahedronAspectRatio()" << endl;
      ratio = 0;
    }
    return ratio;
  }

};

#endif  // CGEOMETRY3D_HPP
