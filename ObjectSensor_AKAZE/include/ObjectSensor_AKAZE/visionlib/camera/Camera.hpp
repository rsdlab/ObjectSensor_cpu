//
// Camera class template
//
// Jaeil Choi (4orward@gmail.com)
// last modified in Oct, 2008
//
// This class is designed for
//   - managing intrinsic and extrinsic parameters
//   - managing camera trajectory (for Structure From Motion)
//   - reading camera parameters from a configuration file
//   - coordinate conversion among world/camera/image coordinates
//   - inverse of projection (a point on the image -> a ray)
// It is supposed to be initiated as Camera<float> or Camera<double>.
// More explanation of each parameter can be found in
//  'Camera Calibration Toolbox for Matlab' by Jean-Yves Bouguet, Intel
//   http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
// For more information on invertible lens distortion model, refer to
//  'Non-metric calibration of wide-angle lenses and polycameras', by R. Swaminathan and S.K. Nayar, PAMI, 2000.
//
// Standard Camera Model ---------------------------------------------
//   z * Xn = Xc   (Xn = [ xc/zc ; yc/zc ])	// normalized(pinhole) image projection
//   Xd = (1 + O(r^6)) Xn + td			// lens distortion (radial + tangential)
//   Xd = (1+dt[0]r^2+dt[1]r^4+dt[4]r^6) Xn + [ 2 dt[2] x y + dt[3](r^2 + 2x^2) ]
//                                            [ dt[2](r^2 + 2y^2) + 2 dt[3] x y ]
//   Xp = [ fc[0] alpha*fc[1] ] Xd + [ cc[0] ]	// pixel coordinates (upper-left is (0,0))
//        [   0      fc[1]    ]      [ cc[1] ]
//   Reduced camera model
//     alpha = dt[2] = dt[3] = dt[4] = 0	// default (recommended)
// Invertible lens distortion model ----------------------------------
//   After the linear (pinhole) model:
//     Xp = [ fc[0] alpha*fc[1] cc[0] ]	[ R T ] [ Xw ]
//          [   0      fc[1]    cc[1] ]         [  1 ]
//          [   0        0        1   ]
//     resulting in  u = xp / zp  and  v = yp / zp,
//   Let r^2 = (u-cc[0])^2 + (v-cc[1])^2, then
//      ud - cc[0] = (u - cc[0]) / sqrt(1 + 2*dt[0] * r^2)
//      vd - cc[1] = (v - cc[1]) / sqrt(1 + 2*dt[0] * r^2)
//   In inverse, with rd^2 = (ud-cc[0])^2 + (vd-cc[1])^2,
//      u - cc[0] = (ud - cc[0]) / sqrt(1 - 2*dt[0] * rd^2)
//      v - cc[1] = (vd - cc[1]) / sqrt(1 - 2*dt[0] * rd^2)
// -------------------------------------------------------------------


#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <iostream>
#include "vm_macros.h"

#ifdef USE_GUIH_CONFIG
#include "guih_config.hpp"
#endif
#ifdef USE_MTH_ROTATION
#include "mth_rotation.hpp"
#endif
#ifdef USE_MTX_MATRIX
#include "mtx_matrix.hpp"
#endif
#ifdef USE_MTX_MATRIX_SOLVER
#include "mtx_matrix.hpp"
#include "mtx_matrix_solver.hpp"
#endif

template<class T_t>
class Camera
{
 public:
  // intrinsic parameters
  int  wh[2];    // image resolution (width and height)
  T_t  fc[2];    // focal length in pixels (f * s)
  T_t  cc[2];    // principal point coordinates
  T_t  afc;      // skew coefficient (alpha * fc[1])
  T_t  dt[5];    // distortion coefficients
  char dt_type;	 // distortion model ( 'S'tandard / 'I'nvertible )

  // extrinsic parameters (Xc = Rcw * Xw + Tcw)
  T_t  *Rcw;     // world-to-camera rotation (at the current frame)
                 //   equivalently, world  axes in camera frame (3 col vectors)
                 //   equivalently, camera axes in world  frame (3 row vectors)
  T_t  *Tcw;     // world origin in camera coordinates (at the current frame)

  // trajectory of camera (sequence of camera poses)
  T_t  *motion;  // sequence of extrinsic parameters (R[9]+T[3])
  int  nframes;  // number of frames ( nframes >= 1 )
  int  cfidx;    // current frame index

  // projection matrix (To update this matrix, use 'updatePmatUsingParam()')
  T_t Pmat[12];	// 3 x 4 projection matrix (linear/pinhole camera model )

 private:
  char  info[256];

 public:
  Camera() : motion(NULL) { clear(false); createMotionFrames(1); };
  ~Camera() { clear(true); };
  void clear(bool bFree=true) {
    memset(dt, 0, sizeof(dt));	// clear intrinsic parameters
    memset(fc, 0, sizeof(fc));
    memset(cc, 0, sizeof(cc));
    afc = 0;
    Rcw = NULL;			// clear extrinsic parameters
    Tcw = NULL;
    if (bFree && motion) free(motion);
    motion = NULL;
    nframes = cfidx = 0;
  }

  // setting intrinsic parameters ------------------------------------
 public:
  void setIntrinsic(int imgw, int imgh, T_t fx, T_t fy, T_t afc, T_t cx, T_t cy,
		    T_t k0=0, T_t k1=0, T_t k2=0, T_t k3=0, T_t k4=0) {
    wh[0] = imgw; wh[1] = imgh;
    fc[0] = fx;   fc[1] = fy;
    this->afc = afc;
    cc[0] = cx;   cc[1] = cy;
    setDistortion( k0, k1, k2, k3, k4 );
  }
  void setIntrinsic(int imgw, int imgh, T_t fc[2], T_t afc, T_t cc[2], T_t dt[5]) {
    wh[0] = imgw; wh[1] = imgh;
    memcpy(this->fc, fc, 2 * sizeof(T_t));
    memcpy(this->cc, cc, 2 * sizeof(T_t));
    this->afc = afc;
    setDistortion( dt[0], dt[1], dt[2], dt[3], dt[4] );
  }
  void setDistortion(T_t dt0, T_t dt1=0, T_t dt2=0, T_t dt3=0, T_t dt4=0) {
    G5V_SET( this->dt, dt0, dt1, dt2, dt3, dt4 );
    dt_type = (dt0==0 ? ' ' : ((dt0<0.0001 && dt1==0 && dt2==0 && dt3==0 && dt4==0) ? 'I' : 'S'));
  }
#ifdef USE_MTX_MATRIX_SOLVER
  void updateParamUsingPmat(void) {
    // Pmat = s * [ fx a*fy cx ] [ R00 R01 R02 T0 ]
    //            [  0  fy  cy ] [ R10 R11 R12 T1 ]
    //            [  0   0   1 ] [ R20 R21 R22 T2 ]
    T_t P[12], K[9], b[3];
    memcpy(P, Pmat, 12*sizeof(T_t));
    // normalize the projection matrix to get normalized pixel points
    // Note that Pmat(2,:) = [ R20 R21 R22 T2 ].
    T_t len = G3V_LENGTH(Pmat+8);
    if (len > 0) for (int i = 0; i < 12; i++) P[i] /= len;
    // decompose the matrix by RQ decomposition
    MTX::MatrixSolver<T_t> ms;
    ms.RQ_3x4( P, K, Rcw );
    // calculate the translation vector
    // Pmat(:,3) = K * Tcw,  since Pmat = K * [ Rcw Tcw ]
    b[0] = P[3];  b[1] = P[7];  b[2] = P[11];
    ms.solveByGaussJordan(3, K, Tcw, b);
    // save intrisic parameters
    fc[0] = K[0];  fc[1] = K[4];
    afc   = K[1];
    cc[0] = K[2];  cc[1] = K[5];
  }
#endif
  void updatePmatUsingParam(void) {
    // Pmat = [ fx afc cx ] [ R0 R1 R2 T0 ]
    //        [  0  fy cy ] [ R3 R4 R5 T1 ]
    //        [  0   0  1 ] [ R6 R7 R8 T2 ]
    Pmat[0] = fc[0] * Rcw[0] + afc * Rcw[3] + cc[0] * Rcw[6];
    Pmat[1] = fc[0] * Rcw[1] + afc * Rcw[4] + cc[0] * Rcw[7];
    Pmat[2] = fc[0] * Rcw[2] + afc * Rcw[5] + cc[0] * Rcw[8];
    Pmat[3] = fc[0] * Tcw[0] + afc * Tcw[1] + cc[0] * Tcw[2];
    Pmat[4] =                fc[1] * Rcw[3] + cc[1] * Rcw[6];
    Pmat[5] =                fc[1] * Rcw[4] + cc[1] * Rcw[7];
    Pmat[6] =                fc[1] * Rcw[5] + cc[1] * Rcw[8];
    Pmat[7] =                fc[1] * Tcw[1] + cc[1] * Tcw[2];
    Pmat[8] =                                         Rcw[6];
    Pmat[9] =                                         Rcw[7];
    Pmat[10]=                                         Rcw[8];
    Pmat[11]=                                         Tcw[2];
  }

  // setting extrinsic parameters (current frame) --------------------
 public:
  void setExtrinsic(T_t fx, T_t fy, T_t fz,
		    T_t ax, T_t ay, T_t az,
		    T_t ux, T_t uy, T_t uz) {
    T_t x[3], y[3], z[3];
    G3V_SET( z, ax - fx, ay - fy, az - fz );	// z axis of local coordiante system
    G3V_NORMALIZE( z );
    G3V_SET( y, -ux, -uy, -uz );		// y axis of local coordiante system
    G3V_SCALED_ADD( y, y, -G3V_DOT(y,z), z );
    G3V_NORMALIZE( y );
    G3V_CROSS( x, y, z );			// x axis of local coordiante system
    G3V_NORMALIZE( x );
    // world-to-camera rotation matrix
    G3M_SET( Rcw,
	     x[0], x[1], x[2],
	     y[0], y[1], y[2],
	     z[0], z[1], z[2] );
    // world origin in camera coordinates
    // R * [fx; fy; fz] + T = [0; 0; 0]
    // T = - R * [fx; fy; fz]
    G3V_SET( Tcw,
	     -(Rcw[0] * fx + Rcw[1] * fy + Rcw[2] * fz),
	     -(Rcw[3] * fx + Rcw[4] * fy + Rcw[5] * fz),
	     -(Rcw[6] * fx + Rcw[7] * fy + Rcw[8] * fz) );
  }
  void setExtrinsic(T_t r00, T_t r01, T_t r02,
		    T_t r10, T_t r11, T_t r12,
		    T_t r20, T_t r21, T_t r22,
		    T_t t0,  T_t t1,  T_t t2) {
    // world-to-camera rotation matrix
    G3M_SET( Rcw, r00, r01, r02, r10, r11, r12, r20, r21, r22 );
    // world origin in camera coordinates
    G3V_SET( Tcw, t0, t1, t2 );
  }
  void setExtrinsic(T_t Rcw[9], T_t Tcw[3]) {
    memcpy( this->Rcw, Rcw, 9 * sizeof(T_t) );
    memcpy( this->Tcw, Tcw, 3 * sizeof(T_t) );
  }
  void setExtrinsicWithCameraPose(T_t Rwc[3*3], T_t Twc[3]) {
    G3M_TRANS ( this->Rcw, Rwc );
    G3M_MUL_MV( this->Tcw, this->Rcw, Twc );
    G3V_MUL_VALUE( this->Tcw, -1 );	// Tcw = - (Rwc)T * Twc
  }

  // projection ------------------------------------------------------
 public:
  void World2Camera(T_t  w_xyz[3], T_t  c_xyz[3]) {
    c_xyz[0] = Rcw[0] * w_xyz[0] + Rcw[1] * w_xyz[1] + Rcw[2] * w_xyz[2]  + Tcw[0];
    c_xyz[1] = Rcw[3] * w_xyz[0] + Rcw[4] * w_xyz[1] + Rcw[5] * w_xyz[2]  + Tcw[1];
    c_xyz[2] = Rcw[6] * w_xyz[0] + Rcw[7] * w_xyz[1] + Rcw[8] * w_xyz[2]  + Tcw[2];
  }
  void Camera2Pixel(T_t  c_xyz[3], T_t  uv[2], bool distort=true) {
    T_t  x = c_xyz[0], y = c_xyz[1], z = c_xyz[2], rd, tdx, tdy, rr, temp;
    if (z != 0) {  x /= z;   y /= z;  }
    if (distort && dt_type == 'S') {   /* standard distortion model */
      rr = x*x + y*y;
      if (dt[4] == 0) rd = 1 + dt[0] * rr + dt[1] * rr * rr;
      else            rd = 1 + dt[0] * rr + dt[1] * rr * rr + dt[4] * rr * rr * rr;
      if (dt[2] == 0) tdx = tdy = 0;
      else {
        tdx =  2 * x * y * dt[2] + (rr+2*x*x) * dt[3];
        tdy = (rr+2*y*y) * dt[2] +  2 * x * y * dt[3];
      }
      x = rd * x + tdx;
      y = rd * y + tdy;
    }
    uv[0] = fc[0] * x +   afc * y + cc[0];
    uv[1] =             fc[1] * y + cc[1];
    if (distort && dt_type == 'I') {   /* invertible distortion model */
      tdx = uv[0] - cc[0];  tdy = uv[1] - cc[1];
      rr  = tdx * tdx + tdy * tdy;
      temp = 1 + 2 * dt[0] * rr;
      temp = sqrt( temp < 0.0001 ? 0.0001 : temp );
      uv[0] = tdx / temp + cc[0];
      uv[1] = tdy / temp + cc[1];
    }
  }
  void World2Pixel (T_t  w_xyz[3], T_t  uv[2], bool distort=true) {
    T_t c_xyz[3];
    World2Camera( w_xyz, c_xyz );
    Camera2Pixel( c_xyz, uv, distort );
  }
  void World2Pixel (T_t  w_xyz[3], int  uv[2], bool distort=true) {
    T_t c_xyz[3], uv2[2];
    World2Camera( w_xyz, c_xyz );
    Camera2Pixel( c_xyz, uv2, distort );
    G2V_SET( uv, (int)uv2[0], (int)(uv2[1]) );
  }

  // inverse of projection -------------------------------------------
public:
  void Pixel2Camera(T_t uv[2], T_t Xc[3], bool undistort=true) {
    // Xp = [ fc[0]   afc  ] Xd + [ cc[0] ]	// pixel coordinates (upper-left is (0,0))
    //      [   0    fc[1] ]      [ cc[1] ]
    // Xd = [ (Xp[0] - cc[0] - afc*Xd[1]) / fc[0] ]
    //      [ (Xp[1] - cc[1]            ) / fc[1] ]
    if (fc[0]==0) { fprintf(stderr,"Error: Camera intrinsic parameters not set\n"); return; }
    if (dt_type == 'I') {
      // invertible distortion model with single parameter
      // In inverse, with rd^2 = (ud-u0)^2 + (vd-v0)^2,
      //    u - u0 = (ud - u0) / sqrt(1 - 2*k * rd^2)
      //    v - v0 = (vd - v0) / sqrt(1 - 2*k * rd^2)
      T_t tdx, tdy, po[2], rr, temp;
      if (undistort) {
	tdx = uv[0] - cc[0];  tdy = uv[1] - cc[1];
	rr  = tdx * tdx + tdy * tdy;
	temp = 1 - 2 * dt[0] * rr;
	temp = sqrt( temp < 0.0001 ? 0.0001 : temp );
	po[0] = tdx / temp + cc[0];
	po[1] = tdy / temp + cc[1];
      } else {
	G2V_COPY( po, uv );
      }
      // convert into camera space
      Xc[1] = (po[1] - cc[1]) / fc[1];
      Xc[0] = (po[0] - cc[0] - afc*Xc[1]) / fc[0];
      Xc[2] = 1.0;
    } else {
      T_t rr;
      // convert into camera space
      Xc[1] = (uv[1] - cc[1]) / fc[1];
      Xc[0] = (uv[0] - cc[0] - afc*Xc[1]) / fc[0];
      if (undistort && dt[0] != 0 && dt_type == 'S') {
	// standard distortion model with up to 5 coefficients
	rr = (Xc[0] * Xc[0] + Xc[1] * Xc[1]);
	Xc[0] /= (1 + dt[0] * rr + dt[1] * rr * rr);
	Xc[1] /= (1 + dt[0] * rr + dt[1] * rr * rr);
	// This is not the exact inverse of the distortion, since r is
	// calculated from distorted coordinates, not the originals.
      }
      Xc[2] = 1.0;
    }
  }
  void Pixel2WorldRay(T_t uv[2], T_t dir[3], T_t from[3], bool undistort=true) {
    // Unproject the point in the image to get
    //   the half-line of the ray in world coordinate system.
    T_t Xc[3]={0,0,0}, temp[3], Xw[3];
    Pixel2Camera( uv, Xc, undistort );
    //printf("Xc  = %5.2f %5.2f %5.2f\n", Xc[0], Xc[1], Xc[2]);
    // get 3D coordinate (Xc = Rcw * Xw + Tcw)
    G3V_SUB( temp, Xc, Tcw );
    Xw[0] = Rcw[0] * temp[0] + Rcw[3] * temp[1] + Rcw[6] * temp[2];
    Xw[1] = Rcw[1] * temp[0] + Rcw[4] * temp[1] + Rcw[7] * temp[2];
    Xw[2] = Rcw[2] * temp[0] + Rcw[5] * temp[1] + Rcw[8] * temp[2];
    // calculate camera position
    getCameraPosition( from );
    // calculate direction
    G3V_SUB( dir, Xw, from );
    G3V_NORMALIZE( dir );
    //printf("dir = %5.2f %5.2f %5.2f\n", dir[0], dir[1], dir[2]);
  }
  void Pixel2WorldRay(int uv[2], T_t dir[3], T_t from[3]) {
    T_t uvf[2] = { uv[0], uv[1] };
    Pixel2WorldRay( uvf, dir, from );
  }

  // camera position -------------------------------------------------
 public:
  void getCameraPosition(T_t c[3], int fidx=-1) {
    // Rcw * Xw + Tcw = [0; 0; 0], therefore Xw = - Rt * Tcw.
    T_t *R, *T;
    if (fidx < 0) { R = Rcw;         T = Tcw;        }  // use current frame
    else          { R = getR(fidx);  T = getT(fidx); }
    c[0] = -( R[0] * T[0] + R[3] * T[1] + R[6] * T[2] );
    c[1] = -( R[1] * T[0] + R[4] * T[1] + R[7] * T[2] );
    c[2] = -( R[2] * T[0] + R[5] * T[1] + R[8] * T[2] );
  }

  // distorting/undistorting -----------------------------------------
 public:
  void distortUV(T_t  undistorted_uv[2], T_t  distorted_uv[2]) {
    // note that undistorted_uv[2] and distorted_uv[2] can be same.
    if (dt_type == 'I') {
      T_t u  = undistorted_uv[0] - cc[0];
      T_t v  = undistorted_uv[1] - cc[1];
      T_t rr = u*u + v*v;
      distorted_uv[0] = (T_t)( u / sqrt(1 + 2 * dt[0] * rr) ) + cc[0];
      distorted_uv[1] = (T_t)( v / sqrt(1 + 2 * dt[0] * rr) ) + cc[1];
    } else  std::cerr << "Error (Camera::distortUV): invalid camera model type" << std::endl;
  }
  void undistortUV(T_t  distorted_uv[2], T_t  undistorted_uv[2]) {
    // note that undistorted_uv[2] and distorted_uv[2] can be same.
    if (dt_type == 'I') {
      T_t u  = distorted_uv[0] - cc[0];
      T_t v  = distorted_uv[1] - cc[1];
      T_t rr = u*u + v*v;
      undistorted_uv[0] = (T_t)( u / sqrt(1 - 2 * dt[0] * rr) ) + cc[0];
      undistorted_uv[1] = (T_t)( v / sqrt(1 - 2 * dt[0] * rr) ) + cc[1];
	} else{
		double _fx, _fy, y, y2, dty, dt1y, _2dt2y, _3dt2y2,dt3y2;
		double x, x2, dtx, d;

		_fx     = 1.0/fc[0];
		_fy = 1.0/fc[1];
		y       = ( distorted_uv[1] - cc[1])*_fy;
		y2      = y*y;
		dty     = 1 + (dt[0] + dt[1]*y2)*y2;
		dt1y    = 2*dt[1]*y2;
		_2dt2y  = 2*dt[2]*y;
		_3dt2y2 = 3*dt[2]*y2;
		dt3y2   = dt[3]*y2;

		x       = ( undistorted_uv[0] - cc[0])*_fx;
		x2      = x*x;
		dtx     = (dt[0] + dt[1]*x2)*x2;
		d       = dtx + dty + dt1y*x2;

		undistorted_uv[0] = fc[0]*(x*(d + _2dt2y) + dt3y2 + (3*dt[3])*x2) + cc[0];
		undistorted_uv[1] = fc[1]*(y*(d + (2*dt[3])*x) + _3dt2y2 + dt[2]*x2) + cc[1];
	}
  }

  // camera trajectory (seq of poses) --------------------------------
 public:
  void createMotionFrames(int nframes) {
    if (nframes < 1) nframes = 1;
    if (nframes != this->nframes) {
      if (motion) free(motion);
      this->nframes = nframes;
      motion = (T_t*) malloc( nframes * (9+3) * sizeof(T_t) );
    }
    for (int fidx = 0; fidx < nframes; fidx++) {
      setR( fidx,  1, 0, 0,  0, 1, 0,  0, 0, 1 );
      setT( fidx,  0, 0, 0 );
    }
    setCurrentFrame(0);
  }

  void   setCurrentFrame(int fidx) {
    if (fidx < 0 || fidx >= nframes) return;
    this->cfidx = fidx;
    this->Rcw = getR(fidx);
    this->Tcw = getT(fidx);
  }
  inline T_t* getR(int fidx=-1) { return motion + (fidx<0 ? cfidx : fidx)*12; }
  inline T_t* getT(int fidx=-1) { return motion + (fidx<0 ? cfidx : fidx)*12 + 9; }
  void   setR(int fidx, T_t R00, T_t R01, T_t R02, T_t R10, T_t R11, T_t R12, T_t R20, T_t R21, T_t R22) {
    T_t *R = getR(fidx);
    G3M_SET( R, R00, R01, R02, R10, R11, R12, R20, R21, R22 );
  }
  void   setT(int fidx, T_t T0, T_t T1, T_t T2 ) {
    T_t *T = getT(fidx);
    G3V_SET( T, T0, T1, T2 );
  }

  // -----------------------------------------------------------------
 public:
  void  printInfo(char *cmmt=NULL, bool show_ext=false) {
    T_t c[3], v[3];
    printf("Camera information %s%s\n", (cmmt ? " : " : ""), (cmmt ? cmmt : ""));
    getCameraPosition(c);
    printf("  located at  Twc = [ %9.4f %9.4f %9.4f ]T  (%d x %d)\n", c[0], c[1], c[2], wh[0], wh[1]);
    printf("  Intrinsic parameters [K]    Extrinsic parameters [Rcw Tcw] (at frame %d)\n", cfidx);
    printf("  [ %7.2f %7.2f %7.2f ] [ %9.4f %9.4f %9.4f %9.4f ]\n",
	   fc[0], afc, cc[0],  Rcw[0], Rcw[1], Rcw[2], Tcw[0]);
    printf("  [ %7.2f %7.2f %7.2f ] [ %9.4f %9.4f %9.4f %9.4f ]\n",
	   0.0,         fc[1], cc[1],  Rcw[3], Rcw[4], Rcw[5], Tcw[1]);
    printf("  [ %7.2f %7.2f %7.2f ] [ %9.4f %9.4f %9.4f %9.4f ]\n",
	   0.0,           0.0,   1.0,  Rcw[6], Rcw[7], Rcw[8], Tcw[2]);
    if (dt_type == 'I') {	// invertible distortion model (as in Andrew Davison's vSLAM)
      printf("  Distortion model : Invertible  rd = r / sqrt(1 + 2*k*r^2) \n");
      printf("    [ %g ]  1/sqrt(1+2*k*300^2) = %.2f\n", dt[0], 1/sqrt(1+2*dt[0]*90000));
    } else {		// standard distortion model (as in Matlab toolbox)
      printf("  Distortion model : Standard  \n");
      printf("    [ %9.4f %9.4f (%9.4f %9.4f) %9.4f ]\n", dt[0], dt[1], dt[2], dt[3], dt[4]);
    }
    if (fc[0] != 0 && fc[1] != 0) {
      // check Pmat[], the linear projection matrix for the pinhole camera model
      T_t d00 = fabs(fc[0] * Rcw[0] + afc * Rcw[3] + cc[0] * Rcw[6] - Pmat[0]);
      T_t d01 = fabs(fc[0] * Rcw[1] + afc * Rcw[4] + cc[0] * Rcw[7] - Pmat[1]);
      T_t d02 = fabs(fc[0] * Rcw[2] + afc * Rcw[5] + cc[0] * Rcw[8] - Pmat[2]);
      T_t d20 = fabs(Rcw[6] - Pmat[8]);
      T_t d21 = fabs(Rcw[7] - Pmat[9]);
      T_t d22 = fabs(Rcw[8] - Pmat[10]);
      T_t d23 = fabs(Tcw[2] - Pmat[11]);
      bool warn = (d00 > 0.0001 || d01 > 0.0001 || d02 > 0.0001 ||
		   d20 > 0.0001 || d21 > 0.0001 || d22 > 0.0001 || d23 > 0.0001);
      if (warn) printf("    Warning: Pmat does not agree with the parameters. \n");
//       printf("  Pmat : The projection matrix for the pinhole model \n");
//       printf("    [ %12.6f %12.6f %12.6f %12.6f ]\n", Pmat[0], Pmat[1], Pmat[2], Pmat[3]);
//       printf("    [ %12.6f %12.6f %12.6f %12.6f ]\n", Pmat[4], Pmat[5], Pmat[6], Pmat[7]);
//       printf("    [ %12.6f %12.6f %12.6f %12.6f ]\n", Pmat[8], Pmat[9], Pmat[10], Pmat[11]);
    }
    // check each frame in the camera motion
    if        (nframes > 1 && !show_ext) {
      printf("  Camera motion [Tcw] : %d / total %d frames\n", cfidx, nframes);
      for (int fidx = 0; fidx < nframes; fidx++) {
	T_t *T = getT(fidx), Twc[3];  getCameraPosition( Twc, fidx );
	printf("    frame %02d  Tcw=(%6.2f %6.2f %6.2f)  Twc=(%6.2f %6.2f %6.2f) \n",
	       fidx, T[0], T[1], T[2], Twc[0], Twc[1], Twc[2]);
      }
    } else if (nframes > 1 && show_ext) {
      printf("  Camera motion [Rcw Tcw] : %d / total %d frames\n", cfidx, nframes);
      for (int fidx = 0; fidx < nframes; fidx++) {
	T_t *R = getR(fidx);
	T_t *T = getT(fidx);
	for (int i = 0; i < 3; i++) {
	  G3V_SET( v, R[ 3*i + 0], R[ 3*i + 1], R[ 3*i + 2] );
	  if (fabs(G3V_LENGTH(v) - 1.0) > 0.0001) {
	    printf("    frame %d : invalid rotation\n", fidx);
	    break;
	  }
	}
	printf("    frame %d \n", fidx);
	printf("    [ %12.7f %12.7f %12.7f %12.7f ]\n", R[0], R[1], R[2], T[0]);
	printf("    [ %12.7f %12.7f %12.7f %12.7f ]\n", R[3], R[4], R[5], T[1]);
	printf("    [ %12.7f %12.7f %12.7f %12.7f ]\n", R[6], R[7], R[8], T[2]);
      }
    }
  }
#ifdef USE_MTX_MATRIX
  T_t evaluateReprojectionError(MTX::Matrix<T_t> &mm, MTX::Matrix<T_t> &rm, bool rm_in_3d) {
    // calculate reprojection error from measurement and reprojection matrix
    if (mm.nCol != rm.nCol) return 0;
    T_t error = 0, dx, dy;
    if (rm_in_3d) {	// 'rm' as 3D world coordinates before the reprojection
      T_t xyz[3], p[2];
      for (int col = 0; col < mm.nCol; col++) {
	G3V_SET( xyz, rm(0, col), rm(1, col), rm(2, col) );
	World2Pixel( xyz, p, true );
	dx = mm(0, col) - p[0];
	dy = mm(1, col) - p[1];
	error += (dx * dx + dy * dy);
      }
    } else {		// 'rm' as 2D image coordinates after the reprojection
      for (int col = 0; col < mm.nCol; col++) {
	dx = mm(0, col) - rm(0, col);
	dy = mm(1, col) - rm(1, col);
	error += (dx * dx + dy * dy);
      }
    }
    return error;
  }
#endif

#ifdef USE_GUIH_CONFIG
  bool readConfigFile(char *filename, char *section, bool read_int=true, bool read_ext=true, bool verbose=true) {
    char  res[80], in0[200], in1[200], ex0[200], ex1[200], ex2[200], dst[200];
    GUIH::Config cfg;
    cfg.set( section, "CameraResolution", CFG_STRING, res );
    cfg.set( section, "CameraIntrinsic0", CFG_STRING, in0 );
    cfg.set( section, "CameraIntrinsic1", CFG_STRING, in1 );
    cfg.set( section, "CameraExtrinsic0", CFG_STRING, ex0 );
    cfg.set( section, "CameraExtrinsic1", CFG_STRING, ex1 );
    cfg.set( section, "CameraExtrinsic2", CFG_STRING, ex2 );
    cfg.set( section, "CameraDistortion", CFG_STRING, dst );
    if (!cfg.process( filename, section )) {
      if (verbose) printf("Error (Camera::readConfigFile): cannot read camera parameters\n");
      return false;
    }
    double dummy, f[2], c[2], a, R[9], T[3], d[5]={0,0,0,0,0};
    if (read_int) {
      if (!cfg.wasProcessed(section, "CameraIntrinsic0")) {
	if (verbose) printf("Error (Camera::readConfigFile): cannot read intrinsic camera parameters\n");
	return false;
      }
      sscanf( res, "%d x %d", wh+0, wh+1 );
      sscanf( in0, "%lf %lf %lf", f+0,   &a, c+0 );
      sscanf( in1, "%lf %lf %lf", &dummy, f+1,   c+1 );
      int n = sscanf( dst, "%lf %lf %lf %lf %lf", d+0, d+1, d+2, d+3, d+4);
      dt_type = (n > 1 ? 'S' : 'I');
      afc = (T_t)a;
      G2V_SET( fc,  (T_t)f[0], (T_t)f[1] );
      G2V_SET( cc,  (T_t)c[0], (T_t)c[1] );
      G5V_SET( dt,  (T_t)d[0], (T_t)d[1], (T_t)d[2], (T_t)d[3], (T_t)d[4] );
    }
    if (read_ext) {
      if (!cfg.wasProcessed(section, "CameraExtrinsic0")) {
	if (verbose) printf("Error (Camera::readConfigFile): cannot read extrinsic camera parameters\n");
	return false;
      }
      sscanf( ex0, "%lf %lf %lf %lf", R+0, R+1, R+2, T+0 );
      sscanf( ex1, "%lf %lf %lf %lf", R+3, R+4, R+5, T+1 );
      sscanf( ex2, "%lf %lf %lf %lf", R+6, R+7, R+8, T+2 );
      G3M_SET( Rcw, (T_t)R[0], (T_t)R[1], (T_t)R[2], (T_t)R[3], (T_t)R[4], (T_t)R[5], (T_t)R[6], (T_t)R[7], (T_t)R[8] );
      G3V_SET( Tcw, (T_t)T[0], (T_t)T[1], (T_t)T[2] );
    }
    updatePmatUsingParam();
    return true;
  }
#endif
};

#endif // CAMERA_HPP
