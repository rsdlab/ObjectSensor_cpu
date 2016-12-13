//
// CameraStereo class template
//
// Jaeil Choi
// last modified in Oct, 2008
//

#ifndef CAMERA_STEREO_HPP
#define CAMERA_STEREO_HPP

#include <iostream>
#include "vm_macros.h"
#include "geometry3d.hpp"
#include "Camera.hpp"


template <class T_t>
class CameraStereo
{
public:
  Camera<T_t>	*camL;		// left  camera
  Camera<T_t>	*camR;		// right camera
private:
public:
  T_t		baseline;       //
  Camera<T_t>	camL_dummy;	// left  camera dummy place holder
  Camera<T_t>	camR_dummy;	// right camera dummy place holder
  T_t		Rqc[9], Tqc[3];	// xform from camera frame to left  cam
  T_t		Rpc[9], Tpc[3];	// xform from camera frame to right cam

public:
  CameraStereo(T_t baseline=0.12) : camL(&camL_dummy), camR(&camR_dummy) { setStereoPoseBaseline(baseline); }
  CameraStereo(Camera<T_t> *left, Camera<T_t> *right, T_t baseline=0) {
    setCamera(left, right);
    if (baseline > 0) setStereoPoseBaseline(baseline);
  }
  ~CameraStereo() {}
  void setCamera(Camera<T_t> *left, Camera<T_t> *right) {
    camL = (left  ? left  : &camL_dummy);
    camR = (right ? right : &camR_dummy);
  }

public:
  // -----------------------------------------------------------------
  // set the intrinsic parameters of the left/right cameras
  // -----------------------------------------------------------------
  void setIntrinsic(int imgw, int imgh,
		    T_t fxL, T_t fyL, T_t afL, T_t cxL, T_t cyL, T_t dtL,
		    T_t fxR, T_t fyR, T_t afR, T_t cxR, T_t cyR, T_t dtR) {
    camL->setIntrinsic( imgw, imgh, fxL, fyL, afL, cxL, cyL, dtL );
    camR->setIntrinsic( imgw, imgh, fxR, fyR, afR, cxR, cyR, dtR );
  }

  // -----------------------------------------------------------------
  // set relative pose of left/right camera
  // -----------------------------------------------------------------
  void setStereoPoseBaseline(T_t baseline) {
    this->baseline = baseline;
    G3M_SET_ID( Rqc );  G3V_SET( Tqc, +baseline/2, 0, 0 );  // camera -> left
    G3M_SET_ID( Rpc );  G3V_SET( Tpc, -baseline/2, 0, 0 );  // camera -> right
    camL->setExtrinsic( Rqc, Tqc );
    camR->setExtrinsic( Rpc, Tpc );
  }
  // -----------------------------------------------------------------
  // set the extrinsic pose of the left/right cameras
  // -----------------------------------------------------------------
  void setExtrinsic(T_t Rcw[9], T_t Tcw[3]) {
    G3M_XFORM_MERGE( camL->Rcw, camL->Tcw,  Rqc, Tqc,  Rcw, Tcw );
    G3M_XFORM_MERGE( camR->Rcw, camR->Tcw,  Rpc, Tpc,  Rcw, Tcw );
  }
  void setExtrinsicWithCameraPose(T_t Rwc[9], T_t Twc[3]) {
    T_t Rcw[9], Tcw[3];
    G3M_XFORM_INVERSE( Rcw, Tcw, Rwc, Twc );
    setExtrinsic( Rcw, Tcw );
  }

  bool triangulatePosition(T_t uvL[2], T_t uvR[2], T_t xyz[3]) {
    T_t xyzL[3], xyzR[3], dirL[3], dirR[3];
    camL->Pixel2WorldRay( uvL, dirL, xyzL );
    camR->Pixel2WorldRay( uvR, dirR, xyzR );
//     printf("RayL (%.2f %.2f %.2f)->(%.2f %.2f %.2f)  RayR (%.2f %.2f %.2f)->(%.2f %.2f %.2f)\n",
// 	   xyzL[0], xyzL[1], xyzL[2], dirL[0], dirL[1], dirL[2],
// 	   xyzR[0], xyzR[1], xyzR[2], dirR[0], dirR[1], dirR[2]);
    Geometry3D<double> geom;
    int mode = -1;   // -1: OnLeftLine,  +1: OnRightLine,  0: InBetween
    return geom.getPointByIntersectingTwoLines( xyzL, dirL, xyzR, dirR, xyz, mode );  //// new function
  }

  void printInfo(char *cmmt=NULL) {
    printf("CameraStereo information %s%s\n", (cmmt ? " : " : ""), (cmmt ? cmmt : ""));
    printf("  baseline = %.3f\n", baseline);
    camL->printInfo("Left");
    camR->printInfo("Right");
  }
};

#endif // CAMERA_STEREO_HPP
