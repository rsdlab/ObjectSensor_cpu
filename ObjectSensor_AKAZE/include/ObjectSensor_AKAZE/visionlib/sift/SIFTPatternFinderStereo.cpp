
//
// SIFTPatternFinderStereo: 
//   3D pose estimation using SIFT feature matching
//
// Jaeil Choi
// last modified in Feb, 2009
//

#include <iostream>
#include "guih_config.hpp"
#include "imgh_file_io.hpp"
#include "imgh_editor.hpp"
#include "mth_rotation.hpp"
#include "SIFTPatternFinderStereo.hpp"
using namespace std;

// ===================================================================
// Loading all the configuration information
// ===================================================================

bool SIFTPatternFinderStereo::loadCameraParameters(char *cfg_file)
{
  // [StereoCamera]
  // CameraStereoBaseline	   = 0.12 
  // CameraDefaultSIFTPattern      = crunky
  //
  // [StereoCameraLeft]
  // CameraResolution =  640 x 480
  // CameraIntrinsic0 =  594.128  0       319.988
  // CameraIntrinsic1 =  0        600.2   240.004
  // CameraDistortion =  9.14364e-07		# invertible distortion model
  //
  // [StereoCameraRight]
  // CameraResolution =  640 x 480
  // CameraIntrinsic0 =  590.434   0         320.028  
  // CameraIntrinsic1 =  0         599.779   240.011  
  // CameraDistortion =  8.73809e-07 		# invertible distortion model
  //
  GUIH::Config cfg;
  cfg.set( "StereoCamera", "CameraStereoBaseline", CFG_DOUBLE, &baseline );
  if (cfg.process( cfg_file, "StereoCamera", false, false)) {
    if (verbose) printf("[sift] loading baseline information .. done (%.3f)\n", baseline);
  } else {
    if (verbose) printf("[sift] loading baseline information .. FAILED\n");
    return false;
  }
  // set base line of the stereo camera
  G3M_SET( Rcq,  1, 0, 0,  0, 1, 0,  0, 0, 1 );  G3V_SET( Tcq, -baseline/2, 0, 0 );
  G3M_SET( Rcp,  1, 0, 0,  0, 1, 0,  0, 0, 1 );  G3V_SET( Tcp, +baseline/2, 0, 0 );
  // we also need to set 'stereo' (CameraStereo<>) for the triangulation
  stereo.setStereoPoseBaseline( baseline );
  
  pfL.loadCameraParameters( cfg_file, "StereoCameraLeft"  );
  pfR.loadCameraParameters( cfg_file, "StereoCameraRight" );
  pfL.verbose = pfR.verbose = false;
  // we also need to set 'stereo' (CameraStereo<>) for the triangulation
  Camera<double> *cL = &pfL.camera, *cR = &pfR.camera;
  stereo.setIntrinsic( cL->wh[0], cL->wh[1], 
		       cL->fc[0], cL->fc[1], cL->afc, cL->cc[0], cL->cc[1], cL->dt[0],
		       cR->fc[0], cR->fc[1], cR->afc, cR->cc[0], cR->cc[1], cR->dt[0] );
  
  if (verbose) printf("[sift] loaded  configuration information\n");
  return true;
}

bool SIFTPatternFinderStereo::loadReferencePattern(char *cfg_file, char *pattern)
{
  // [crunky]
  // RefImgFile   = images/siftref_crunky.png	# image of the reference pattern 
  // RefImgRegion  = 326:266:116:116			# image region of the reference pattern
  // RefImgRegion2 = 843:268:116:116			# image region of the reference pattern
  // CameraExtrinsic0 = 1.0   0.00   0.00   0.002	# pose of the reference pattern
  // CameraExtrinsic1 = 0.0   0.00  -1.00   0.132	# [Rco Tco] 
  // CameraExtrinsic2 = 0.0   1.00   0.00   0.555	# size w:0.106 h:0.106
  GUIH::Config cfg;
  char  pattern_dummy[520];
  pfL.verbose = pfR.verbose = verbose;
  
  if (!pattern || !pattern[0]) {
    char *cfg_section = "StereoCamera";
    cfg.set( cfg_section, "CameraDefaultSIFTPattern", CFG_STRING, pattern_dummy );
    if (! cfg.process( cfg_file, cfg_section, false, false )) {
      if (verbose) printf("[sift] loading ref. patterns .. FAILED ('CameraDefaultSIFTPattern' at %s in %s)\n", cfg_section, cfg_file);
      return false;
    }
    pattern = pattern_dummy;
  }
  // read the reference image
  pfL.loadReferencePattern( cfg_file, pattern, -1 );
  pfR.loadReferencePattern( cfg_file, pattern, +1 );
  if (verbose) printf("[sift] loaded images of the ref pattern '%s' for stereo\n", pattern);
  // read the pose of the reference pattern [ Rco Tco ]
  Camera<double> cam;
  if (! cam.readConfigFile( cfg_file, pattern, false, true, false )) {
    if (verbose) printf("NOT FOUND\n");
    return false;
  }
  // set the pattern pose for the left camera
  memcpy( pfL.Rcf, cam.Rcw, 9*sizeof(double) );
  memcpy( pfL.Tcf, cam.Tcw, 3*sizeof(double) );
  // adjust the pattern pose for stereo
  double Rcf[9], Tcf[3], Rcc[9]={1,0,0,0,1,0,0,0,1}, Tcc[3];
  G3V_SET( Tcc, +baseline/2, 0, 0 );	// adjust the pattern pose for left camera
  G3M_XFORM_MERGE( Rcf, Tcf, Rcc, Tcc, pfL.Rcf, pfL.Tcf );
  G3M_COPY( pfL.Rcf, Rcf );
  G3V_COPY( pfL.Tcf, Tcf );  // G3M_XFORM_PRINTF( pfL.Rcf, pfL.Tcf );
  // set the pattern pose for the right camera
  memcpy( pfR.Rcf, cam.Rcw, 9*sizeof(double) );
  memcpy( pfR.Tcf, cam.Tcw, 3*sizeof(double) );
  G3V_SET( Tcc, -baseline/2, 0, 0 );	// adjust the pattern pose for right camera
  G3M_XFORM_MERGE( Rcf, Tcf, Rcc, Tcc, pfR.Rcf, pfR.Tcf );
  G3M_COPY( pfR.Rcf, Rcf );
  G3V_COPY( pfR.Tcf, Tcf );  // G3M_XFORM_PRINTF( pfR.Rcf, pfR.Tcf );
  if (verbose) printf("[sift] loaded pose   of the ref pattern '%s' for stereo\n", pattern);
  
  pfL.verbose = pfR.verbose = false;
  return true;
}

// ===================================================================
// Setting the input (scene) image
// ===================================================================

bool SIFTPatternFinderStereo::setupInputImage(char cmode, char *fname)
{
  // Set the input image.
  IMGH::FileIO imgfile;
  IMGH::Image  img;
  if (! imgfile.readFile( fname, &img, IMGH::PIXEL_FLOAT )) return false;
  
  return setupInputImageWithFloatBuffer( cmode, img.w, img.h, (float*)img.data );
}

bool SIFTPatternFinderStereo::setupInputImage(char cmode, IMGH::Image *img )
{
  // Set the input image. (create a new copy)
  if (!img) return false;
  if      (cmode=='L') return pfL.setupInputImage( img );
  else if (cmode=='R') return pfR.setupInputImage( img );
  else {
    IMGH::Image tmp;
    if (img->type != IMGH::PIXEL_FLOAT) {
      IMGH::ImageConverter conv;
      conv.convertImage( img, &tmp, IMGH::PIXEL_FLOAT );
      img = &tmp;
    }
    pfL.scene.iimg.setImage( img->w/2, img->h, IMGH::PIXEL_FLOAT );
    pfR.scene.iimg.setImage( img->w/2, img->h, IMGH::PIXEL_FLOAT );
    pfL.scene.iimg.copySubregionFrom( 0, 0, img->w/2, img->h, img, 0,       0 );
    pfR.scene.iimg.copySubregionFrom( 0, 0, img->w/2, img->h, img, img->w/2, 0 );
    pfL.scene.xywh_given = false;  pfL.scene.clearFeatures();
    pfR.scene.xywh_given = false;  pfR.scene.clearFeatures();
    pfL.match.clear();
    pfR.match.clear();
    return true;
  }
}

bool SIFTPatternFinderStereo::setupInputImageWithFloatBuffer(char cmode, int w, int h, float *buf)
{
  // Set the input image. (using the original buffer; no copy)
  if      (cmode=='L') return pfL.setupInputImageWithFloatBuffer( w, h, buf );
  else if (cmode=='R') return pfR.setupInputImageWithFloatBuffer( w, h, buf );
  else {
    IMGH::Image tmp( w, h, IMGH::PIXEL_FLOAT, buf );
    pfL.scene.iimg.setImage( tmp.w/2, tmp.h, IMGH::PIXEL_FLOAT );
    pfR.scene.iimg.setImage( tmp.w/2, tmp.h, IMGH::PIXEL_FLOAT );
    pfL.scene.iimg.copySubregionFrom( 0, 0, tmp.w/2, tmp.h, &tmp, 0,       0 );
    pfR.scene.iimg.copySubregionFrom( 0, 0, tmp.w/2, tmp.h, &tmp, tmp.w/2, 0 );
    pfL.scene.xywh_given = false;  pfL.scene.clearFeatures();
    pfR.scene.xywh_given = false;  pfR.scene.clearFeatures();
    pfL.match.clear();
    pfR.match.clear();
    return true;
  }
}

// ===================================================================
// Finding specified pattern in the scene
// ===================================================================

bool SIFTPatternFinderStereo::findSIFTPattern(char cmode, double Rco[9], double Tco[3])
{
  // Find the SIFT pattern with the pattern ID 'pid' in the scene.
  foundL = foundR = false;
  if (cmode != 'R') foundL = pfL.findSIFTPattern();
  if (cmode != 'L') foundR = pfR.findSIFTPattern();
  // get the result
  if (Rco || Tco) getResult3DPose( cmode, Rco, Tco );
  return (foundL || foundR);
}

bool SIFTPatternFinderStereo::getResult3DPose(char cmode, double R[9], double T[3])
{
  // Return the 3D pose of the found pattern.
  // Make sure this function is called only after 'findPattern()'.
  double Rqo[9], Tqo[3], Rpo[9], Tpo[3];
  pfL.verbose = pfR.verbose = verbose;
  foundLPose = foundRPose = false;
  if (cmode != 'R' && foundL) foundLPose = pfL.getResult3DPose( Rqo, Tqo );
  if (cmode != 'L' && foundR) foundRPose = pfR.getResult3DPose( Rpo, Tpo );
//   G3M_XFORM_PRINTF( Rqo, Tqo );
//   G3M_XFORM_PRINTF( Rpo, Tpo );
  
  if (foundRPose && foundLPose) {
    MTH::Rotation<double> rotat;
    double RL[9], TL[3], RR[9], TR[3], RvL[3], RvR[3], Rv[3];
    G3M_XFORM_MERGE( RL, TL, Rcq, Tcq, Rqo, Tqo );
    G3M_XFORM_MERGE( RR, TR, Rcp, Tcp, Rpo, Tpo );
    rotat.R2Rv( RL, RvL );
    rotat.R2Rv( RR, RvR );
    G3V_AVERAGE2( Rv, RvL, RvR );
    rotat.Rv2R( Rv, R );
    G3V_AVERAGE2( T,  TL,  TR );
#if 0
    // estimate the position using triangulation
    double uvL[2], uvR[2], cxyz[3];
    pfL.getResult2DPose( NULL, uvL, NULL);
    pfR.getResult2DPose( NULL, uvR, NULL);
    stereo.triangulatePosition( uvL, uvR, cxyz );
    //stereo.printInfo();
    //printf("(%.2f %.2f %.2f) vs. (%.2f %.2f %.2f)\n", T[0], T[1], T[2], cxyz[0], cxyz[1], cxyz[2]);
#endif
  } else if (foundRPose) {
    G3M_XFORM_MERGE( R, T, Rcp, Tcp, Rpo, Tpo ); // Rco Tco
  } else if (foundLPose) {
    G3M_XFORM_MERGE( R, T, Rcq, Tcq, Rqo, Tqo ); // Rco Tco
  } else {
    memset(T, 0, 3*sizeof(double));  
    return false;
  }
  if (verbose) {
    printf("[sift] 3D Pose : Rco[9] Tco[3]=(%5.2f %5.2f %5.2f)\n", T[0], T[1], T[2]);
  }
  return true;
}

// ===================================================================
// 
// ===================================================================

bool SIFTPatternFinderStereo::getResultImage(char cmode, IMGH::Image *image, bool show_features, bool show_matches, bool show_3d_pose)
{
  // Visualize the results of SIFT feature extraction and pattern matching.
  bool gotL=false, gotR=false;
  IMGH::Image imgL, imgR;
  if (cmode != 'L') gotR = pfR.getResultImage( &imgR, show_features, show_matches, show_3d_pose );
  if (cmode != 'R') gotL = pfL.getResultImage( &imgL, show_features, show_matches, show_3d_pose );
  IMGH::ImageEditor imgedt;
  if (imgL.w == 2*pfL.camera.wh[0]) imgedt.crop( &imgL, 0, 0, imgL.w/2, imgL.h );
  if (imgR.w == 2*pfR.camera.wh[0]) imgedt.crop( &imgR, 0, 0, imgR.w/2, imgR.h );
  
  if (gotR && gotL) {
    image->setImage( imgL.w + imgR.w, imgL.h, IMGH::PIXEL_RGB );
    image->copySubregionFrom ( 0,      0, imgL.w, imgL.h, &imgL, 0, 0 );
    image->copySubregionFrom ( imgL.w, 0, imgR.w, imgR.h, &imgR, 0, 0 );
  } else if (gotR) {
    image->swapImage( &imgR );
  } else if (gotL) {
    image->swapImage( &imgL );
  } else return false;
  return true;
}

bool SIFTPatternFinderStereo::saveResultImageWithNewName(char cmode, char *oldname)
{
  if (!oldname) return false;
  IMGH::Image img;
  this->getResultImage( cmode, &img, false, false );
  IMGH::FileIO imgfile;
  char newname[320], base[320], bufL[80], bufR[80];
  sprintf(newname, "%s_debug.png", imgfile.getFileBaseName(oldname, base));
  imgfile.writeFile( newname, &img );
  if (pfL.found) sprintf(bufL, "L(%3d s%3.1f r%2.0f)", pfL.match.matched_count, pfL.match.xform.getScale(), pfL.match.xform.getRotat());
  else           sprintf(bufL, "L(%12s)", "not found");
  if (pfR.found) sprintf(bufR, "R(%3d s%3.1f r%2.0f)", pfR.match.matched_count, pfR.match.xform.getScale(), pfR.match.xform.getRotat());
  else           sprintf(bufR, "R(%12s)", "not found");
  if (verbose) printf("[sift] Image saved as '%s' .. %s %s\n", newname, bufL, bufR);
  return true;
}

void SIFTPatternFinderStereo::printInfo(char *cmmt)
{
  printf("%s\n", (cmmt ? cmmt : "PatternFinder Stereo"));
  printf("  baseline=%.3f  foundL:%s,%s  foundR:%s,%s\n", baseline,
	 (foundL ? "T":"F"), (foundLPose ? "T":"F"), 
	 (foundR ? "T":"F"), (foundRPose ? "T":"F"));
  if (pfL.refer.iimg.w>0 || pfL.scene.iimg.w>0) pfL.printInfo("  [Left]");
  if (pfR.refer.iimg.w>0 || pfR.scene.iimg.w>0) pfR.printInfo("  [Right]");
}
