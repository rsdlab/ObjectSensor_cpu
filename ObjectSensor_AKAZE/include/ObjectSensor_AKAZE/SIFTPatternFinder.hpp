
//
// SIFTPatternFinder: 
//   3D pose estimation using SIFT feature matching
//
// Jaeil Choi
// last modified in Feb, 2009
//

#ifndef SIFT_PATTERN_FINDER_HPP
#define SIFT_PATTERN_FINDER_HPP

#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif
#include "vm_macros.h"
#include "imgh_common.hpp"
#include "util_timer.hpp"
#include "camera/Camera.hpp"
#include "SIFT.hpp"
#include "SIFTMatching.hpp"
using namespace std;


// ===================================================================
// 
// ===================================================================

class SIFTPatternFinder
{
public:
  SIFT		refer;
  SIFT		scene;
  SIFTMatching	match;
  Camera<double>camera;		// camera model (extrinsics are irrelevant)
  double	Rcf[9], Tcf[3];	// pose of refer pattern in camera space
  double	Rco[9], Tco[3];	// pose of scene pattern in camera space
  double   H[9];
  bool		found;
  bool		verbose;
  bool		use_gpu;
  
  bool		sift_verbose;
  int		sift_nOctaves;
  int		sift_nLevels;
  float		sift_sigma;
  float		sift_ethreshold;
  
  double	timer_sec[3];
public:
  SIFTPatternFinder() : found(false), verbose(false), use_gpu(true),
			sift_verbose(false), sift_nOctaves(0), 
			sift_nLevels(3), sift_sigma(1.6f), 
			sift_ethreshold(10.0f) {
    match.refer = &refer;   match.scene = &scene;
    memset( timer_sec, 0, sizeof(timer_sec) );
  }
  ~SIFTPatternFinder() {}
  
  // =================================================================
  // 
  // =================================================================
public:
  // setting reference pattern and its pose
  bool loadCameraParameters(char *cfg_file, char *cfg_section);
  bool loadReferencePattern(char *cfg_file, char *cfg_section, int left_right=0);
  bool loadReferencePattern(char *img_file, int xywh[4], double Rcf[9], double Tcf[3]);
  // setting the input (scene) image
  bool setupInputImage(char *fname);
  bool setupInputImage(IMGH::Image *img);
  bool setupInputImageWithFloatBuffer(int w, int h, float *buf);
  // setting up extracted SIFT features for the scene
  bool setupSIFTFeaturesWithBuffer(int n, SIFTFeature *fbuf);
  
  // finding the pattern 
  bool findSIFTPattern(double Rco[9]=NULL, double Tco[3]=NULL);
  bool findSIFTPatternwithH(double Ho[9], double Rco[9] = NULL, double Tco[3] = NULL);
  // inquiry of the results
  bool getResult2DPose(double corners[4][2]=NULL, double center[2]=NULL, double sa[2]=NULL);
  bool getResult3DPose(double Rco[9], double Tco[3]);
  bool getResult3DPosewithH(double Ho[9], double Rco[9], double Tco[3]);
  bool getResultImage (IMGH::Image *image, bool show_features=true, bool show_matches=true, bool show_pose=true);
  //
  void printInfo(char *cmmt=NULL);
  
private:
  bool estimateHomographyRectified(double homog[9]);
  void optimizePoseForHomography(double Rco[9], double Tco[3]);
  void getReferencePatternSize(double wh[2], double objp[4][3]=NULL);
  void visualizePatternPose(IMGH::Image *img, double R[9], double T[3], double size, int uoff=0, int voff=0);
  void visualizeObjBoundary(IMGH::Image *img, double R[9], double T[3], double width, double height, double uoff=0, double voff=0);
};

#endif // SIFT_PATTERN_FINDER_HPP
