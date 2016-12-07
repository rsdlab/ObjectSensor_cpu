
//
// SIFTPatternFinderStereo: 
//   3D pose estimation using SIFT feature matching
//
// Jaeil Choi
// last modified in Feb, 2009
//

#ifndef SIFT_PATTERN_FINDER_STEREO_HPP
#define SIFT_PATTERN_FINDER_STEREO_HPP

#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif
#include "vm_macros.h"
#include "imgh_common.hpp"
#include "camera/CameraStereo.hpp"
#include "SIFTPatternFinder.hpp"
using namespace std;


// ===================================================================
// 
// ===================================================================

class SIFTPatternFinderStereo
{
public:
  double		baseline;
  CameraStereo<double>	stereo;
  SIFTPatternFinder	pfL;		// left  camera
  SIFTPatternFinder	pfR;		// right camera
  double		Rcq[9], Tcq[3];	// left  camera pose in common camera frame
  double		Rcp[9], Tcp[3];	// right camera pose in common camera frame
  
  bool			foundL;		// found the pattern on left  image
  bool			foundR;		// found the pattern on right image
  bool			foundLPose;	// estimated 3D pose on left  image
  bool			foundRPose;	// estimated 3D pose on right image
  bool			verbose;
  
public:
  SIFTPatternFinderStereo() : foundL(false), foundR(false),
			      foundLPose(false), foundRPose(false), verbose(false) {}
  ~SIFTPatternFinderStereo() {}
  
  // =================================================================
  // 
  // =================================================================
public:
  // setting reference pattern and its pose
  bool loadCameraParameters(char *cfg_file);
  bool loadReferencePattern(char *cfg_file, char *pattern);
  
  // setting the input scene
  bool setupInputImage(char cmode, char *fname);
  bool setupInputImage(char cmode, IMGH::Image *img);
  bool setupInputImageWithFloatBuffer(char cmode, int w, int h, float *buf);
  
  // finding the pattern 
  bool findSIFTPattern(char cmode, double Rco[9]=NULL, double Tco[3]=NULL);
  
  // inquiry of the results
  bool getResult3DPose(char cmode, double Rco[9], double Tco[3]);
  bool getResultImage (char cmode, IMGH::Image *image, bool show_features=true, bool show_matches=true, bool show_pose=true);
  bool saveResultImageWithNewName(char cmode, char *oldname);
  //
  void printInfo(char *cmmt=NULL);
  
};

#endif // SIFT_PATTERN_FINDER_STEREO_HPP
