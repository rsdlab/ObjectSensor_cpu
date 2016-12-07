
//
// SIFTMatching:
//
// Jaeil Choi
// last modified in Feb, 2009
//

#ifndef SIFT_MATCHING_HPP
#define SIFT_MATCHING_HPP

#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif
#include "vm_macros.h"
#include "util_timer.hpp"
#include "imgh_common.hpp"
#include "SIFT.hpp"
using namespace std;


// ===================================================================
// 
// ===================================================================

typedef struct { SIFTFeature *rfp, *sfp; } sift_match;

class AffXform {
public:
  // Note that the transformation is not     //  [ ra, rb, tx ] [ rx-cx ] = [ sx-cx ]
  //   in the image coordinate system. Its   //  [ rc, rd, ty ] [ ry-cy ]   [ sy-cx ]
  //   origin is the center of the pattern.  //  [  0,  0,  1 ] [   1   ]   [   1   ]
  double R[4], T[2];
  // evaluation of the hypothesis	// etc[0]: # of matched features, 
  double etc[2];			// etc[1]: average image distance
public:
  AffXform() {}
  ~AffXform() {}
public:
  inline void clear(void) { memset( R, 0, sizeof(R) ); memset( T, 0, sizeof(T) ); }
  inline bool isInvalid(void) { return (R[0]==0 && R[2]==0); }
  double getScale(void);
  double getRotat(void);
  void   getTransformedPoint(int xywh[4], double px, double py, double np[2]);
  char*  getInfo(char buf[]);
};
  
// ===================================================================
// 
// ===================================================================

class SIFTMatching 
{
public:
  SIFT		*refer;
  SIFT		*scene;
  IMGH::Image	fmap;		// image of pointers to the SIFT features (PIXEL_VOIDP)
  
  // SIFT feature matching
  sift_match   *matches;
  int		matched_count, rejected_count[5];
  AffXform	xform;		// affine transformation of the machted pattern
  double	homography[9];  // homography of the target pattern
  
public:
  bool		verbose;
  bool  	use_timer;
  bool		use_simple_match;
  int		ftidx;
  
protected:
  UTIL::Timer timer;
  double      timer_log[6];
  
public:
  SIFTMatching() 
    : matches(NULL), matched_count(0), verbose(false), use_timer(false), 
      use_simple_match(false), ftidx(-1) {}
  ~SIFTMatching() { clear(); }
  void clear(void);
  inline int     getMatchedCount(void) { return matched_count; }
  inline double* getAffineR(void) { return xform.R; }
  inline double* getAffineT(void) { return xform.T; }
  inline double* getHomography(void) { return homography; }
  inline bool    getMatchedFeatures(int idx, SIFTFeature **rfp, SIFTFeature **sfp) {
    if (idx < 0 || idx >= matched_count) { *rfp = *sfp = NULL; return false; }
    *rfp = matches[idx].rfp;
    *sfp = matches[idx].sfp;
  }
  
  // =================================================================
  // public functions to use
  // =================================================================
public:
  bool findSIFTPattern(SIFT *refer, SIFT *scene);
  bool getResultPose(double corners[4][2]=NULL, double center[2]=NULL, double sr[2]=NULL);
  void printInfo(char *cmmt=NULL);
  
  // =================================================================
  // private function
  // =================================================================
public:
  double getFeatureDistance(SIFTFeature* f1, SIFTFeature* f2);
  int  findMatchesWithTransformation(AffXform *xf, bool eval_only=false, double result_dist[3]=NULL);
  void finalizeTransformation(AffXform *f);
  void analyzeMatching(double mmv[3]);
private:
  int  findMatchesBrutalSearch(double max_ratio);
  int  findBestMatchesBrutalSearch(int min_matches, double max_ratio);
  SIFTFeature* findClosestFeature(SIFTFeature *rfp, double x, double y, int max_dist=-1,
				  double ori=0, double ori_tol=-1, 
				  double scl=1, double scl_tol=-1, double *fdist=NULL);
  bool estimateHomography(int min_matches);
public:
  void guessAffineTransformation(SIFTFeature *rfp, SIFTFeature *sfp, AffXform *xf);
public:
  void showMatches(IMGH::Image *img, int r_uoff=0, int r_voff=0, int uoff=0, int voff=0);
  void showTransformation(IMGH::Image *img, int rxywh[4], int uoff=0, int voff=0);
  
};

#endif // SIFT_MATCHING_HPP
