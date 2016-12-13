
//
// SIFT: SIFT features of an image
//
// Jaeil Choi
// last modified in Feb, 2009
//

#ifndef SIFT_HPP
#define SIFT_HPP

#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif
#include "vm_macros.h"
#include "util_timer.hpp"
#include "imgh_common.hpp"
using namespace std;


// ===================================================================
// The SIFT feature in a memory buffer of (float)[140]
// ===================================================================

#define SIFT_SIZE 128

class SIFTFeature {
public:
  union {
    float		data[140];
    struct {
      float		x;		// X
      float		y;		// Y
      float		s;		// scale
      float		sh;		// 
      float		edge;		// edgeness
      float		o;		// orientation (radian)
      float		sco;		// 
      float		mratio;		// [matching] match ratio ([0~1] smaller, better)
      float		desc[128];
      SIFTFeature*	match;		// [matching] pointer to matched feature
      SIFTFeature*	next;		// [matching] pointer to the next for feature map
    } f;
  };
public:
  SIFTFeature() {}
  ~SIFTFeature() {}
  void printInfo(const char *cmmt=NULL, bool show_desc=false) {
    printf("%s(%.1f %.1f %.1f %.0f)\n", (cmmt ? cmmt:"SIFT"), f.x, f.y, f.s, f.o*180/M_PI);
    if (show_desc) {
      for (int i=0; i < 128; i++) { 
	if (i%8 == 0) printf("  ");
	printf("%7.4f ", f.desc[i]);
	if (i%8 == 7) printf("\n"); 
      } 
    }
  }
};


// ===================================================================
// SIFT : Scale Invariant Feature Transform
// ===================================================================

class SIFT 
{
public:
  IMGH::Image	iimg;		// input GRAYSCALE image  (PIXEL_FLOAT)
  IMGH::Image	fimg;		// SIFT features in FLOAT image (w=140 x h=N)
  int		xywh[4];	// (ref.  only) SIFT feature region in the input image
  
  bool	verbose;
  bool  use_timer;
  bool  xywh_given;
  int	ftidx;
  
private:  
  UTIL::Timer timer;
  double      timer_log[6];
  
public:
  SIFT() 
    : verbose(false), use_timer(false), xywh_given(false), ftidx(-1) { memset(xywh,0,4*sizeof(int)); }
  ~SIFT() { clear(); iimg.clear(); }
  inline void clear(void) { fimg.clear(); fimg.clear(); G6V_SET(timer_log, 0,0,0,0,0,0); }
  inline void clearFeatures(void) { fimg.clear(); }
  
  // =================================================================
  // 
  // =================================================================
public:
  bool setupInputImage(IMGH::Image *img, int xywh[4]=NULL);
  bool setupInputImageWithFloatBuffer(int w, int h, float *buf, int xywh[4]=NULL);
  
  int  extractFeatures(int nOctaves=0, int nLevels=3, float sigma=1.6, float ethreshold=10.5, bool use_gpu=false, IMGH::Image *features=NULL);
  bool setupSIFTFeaturesWithBuffer(int n, void *feats);
  
  void showScene(IMGH::Image *img, int uoff=0, int voff=0, int xywh[4]=NULL);
  void showFeatures(IMGH::Image *img, int uoff=0, int voff=0);
  void showFeatureInDetail(IMGH::Image *img, int fidx, int uoff=0, int voff=0, int r=-1, int g=-1, int b=-1);
  
  void printFeatures(char *cmmt=NULL, int fidx_min=0, int fidx_max=0, bool desc=false);
  void printInfo(char *cmmt=NULL);
  
};

#endif // SIFT_HPP
