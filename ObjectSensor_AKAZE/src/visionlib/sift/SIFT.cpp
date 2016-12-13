
//
// SIFT: SIFT features of an image
//
// Jaeil Choi
// last modified in Feb, 2009
//

#include <iostream>
#include "vm_macros.h"
#include "imgh_common.hpp"
#include "imgh_drawer.hpp"
#include "imgh_converter.hpp"
#include "imgh_file_io.hpp"
#include "util_timer.hpp"
#include "SIFT.hpp"
#include "siftpp_core.hpp"

using namespace std;

// ===================================================================
// 
// ===================================================================

bool SIFT::setupInputImage(IMGH::Image *img, int xywh[4])
{
  // Set up the input image by copying into its own buffer.
  if (!img) return false;
  if (img != &iimg) {
    IMGH::ImageConverter conv;
    conv.convertImage( img, &iimg, IMGH::PIXEL_FLOAT );
  }
  if (xywh) G4V_COPY( this->xywh, xywh );
  else      G4V_SET ( this->xywh, 0, 0, iimg.w, iimg.h );
  xywh_given = (xywh && xywh[2]>0 && xywh[3]>0);
  return true;
}

bool SIFT::setupInputImageWithFloatBuffer(int w, int h, float *buf, int xywh[4])
{
  // Set up the input image without copying all the pixel data.
  if (!buf) return false;
  iimg.setImage( w, h, IMGH::PIXEL_FLOAT, buf );
  if (xywh) G4V_COPY( this->xywh, xywh );
  else      G4V_SET ( this->xywh, 0, 0, iimg.w, iimg.h );
  xywh_given = (xywh && xywh[2]>0 && xywh[3]>0);
  return true;
}

// ===================================================================
// Extracting SIFT Features
// ===================================================================

int SIFT::extractFeatures(int nOctaves, int nLevels, float sigma, float ethreshold,
			  bool use_gpu, IMGH::Image *features)
{ 
	// Extract SIFT features from the input image 'this->iimg', and save
	//   the features in 'features' (140xN PIXEL_FLOAT image).
	//   If 'features==NULL', then they will be saved in 'this->fimg'.
	//   nOctaves  : number of octaves (if positive.  0:auto(default), -1:auto with upsampling)
	//   nLevels   : number of blurring levels in each octave [3]
	//   sigma     : size of the initial Gaussian filter [GPU:1.6 CPU:0.5]
	//   ethreshold: rejection threshold for keypoints on edge [10.0]
	if (verbose) {
		if (xywh_given) printf("Extracting SIFT features from (%d x %d) image  with xywh=[%d %d %d %d]\n", iimg.w, iimg.h, xywh[0], xywh[1], xywh[2], xywh[3]);
		else            printf("Extracting SIFT features from (%d x %d) image\n", iimg.w, iimg.h);
		printf("  nOctaves=%d  nLevels=%d  sigma=%.2f  ethreshold=%.2f  use_GPU=%d\n", nOctaves, nLevels, sigma, ethreshold, use_gpu);
	}
	clearFeatures();
	if (!features) features = &fimg;
    
// CPU-based SIFT feature extraction -----------------------------
    
    // initialize SIFT++ (initialize Gaussian scale space)
    int   O      = nOctaves ;
    int   omin   = (nOctaves < 0 ? -1 : 0);  // the first octave (it can be -1)
    float sigman = sigma;
    float sigma0 = (float)(1.6 * powf(2.0f, 1.0f / nLevels));
    float threshold = 0.04f / nLevels / 2.0f;
    
    if (O < 1) {  // autoselect the number of octaves (downsample up to 8x8 patches)
      O = int(std::floor(log((double)(iimg.w < iimg.h ? iimg.w : iimg.h))/log(2.0)) - omin -3);
      if (O < 1) O = 1;
      //O = std::max( int(std::floor(log2(std::min(iimg.w,iimg.h))) - omin -3), 1 );
    }
    if (verbose) printf("  numOctaves: %d,  firstOctave: %d,  nLevels: %d\n", O, omin, nLevels);
    if (verbose) printf("  sigman=%g sigma0=%g  threshold=%g,  ethreshold=%g\n", sigman, sigma0, threshold, ethreshold);
    
    VL::Sift siftpp( (const VL::pixel_t*)(iimg.data), iimg.w, iimg.h, 
		     sigman, sigma0, O, nLevels, omin, -1, nLevels+1, verbose ) ;
    siftpp.detectKeypoints(threshold, ethreshold);	// find extrema of DOG
    siftpp.setNormalizeDescriptor( true );		// set descriptor options
    siftpp.setMagnification( 3.0 );			// set descriptor options
    
    SIFTFeature *fbuf = (SIFTFeature*) calloc( siftpp.keypoints.size()*2, sizeof(SIFTFeature) );
    SIFTFeature *fp=fbuf;
    int nf = 0;
    int xymin[2] = { xywh[0], xywh[1] };
    int xymax[2] = { xywh[0] + xywh[2], xywh[1] + xywh[3] };
    for( VL::Sift::KeypointsConstIter iter = siftpp.keypointsBegin() ;
	 iter != siftpp.keypointsEnd() ; ++iter ) {
      if ( xywh_given && (iter->x < xymin[0] || iter->x >= xymax[0] ||
			  iter->y < xymin[1] || iter->y >= xymax[1] )) continue;
      // detect orientations
      float angles[4];  
      int a, nangles = siftpp.computeKeypointOrientations(angles, *iter);
      // compute descriptors
      for(a = 0; a < nangles ; ++a) {
	//printf("  %3d x=%.2f y=%.2f s=%.2f angle=%.2f\n", nf, iter->x, iter->y, iter->sigma, angles[a]*180/M_PI);
	G6V_SET( fp->data, iter->x, iter->y, iter->sigma, 0, 0, angles[a] );
	siftpp.computeKeypointDescriptor( fp->f.desc, *iter, angles[a] );
	nf++;  fp++;
      }
    }
    fbuf = (SIFTFeature*) realloc( fbuf, nf * sizeof(SIFTFeature) );
    features->setImageWithAllocatedBuffer( 140, nf, IMGH::PIXEL_FLOAT, fbuf );

	return features->h;
}

bool SIFT::setupSIFTFeaturesWithBuffer(int n, void *feats)
{
  // Set up SIFT features using the values in an existing buffer,
  //   in cases where the features are provided by another module.
  // Note that 'feats' must be either (SIFTFeature*) or (float*).
  if (!feats) return false;
  fimg.setImage( 140, n, IMGH::PIXEL_FLOAT, feats );
  return true;
}

// ===================================================================
// 
// ===================================================================

void SIFT::showScene(IMGH::Image *img, int uoff, int voff, int xywh[4])
{
  int x, y, xx, yy, dummy[4];
  if (xywh == NULL) { xywh = dummy;  G4V_SET( xywh, 0, 0, iimg.w, iimg.h ); }
  if (img->w < xywh[2] + uoff || img->h < xywh[3] + voff) return;
  for (y = 0; y < xywh[3]; y++) {
    yy = xywh[1] + y;
    for (x = 0; x < xywh[2]; x++) {
      xx = xywh[0] + x;
      unsigned int v = (unsigned int)(iimg.getFloat( xx, yy ) * 255);
      img->setRGB( xx+uoff, yy+voff, v, v, v );
    }
  }
}

void SIFT::showFeatures(IMGH::Image *img, int uoff, int voff)
{
  // Visualize the SIFT features using arrows,
  //   assumming 'img' is already set appropriately.
  //   'uoff' and 'voff' defines the offset by which the features will be translated.
  if (!img) return;
  IMGH::Drawer<double> drw(img);
  if (xywh_given && xywh[2]>0 && xywh[3]>0) {
    // draw the boundary of the region
    drw.setColor( 0, 0, 255 );
    drw.drawRect( xywh[0]+uoff, xywh[1]+voff, xywh[0]+xywh[2]-1+uoff, xywh[1]+xywh[3]-1+voff );
  }
  for (int i=0; i < fimg.h; i++) {
    SIFTFeature *fp = (SIFTFeature*)fimg.getRow(i);
    if (ftidx>=0 && i==ftidx%fimg.h) drw.setColor( 255, 255,   0 ); else
    if (fp->f.match) drw.setColor( 255,   0,   0 );  // Red  (matched features)
    else      	     drw.setColor( 255, 120, 120 );  // Pink (unmatched features)
    //drw.setColor( 255, 0, 0 );  // Red
    drw.drawArrow(fp->f.x + uoff, fp->f.y + voff, fp->f.s*3, -fp->f.o);
    if (false) showFeatureInDetail( img, i, uoff, voff, 200, 0, 200 );
  }
}

void SIFT::showFeatureInDetail(IMGH::Image *img, int fidx, int uoff, int voff, int r, int g, int b)
{
  // Visualize the given SIFT feature with index 'fidx' in detail.
  if (fidx < 0 || fidx >= fimg.h) return;
  IMGH::Drawer<double> drw(img);
  SIFTFeature *fp = (SIFTFeature*)fimg.getRow(fidx);
  double cc[2]={fp->f.x + uoff, fp->f.y + voff}, radius=fp->f.s*6;
  double xyoff[2], roff[2], dvalue, dsize, drad, R[2];
  double rot = M_PI*1.5 - fp->f.o;
  double cosv=cos(rot), sinv=sin(rot);
  G2M_SET( R, +cosv, -sinv, +sinv, +cosv );
  for (int j = 0; j < 4; j++) {
    xyoff[1] = -(j-1.5f) * radius/2;	// image coordinate is different from general 2D
    for (int i = 0; i < 4; i++) {
      xyoff[0] = (i-1.5f) * radius/2;
      G2M_MUL_MV( roff, R, xyoff );
      roff[1] *= -1;			// back to image coordinate
      for (int k = 0; k < 8; k++) {
	dvalue = fp->f.desc[ (4*j+i)*8 + k ];	// descriptor value
	dsize  = radius * sqrt(dvalue) / 2;	// arrow size 
	drad   = M_PI * k / 4;		// arrow direction
	drw.drawArrow( cc[0]+roff[0], cc[1]+roff[1], dsize, rot+drad, r, g, b, true, false );
      }
    }
  }
}

void SIFT::printFeatures(char *cmmt, int fidx_min, int fidx_max, bool desc)
{
  // print information with summary of all the features
  printf("%s  total=%d  xywh=(%d %d %d %d)\n", 
	 (cmmt ? cmmt : "SIFT Features"), fimg.h, xywh[0], xywh[1], xywh[2], xywh[3]);
  if (fidx_max==0) fidx_max = fimg.h-1;
  for (int i = fidx_min; i <= fidx_max && i <= fimg.h; i++) {
    SIFTFeature *fp = (SIFTFeature*)fimg.getRow(i);
    printf("  F%02d : (%5.1f %5.1f)  s=%4.2f o=%3.0f\n", i, fp->f.x, fp->f.y, fp->f.s, fp->f.o*180/M_PI);
    for (int i=0; i<128 && desc; i++) {
      if (i%8 == 0) printf("  ");
      printf("%7.4f ", fp->f.desc[i]);
      if (i%8 == 7) printf("\n");
    }
  }
}


void SIFT::printInfo(char *cmmt)
{
  // print information with summary of all the features
  printf("%s  total=%d  xywh=(%d %d %d %d)\n", 
	 (cmmt ? cmmt : "SIFT Features"), fimg.h, xywh[0], xywh[1], xywh[2], xywh[3]);
  iimg.printInfo("  greyscale image");
  double smin=0, smax=0;
  for (int i=0; i<fimg.h; i++) {
    SIFTFeature *fp = (SIFTFeature*)fimg.getRow(i);
    if (i==0 || fp->f.s < smin) smin = fp->f.s;
    if (i==0 || fp->f.s > smax) smax = fp->f.s;
  }
  printf("  range of feature scale: %.2f ~ %.2f\n", smin, smax);
  if (use_timer) 
    printf("  stopwatch: SIFT:%6.4f Tree:%6.4f (%6.4f %6.4f) Match:%6.4f Pose:%6.4f\n",
	   timer_log[0], timer_log[1], timer_log[2], timer_log[3], timer_log[4], timer_log[5]);
}

#include "imgh_editor.hpp"
void save_fbuf(float *data, int w, int h, char *fname)
{
  IMGH::Image tmp;
#if 0
  tmp.setImage( w, h, IMGH::PIXEL_RGBA, data );	// saving as RGBA image is not working
#else
  // sa
  tmp.setImage( w, h, IMGH::PIXEL_RGB );
  IMGH::ImageConverter conv;
  conv.convertImage( data, tmp.data, w, h, IMGH::PIXEL_FLOAT, IMGH::PIXEL_RGB );
#endif
  IMGH::FileIO ifile;
  ifile.writeFile( fname, &tmp );
  
//   // check symmetry
//   IMGH::Image img( w, h, IMGH::PIXEL_FLOAT, data );
//   IMGH::ImageEditor edt;
//   edt.checkSymmetry( &img, &tmp, true );
//   //tmp.printInfo("symm");
//   IMGH::Image tmp2( w, h, IMGH::PIXEL_RGB );
//   conv.convertFloatImageToRGB( tmp.data, tmp2.data, w, h, "Sign" );
//   ifile.writeFile( fname, &tmp2 );
}

