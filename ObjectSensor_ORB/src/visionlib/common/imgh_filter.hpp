
//
// IMGH::ImageFilter
//
// Jaeil Choi
// last modified in Dec, 2006
//
// -------------------------------------------------------------------
//
// This code is a part of IMGH library (IMaGe library in Header files).
// IMGH is a very easy-to-use, light-weight and cross-platform image library,
//   which is defined entirely in header files. The purpose of this library
//   is to eliminate the installation and other setup, while providing
//   most functions of any image processing libraries.
// IMGH library has several advantages:
//   - No installation required. Just copy, and '#include' them.
//   - Operating System independent (Windows/Linux/Mac)
//   - Conversion from/to different image format can be easily done without copying.
//   - Provides a tool to draw lines, circles and polygons on the image.
//   - Supports pixel formats of 'integer', 'float', or even 'void* pointer',
//     which are useful for scalar field, or a map of pointers to complex data structures.
// IMGH library consists of:
//   IMGH::Image		in imgh_common.hpp	for creation & basic handling
//   IMGH::FileIO		in imgh_file_io.hpp	for file I/O (JPG,PNG,PGM, and more)
//   IMGH::ImageConverter	in imgh_converter.hpp	for pixel format conversion
//   IMGH::ImageEditor		in imgh_editor.hpp	for image edition (cut,resize,merge,etc.)
//   IMGH::Drawer		in imgh_drawer.hpp	for drawing on the image
//   IMGH::ImageFilter		in imgh_filter.hpp	for Gaussian filtering
//   IMGH::ImageGradient	in imgh_gradient.hpp	for gradient images
//   IMGH::CornerDetector	in imgh_corner_detector.hpp
//   IMGH::EdgeDetector		in imgh_edge_detector.hpp
//   IMGH::SuperPixels		in imgh_super_pixels.hpp
//   IMGH::IntegralImage	in imgh_integral.hpp
//   and some more. (The rest are under development, and have not been tested enough)
//
// --------------------------------------------------------------------------
//
//   This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the
//  Free Software Foundation; either version 2 of the License or later.
//  This program is distributed in the hope that it will be useful, but
//  WITHOUT ANY WARRANTY. See the GNU General Public License for more details.
//
// -------------------------------------------------------------------
//

#ifndef IMGH_IMAGE_FILTER_HPP
#define IMGH_IMAGE_FILTER_HPP

#include <iostream>
#include "imgh_common.hpp"

namespace IMGH {

// behavior pattern for boundary pixels
typedef enum { IMGH_FILTER_CLIP=0,	// do nothing on the boundary
	       IMGH_FILTER_ZERO,	// use zero for pixel value
	       IMGH_FILTER_REPEAT,	// use pixel value  on the edge
	       IMGH_FILTER_CYCLE,	// use pixel  values on the opposite boundary
	       IMGH_FILTER_MIRROR	// use mirror values on the same boundary
} behavior_t;

// ===================================================================
//
// ===================================================================

template <class T>
class ImageConvolution {
private:
  float *mask;
  int    size;
public:
  ImageConvolution(float *mask, int mask_size) { this->mask = mask; size = mask_size; }
  ~ImageConvolution() {}
public:
  void convolute1(void *src, void *dst, int length, int stride, int behavior) {
    int   i, j, half = (int)size/2;
    float sum, wsum;
    T     *sp = (T*)src, *dp = (T*)dst;
    // boundary pixels at the beginning
    switch (behavior) {
    case IMGH_FILTER_CLIP: sp += half*stride; dp += half*stride; break;
    case IMGH_FILTER_REPEAT:
    default:
      for (i = 0; i < half; i++, dp+=stride, sp+=stride) {
	for (j = -i, sum = wsum = 0; j <= +half; j++) {
	  sum  += mask[half+j] * sp[ j * stride ];
	  wsum += mask[half+j];
	}
	*dp = (T)(wsum!=0 ? sum / wsum : 0);
      }
    }
    // check if the mask is symmetric
    bool  symmetric=true;
    for (i=0; i<half; i++) if (mask[i] != mask[size-1-i]) { symmetric = false; break; }
    // non-boundary pixels
    if (symmetric) {
      for (i = half; i < length-half; i++, dp+=stride, sp+=stride) {
	sum = mask[half]*sp[0];
	if (stride == 1)
	  for (j = 1; j <= half; j++) sum += mask[half+j] *(sp[j] + sp[-j]);
	else
	  for (j = 1; j <= half; j++) sum += mask[half+j] *(sp[j*stride] + sp[-j*stride]);
	*dp = (T)sum;
      }
    } else {
      for (i = half; i < length-half; i++, dp+=stride, sp+=stride) {
	sum = 0.0;
	if (stride == 1)
	  for (j = 0; j < size; j++) sum += mask[j] * sp[j-half];
	else
	  for (j = 0; j < size; j++) sum += mask[j] * sp[(j-half)*stride];
	*dp = (T)sum;
      }
    }
    // boundary pixels at the end
    switch (behavior) {
    case IMGH_FILTER_CLIP:  break;
    case IMGH_FILTER_REPEAT:
    default:
      for (i = length-half; i < length; i++, dp+=stride, sp+=stride) {
	sum = wsum = 0;
	for (j = -half; j <= length - 1 - i; j++) {
	  sum  += mask[half+j] * sp[ j * stride ];
	  wsum += mask[half+j];
	}
	*dp = (T)(wsum!=0 ? sum / wsum : 0);
      }
    }
  }
  void convolute3(void *src, void *dst, int length, int stride, int behavior) {
    int   i, j, half = (int)size/2, pos;
    float sum[3], wsum;
    T     *p, *sp = (T*)src, *dp = (T*)dst;
    // boundary pixels at the beginning
    switch (behavior) {
    case IMGH_FILTER_CLIP: sp += half*stride; dp += half*stride; break;
    case IMGH_FILTER_REPEAT:
    default:
      for (i = 0; i < half; i++, dp+=stride, sp+=stride) {
	IMGH3V_SET( sum,  0, 0, 0 );
	for (j = -i, wsum = 0; j <= +half; j++) {
	  p = sp + j * stride;
	  IMGH3V_SADD( sum, sum, mask[half+j], p );
	  wsum += mask[half+j];
	}
	IMGH3V_DIV( sum, wsum );  IMGH3V_SET( dp, (T)sum[0], (T)sum[1], (T)sum[2] );
      }
    }
    // check if the mask is symmetric
    bool  symmetric=true;
    for (i = 0; i < half; i++) if (mask[i] != mask[size-1-i]) { symmetric = false; break; }
    // non-boundary pixels
    if (symmetric) {
      for (i = half; i < length-half; i++, dp+=stride, sp+=stride) {
	IMGH3V_SET( sum, mask[half]*sp[0], mask[half]*sp[1], mask[half]*sp[2] );
	for (j = 1; j <= half; j++) {
	  pos = j * stride;
	  sum[0] += mask[half+j] *((float)sp[pos+0] + (float)sp[-pos+0]);
	  sum[1] += mask[half+j] *((float)sp[pos+1] + (float)sp[-pos+1]);
	  sum[2] += mask[half+j] *((float)sp[pos+2] + (float)sp[-pos+2]);
	}
	IMGH3V_SET( dp, (T)sum[0], (T)sum[1], (T)sum[2] );
      }
    } else {
      for (i = half; i < length-half; i++, dp+=stride, sp+=stride) {
	IMGH3V_SET( sum, 0, 0, 0 );
	for (j = 0; j < size; j++) {
	  pos = (j-half) * stride;
	  sum[0] += mask[j] * sp[pos+0];
	  sum[1] += mask[j] * sp[pos+1];
	  sum[2] += mask[j] * sp[pos+2];
	}
	IMGH3V_SET( dp, (T)sum[0], (T)sum[1], (T)sum[2] );
      }
    }
    // boundary pixels at the end
    switch (behavior) {
    case IMGH_FILTER_CLIP: break;
    case IMGH_FILTER_REPEAT:
    default:
      for (i = length-half; i < length; i++, dp+=stride, sp+=stride) {
	IMGH3V_SET( sum,  0, 0, 0 );
	for (j = -half, wsum = 0; j <= length - 1 - i; j++) {
	  p = sp + j * stride;
	  IMGH3V_SADD( sum, sum, mask[half+j], p );
	  wsum += mask[half+j];
	}
	IMGH3V_DIV( sum, wsum );  IMGH3V_SET( dp, (T)sum[0], (T)sum[1], (T)sum[2] );
      }
    }
  }

  void average(void *src, void *dst, int wsize, int length, int stride, int behavior) {
    int   i, half = (int)wsize/2;
    float sum=0;
    T     *sp = (T*)src, *dp = (T*)dst;
    // boundary pixels at the beginning
    switch (behavior) {
    case IMGH_FILTER_CLIP:
      for (i = 0; i < wsize-1; i++) sum += sp[i*stride];
      for (i = 0; i < half; i++, dp+=stride, sp+=stride);
      break;
    case IMGH_FILTER_ZERO:
      for (i = 0; i < half; i++) sum += (sp[i*stride]);
      for (i = 0; i < half; i++, dp+=stride, sp+=stride) {
	sum += sp[+half*stride];
	*dp = (T)(sum / wsize);
      }
      break;
    case IMGH_FILTER_REPEAT:
    default:
      for (i = -half; i < half; i++) sum += (i<0 ? *sp : sp[i*stride]);
      for (i = 0; i < half; i++, dp+=stride, sp+=stride) {
	sum += sp[+half*stride];
	*dp = (T)(sum / wsize);
	sum -= sp[-i*stride];
      }
      break;
    }
    // non-boundary pixels
    for (; i < length-half; i++, dp+=stride, sp+=stride) {
      sum += sp[+half*stride];
      *dp = (T)(sum / wsize);
      sum -= sp[-half*stride];
    }
    // boundary pixels at the end
    switch (behavior) {
    case IMGH_FILTER_CLIP: /* do nothing */  break;
    case IMGH_FILTER_ZERO:
      for (; i < length; i++, dp+=stride, sp+=stride) {
	*dp = (T)(sum / wsize);
	sum -= sp[-half*stride];
      }
      break;
    case IMGH_FILTER_REPEAT:
    default:
      for (; i < length; i++, dp+=stride, sp+=stride) {
	sum += sp[(length-i-1)*stride];
	*dp = (T)(sum / wsize);
	sum -= sp[-half*stride];
      }
      break;
    }
  }
};

// ===================================================================
//
// ===================================================================

class ImageFilter
{
 public:
  float *mask;
  int   size;

public:
  ImageFilter () : mask(NULL), size(0) { }
  ~ImageFilter() { clear(true);  }
  void clear(bool bFree=true) { if (bFree && mask) free(mask); mask=NULL; size=0; }

  // -----------------------------------------------------------------
  // Gaussian filter
  // -----------------------------------------------------------------
public:
  void convoluteWithGaussian(IMGH::Image *src, IMGH::Image *dst=NULL,
			     int filter_size=3, double sigma=0.0, double scale=1.0,
			     behavior_t behavior=IMGH_FILTER_REPEAT) {
    this->createGaussianFilter( filter_size, sigma, scale );
    this->convolute( src, dst, behavior );
  }

  // -----------------------------------------------------------------
  void createGaussianFilter(int size=5, double sigma=0.0, double scale=1.0) {
    // Create Gaussian filter into the member variable 'mask'.
    //   size   : size of the mask (should be an odd number, like 3, 5 or 7)
    //   sigma  : standard deviation
    //   scale  : sum of all the elements (normalized when scale=1.0)
    clear();
    if (size < 3) { printf("Error (ImageFilter:createGaussianFilter): filter size < 3\n"); return; }
    else if (size % 2 == 0) size++;
    float x, norm;
    int   i, half = (int)size/2;
    mask = (float*) malloc(size * sizeof(float));
    this->size = size;
    if (sigma <= 0.0) sigma = (size / 6.0f);	// calculate variance, if necessary
    for (i = 0, norm = 0.0; i < size; i++) {	// calculate mask values
      x = (float)(i - half);
      mask[i] = (float)exp(-0.5*x*x/(sigma*sigma));
      norm += mask[i];
    }
    norm /= (float)scale;			// normalize
    for (i = 0; i < size; i++) mask[i] /= norm;
  }

  // -----------------------------------------------------------------
public:
  void convolute(IMGH::Image *src, IMGH::Image *dst=NULL,
		 behavior_t behavior=IMGH_FILTER_REPEAT) {
    // Apply the filter mask using convolution.
    // Note that 'src' and 'dst' can be same.
    if (!src || !src->data || !mask) return;
    int         i, j, pos;
    IMGH::Image tmp( src->w, src->h, src->type );
    if (dst) dst->setImage( src->w, src->h, src->type );
    else dst = src;
    ImageConvolution<unsigned char> ccv( mask, size );
    ImageConvolution<float>         fcv( mask, size );
    switch (src->type) {
    case PIXEL_GRAY:
      for (i = 0; i < src->h; i++) {	// convolution along X axis (src -> tmp)
	pos = i * src->w;
	ccv.convolute1(src->data+pos, tmp.data+pos, src->w, 1, behavior);
      }
      for (j = 0; j < src->w; j++)	// convolution along Y axis (tmp -> dst)
	ccv.convolute1(tmp.data+j, dst->data+j, src->h, src->w, behavior);
      break;
    case PIXEL_RGB:
      for (i = 0; i < src->h; i++) {	// convolution along X axis (src -> tmp)
	pos = i * 3 * src->w;
	ccv.convolute3(src->data+pos, tmp.data+pos, src->w, 3, behavior);
      }
      for (j = 0; j < src->w; j++)	// convolution along Y axis (tmp -> dst)
	ccv.convolute3(tmp.data+j*3, dst->data+j*3, src->h, 3*src->w, behavior);
      break;
    case PIXEL_FLOAT:
      for (i = 0; i < src->h; i++) {	// convolution along X axis (src -> tmp)
	pos = i * src->w;
	fcv.convolute1(src->data+pos, tmp.data+pos, src->w, 1, behavior);
      }
      for (j = 0; j < src->w; j++)	// convolution along Y axis (tmp -> dst)
	fcv.convolute1(tmp.data+j, dst->data+j, src->h, src->w, behavior);
      break;
    default: break;
    }
  }

  // -----------------------------------------------------------------
  // Min/Max filter
  // -----------------------------------------------------------------
public:
  void getMinMax(IMGH::Image *src, IMGH::Image *dst, int wsize, bool for_max) {
    // Note that 'src' and 'dst' can be same.
    if (!src || !dst || src->type != PIXEL_RGB) return;
    int i, j, pos;
    if (wsize < 1) wsize = 1;
    if (wsize%2 == 0) wsize++;	// window size must be odd.
    // get min/max value along x axis, and save the result in 'tmp'
    IMGH::Image tmp(src->w, src->h, PIXEL_RGB);
    for (i = 0; i < src->h; i++) {
      pos = i * 3 * src->w;
      getMinMaxWithBuffer(src->data+pos, tmp.data+pos, wsize, src->w, 3, for_max);
    }
    // get min/max value along y axis, and save the result in 'dst'
    dst->setImage( src->w, src->h, PIXEL_RGB );
    for (j = 0; j < src->w; j++) {
      getMinMaxWithBuffer(tmp.data+j*3, dst->data+j*3, wsize, src->h, 3*src->w, for_max);
    }
  }
  void getMinMaxWithBuffer(unsigned char *src, unsigned char *dst,
			   int wsize, int length, int stride, bool for_max) {
    int   i, j, s, e, half = (int)wsize/2;
    unsigned char *p, final_value[3];
    if (for_max) {
      for (i = 0; i < length; i++, dst+=stride, src+=stride) {
	s = (i - half >=     0 ? -half : -i);
	e = (i + half < length ? +half : length - i - 1);
	for (j = s; j <= e; j++) {
	  p = src + j * stride;
	  if (j == s || p[0] > final_value[0]) final_value[0] = p[0];
	  if (j == s || p[1] > final_value[1]) final_value[1] = p[1];
	  if (j == s || p[2] > final_value[2]) final_value[2] = p[2];
	}
	IMGH3V_COPY( dst, final_value );
      }
    } else {
      for (i = 0; i < length; i++, dst+=stride, src+=stride) {
	s = (i - half >=     0 ? -half : -i);
	e = (i + half < length ? +half : length - i - 1);
	for (j = s; j <= e; j++) {
	  p = src + j * stride;
	  if (j == s || p[0] < final_value[0]) final_value[0] = p[0];
	  if (j == s || p[1] < final_value[1]) final_value[1] = p[1];
	  if (j == s || p[2] < final_value[2]) final_value[2] = p[2];
	}
	IMGH3V_COPY( dst, final_value );
      }
    }
  }

  // -----------------------------------------------------------------
  //
  // -----------------------------------------------------------------
public:
  void printInfo(char *comment=NULL) {
    if (comment) std::cout << comment << std::endl;
    else         std::cout << "ImageFilter" << std::endl;
    int i;
    float sum=0;
    for (i = 0; i < size; i++) sum += mask[i];
    printf("  size=%d  sum=%.3f  ( ", size, sum);
    for (i = 0; i < size; i++) printf("%.4f ", mask[i]);
    printf(")\n");
  }
};


}	// namespace IMGH


#endif	// IMGH_IMAGE_FILTER_HPP
