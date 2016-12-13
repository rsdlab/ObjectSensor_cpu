
//
// ImageEditor
//
// Jaeil Choi
// last modified in Feb, 2007
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
// This class is designed for:
//   - combining multiple input images into single image
//   - editing an image (crop, flip, resize, etc)
//   - changing pixel values (by flooding) in the image
//   - rectification/distortion of image
//

#ifndef IMGH_IMAGE_EDITOR_HPP
#define IMGH_IMAGE_EDITOR_HPP

#include <iostream>
#include "imgh_common.hpp"
#include "imgh_converter.hpp"
#include "imgh_filter.hpp"
#include "imgh_file_io.hpp"

namespace IMGH {

class ImageEditor
{
public:
  ImageEditor() {}
  ~ImageEditor() {}

public:
  // -----------------------------------------------------------------
  // editing a single image
  // -----------------------------------------------------------------
public:
  void rotate(IMGH::Image *img, bool ccw=false, IMGH::Image *dst=NULL) {
    if (!img || img->w == 0 || img->h == 0) return;
    IMGH::Image dummy;
    if (dst == NULL) dst = &dummy;
    dst->setImage( img->h, img->w, img->type );
    unsigned char pv[4];
    for (int j = 0; j < img->h; j++)
      for (int i = 0; i < img->w; i++) {
	img->getPixel( i, j, pv );
	if (ccw) dst->setPixel( j, img->w - 1 - i, pv );
	else     dst->setPixel( img->h - 1 - j, i, pv );
      }
    if (dst == &dummy) {
      if (img->allocated) img->copyFrom( &dummy );
      else img->swapImage( &dummy );
    }
  }

  void invert(IMGH::Image *img) {
    int i, *srci=(int*)img->data, total=img->w*img->h;
    float *srcf=(float*)img->data;
    switch (img->type) {
    case PIXEL_INT:   for (i=0; i<total; i++) srci[i] *= -1;   break;
    case PIXEL_FLOAT: for (i=0; i<total; i++) srcf[i] *= -1;   break;
    case PIXEL_GRAY:  case PIXEL_RGB:  case PIXEL_BGR:
      total = img->w * img->h * img->pixel_size;
      for (i=0; i<total; i++) img->data[i] = 255 - img->data[i];
      break;
    case PIXEL_GRAYA:
    case PIXEL_RGBA:
    default: break;
    }
  }

  // -----------------------------------------------------------------

private:
  bool adjustRect(IMGH::Image *img, int &x, int &y, int &w, int &h, int m=0) {
    if (x < m) x = m;  if (y < m) y = m;
    if (x+w+m > img->w) w = img->w - x - m;  if (w < 1) return false;
    if (y+h+m > img->h) h = img->h - y - m;  if (h < 1) return false;
    //printf(" x=%d y=%d w=%d h=%d\n", x, y, w, h);
    return true;
  }

public:
  bool markRect(IMGH::Image *img, int x, int y, int w, int h, int r=255, int g=0, int b=0) {
    if (!adjustRect( img, x, y, w, h )) return false;
    for (int j = 0; j < h; j++)
      for (int i = 0; i < w; i++) {
	uchar_t *p = img->pixel( (x+i), (y+j) );
	if (i==0 || i==w-1 || j==0 || j==h-1)
	  switch (img->type) {
	  case PIXEL_GRAY:  p[0] = r;  break;
	  case PIXEL_GRAYA: p[0] = r; p[1] = 255;  break;
	  case PIXEL_RGB:   p[0] = r; p[1] = g; p[2] = b;  break;
	  case PIXEL_RGBA:  p[0] = r; p[1] = g; p[2] = b; p[3] = 255;  break;
	  default: break;
	  }
      }
    return true;
  }

  bool fillRect(IMGH::Image *img, int x, int y, int w, int h, int r, int g, int b) {
    if (img->type != PIXEL_RGB && img->type != PIXEL_RGBA) return false;
    if (!adjustRect( img, x, y, w, h )) return false;
    for (int j = 0; j < h; j++)
      for (int i = 0; i < w; i++) img->setRGB( (x+i), (y+j), r, g, b );
    return true;
  }

  bool fillCircle(IMGH::Image *img, float x, float y, float radius, int r, int g, int b) {
    if (img->type != PIXEL_RGB && img->type != PIXEL_RGBA) return false;
    for (int j = 0; j < img->h; j++)
      for (int i = 0; i < img->w; i++) {
	float dy = j + 0.5f - y;
	float dx = i + 0.5f - x;
	if (dx*dx + dy*dy > radius * radius) continue;
	if (dy < 0) continue;
	img->setRGB( i, j, r, g, b );
      }
    return true;
  }

  void fillChecker(IMGH::Image *img, int size, int b0=0, int b1=0, int b2=0, int w0=0, int w1=0, int w2=0) {
    // draw checker pattern
    uchar_t pv[2][4]={{b0,b1,b2,255},{w0,w1,w2,255}};
    for (int y = 0; y < img->h; y++)
      for (int x = 0; x < img->w; x++)
	img->setPixel(x, y, (((x/size)%2 + (y/size)%2)%2 == 0 ? pv[0] : pv[1]));
  }

  bool crop(IMGH::Image *img, int x, int y, int w, int h, IMGH::Image *dst=NULL) {
    // Crop a region from input image 'img'.
    // This function replaces 'img', unless the output image 'dst' is given,
    if (!img || img->w == 0 || img->h == 0) return false;
    IMGH::Image dummy;
    if (dst == NULL) dst = &dummy;
    if (x + w >= img->w) w = img->w - x;
    if (y + h >= img->h) h = img->h - y;
    dst->setImage( w, h, img->type );
    img->copySubregionTo( x, y, w, h, dst, 0, 0 );
    if (dst == &dummy) {
      if (img->allocated) img->copyFrom( dst );
      else img->swapImage( dst );
    }
    return true;
  }

  bool flip(IMGH::Image *img, bool horizontal=true) {
    if (!img || img->w == 0 || img->h == 0) return false;
    int  hw = img->w/2, hh = img->h/2;
    uchar_t *p0, *p1;
    if (horizontal) {
      uchar_t temp[32];
      for (int y = 0; y < img->h; y++) {
	p0 = img->data + y * img->row_size ;
	p1 = img->data + y * img->row_size + (img->w-1) * img->pixel_size;
	for (int x = 0; x < hw; x++) {
	  memcpy( temp, p0, img->pixel_size );
	  memcpy( p0,   p1, img->pixel_size );
	  memcpy( p1, temp, img->pixel_size );
	  p0 += img->pixel_size;
	  p1 -= img->pixel_size;
	}
      }
    } else {
      uchar_t *temp = (uchar_t*)malloc(img->row_size);
      for (int y = 0; y < hh; y++) {
	p0 = img->data + y * img->row_size;
	p1 = img->data + (img->h - 1 - y) * img->row_size;
	memcpy( temp, p0, img->row_size );
	memcpy( p0,   p1, img->row_size );
	memcpy( p1, temp, img->row_size );
      }
      free(temp);
    }
    return true;
  }

  bool half(IMGH::Image *img, IMGH::Image *dst=NULL) {
    // Reduce the size of the image by half.
    // If 'dst==NULL', the 'img' will be changed as output.
    if (!img || img->w == 0 || img->h == 0) return false;
    IMGH::Image dummy;
    if (dst == NULL) dst = &dummy;
    int  i, j, y, x, idx, w=img->w/2, h=img->h/2, sr, sg, sb, sa;
    uchar_t pv[4];
    dst->setImage( w, h, img->type );
    switch (img->type) {
    case PIXEL_RGB: case PIXEL_BGR: case PIXEL_YUV:
      for (y = idx = 0; y < h; y++)
	for (x = 0; x < w; x++, idx++) {
	  for (i = 0, sr=sg=sb=0; i < 2; i++)	// get average color
	    for (j = 0; j < 2; j++) {
	      img->getPixel( 2*x + j, 2*y + i, pv );
	      sr += pv[0];  sg += pv[1];  sb += pv[2];
	    }
	  dst->setRGB( idx, (uchar_t)(sr/4), (uchar_t)(sg/4), (uchar_t)(sb/4) );
	}
      break;
    case PIXEL_RGBA:
      for (y = idx = 0; y < h; y++)
	for (x = 0; x < w; x++, idx++) {
	  for (i = 0, sr=sg=sb=sa=0; i < 2; i++)	// get average color
	    for (j = 0; j < 2; j++) {
	      img->getPixel( 2*x + j, 2*y + i, pv );
	      sr += pv[0];  sg += pv[1];  sb += pv[2];  sa += pv[3];
	    }
	  dst->setRGB  ( idx, (uchar_t)(sr/4), (uchar_t)(sg/4), (uchar_t)(sb/4) );
	  dst->setAlpha( idx, (uchar_t)(sa/4) );
	}
      break;
    case PIXEL_GRAY:
      for (y = idx = 0; y < h; y++)
	for (x = 0; x < w; x++, idx++) {
	  for (i = 0, sg=0; i < 2; i++)	// get average grayscale value
	    for (j = 0; j < 2; j++)  sg += img->getChar( 2*x + j, 2*y + i );
	  dst->setChar( idx, (uchar_t)(sg/4) );
	}
      break;
    case PIXEL_GRAYA:
      for (y = idx = 0; y < h; y++)
	for (x = 0; x < w; x++, idx++) {
	  for (i = 0, sg=sa=0; i < 2; i++)	// get average grayscale value
	    for (j = 0; j < 2; j++) {
	      img->getPixel( 2*x + j, 2*y + i, pv );
	      sg += pv[0];  sa += pv[1];
	    }
	  pv[0] = (uchar_t)(sg/4);  pv[1] = (uchar_t)(sa/4);
	  dst->setPixel( idx, pv );
	}
      break;
    default:
      std::cerr << "Error (IMGH::ImageEditor::half): invalid pixel type" << std::endl;
      return false;
    }
    if (dst == &dummy) {
      if (img->allocated) img->copyFrom( dst );
      else img->swapImage( dst );
    }
    return true;
  }

  void resize(IMGH::Image *img, int percent) {
    resize( img, (img->w * percent) / 100, (img->h * percent) / 100 );
  }
  void resize(IMGH::Image *img, int w, int h, IMGH::Image *dst=NULL) {
    int    x, y, idx;
    float nx, ny, xscale, yscale, scale;
    IMGH::Image dst_dummy;
    if (dst == NULL) dst = &dst_dummy;
    dst->setImage( w, h, img->type );
    xscale = (float) dst->w / (float) img->w;
    yscale = (float) dst->h / (float) img->h;
    scale = (xscale > yscale ? xscale : yscale);
    if      (xscale==1.00 && yscale==1.00) dst->copyFrom( img );
    else if (xscale==0.50 && yscale==0.50) half( img, dst );
    else if (xscale==0.25 && yscale==0.25) { half( img, dst ); half( dst ); }
    else {
      float xs = 1/xscale, ys = 1/yscale;
      for (y = idx = 0; y < dst->h; y++) {
	ny = (y + 0.5f) / yscale;
	for (x = 0; x < dst->w; x++, idx++) {
	  nx = (x + 0.5f) / xscale;
	  superSamplePixel( img, dst->getPixel(idx), nx, ny, xs, ys );
	}
      }
    }
    if (dst == &dst_dummy) {
      if (img->allocated) img->copyFrom( dst );
      else img->swapImage( dst );
    }
  }

  void* samplePixel(IMGH::Image *img, void *p, float x, float y) {
    // Sample pixel values at the given location with given pixel scale.
    //   The pixel scale is calculated by (new pixel size / old pixel size)
    //   Note that the range of 'x' and 'y' is [0,w] and [0,h].
    uchar_t *pv = (uchar_t*)p;
    uchar_t av[4], bv[4], cv[4], dv[4];
    int     x0 = int(x-0.5),  x1 = int(x-0.5)+1,  y0 = int(y-0.5),  y1 = int(y-0.5)+1;
    float   xr = (float)(x-0.5 - x0),  yr = (float)(y-0.5 - y0), abv[4], cdv[4], fv[4];
    if (x0 < 0) x0 = 0;  if (x1 > img->w-1) x1 = img->w-1;
    if (y0 < 0) y0 = 0;  if (y1 > img->h-1) y1 = img->h-1;
    switch (img->type) {
    case PIXEL_GRAY:  case PIXEL_GRAYA:  case PIXEL_RGBA:
    case PIXEL_RGB:  case PIXEL_BGR:  case PIXEL_YUV:
      img->getPixel( x0, y0, av );
      img->getPixel( x1, y0, bv );
      img->getPixel( x0, y1, cv );
      img->getPixel( x1, y1, dv );
      if (img->pixel_size == 1) {
	abv[0] = (1-xr) * av[0] + xr * bv[0];
	cdv[0] = (1-xr) * cv[0] + xr * dv[0];
	fv[0]  = (1-yr) * abv[0] + yr * cdv[0];
	pv[0] = (uchar_t)fv[0];
      } else if (img->pixel_size == 2) {
	IMGH2V_WADD( abv, (1-xr), av, xr, bv );
	IMGH2V_WADD( cdv, (1-xr), cv, xr, dv );
	IMGH2V_WADD( fv, (1-yr), abv, yr, cdv );
	pv[0] = (uchar_t)fv[0];  pv[1] = (uchar_t)fv[1];
      } else if (img->pixel_size == 3) {
	IMGH3V_WADD( abv, (1-xr), av, xr, bv );
	IMGH3V_WADD( cdv, (1-xr), cv, xr, dv );
	IMGH3V_WADD( fv, (1-yr), abv, yr, cdv );
	pv[0] = (uchar_t)fv[0];  pv[1] = (uchar_t)fv[1];  pv[2] = (uchar_t)fv[2];
      } else if (img->pixel_size == 4) {
	IMGH4V_WADD( abv, (1-xr), av, xr, bv );
	IMGH4V_WADD( cdv, (1-xr), cv, xr, dv );
	IMGH4V_WADD( fv, (1-yr), abv, yr, cdv );
	pv[0] = (uchar_t)fv[0];  pv[1] = (uchar_t)fv[1];  pv[2] = (uchar_t)fv[2];  pv[3] = (uchar_t)fv[3];
      }
      break;
    default: break;
    }
    return p;
  }

  void* superSamplePixel(IMGH::Image *img, void *p, float x, float y, float xs, float ys) {
    // Super-sample pixel values at the given location with X/Y scale.
    //   X/Y scale is given by the new pixel size.
    uchar_t *pv = (uchar_t*)p;
    float xs_half = (xs>0.5 ? xs/2 - 0.25f : 0.25f);
    float ys_half = (xs>0.5 ? xs/2 - 0.25f : 0.25f);
    float xr=1, yr=1, sum[4]={0,0,0,0};
    int   i, cnt=0, nch = img->pixel_size;
    for (xr = 0; xr < xs_half; xr += 0.5) {
      for (yr = 0; yr < ys_half; yr += 0.5) {
	if (xr == 0 && yr == 0) {
	  samplePixel( img, pv, x   , y    );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	} else if (xr == 0) {
	  samplePixel( img, pv, x   , y+yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	  samplePixel( img, pv, x   , y-yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	} else if (yr == 0) {
	  samplePixel( img, pv, x+xr, y    );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	  samplePixel( img, pv, x-xr, y    );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	} else {
	  samplePixel( img, pv, x+xr, y+yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	  samplePixel( img, pv, x+xr, y-yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	  samplePixel( img, pv, x-xr, y+yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	  samplePixel( img, pv, x-xr, y-yr );  for(i=0;i<nch;i++) sum[i]+=pv[i];  cnt++;
	}
      }
    }
    for (i=0; i<nch; i++) pv[i] = (uchar_t)(sum[i] / cnt);
    return p;
  }

  // -----------------------------------------------------------------
  // combining multiple images
  // -----------------------------------------------------------------
public:
  bool mixAt(IMGH::Image *dst, int dx, int dy,
	     IMGH::Image *src, int sx=0, int sy=0, int w=0, int h=0) {
    // Mix a region (sx,sy,w,h) of 'src' into 'dst' at (dx,dy) using alpha channel
    if (w <= 0) w = src->w;  if (h <= 0) h = src->h;
    if (dx + w >= dst->w) w = dst->w - dx;
    uchar_t *sp, *dp;
    float         alpha;
    if (src->type == PIXEL_GRAYA) {
      if (dst->type != PIXEL_GRAY) {
	std::cerr << "Error (IMGH::ImageEditor::mixAt): cannot mix GRAYA into non-GRAY" << std::endl;
	return false;
      }
      for (int y = 0; y < h; y++) {
	if (dy + y >= dst->h) break;
	for (int x = 0; x < w; x++) {
	  sp = src->data + (sy+y) * src->row_size + (sx+x) * src->pixel_size;
	  dp = dst->data + (dy+y) * dst->row_size + (dx+x) * dst->pixel_size;
	  alpha = sp[1] / 255.0f;
	  dp[0] = (uchar_t)(dp[0] * (1-alpha) + sp[0] * alpha);
	}
      }
    } else if (src->type == PIXEL_RGBA) {
      if (dst->type != PIXEL_RGB) {
	std::cerr << "Error (IMGH::ImageEditor::mixAt): cannot mix RGBA into non-RGB" << std::endl;
	return false;
      }
      for (int y = 0; y < h; y++) {
	if (dy + y >= dst->h) break;
	for (int x = 0; x < w; x++) {
	  sp = src->data + (sy+y) * src->row_size + (sx+x) * src->pixel_size;
	  dp = dst->data + (dy+y) * dst->row_size + (dx+x) * dst->pixel_size;
	  alpha = sp[3] / 255.0f;
	  dp[0] = (uchar_t)(dp[0] * (1-alpha) + sp[0] * alpha);
	  dp[1] = (uchar_t)(dp[1] * (1-alpha) + sp[1] * alpha);
	  dp[2] = (uchar_t)(dp[2] * (1-alpha) + sp[2] * alpha);
	}
      }
    } else {
      std::cerr << "Error (IMGH::ImageEditor::mixAt): cannot mix an image without alpha channel" << std::endl;
      return false;
    }
    return true;
  }
  bool diffAt(IMGH::Image *dst, int dx, int dy,
	      IMGH::Image *src, int sx=0, int sy=0, int w=0, int h=0) {
    // Subtract a region (sx,sy,w,h) of 'src' from 'dst' at (dx,dy).
    if (!dst || !src || src->type != dst->type) {
      std::cerr << "Error (IMGH::ImageEditor::diffAt): cannot diff images without different pixel types" << std::endl;
      return false;
    }
    IMGH::Image dummy;
    if (w <= 0) w = src->w;  if (h <= 0) h = src->h;
    uchar_t *sp, *dp;  float *sfp, *dfp;  int *sip, *dip;
    if (dx + w >= dst->w) w = dst->w - dx;
    for (int y = 0; y < h; y++) {
      if (dy + y >= dst->h) break;
      for (int x = 0; x < w; x++) {
	sp = src->data + (sy+y) * src->row_size + (sx+x) * src->pixel_size;
	dp = dst->data + (dy+y) * dst->row_size + (dx+x) * dst->pixel_size;
	switch (dst->type) {
	case PIXEL_GRAY: *dp = abs(*dp - *sp);  break;
	case PIXEL_RGB:  dp[0]=abs(dp[0]-sp[0]); dp[1]=abs(dp[1]-sp[1]); dp[2]=abs(dp[2]-sp[2]);  break;
	case PIXEL_BGR:  dp[0]=abs(dp[0]-sp[0]); dp[1]=abs(dp[1]-sp[1]); dp[2]=abs(dp[2]-sp[2]);  break;
	case PIXEL_YUV:  dp[0]=abs(dp[0]-sp[0]); dp[1]=abs(dp[1]-sp[1]); dp[2]=abs(dp[2]-sp[2]);  break;
	case PIXEL_INT:   sip = (int*)sp;  dip = (int*)dp;  *dip -= *sip;  break;
	case PIXEL_FLOAT: sfp = (float*)sp;  dfp = (float*)dp;  *dfp -= *sfp;  break;
	default: break;
	}
      }
    }
    return true;
  }
//  inline bool mergev(Image *a, Image *b, Image *result) { return merge(a, b, result, true); }
//  inline bool mergeh(Image *a, Image *b, Image *result) { return merge(a, b, result, false); }
  bool merge(IMGH::Image *a, IMGH::Image *b, IMGH::Image *result, bool vertically=true) {
    // Merge two images into a single image.
    // The output image 'result' should not be the same with 'a' or 'b',
    //    and it will be set in this function with appropriate size.
    if (!a || !b || !result) return false;
    if (vertically) {
      int w = (a->w > b->w ? a->w : b->w);
      int h = a->h + b->h;
      result->setImage( w, h, (a->type==b->type ? a->type : PIXEL_RGB), true );
      a->copySubregionTo( 0, 0, a->w, a->h, result, 0, 0 );
      b->copySubregionTo( 0, 0, b->w, b->h, result, 0, a->h );
    } else {
      int w = a->w + b->w;
      int h = (a->h > b->h ? a->h : b->h);
      result->setImage( w, h, (a->type==b->type ? a->type : PIXEL_RGB), true );
      a->copySubregionTo( 0, 0, a->w, a->h, result, 0,    (h - a->h)/2 );
      b->copySubregionTo( 0, 0, b->w, b->h, result, a->w, (h - b->h)/2 );
    }
    return true;
  }

  // -----------------------------------------------------------------
  // changing pixel values
  // -----------------------------------------------------------------
public:
  bool flood(IMGH::Image *img, int x, int y, int r, int g, int b, int tolerance) {
    if (img->type != PIXEL_RGB && img->type != PIXEL_RGBA) return false;
    if (x < 0 || y < 0 || x >= img->w || y >= img->h) return false;
    uchar_t old_rgb[4], new_rgb[3] = { r, g, b };
    img->getPixel(x, y, old_rgb);
    floodRecursive( img, x, y, old_rgb, new_rgb, tolerance );
    return true;
  }
  bool floodAlpha(IMGH::Image *img, int x, int y, int alpha_value, int tolerance) {
    if (img->type != PIXEL_GRAYA && img->type != PIXEL_RGBA) return false;
    if (x < 0 || y < 0 || x >= img->w || y >= img->h) return false;
    uchar_t old_rgb[4];
    img->getPixel(x, y, old_rgb);
    floodAlphaRecursive( img, x, y, old_rgb, alpha_value, tolerance );
    return true;
  }

private:
  void floodRecursive(IMGH::Image *img, int x, int y, uchar_t old_rgb[3],
		      uchar_t new_rgb[3], int tolerance) {
    if (x < 0 || y < 0 || x >= img->w || y >= img->h) return;
    uchar_t rgb[4];
    img->getPixel(x, y, rgb);
    if (abs(rgb[0] - old_rgb[0]) > tolerance) return;
    if (abs(rgb[1] - old_rgb[1]) > tolerance) return;
    if (abs(rgb[2] - old_rgb[2]) > tolerance) return;
    img->setRGB(x, y, new_rgb[0], new_rgb[1], new_rgb[2]);
    floodRecursive( img, x-1, y-1, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x-1, y+0, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x-1, y+1, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x+0, y-1, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x+0, y+1, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x+1, y-1, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x+1, y+0, old_rgb, new_rgb, tolerance );
    floodRecursive( img, x+1, y+1, old_rgb, new_rgb, tolerance );
  }
  void floodAlphaRecursive(IMGH::Image *img, int x, int y, uchar_t old_rgb[3],
			   int alpha_value, int tolerance) {
    if (x < 0 || y < 0 || x >= img->w || y >= img->h) return;
    uchar_t rgba[4];
    if (img->type == PIXEL_RGBA) {
      img->getPixel(x, y, rgba);
      if (abs(rgba[0] - old_rgb[0]) > tolerance) return;
      if (abs(rgba[1] - old_rgb[1]) > tolerance) return;
      if (abs(rgba[2] - old_rgb[2]) > tolerance) return;
      if (rgba[3] == alpha_value) return;
    } else if (img->type == PIXEL_GRAYA) {
      img->getPixel(x, y, rgba);
      if (abs(rgba[0] - old_rgb[0]) > tolerance) return;
      if (rgba[1] == alpha_value) return;
    } else return;
    img->setAlpha(x, y, alpha_value);
    floodAlphaRecursive( img, x-1, y-1, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x-1, y+0, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x-1, y+1, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x+0, y-1, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x+0, y+1, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x+1, y-1, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x+1, y+0, old_rgb, alpha_value, tolerance );
    floodAlphaRecursive( img, x+1, y+1, old_rgb, alpha_value, tolerance );
  }

  // -----------------------------------------------------------------
  // camera distortion / rectification
  // Invertible distortion model with single parameter
  //   let r^2 = (u-cc[0])^2 + (v-cc[1])^2, then
  //     ud - cc[0] = (u - cc[0]) / sqrt(1 + 2*dt[0] * r^2)
  //     vd - cc[1] = (v - cc[1]) / sqrt(1 + 2*dt[0] * r^2)
  //   In inverse, with rd^2 = (ud-u0)^2 + (vd-v0)^2,
  //     u - u0 = (ud - u0) / sqrt(1 - 2*k * rd^2)
  //     v - v0 = (vd - v0) / sqrt(1 - 2*k * rd^2)
  // -----------------------------------------------------------------
public:
  void rectify(IMGH::Image *src, IMGH::Image *dst, double cx, double cy, double dp, double *scale=NULL) {
    // Rectify the image.  If 'scale' is given, rectify the image
    //   with scaling to get maximum coverage with the same resolution.
    //   This means the intrinsic parameters (fx and fy) have to be scaled, as well.
    dst->setImage( src->w, src->h, src->type );
    double tdx, tdy, rr, tmp, x, y, hh=0, ratio=1.0;
    if (scale) {
      hh = (cy > src->h/2.0 ? src->h-cy : cy);
      ratio = 1.0 / sqrt( 1 - 2*dp*hh*hh );  // >= 1.0
    }
    unsigned char zero[4]={0,0,0,0}, value[4];  // should work for any pixel type
    for (int j = 0; j < dst->h; j++) {
      for (int i = 0; i < dst->w; i++) {
	if (scale) { tdx = (i - cx)*ratio;  tdy = (j - cy)*ratio; }
	else       { tdx = (i - cx);        tdy = (j - cy); }
	rr  = tdx * tdx + tdy * tdy;
	tmp = 1 + 2 * dp * rr;
	tmp = sqrt( tmp < 0.0001 ? 0.0001 : tmp );
	x = tdx / tmp + cx;   y = tdy / tmp + cy;
	if (x < 0 || x >= src->w || y < 0 || y >= src->h)
	  dst->setPixel( i, j, zero );
	else
	  dst->setPixel( i, j, samplePixel(src, value, (float)x, (float)y) );
      }
    }
    // save the scale factor for camera intrinsic parameters, fx and fy.
    if (scale) *scale = 1/ratio;  // [0,1]    new_fx = old_fx * (1/ratio)
  }

  void distort(IMGH::Image *src, IMGH::Image *dst, double cx, double cy, double dp) {
    dst->setImage( src->w, src->h, src->type );
    int i, j;
    unsigned char zero[4]={0,0,0,0}, value[4];  // should work for any pixel type
    for (j = 0; j < dst->h; j++) {
      for (i = 0; i < dst->w; i++) {
	double tdx, tdy, rr, tmp, x, y;
	tdx = i - cx;  tdy = j - cy;
	rr  = tdx * tdx + tdy * tdy;
	tmp = 1 - 2 * dp * rr;
	tmp = sqrt( tmp < 0.0001 ? 0.0001 : tmp );
	x = tdx / tmp + cx;   y = tdy / tmp + cy;
	if (x < 0 || x >= src->w || y < 0 || y >= src->h)
	  dst->setPixel( i, j, zero );
	else
	  dst->setPixel( i, j, samplePixel(src, value, (float)x, (float)y) );
      }
    }
  }

  // -----------------------------------------------------------------
  // chekcking
  // -----------------------------------------------------------------

  void checkSymmetry(IMGH::Image *img, IMGH::Image *dst, bool horizontal=true) {
    if (img->type != PIXEL_FLOAT) return;
    dst->copyFrom( img );
    flip( dst, horizontal );
    diffAt( dst, 0, 0, img );	// flip - img
    invert( dst );		// img - flip
  }


};


}	// namespace IMGH


#endif	// IMGH_IMAGE_EDITOR_HPP

