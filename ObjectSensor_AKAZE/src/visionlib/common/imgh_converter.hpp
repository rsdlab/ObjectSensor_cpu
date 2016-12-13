
//
// IMGH::ImageConverter
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

#ifndef IMGH_IMAGE_CONVERTER_HPP
#define IMGH_IMAGE_CONVERTER_HPP

#include <iostream>
#include <cmath>
#include "imgh_common.hpp"

namespace IMGH {

// -------------------------------------------------------------------

#define IMGH_RGB_GRAY(rgb, gray) \
   do { gray = ((6969*(int)(rgb)[0] + 23434*(int)(rgb)[1] + 2365*(int)(rgb)[2])/32768); } while(0)

#define IMGH_BGR_GRAY(bgr, gray) \
   do { gray = ((6969*(int)(bgr)[2] + 23434*(int)(bgr)[1] + 2365*(int)(bgr)[0])/32768); } while(0)

#define IMGH_RGB_YUV_YUV(r, g, b, y, u, v, yuv) \
   do { \
     y = (int)( 0.257 * r + 0.504 * g + 0.098 * b) +  16; \
     v = (int)( 0.439 * r - 0.368 * g - 0.071 * b) + 128; \
     u = (int)(-0.148 * r - 0.291 * g + 0.439 * b) + 128; \
     if (y < 0) y = 0;  if (y > 255) y = 255; \
     if (u < 0) u = 0;  if (u > 255) u = 255; \
     if (v < 0) v = 0;  if (v > 255) v = 255; \
     (yuv)[0] = y;  (yuv)[1] = u;  (yuv)[2] = v; \
   } while (0)

#define IMGH_YUV_RGB_RGB(y, u, v, r, g, b, rgb) \
   do { \
     b = (int)(1.164*(y - 16)                   + 2.018*(u - 128)); \
     g = (int)(1.164*(y - 16) - 0.813*(v - 128) - 0.391*(u - 128)); \
     r = (int)(1.164*(y - 16) + 1.596*(v - 128)); \
     if (r < 0) r = 0;  if (r > 255) r = 255; \
     if (g < 0) g = 0;  if (g > 255) g = 255; \
     if (b < 0) b = 0;  if (b > 255) b = 255; \
     (rgb)[0] = r;  (rgb)[1] = g;  (rgb)[2] = b; \
   } while (0)

#define IMGH_YUV_BGR_BGR(y, u, v, b, g, r, bgr) \
   do { \
     b = (int)(1.164*(y - 16)                   + 2.018*(u - 128)); \
     g = (int)(1.164*(y - 16) - 0.813*(v - 128) - 0.391*(u - 128)); \
     r = (int)(1.164*(y - 16) + 1.596*(v - 128)); \
     if (r < 0) r = 0;  if (r > 255) r = 255; \
     if (g < 0) g = 0;  if (g > 255) g = 255; \
     if (b < 0) b = 0;  if (b > 255) b = 255; \
     (bgr)[2] = r;  (bgr)[1] = g;  (bgr)[0] = b; \
   } while (0)

#define IMGH_YUYV_RGB_RGB(y1, u, y2, v, r, g, b, rgb) \
   do { \
     IMGH_YUV_RGB_RGB( y1, u, v, r, g, b, (rgb+0) ); \
     IMGH_YUV_RGB_RGB( y2, u, v, r, g, b, (rgb+3) ); \
   } while (0)

#define IMGH_YUYV_BGR_BGR(y1, u, y2, v, b, g, r, bgr) \
   do { \
     IMGH_YUV_BGR_BGR( y1, u, v, b, g, r, (bgr+0) ); \
     IMGH_YUV_BGR_BGR( y2, u, v, b, g, r, (bgr+3) ); \
   } while (0)

#define IMGH_YUYV_YUVYUV(y1, u, y2, v, yuvyuv) \
   do { \
     (yuvyuv)[0] = y1;  (yuvyuv)[1] = u;  (yuvyuv)[2] = v; \
     (yuvyuv)[3] = y2;  (yuvyuv)[4] = u;  (yuvyuv)[5] = v; \
   } while (0)

// -------------------------------------------------------------------

class ImageConverter
{
public:
  ImageConverter() {}
  ~ImageConverter() {}
public:
  bool convertImage(Image *src, Image *dst, pixel_t type=PIXEL_UNKNOWN) {
    if (!src || !dst) return false;
    if (type == PIXEL_UNKNOWN) type = dst->type;
    dst->setImage( src->w, src->h, type );
    return convertImage( src->data, dst->data, src->w, src->h, src->type, dst->type );
  }
  bool convertImage(void *src, void *dst, int w, int h, pixel_t src_type, pixel_t dst_type) {
    // Convert an image to a same image with another pixel type,
    //   assumming 'dst' has been set to appropriate size and pixel type.
    if (src == NULL || dst == NULL || src == dst) return false;
    uchar_t *sp = (uchar_t*)src, *dp = (uchar_t*)dst;
    float   *sfp = (float*)src,  *dfp = (float*)dst, minv, maxv;
    void    **spp = (void**)src;
    int     i, total=w*h, r, g, b, y, u, v, *dip = (int*)dst;
    switch (src_type) {
    case PIXEL_GRAY:
      switch (dst_type) {
      case PIXEL_GRAY: memcpy( dp, sp, 1 * total);  break;
      case PIXEL_GRAYA:for (i=0; i<total; i++,sp++,dp+=2) { dp[0] = sp[0]; dp[1] = 255; }  break;
      case PIXEL_RGB:  for (i=0; i<total; i++,sp++,dp+=3) dp[0] = dp[1] = dp[2] = *sp;  break;
      case PIXEL_BGR:  for (i=0; i<total; i++,sp++,dp+=3) dp[0] = dp[1] = dp[2] = *sp;  break;
      case PIXEL_YUV:  std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      case PIXEL_YUYV: std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      case PIXEL_FLOAT:for (i=0; i<total; i++,sp++,dfp++) dfp[0] = sp[0] / 255.0f;  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_RGB:
      switch (dst_type) {
      case PIXEL_GRAY: for (i=0; i<total; i++,sp+=3,dp+=1) IMGH_RGB_GRAY(sp, dp[0]);  break;
      case PIXEL_GRAYA:for (i=0; i<total; i++,sp+=3,dp+=2) { IMGH_RGB_GRAY(sp, dp[0]); dp[1] = 255; } break;
      case PIXEL_RGB:  memcpy( dp, sp, 3 * total);  break;
      case PIXEL_RGBA: for (i=0; i<total; i++,sp+=3,dp+=4) { dp[0] = sp[0]; dp[1] = sp[1]; dp[2] = sp[2]; dp[3] = 255; }  break;
      case PIXEL_BGR:  for (i=0; i<total; i++,sp+=3,dp+=3) { dp[0] = sp[2]; dp[1] = sp[2]; dp[2] = sp[0]; }  break;
      case PIXEL_YUV:  for (i=0; i<total; i++,sp+=3,dp+=3) IMGH_RGB_YUV_YUV( sp[0], sp[1], sp[2], y, u, v, dp );  break;
      case PIXEL_YUYV: std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      case PIXEL_INT:  for (i=0; i<total; i++,sp+=3,dip++) { dip[0] = (sp[0]+sp[1]+sp[2])/3; }  break;
      case PIXEL_FLOAT:for (i=0; i<total; i++,sp+=3,dfp++) { dfp[0] = ((float)sp[0]+sp[1]+sp[2]) / 3.0f / 255.0f; }  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_BGR:
      switch (dst_type) {
      case PIXEL_GRAY: for (i=0; i<total; i++,sp+=3,dp+=1) IMGH_BGR_GRAY(sp, dp[0]);  break;
      case PIXEL_GRAYA:for (i=0; i<total; i++,sp+=3,dp+=2) { IMGH_RGB_GRAY(sp, dp[0]); dp[1] = 255; } break;
      case PIXEL_RGB:  for (i=0; i<total; i++,sp+=3,dp+=3) { dp[0] = sp[2]; dp[1] = sp[2]; dp[2] = sp[0]; }  break;
      case PIXEL_RGBA: for (i=0; i<total; i++,sp+=3,dp+=4) { dp[0] = sp[2]; dp[1] = sp[2]; dp[2] = sp[0]; dp[3] = 255; } break;
      case PIXEL_BGR:  memcpy( dp, sp, 3 * total);  break;
      case PIXEL_YUV:  for (i=0; i<total; i++,sp+=3,dp+=3) IMGH_RGB_YUV_YUV( sp[2], sp[1], sp[0], y, u, v, dp );  break;
      case PIXEL_YUYV: std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_YUV:
      switch (dst_type) {
      case PIXEL_GRAY: for (i=0; i<total; i++,sp+=3,dp+=1) dp[0] = sp[0];  break;
      case PIXEL_RGB:  for (i=0; i<total; i++,sp+=3,dp+=3) IMGH_YUV_RGB_RGB( sp[0], sp[1], sp[2], r, g, b, dp );  break;
      case PIXEL_RGBA: for (i=0; i<total; i++,sp+=3,dp+=4) { IMGH_YUV_RGB_RGB( sp[0], sp[1], sp[2], r, g, b, dp ); dp[3] = 255; } break;
      case PIXEL_BGR:  for (i=0; i<total; i++,sp+=3,dp+=3) IMGH_YUV_BGR_BGR( sp[0], sp[1], sp[2], b, g, r, dp );  break;
      case PIXEL_YUV:  memcpy( dp, sp, 3 * total);  break;
      case PIXEL_YUYV: std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_YUYV:
      switch (dst_type) {
      case PIXEL_GRAY: for (i=0; i<total; i++, sp+=2,dp+=1) dp[0] = sp[0];  break;
      case PIXEL_RGB:  for (i=0; i<total; i+=2,sp+=4,dp+=6) IMGH_YUYV_RGB_RGB( sp[0], sp[1], sp[2], sp[3], r, g, b, dp ); break;
      case PIXEL_BGR:  for (i=0; i<total; i+=2,sp+=4,dp+=6) IMGH_YUYV_BGR_BGR( sp[0], sp[1], sp[2], sp[3], b, g, r, dp ); break;
      case PIXEL_YUV:  for (i=0; i<total; i+=2,sp+=4,dp+=6) IMGH_YUYV_YUVYUV ( sp[0], sp[1], sp[2], sp[3], dp ); break;
      case PIXEL_YUYV: memcpy( dp, sp, 2 * total);  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_GRAYA:
      switch (dst_type) {
      case PIXEL_GRAY:  for (i=0; i<total; i++,sp+=2,dp+=1) dp[0] = sp[0];  break;
      case PIXEL_GRAYA: memcpy( dp, sp, 2 * total);  break;
      case PIXEL_RGB:   for (i=0; i<total; i++,sp+=2,dp+=3) dp[0] = dp[1] = dp[2] = sp[0];  break;
      case PIXEL_RGBA:  for (i=0; i<total; i++,sp+=2,dp+=4) { dp[0] = dp[1] = dp[2] = sp[0]; dp[3] = sp[1]; } break;
      case PIXEL_FLOAT: for (i=0; i<total; i++,sp+=2,dfp++) { dfp[0] = (sp[0] / 255.0f) * (sp[1] / 255.0f); }  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_RGBA:
      switch (dst_type) {
      case PIXEL_GRAY:  for (i=0; i<total; i++,sp+=4,dp+=1) IMGH_RGB_GRAY( sp, dp[0] ); break;
      case PIXEL_GRAYA: for (i=0; i<total; i++,sp+=4,dp+=2) { IMGH_RGB_GRAY( sp, dp[0] ); dp[1] = sp[3]; } break;
      case PIXEL_RGB:   for (i=0; i<total; i++,sp+=4,dp+=3) IMGH3V_COPY( dp, sp ); break;
      case PIXEL_RGBA:  memcpy( dp, sp, 4 * total);  break;
      case PIXEL_INT:   for (i=0; i<total; i++,sp+=4,dip++) { dip[0] = (sp[0]+sp[1]+sp[2])/3; }  break;
      case PIXEL_FLOAT: for (i=0; i<total; i++,sp+=4,dfp++) { dfp[0] = ((float)sp[0]+sp[1]+sp[2])/3.0f/255.0f; }  break;
      case PIXEL_VOIDP: memcpy( dp, sp, 4 * total);  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_FLOAT:
      if (src) { IMGH::Image tmp(w, h, PIXEL_FLOAT, src);  tmp.getPixelStat(&minv, &maxv); }
      if (minv >= 0 && maxv <= 255) { minv = 0.0f;  maxv = (maxv <= 1.5f ? 1.0f : 255.0f); }
      switch (dst_type) {
      case PIXEL_GRAY:
	if      (minv <  0) convertFloatImageToGray( src, dst, w, h, minv, maxv );
	else if (maxv<=  1) convertFloatImageToGray( src, dst, w, h, 0.0f, 1.0f );
	else if (maxv<=256) convertFloatImageToGray( src, dst, w, h, 0.0f, 255  );
	else convertFloatImageToGray( src, dst, w, h, minv, maxv );
	break;
      case PIXEL_RGB:
	if      (maxv==  1) convertFloatImageToRGB( src, dst, w, h, "Grayscale", 0, 1  );
	else if (maxv==255) convertFloatImageToRGB( src, dst, w, h, "Grayscale", 0, 255 );
	else                convertFloatImageToRGB( src, dst, w, h, "Grayscale", minv, maxv );
	break;
      case PIXEL_FLOAT: memcpy( dfp, sfp, total * sizeof(float) );  break;
      case PIXEL_BGR:  std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      case PIXEL_YUV:  std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    case PIXEL_VOIDP:
      switch (dst_type) {
      case PIXEL_GRAY:
	for (i=0; i<total; i++, spp++, dp++) {
	  if (*spp == NULL) continue;
	  srand( (unsigned int)(long)(size_t)(*spp) );
	  dp[0] = 100 + rand()%156;
	}
	break;
      case PIXEL_RGB:
	for (i=0; i<total; i++, spp++, dp+=3) {
	  if (*spp == NULL) continue;
	  srand( (unsigned int)(long)(size_t)(*spp) );
	  dp[0] = 100+rand()%156;  dp[1] = 100+rand()%156;  dp[2] = 100+rand()%156;
	}
	break;
      case PIXEL_BGR:  std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      case PIXEL_YUV:  std::cerr << "Error (IMGH::ImageConverter::convert): Not implemented yet" << std::endl;  break;
      default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid DST pixel type" << std::endl;  return false;
      }
      break;
    default:  std::cerr << "Error (IMGH::ImageConverter::convert): invalid SRC pixel type" << std::endl;  return false;
    }
    return true;
  }

  bool convertFloatImageToGray(void *src, void *dst, int w, int h, float minv=0, float maxv=0) {
    // Convert a FLOAT image by mapping to grayscale values in the new image.
    int     i, total=w*h;
    float   *sp=(float*)src;
    uchar_t *dp=(uchar_t*)dst;
    if (minv>=maxv) { Image tmp(w, h, PIXEL_FLOAT, src); tmp.getPixelStat(&minv, &maxv); }
    for (i=0; i<total; i++, sp++, dp++)
      dp[0] = (int)(255 * (sp[0] - minv) / (maxv - minv));
    return true;
  }

  bool convertFloatImageToRGB(void *src, void *dst, int w, int h, char *mode, float minv=0, float maxv=0) {
    // Visualize the values of FLOAT image, using colors of a new RGB image.
    int     i, total=w*h, v;
    float   *sp=(float*)src;
    uchar_t *dp=(uchar_t*)dst;
    if (minv>=maxv) { Image tmp(w, h, PIXEL_FLOAT, src); tmp.getPixelStat(&minv, &maxv); }
    if (strcmp(mode,"Grayscale")==0) {
      for (i=0; i<total; i++, sp++, dp+=3) {
	v = (int)((*sp-minv)/(maxv-minv)*256 + 0.5f);
	dp[0] = dp[1] = dp[2] = (unsigned char)(v<0 ? 0 : (v>255 ? 255 : v));
      }
    } else if (strcmp(mode,"Rainbow")==0) {
      int nc, colors[6][3];
      colors[0][0] =   0;  colors[0][1] =   0;  colors[0][2] =   0;  // black
      colors[1][0] =   0;  colors[1][1] =   0;  colors[1][2] = 255;  // blue
      colors[2][0] =   0;  colors[2][1] = 255;  colors[2][2] =   0;  // green
      colors[3][0] = 255;  colors[3][1] =   0;  colors[3][2] =   0;  // red
      colors[4][0] = 255;  colors[4][1] = 255;  colors[4][2] =   0;  // yellow
      colors[5][0] = 255;  colors[5][1] = 255;  colors[5][2] = 255;  // white
      nc = 6;
      for (i=0; i<total; i++, sp++, dp+=3) {
	float ratio = (*sp - minv) / (maxv - minv);
	float step  = (float)(1.0 / (nc - 1));
	int   idx = (int)(ratio/step),  idx2 = (int)(ratio/step)+1;
	float off = (ratio - (idx * step)) / step;
	dp[0] = (unsigned char)((1-off) * colors[idx][0] + (off) * colors[idx2][0]);
	dp[1] = (unsigned char)((1-off) * colors[idx][1] + (off) * colors[idx2][1]);
	dp[2] = (unsigned char)((1-off) * colors[idx][2] + (off) * colors[idx2][2]);
      }
    } else if (strcmp(mode,"Sign")==0) {
      IMGH::Image tmp(w, h, PIXEL_RGB, dp); tmp.clearImage(0);
      float size = (fabs(maxv) > fabs(minv) ? fabs(maxv) : fabs(minv));
      for (i=0; i<total; i++, sp++, dp+=3) {
	v = (int)(fabs(*sp) / size * 255 + 0.5f);
	dp[ (*sp>=0 ? 0 : 2) ] = (v>255 ? 255 : v);
      }
    } else {
      fprintf(stderr, "Error: invalid conversion from FLOAT to RGB : '%s'\n", mode);
      return false;
    }
    return true;
  }

  bool removeAlphaChannel(Image *img) {
    int i, total = img->w * img->h, psize;
    uchar_t *src = img->data, *dst = img->data;
    switch (img->type) {
    case PIXEL_GRAYA:
      psize = img->getPixelSize(PIXEL_GRAY);  // 1
      for (i=0; i<total; i++, src+=2, dst+=1) dst[0] = src[0];
      if (img->allocated)
	img->data = (uchar_t*) realloc( img->data, total * psize );
      img->type = PIXEL_GRAY;
      img->pixel_size = img->getPixelSize(PIXEL_GRAY);
      img->row_size   = img->getPixelSize(PIXEL_GRAY) * img->w;
      break;
    case PIXEL_RGBA:
      psize = img->getPixelSize(PIXEL_RGB);  // 3
      for (i=0; i<total; i++, src+=4, dst+=3) memcpy( dst, src, psize );
      if (img->allocated)
	img->data = (uchar_t*) realloc( img->data, total * psize );
      img->type = PIXEL_RGB;
      img->pixel_size = img->getPixelSize(PIXEL_RGB);
      img->row_size   = img->getPixelSize(PIXEL_RGB) * img->w;
      break;
    default:  std::cerr << "Error (IMGH::ImageConverter::removeAlphaChannel): invalid pixel type" << std::endl;  break;
    }
    return true;
  }

};


}	// namespace IMGH


#endif	// IMGH_IMAGE_CONVERTER_HPP


// ===================================================================
#if 0	// start of the example code
// ===================================================================
#include <iostream>
#include "imgh_file_io.hpp"
#include "imgh_converter.hpp"
using namespace std;
int main(int argc, char **argv)
{
  // read the image from a file
  IMGH::Image  img;
  IMGH::FileIO ifile;
  ifile.readFile( argv[1], &img, IMGH::PIXEL_RGB );
  IMGH::Image  img2( img.w, img.h, IMGH::YUV );
  IMGH::ImageConverter conv;
  // converting pixel formats (RGB -> YUV)
  if (1) conv.convertImage( &img, &img2, IMGH::PIXEL_YUV );
  else   conv.convertImage( img.data, img2.data, img.w, img.h, IMGH::PIXEL_RGB, IMGH::PIXEL_YUV );
  img2.printInfo();
  // converting pixel formats (YUV -> RGBA)
  if (1) conv.convertImage( &img2, &img, IMGH::PIXEL_RGBA );
  else {
    img.setImage( img2.w, img2.h, IMGH::PIXEL_RGBA );
    conv.convertImage( img2.data, img.data, img.w, img.h, IMGH::PIXEL_YUV, IMGH::PIXEL_RGBA );
  }
  img.printInfo();
  // removing alpha channel
  conv.removeAlphaChannel( img );
  img.printInfo();
  return EXIT_SUCCESS;
}
// ===================================================================
#endif	// end of the example code
// ===================================================================
