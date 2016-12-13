
//
// IMGH::Image
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
// This file defines IMGH::Image class, which is
//   the encapsulation of an image (of any pixel format) and its functions.
//
// Image coordinate : (0,0) +-------> X
//                          |       .
//                        Y V . . . .
// Available functions :
//   void clear();			// clear the image settings, and destroy its buffer
//   void clearImage(255);		// set all the pixels with the given value
//   bool sameSize( &img2 );
//   bool sameSize( w, h, PIXEL_UNKNOWN );
//   void setImage( w, h, PIXEL_RGB );		// set image with new memory allocation
//   void setImage( w, h, PIXEL_RGB, buffer );	// set image with the given data buffer
//   void setPixel(x,y,values);	void* getPixel(x,y);
//   void setRGB (x,y,r,g,b);	void  getRGB (x,y,&r,&g,&b);
//   void setRGBA(x,y,r,g,b,a);	void  getRGBA(x,y,&r,&g,&b,&a);
//   void setInt  (x,y,value);	int     getInt  (x,y);
//   void setChar (x,y,value);	char    getChar (x,y);
//   void setUChar(x,y,value);	uchar_t getUChar(x,y);
//   void setFloat(x,y,value);	float   getFloat(x,y);
//   void setPointer(x,y,ptr);	void*   getPointer(x,y);
//   void setAlpha(x,y,a);	uchar_t getAlpha(x,y);
//   void copyFrom( &img2 );
//   void swapImage( &img2 );
//   void copyFromBuffer( w, h, PIXEL_GRAY, buffer );
//   void copyToBuffer( buffer );
//   bool copySubregionFrom( x, y, w, h, &src_image, src_x, src_y );
//   bool copySubregionTo  ( x, y, w, h, &dst_image, dst_x, dst_y );
//   bool getImageFromChannel( &src_image, 'A' );	// 'R', 'G', 'B', or 'A'
//   void getImageInfo( buffer );
//   void printInfo( "image A" );
//

#ifndef IMGH_IMAGE_HPP
#define IMGH_IMAGE_HPP

#ifdef WIN32
#include <windows.h>
#else
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#endif

#include <iostream>

#ifndef ROUNDI
#define ROUNDI(a)   ((int)((a)+0.5))
#endif

namespace IMGH {

#ifndef IMGH_VM_DEFS
#define IMGH_VM_DEFS
#define IMGH2V_GET( p, v0, v1 )  do { v0 = (p)[0]; v1 = (p)[1]; } while(0)
#define IMGH2V_SET( p, v0, v1 )  do { (p)[0] = v0; (p)[1] = v1; } while(0)
#define IMGH3V_GET( p, v0, v1, v2 )  do { v0 = (p)[0]; v1 = (p)[1]; v2 = (p)[2]; } while(0)
#define IMGH3V_SET( p, v0, v1, v2 )  do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; } while(0)
#define IMGH4V_GET( p, v0, v1, v2, v3 )  do { v0 = (p)[0]; v1 = (p)[1]; v2 = (p)[2]; v3 = (p)[3]; } while(0)
#define IMGH4V_SET( p, v0, v1, v2, v3 )  do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; (p)[3] = v3; } while(0)
#define IMGH5V_GET( p, v0, v1, v2, v3, v4 )  do { v0 = (p)[0]; v1 = (p)[1]; v2 = (p)[2]; v3 = (p)[3]; v4 = (0)[4]; } while(0)
#define IMGH5V_SET( p, v0, v1, v2, v3, v4 )  do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; (p)[3] = v3; (p)[4] = v4; } while(0)
#define IMGH2V_COPY( d, s )  do { (d)[0] = (s)[0]; (d)[1] = (s)[1]; } while(0)
#define IMGH3V_COPY( d, s )  do { (d)[0] = (s)[0]; (d)[1] = (s)[1]; (d)[2] = (s)[2]; } while(0)
#define IMGH4V_COPY( d, s )  do { (d)[0] = (s)[0]; (d)[1] = (s)[1]; (d)[2] = (s)[2]; (d)[3] = (s)[3]; } while(0)
#define IMGH2V_ADD( r, a, b )  do { (r)[0] = (a)[0]+(b)[0]; (r)[1] = (a)[1]+(b)[1]; } while(0)
#define IMGH3V_ADD( r, a, b )  do { (r)[0] = (a)[0]+(b)[0]; (r)[1] = (a)[1]+(b)[1]; (r)[2] = (a)[2]+(b)[2]; } while(0)
#define IMGH2V_SUB( r, a, b )  do { (r)[0] = (a)[0]-(b)[0]; (r)[1] = (a)[1]-(b)[1]; } while(0)
#define IMGH3V_SUB( r, a, b )  do { (r)[0] = (a)[0]-(b)[0]; (r)[1] = (a)[1]-(b)[1]; (r)[2] = (a)[2]-(b)[2]; } while(0)
#define IMGH2V_DOT( a, b )  ( (a)[0] * (b)[0] + (a)[1] * (b)[1] )
#define IMGH3V_DOT( a, b )  ( (a)[0] * (b)[0] + (a)[1] * (b)[1] + (a)[2] * (b)[2] )
#define IMGH2V_LEN( a )  sqrt( (a)[0] * (a)[0] + (a)[1] * (a)[1] )
#define IMGH3V_LEN( a )  sqrt( (a)[0] * (a)[0] + (a)[1] * (a)[1] + (a)[2] * (a)[2] )
#define IMGH2V_MUL( a, m )  do { (a)[0] *= m; (a)[1] *= m; } while(0)
#define IMGH3V_MUL( a, m )  do { (a)[0] *= m; (a)[1] *= m; (a)[2] *= m; } while(0)
#define IMGH2V_DIV( a, q )  do { (a)[0] /= q; (a)[1] /= q; } while(0)
#define IMGH3V_DIV( a, q )  do { (a)[0] /= q; (a)[1] /= q; (a)[2] /= q; } while(0)
#define IMGH2V_SADD( r, a, s, b )  do { (r)[0] = (a)[0]+(s)*(b)[0]; (r)[1] = (a)[1]+(s)*(b)[1]; } while(0)
#define IMGH3V_SADD( r, a, s, b )  do { (r)[0] = (a)[0]+(s)*(b)[0]; (r)[1] = (a)[1]+(s)*(b)[1]; (r)[2] = (a)[2]+(s)*(b)[2]; } while(0)
#define IMGH2V_WADD( r, sa, a, sb, b )  do { (r)[0] = (sa)*(a)[0]+(sb)*(b)[0]; (r)[1] = (sa)*(a)[1]+(sb)*(b)[1]; } while(0)
#define IMGH3V_WADD( r, sa, a, sb, b )  do { (r)[0] = (sa)*(a)[0]+(sb)*(b)[0]; (r)[1] = (sa)*(a)[1]+(sb)*(b)[1]; (r)[2] = (sa)*(a)[2]+(sb)*(b)[2]; } while(0)
#define IMGH4V_WADD( r, sa, a, sb, b )  do { (r)[0] = (sa)*(a)[0]+(sb)*(b)[0]; (r)[1] = (sa)*(a)[1]+(sb)*(b)[1]; (r)[2] = (sa)*(a)[2]+(sb)*(b)[2]; (r)[3] = (sa)*(a)[3]+(sb)*(b)[3]; } while(0)
#define IMGH2V_AVG( r, a, b )  do { (r)[0] = ((a)[0]+(b)[0])/2; (r)[1] = ((a)[1]+(b)[1])/2; } while(0)
#define IMGH3V_AVG( r, a, b )  do { (r)[0] = ((a)[0]+(b)[0])/2; (r)[1] = ((a)[1]+(b)[1])/2; (r)[2] = ((a)[2]+(b)[2])/2; } while(0)
#define IMGH2V_MIN( a, b )    ( (a) < (b) ? (a) : (b) )
#define IMGH3V_MIN( a, b, c ) ( (a) < (b) ? ((a) < (c) ? (a) : (c)) : ((b) < (c) ? (b) : (c)) )
#define IMGH2V_MAX( a, b )    ( (a) > (b) ? (a) : (b) )
#define IMGH3V_MAX( a, b, c ) ( (a) > (b) ? ((a) > (c) ? (a) : (c)) : ((b) > (c) ? (b) : (c)) )
#define IMGH2V_DIST( a, b )  ( sqrt(((a)[0]-(b)[0])*((a)[0]-(b)[0]) + ((a)[1]-(b)[1])*((a)[1]-(b)[1])) )
#define IMGH2V_LPDIST( l, p )  ( (l)[0] * (p)[0] + (l)[1] * (p)[1] + (l)[2] )
#endif

typedef unsigned char uchar_t;
typedef enum { PIXEL_UNKNOWN=0,	// default value (not meant for use)
	       PIXEL_GRAY,	// grayscale		(1 channel /pixel) (the smallest)
	       PIXEL_GRAYA,  	// grayscale + alpha	(2 channels/pixel)
	       PIXEL_RGB, 	// RGB			(3 channels/pixel)
	       PIXEL_RGBA,  	// RGB + alpha		(4 channels/pixel)
	       PIXEL_BGR, 	// BGR			(3 channels/pixel)
	       PIXEL_YUV,  	// YUV			(3 channels/pixel)
	       PIXEL_YUYV,  	// Y and alternating UV	(2 channels/pixel)
	       PIXEL_INT,  	// integer values	(1 channels/pixel)
	       PIXEL_FLOAT,  	// float values		(1 channels/pixel)
	       PIXEL_VOIDP,  	// void pointer		(1 channels/pixel)
	       PIXEL_F3V  	// float[3]		(3 channels/pixel)
}  pixel_t;

class Image
{
public:
  int		w, h;		// image width and height
  pixel_t	type;		// pixel format (GRAY, RGB, BGR, YUV)
  uchar_t	*data;		// pixel values
  bool		allocated;	// whether or not the pixel buffer was allocated
  int		row_size;	// the size of a row   in bytes
  int		pixel_size;	// the size of a pixel in bytes
public:
  Image () : w(0), h(0), type(PIXEL_UNKNOWN), data(NULL), allocated(false), row_size(0), pixel_size(0) { }
  Image (int w, int h, pixel_t type=PIXEL_RGB, void *buffer=NULL)
    : w(0), h(0), type(PIXEL_UNKNOWN), data(NULL), allocated(false), row_size(0), pixel_size(0) {
    if (buffer) setImage(w, h, type, buffer, false);
    else        setImage(w, h, type, false);
  }
  Image (Image *src) : w(0), h(0), data(NULL), allocated(false) { copyFrom(src); }
  ~Image() { if (allocated && data) free(data); }

public:
  int  getPixelSize(pixel_t type=PIXEL_UNKNOWN) {
    if (type == PIXEL_UNKNOWN) type = this->type;
    int size;		// in bytes
    switch(type) {
    case PIXEL_GRAY:  size = 1;  break;
    case PIXEL_RGB:   size = 3;  break;
    case PIXEL_BGR:   size = 3;  break;
    case PIXEL_RGBA:  size = 4;  break;
    case PIXEL_GRAYA: size = 2;  break;
    case PIXEL_YUV:   size = 3;  break;
    case PIXEL_YUYV:  size = 2;  break;
    case PIXEL_INT:   size = 1 * sizeof(int);  break;
    case PIXEL_FLOAT: size = 1 * sizeof(float);  break;
    case PIXEL_VOIDP: size = 1 * sizeof(void*);  break;
    case PIXEL_F3V:   size = 3 * sizeof(float);  break;
    default: size = 0;  break;
    }
    return size;
  }
  inline int  getIndex(int x, int y)       { return y * w + x; }
  inline uchar_t *operator()(int idx)      { return data + idx * pixel_size; }
  inline uchar_t *operator()(int x, int y) { return data + row_size * y + x * pixel_size; }
  inline uchar_t *pixel(int idx)           { return data + idx * pixel_size; }
  inline uchar_t *pixel(int x, int y)      { return data + row_size * y + x * pixel_size; }
  inline uchar_t *getRow(int y)            { return data + row_size * y; }
  void clear(void) { w = h = 0;  if (allocated && data) free(data);  data = NULL; }
  void clearImage(uchar_t v=0) { if (data) memset(data, v, w * h * pixel_size ); }
  void clearImage(int r, int g, int b, int a=255) {
    if      (type==PIXEL_RGB)  for (int i=0, total=w*h; i < total; i++) setRGB(i, r, g, b);
    else if (type==PIXEL_RGBA) for (int i=0, total=w*h; i < total; i++) setRGBA(i, r, g, b, a);
    else return;
  }
  void clearAlphaChannel(int a=255) { for (int i=0, total=w*h; i < total; i++) setAlpha(i, a); }
  bool sameSize(Image *img, bool check_type=false) {
    if (w != img->w || h != img->h) return false;
    if (!check_type) return true;
    else return (w==0 || type == img->type);
  }
  bool sameSize(int w, int h, pixel_t type=PIXEL_UNKNOWN) {
    if (this->w != w || this->h != h) return false;
    if (type == PIXEL_UNKNOWN) return true;
    else return (w == 0 || this->type == type);
  }

  // -----------------------------------------------------------------

  void setImage(int w, int h, pixel_t type=PIXEL_RGB, bool clear_image=false) {
    // Set up the image by allocating a new memory block (if necessary)
    // Note the buffer will be freed automatically when the image is destroyed.
    if (w <= 0 || h <= 0) { clear(); return; }
    if (!sameSize( w, h, type )) {
      clear();
      this->w = w;  this->h = h;  this->type = type;
      this->pixel_size = getPixelSize();  this->row_size = w * pixel_size;
      this->data = (uchar_t*)malloc( w * h * pixel_size );
      this->allocated = true;
    }
    if (clear_image) {
      clearImage();
      if (type==PIXEL_RGBA || type==PIXEL_GRAYA) clearAlphaChannel(255);
    }
  }
  void setImage( int w, int h, pixel_t type, void *buffer, bool clear_image=false) {
    // Set up the image using the given (fixed) memory buffer.
    // Note the buffer will NOT be freed when the image is destroyed.
    clear();
    if (w <= 0 || h <= 0 || buffer == NULL) return;//ƒGƒ‰[
    this->w = w;  this->h = h;  this->type = type;
    this->pixel_size = getPixelSize();  this->row_size = w * pixel_size;
    this->data = (uchar_t*)buffer;
    this->allocated = false;
    if (clear_image) {
      clearImage();
      if (type==PIXEL_RGBA || type==PIXEL_GRAYA) clearAlphaChannel(255);
    }
  }
  inline void setImageWithAllocatedBuffer( int w, int h, pixel_t type, void *buffer, bool clear_image=false) {
    // Set up the image using the given memory buffer.
    // Note the buffer will be freed when the image is destroyed.
    setImage( w, h, type, buffer, clear_image );
    this->allocated = true;
  }

  // -----------------------------------------------------------------
  // Note that x is column index and y is row index

  // for all pixel types
  inline void* getPixel(int x, int y) { return (void*)(data + (y*w+x)*pixel_size); }
  inline void* getPixel(int idx)      { return (void*)(data +  (idx) *pixel_size); }
  inline void getPixel(int x, int y, void *value) { uchar_t *src = data + (y*w+x)*pixel_size; memcpy(value, src, pixel_size); }
  inline void getPixel(int idx,      void *value) { uchar_t *src = data +   idx * pixel_size; memcpy(value, src, pixel_size); }
  inline void setPixel(int x, int y, void *value) { uchar_t *dst = data + (y*w+x)*pixel_size; memcpy(dst, value, pixel_size); }
  inline void setPixel(int idx,      void *value) { uchar_t *dst = data +   idx * pixel_size; memcpy(dst, value, pixel_size); }
  // for PIXEL_RGB, PIXEL_RGBA, PIXEL_BGR, PIXEL_YUV
  inline void setGray(int x, int y, int v) { *(data+(y*w+x)*pixel_size) = (uchar_t)v; }
  inline void setGray(int idx,      int v) { *(data+ (idx) *pixel_size) = (uchar_t)v; }
  inline void getRGB(int x, int y, int *r, int *g, int *b) { uchar_t *p = data+(y*w+x)*pixel_size; IMGH3V_GET( p, *r, *g, *b ); }
  inline void getRGB(int idx,      int *r, int *g, int *b) { uchar_t *p = data+ (idx) *pixel_size; IMGH3V_GET( p, *r, *g, *b ); }
  inline void setRGB(int x, int y, int  r, int  g, int  b) { uchar_t *p = data+(y*w+x)*pixel_size; IMGH3V_SET( p,  r,  g,  b ); }
  inline void setRGB(int idx,      int  r, int  g, int  b) { uchar_t *p = data+ (idx) *pixel_size; IMGH3V_SET( p,  r,  g,  b ); }
  inline void mixRGB(int x, int y, int  r, int  g, int  b, double op) { uchar_t *p = data+(y*w+x)*pixel_size; double val[3]={r,g,b}; IMGH3V_WADD( val, (1-op), p, op, val ); IMGH3V_SET( p, (uchar_t)val[0], (uchar_t)val[1], (uchar_t)val[2]); }
  inline void mixRGB(int idx,      int  r, int  g, int  b, double op) { uchar_t *p = data+ (idx) *pixel_size; double val[3]={r,g,b}; IMGH3V_WADD( val, (1-op), p, op, val ); IMGH3V_SET( p, (uchar_t)val[0], (uchar_t)val[1], (uchar_t)val[2]); }
  inline void getRGBA(int x, int y, int *r, int *g, int *b, int *a) { uchar_t *p = data+(y*w+x)*pixel_size; IMGH4V_GET( p, *r, *g, *b, *a ); }
  inline void getRGBA(int idx,      int *r, int *g, int *b, int *a) { uchar_t *p = data+ (idx) *pixel_size; IMGH4V_GET( p, *r, *g, *b, *a ); }
  inline void setRGBA(int x, int y, int  r, int  g, int  b, int  a) { uchar_t *p = data+(y*w+x)*pixel_size; IMGH4V_SET( p,  r,  g,  b,  a ); }
  inline void setRGBA(int idx,      int  r, int  g, int  b, int  a) { uchar_t *p = data+ (idx) *pixel_size; IMGH4V_SET( p,  r,  g,  b,  a ); }
  // for PIXEL_RGBA, PIXEL_GRAYA
  inline uchar_t getAlpha(int idx     ) { return data[ (idx+1)  * pixel_size - 1]; }
  inline uchar_t getAlpha(int x, int y) { return data[(w*y+x+1) * pixel_size - 1]; }
  inline void setAlpha(int idx,      uchar_t  v) { data[ (idx+1)  * pixel_size - 1] = v; }
  inline void setAlpha(int x, int y, uchar_t  v) { data[(w*y+x+1) * pixel_size - 1] = v; }
  // for non-graphic pixel images (PIXEL_INT, PIXEL_FLOAT, PIXEL_VOIDP, etc)
  inline int   getInt(int idx)      { return *(((int *)data)+ (idx));  }	// for PIXEL_INT only
  inline int   getInt(int x, int y) { return *(((int *)data)+(w*y+x)); }	// for PIXEL_INT only
  inline void  setInt(int idx,      int v) { *(((int *)data)+ (idx))  = v; }	// for PIXEL_INT only
  inline void  setInt(int x, int y, int v) { *(((int *)data)+(w*y+x)) = v; }	// for PIXEL_INT only
  inline float getFloat(int idx)        { return *(((float*)data)+ (idx)); }		// for PIXEL_FLOAT only
  inline float getFloat(int x, int y)   { return *(((float*)data)+(w*y+x)); }		// for PIXEL_FLOAT only
  inline void  setFloat(int idx,      float v) { *(((float*)data)+ (idx))  = v; }	// for PIXEL_FLOAT only
  inline void  setFloat(int x, int y, float v) { *(((float*)data)+(w*y+x)) = v; }	// for PIXEL_FLOAT only
  inline void* getPointer(int idx)        { return *(((void**)data)+ (idx)); }		// for PIXEL_VOIDP only
  inline void* getPointer(int x, int y)   { return *(((void**)data)+(w*y+x)); }		// for PIXEL_VOIDP only
  inline void  setPointer(int idx,      void* v) { *(((void**)data)+ (idx))  = v; }	// for PIXEL_VOIDP only
  inline void  setPointer(int x, int y, void* v) { *(((void**)data)+(w*y+x)) = v; }	// for PIXEL_VOIDP only
  inline char  getChar(int idx)      { return *(((char*)data)+ (idx)); }	// for PIXEL_GRAY only
  inline char  getChar(int x, int y) { return *(((char*)data)+(w*y+x)); }	// for PIXEL_GRAY only
  inline void  setChar(int idx,      char v) { *(((char*)data)+ (idx))  = v; }	// for PIXEL_GRAY only
  inline void  setChar(int x, int y, char v) { *(((char*)data)+(w*y+x)) = v; }	// for PIXEL_GRAY only
  inline uchar_t getUChar(int idx)      { return *(((uchar_t*)data)+ (idx)); }	// for PIXEL_GRAY only
  inline uchar_t getUChar(int x, int y) { return *(((uchar_t*)data)+(w*y+x)); }	// for PIXEL_GRAY only
  inline void    setUChar(int idx,      uchar_t v) { *(((uchar_t*)data)+ (idx))  = v; }	// for PIXEL_GRAY only
  inline void    setUChar(int x, int y, uchar_t v) { *(((uchar_t*)data)+(w*y+x)) = v; }	// for PIXEL_GRAY only
  inline int getGrayFromRGB(int idx)      { int r,g,b; getRGB(idx,&r,&g,&b); return ((6969*r + 23434*g + 2365*b)/32768); }
  inline int getGrayFromRGB(int x, int y) { int r,g,b; getRGB(x,y,&r,&g,&b); return ((6969*r + 23434*g + 2365*b)/32768); }

  // -----------------------------------------------------------------

  void copyFrom(IMGH::Image *img) {
    if (!img) return;
    setImage( img->w, img->h, img->type );
    memcpy( data, img->data, w * h * pixel_size );
  }
  void swapImage(IMGH::Image *img) {
    char tmp[sizeof(IMGH::Image)];
    memcpy( tmp, this, sizeof(IMGH::Image) );
    memcpy( this, img, sizeof(IMGH::Image) );
    memcpy( img,  tmp, sizeof(IMGH::Image) );
  }
  void copyFromBuffer(int w, int h, pixel_t type, uchar_t *buffer) {
    setImage( w, h, type );
    memcpy( this->data, buffer, pixel_size * w * h * sizeof(uchar_t) );
  }
  void copyToBuffer(uchar_t *buffer) {
    memcpy( buffer, this->data, pixel_size * w * h * sizeof(uchar_t) );
  }

  bool copySubregionFrom(int dx, int dy, int dw, int dh, IMGH::Image *src, int sx=0, int sy=0) {
    // Copy a region (sx,sy,w,h) of 'this' from 'src' starting at (dx,dy).
    // This function assumes 'this' image has been set already with desired size and type.
    if (!src || src->type != this->type || dw<=0 || dh<=0) return false;
    if (dx+dw > this->w) dw = this->w-dx;  if (sx+dw > src->w) dw = src->w-sx;
    if (dy+dh > this->h) dh = this->h-dy;  if (sy+dh > src->h) dh = src->h-sy;
    int      y, size = dw * this->pixel_size;
    uchar_t *dp = this->data + dy * this->row_size + dx * this->pixel_size;
    uchar_t *sp =  src->data + sy *  src->row_size + sx *  src->pixel_size;
    for (y = 0; y < dh; y++, dp += this->row_size, sp += src->row_size)
      memcpy ( dp, sp, size );
    return true;
  }
  bool copySubregionTo(int sx, int sy, int sw, int sh, IMGH::Image *dest, int dx=0, int dy=0) {
    // Copy a region (sx,sy,w,h) of 'this' to 'dest' at (dx,dy).
    // This function assumes 'dest' image has been set already with desired size and type.
    if (!dest || dest->type != this->type || sw<=0 || sh<=0) return false;
    if (sx+sw > this->w) sw = this->w-sx;  if (dx+sw > dest->w) sw = dest->w-dx;
    if (sy+sh > this->h) sh = this->h-sy;  if (dy+sh > dest->h) sh = dest->h-dy;
    int      y, size = sw * this->pixel_size;
    uchar_t *sp = this->data + sy * this->row_size + sx * this->pixel_size;
    uchar_t *dp = dest->data + dy * dest->row_size + dx * dest->pixel_size;
    for (y = 0; y < sh; y++, sp += this->row_size, dp += dest->row_size)
      memcpy ( dp, sp, size );
    return true;
  }

  bool getImageFromChannel(Image *src, char ch='A') {
    if (!src) return false;
    setImage( src->w, src->h, PIXEL_GRAY );
    uchar_t *sp = src->data;
    int   i, total = src->w * src->h, s = src->pixel_size, k;
    if      (src->type==PIXEL_GRAY)  k = 0;
    else if (src->type==PIXEL_GRAYA) k = (ch=='A' ? 1 : 0);
    else if (src->type==PIXEL_RGB)   k = (ch=='R' ? 0 : (ch=='G' ? 1 : 2));
    else if (src->type==PIXEL_RGBA)  k = (ch=='R' ? 0 : (ch=='G' ? 1 : (ch=='B' ? 2 : 3)));
    else k = 0;
    for (i=0; i<total; i++, sp+=s) this->data[i] = sp[k];
    return true;
  }

  void setPixelsChecker(int size, int br=100, int bg=100, int bb=100, int wr=156, int wg=156, int wb=156) {
    // draw checker pattern
    uchar_t pv[2][4]={{br,bg,bb,255},{wr,wg,wb,255}};
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
	setPixel(x, y, (((x/size)%2 + (y/size)%2)%2 == 0 ? pv[0] : pv[1]));
  }

  // -----------------------------------------------------------------
public:
  void getPixelStat(void *minp, void *maxp, float *alpha_per=NULL) {
    int i, total = w * h;
    uchar_t *src = data, *minv = (uchar_t*)minp, *maxv = (uchar_t*)maxp;
    int   *srci = (int*)data,   *mini = (int*)minp,   *maxi = (int*)maxp;
    float *srcf = (float*)data, *minf = (float*)minp, *maxf = (float*)maxp;
    double alpha_sum=0.0;
    switch(type) {
    case PIXEL_GRAY:
      *minv = 255;  *maxv = 0;
      for (i=0; i<total; i++, src++) {
	if (*src > *maxv) *maxv = *src;  if (*src < *minv) *minv = *src;
      }
      break;
    case PIXEL_GRAYA:
      *minv = 255;  *maxv = 0;
      for (i=0; i<total; i++, src+=2) {
	if (*src > *maxv) *maxv = *src;  if (*src < *minv) *minv = *src;
	alpha_sum += src[1]/255.0;
      }
      break;
    case PIXEL_RGB:
    case PIXEL_BGR:
    case PIXEL_YUV:
      IMGH3V_SET( minv, 255, 255, 255 );  IMGH3V_SET( maxv, 0, 0, 0 );
      for (i=0; i<total; i++, src+=3) {
	if (src[0] > maxv[0]) maxv[0] = src[0];  if (src[0] < minv[0]) minv[0] = src[0];
	if (src[1] > maxv[1]) maxv[1] = src[1];  if (src[1] < minv[1]) minv[1] = src[1];
	if (src[2] > maxv[2]) maxv[2] = src[2];  if (src[2] < minv[2]) minv[2] = src[2];
      }
      break;
    case PIXEL_RGBA:
      IMGH3V_SET( minv, 255, 255, 255 );  IMGH3V_SET( maxv, 0, 0, 0 );
      for (i=0; i<total; i++, src+=4) {
	if (src[0] > maxv[0]) maxv[0] = src[0];  if (src[0] < minv[0]) minv[0] = src[0];
	if (src[1] > maxv[1]) maxv[1] = src[1];  if (src[1] < minv[1]) minv[1] = src[1];
	if (src[2] > maxv[2]) maxv[2] = src[2];  if (src[2] < minv[2]) minv[2] = src[2];
	alpha_sum += src[3]/255.0;
      }
      break;
    case PIXEL_INT:
      *mini = 65535;  *maxi = -65535;
      for (i=0; i<total; i++, srci++) {
	if (*srci > *maxi) *maxi = *srci;  if (*srci < *mini) *mini = *srci;
      }
      break;
    case PIXEL_FLOAT:
      *minf = +1e9;  *maxf = -1e9;
      for (i=0; i<total; i++, srcf++) {
	if (*srcf > *maxf) *maxf = *srcf;  if (*srcf < *minf) *minf = *srcf;
      }
      break;
    default:  break;
    }
    if (alpha_per) *alpha_per = (float)(alpha_sum * 100 / (w * h));
  }

public:
  void getImageInfo(char buf[]) {
    // Get image information.  Note that buf[] must be larger than 120
    if (!buf) return;
    buf[0] = '\0';
    char *type_str=NULL;
    switch(type) {
    case PIXEL_GRAY:  strcpy(type_str, "GRAY "); break;
    case PIXEL_RGB:   strcpy(type_str, "RGB  "); break;
    case PIXEL_BGR:   strcpy(type_str, "BGR  "); break;
    case PIXEL_RGBA:  strcpy(type_str, "RGBA "); break;
    case PIXEL_GRAYA: strcpy(type_str, "GRAYA"); break;
    case PIXEL_YUV:   strcpy(type_str, "YUV  "); break;
    case PIXEL_YUYV:  strcpy(type_str, "YUYV "); break;
    case PIXEL_INT:   strcpy(type_str, "INT  "); break;
    case PIXEL_FLOAT: strcpy(type_str, "FLOAT"); break;
    case PIXEL_VOIDP: strcpy(type_str, "VOID*"); break;
    default:          strcpy(type_str, "---- "); break;
    }
    if (!data) { sprintf( buf, "(%4d x %4d) %s(%d)  data==NULL  ", w, h, type_str, pixel_size); return; }
    int		i, total = w * h, count=0, *ip;
    uchar_t	minv[3], maxv[3], *cp;
    float	minf, maxf, *fp, alpha_per=0;
    int		mini, maxi;
    void **tmp = (void**)data;
    switch(type) {
    case PIXEL_GRAY:    case PIXEL_GRAYA:
      getPixelStat( minv, maxv, &alpha_per );
      for (i=0, cp = (uchar_t*)data; i<total; i++) if (cp[i]!=0) count++;
      sprintf(buf,"(%4d x %4d) %s(%d)  min=%3d  max=%3d  NonZero=%d(%d%%)", w, h, type_str, pixel_size, minv[0], maxv[0], count, count*100/total);
      if (type==PIXEL_GRAYA) sprintf(buf+strlen(buf), "a=%.0f%%  ", alpha_per);
      break;
    case PIXEL_RGB:   case PIXEL_BGR:   case PIXEL_YUV:   case PIXEL_RGBA:
      getPixelStat( minv, maxv, &alpha_per );
      sprintf(buf,"(%4d x %4d) %s(%d)  min[3]=(%3d %3d %3d) max[3]=(%3d %3d %3d)  ", w, h, type_str, pixel_size, minv[0], minv[1], minv[2], maxv[0], maxv[1], maxv[2]);
      if (type==PIXEL_RGBA) sprintf(buf+strlen(buf), "a=%.0f%%  ", alpha_per);
      break;
    case PIXEL_INT:
      getPixelStat( &mini, &maxi );
      for (i=0, ip = (int*)data; i<total; i++) if (ip[i]!=0) count++;
      sprintf(buf,"(%4d x %4d) %s(%d)  min=%3d  max=%3d  NonZero=%d(%d%%)", w, h, type_str, pixel_size, mini, maxi, count, count*100/total);
      break;
    case PIXEL_FLOAT:
      getPixelStat( &minf, &maxf );
      for (i=0, fp = (float*)data; i<total; i++) if (fp[i]!=0) count++;
      sprintf(buf,"(%4d x %4d) %s(%d)  min=%g  max=%g  NonZero=%d(%d%%)", w, h, type_str, pixel_size, minf, maxf, count, count*100/total);
      break;
    case PIXEL_VOIDP:
      for (i=0; i<total; i++, tmp++) if (tmp[0]) count++;
      sprintf(buf,"(%4d x %4d) %s(%d)  NonNullPixels = %d (%.0f%%)  ", w, h, type_str, pixel_size, count, count*100/(float)total);
      break;
    default:
      sprintf(buf,"(%4d x %4d) Invalid pixel type", w, h);
      break;
    }
  }

  void printInfo(char *comment=NULL) {
    // Print information for debugging.
    char buf[160];
    getImageInfo( buf );
    printf("%s %s\n", (comment ? comment : "IMG"), buf );
  }
};


}	// namespace IMGH


#endif	// IMGH_IMAGE_HPP


// ===================================================================
#if 0	// start of the example code
// ===================================================================
#include <iostream>
#include "imgh_common.hpp"
#include "imgh_file_io.hpp"	// only for IMGH::FileIO
using namespace std;
int main(int argc, char **argv)
{
  IMGH::Image  img;
  // create a new image
  img.setImage( 320, 240, IMGH::PIXEL_GRAY );
  img.clearImage( 255 );  // all white image
  // setup an image using existing pixel buffer
  img.setImage( 320, 240, IMGH::PIXEL_RGB, your_buffer );
  // read the image from a file
  IMGH::FileIO ifile;
  ifile.readFile( argv[1], &img );
  // copying
  IMGH::Image img2;
  img2.copyFrom( &img );
  img2.copyFromBuffer( img.w, img.h, IMGH::PIXEL_RGB, img.data );
  img2.copyToBuffer( img.w, img.h, IMGH::PIXEL_RGB, img.data );
  // swapping
  img2.swap( &img );
  // accessing each pixel
  IMGH::Image img3( img.w, img.h, IMGH::PIXEL_FLOAT );  // scalar field!
  unsigned char rgb[3];
  for (int i=0; i<img2.w*img2.h; i++) {
    img2.getRGB  ( i, rgb );
    img3.setFloat( i, rgb[0]/255.0f );
  }
  // print information
  img2.printInfo();
  img3.printInfo();
  return EXIT_SUCCESS;
}
// ===================================================================
#endif	// end of the example code
// ===================================================================
