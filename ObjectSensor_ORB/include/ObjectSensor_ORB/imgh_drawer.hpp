//
// IMGH::Drawer
//
// Jaeil Choi
// last modified in Nov, 2006
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
// Image coordinate : (0,0) +-------> X
//                          |       .
//                        Y V . . . .
//

#ifndef IMGH_IMAGE_DRAWING_HPP
#define IMGH_IMAGE_DRAWING_HPP

#include <iostream>
#include <cmath>
#include "imgh_common.hpp"

#ifndef ROUND
#define ROUND(a) ((int)(a+0.5f))
#endif
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

namespace IMGH {

template <class T_t>
class Drawer
{
public:
  int  r, g, b;
  T_t  line_thickness;
private:
  IMGH::Image *imgp, img_dummy;
public:
  Drawer(IMGH::Image *img) : r(255), g(255), b(255), line_thickness(1), imgp(NULL) {
    imgp = img;
  }
  Drawer(int w, int h, pixel_t type, void *buffer) : line_thickness(1) {
    img_dummy.setImage(w, h, type, buffer); imgp = &img_dummy;
    r = g = b = 255;
  }
  ~Drawer() { }

public:
  void setup(IMGH::Image *img) { imgp = img; }
  void setup(int w, int h, pixel_t type, void *buffer) {
    img_dummy.setImage(w, h, type, buffer); imgp = &img_dummy;
  }
  void setColor(int r, int g, int b) { this->r = r;  this->g = g;  this->b = b; }

public:
  void clearImage(uchar_t v=0) { imgp->clearImage(v); }

  void setPixel(int idx, int r, int g=0, int b=0) {
    switch (imgp->type) {
    case PIXEL_GRAY: imgp->setChar(idx, r);  break;
    case PIXEL_RGB:  imgp->setRGB (idx, r, g, b);  break;
    case PIXEL_BGR:  imgp->setRGB (idx, b, g, r);  break;
    default: break;
    }
  }

  // -----------------------------------------------------------------

  void drawPixel(T_t x, T_t y, int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    if (r < 0) { r = this->r;  g = this->g;  b = this->b; }
    int idx = imgp->w * (int)y + (int)x;
    if (check_bd) {
      if (x < 0 || x >= imgp->w) return;
      if (y < 0 || y >= imgp->h) return;
      switch (imgp->type) {
      case PIXEL_GRAY: imgp->setChar(idx, r);  break;
      case PIXEL_RGB:  imgp->setRGB (idx, r, g, b );  break;
      case PIXEL_BGR:  imgp->setRGB (idx, b, g, r );  break;
      case PIXEL_RGBA: imgp->setRGB (idx, r, g, b );  break;
      default: break;
      }
    } else {
      switch(imgp->type) {
      case PIXEL_GRAY: imgp->setChar(idx, r);  break;
      case PIXEL_RGB:  imgp->setRGB (idx, r, g, b );  break;
      case PIXEL_BGR:  imgp->setRGB (idx, b, g, r );  break;
      case PIXEL_RGBA: imgp->setRGB (idx, r, g, b );  break;
      default: std::cout << "Error (IMGH::Drawer::drawLine): invalid pixel type" << std::endl;  break;
      }
    }
  }

  // -----------------------------------------------------------------
  // Line
  // -----------------------------------------------------------------

  inline void drawLine(T_t p0[2], T_t p1[2], int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    drawLine( p0[0], p0[1], p1[0], p1[1], r, g, b, check_bd );
  }
  void drawLine(T_t x0, T_t y0, T_t x1, T_t y1, int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    int    i, d;
    double dx = (x1-x0), dy = (y1-y0), cx = x0, cy = y0, sx, sy;
    if (r < 0) { r = this->r;  g = this->g;  b = this->b; }
    if (fabs(dx) > fabs(dy)) { d = ROUNDI(fabs(dx));  sx = (dx>0 ? +1 : -1);  sy = dy/fabs(dx); }
    else                     { d = ROUNDI(fabs(dy));  sx = dx/fabs(dy);  sy = (dy>0 ? +1 : -1); }
    if (check_bd) {
      double x, y;
      for (i=0; i<=d; i++, cx+=sx, cy+=sy) {
	x=ROUNDI(cx);  if (x < 0 || x >= imgp->w) continue;
	y=ROUNDI(cy);  if (y < 0 || y >= imgp->h) continue;
	switch (imgp->type) {
	case PIXEL_GRAY: imgp->setChar((int)x, (int)y, r);  break;
	case PIXEL_RGB:  imgp->setRGB((int)x, (int)y, r, g, b );  break;
	case PIXEL_BGR:  imgp->setRGB((int)x, (int)y, b, g, r );  break;
	case PIXEL_RGBA: imgp->setRGB((int)x, (int)y, r, g, b );  break;
	default: std::cout << "Error (IMGH::Drawer::drawLine): invalid pixel type" << std::endl; break;
	}
      }
    } else {
      switch(imgp->type) {
      case PIXEL_GRAY: for (i=0; i<=d; i++, cx+=sx, cy+=sy) imgp->setChar(ROUNDI(cx), ROUNDI(cy), r);  break;
      case PIXEL_RGB:  for (i=0; i<=d; i++, cx+=sx, cy+=sy) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), r, g, b );  break;
      case PIXEL_BGR:  for (i=0; i<=d; i++, cx+=sx, cy+=sy) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), b, g, r );  break;
      case PIXEL_RGBA: for (i=0; i<=d; i++, cx+=sx, cy+=sy) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), r, g, b );  break;
      default: std::cout << "Error (IMGH::Drawer::drawLine): invalid pixel type" << std::endl;  break;
      }
    }
    if (line_thickness>1) {
      T_t dir[2];  IMGH2V_SET( dir, -(y1-y0), (x1-x0) );
      T_t len = IMGH2V_LEN (dir);  IMGH2V_SET( dir, dir[0]/len, dir[1]/len );
      T_t stt[2], end[2];
      T_t th, lth = line_thickness - 1;  line_thickness = 1;
      for (th = 0.25; th <= lth/2; th += 0.25) {
	IMGH2V_SET( stt, x0 + th/2 * dir[0], y0 + th/2 * dir[1] );
	IMGH2V_SET( end, x1 + th/2 * dir[0], y1 + th/2 * dir[1] );
	drawLine( stt[0], stt[1], end[0], end[1], r, g, b, check_bd );
      }
      for (th = 0.25; th <= lth/2; th += 0.25) {
	IMGH2V_SET( stt, x0 - th/2 * dir[0], y0 - th/2 * dir[1] );
	IMGH2V_SET( end, x1 - th/2 * dir[0], y1 - th/2 * dir[1] );
	drawLine( stt[0], stt[1], end[0], end[1], r, g, b, check_bd );
      }
      line_thickness = lth + 1;
    }
  }

  inline void drawDottedLine(T_t p0[2], T_t p1[2], int r=-1, int g=-1, int b=-1, int len=3) {
    drawDottedLine( p0[0], p0[1], p1[0], p1[1], r, g, b, len );
  }
  void drawDottedLine(T_t x0, T_t y0, T_t x1, T_t y1, int r=-1, int g=-1, int b=-1, int len=3) {
    int    i, d;
    double dx = (x1-x0), dy = (y1-y0), cx = x0, cy = y0, sx, sy;
    if (r < 0) { r = this->r;  g = this->g;  b = this->b; }
    if (fabs(dx) > fabs(dy)) { d = ROUNDI(fabs(dx));  sx = (dx>0 ? +1 : -1);  sy = dy/fabs(dx); }
    else                     { d = ROUNDI(fabs(dy));  sx = dx/fabs(dy);  sy = (dy>0 ? +1 : -1); }
    switch(imgp->type) {
    case PIXEL_GRAY: for (i=0; i<=d; i++, cx+=sx, cy+=sy) if (i/len%2==0) imgp->setChar(ROUNDI(cx), ROUNDI(cy), r);  break;
    case PIXEL_RGB:  for (i=0; i<=d; i++, cx+=sx, cy+=sy) if (i/len%2==0) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), r, g, b );  break;
    case PIXEL_BGR:  for (i=0; i<=d; i++, cx+=sx, cy+=sy) if (i/len%2==0) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), b, g, r );  break;
    case PIXEL_RGBA: for (i=0; i<=d; i++, cx+=sx, cy+=sy) if (i/len%2==0) imgp->setRGB(ROUNDI(cx), ROUNDI(cy), r, g, b );  break;
    default: std::cout << "Error (IMGH::Drawer::drawLine): invalid pixel type" << std::endl;  break;
    }
  }

  inline void drawTransparentLine(T_t p0[2], T_t p1[2], double opaque,
				  int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    drawTransparentLine( p0[0], p0[1], p1[0], p1[1], opaque, r, g, b, check_bd );
  }
  void drawTransparentLine(T_t x0, T_t y0, T_t x1, T_t y1, double opaque,
			   int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    if (imgp->type != PIXEL_RGB && imgp->type != PIXEL_RGBA) return;
    int    i, d;
    double dx = (x1-x0), dy = (y1-y0), cx = x0, cy = y0, sx, sy;
    if (r < 0) { r = this->r;  g = this->g;  b = this->b; }
    if (fabs(dx) > fabs(dy)) { d = ROUNDI(fabs(dx));  sx = (dx>0 ? +1 : -1);  sy = dy/fabs(dx); }
    else                     { d = ROUNDI(fabs(dy));  sx = dx/fabs(dy);  sy = (dy>0 ? +1 : -1); }
    if (check_bd) {
      double x, y;
      for (i=0; i<=d; i++, cx+=sx, cy+=sy) {
	x=ROUNDI(cx);  if (x < 0 || x >= imgp->w) continue;
	y=ROUNDI(cy);  if (y < 0 || y >= imgp->h) continue;
	imgp->mixRGB((int)x, (int)y, r, g, b, opaque );
      }
    } else {
      for (i=0; i<=d; i++, cx+=sx, cy+=sy) imgp->mixRGB(ROUNDI(cx), ROUNDI(cy), r, g, b, opaque );
    }
  }

  // -----------------------------------------------------------------
  // Triangle
  // -----------------------------------------------------------------

  void drawFilledTriangle(T_t p0[2], T_t p1[2], T_t p2[2], void *pvalue) {
    uchar_t *pv = (uchar_t*)pvalue;
    switch (imgp->pixel_size) {
    case 1:  drawFilledTriangle( p0, p1, p2, pv[0], -1,    -1,    -1    );  break;
    case 2:  drawFilledTriangle( p0, p1, p2, pv[0], pv[1], -1,    -1    );  break;
    case 3:  drawFilledTriangle( p0, p1, p2, pv[0], pv[1], pv[2], -1    );  break;
    case 4:  drawFilledTriangle( p0, p1, p2, pv[0], pv[1], pv[2], pv[3] );  break;
    default: break;
    }
  }
  void drawFilledTriangle(T_t p0[2], T_t p1[2], T_t p2[2], int r, int g, int b, int a=-1) {
    // Fill the triangle given by three points with pixel value 'pixval[]',
    //   assumming points are in CCW order.  (for any pixel types)
    // Note that if r,g,b,a is less than zero, the value in the channel will not be changed.
    if      (p1[1]<p0[1] && p1[1]<p2[1]) { T_t *tp=p0; p0=p1; p1=p2; p2=tp; }
    else if (p2[1]<p0[1] && p2[1]<p0[1]) { T_t *tp=p0; p0=p2; p2=p1; p1=tp; }
    int    w=imgp->w, h=imgp->h, x, y;
    double ymax=(p1[1]>p2[1] ? p1[1]:p2[1]), nstep, ychange=(p1[1]<p2[1] ? p1[1]:p2[1]);
    double y10=(p1[1]-p0[1]), y20=(p2[1]-p0[1]), lstep, rstep, lb, rb;
    if (y10 > 0) {
      lstep = (p1[0] - p0[0]) / (double)(p1[1] - p0[1]);// slope of left  boundary
      lb = p0[0] + lstep * (ROUND(p0[1])-p0[1]);	// position of left  boundary
    } else {
      lstep = (p2[1]-p1[1]==0 ? 0 : (p2[0]-p1[0])/(double)(p2[1]-p1[1]));
      lb = p1[0] + lstep * (ROUND(p0[1])-p0[1]-y10);
    }
    if (y20 > 0) {
      rstep = (p2[0] - p0[0]) / (double)(p2[1] - p0[1]);// slope of right boundary
      rb = p0[0] + rstep * (ROUND(p0[1])-p0[1]);	// position of right boundary
    } else {
      rstep = (p1[1]-p2[1]==0 ? 0 : (p1[0]-p2[0])/(double)(p1[1]-p2[1]));
      rb = p2[0] + rstep * (ROUND(p0[1])-p0[1]-y20);
    }
    //printf("drawFilledTriangle (%4.0f %4.0f) (%4.0f %4.0f) (%4.0f %4.0f) lstep=%g  rstep=%g\n",
    //       p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], lstep, rstep);
    for (y = (int)ROUND(p0[1]); y <= ymax && y < h;) {
      if (y > ychange && y <= ychange+1) {
	if        (y > p1[1]) {      // change the ratio of left boundary
	  nstep = (p2[1]-p1[1]==0 ? 0 : (p2[0]-p1[0])/(double)(p2[1]-p1[1]));
	  lb += (-lstep + nstep) * (y - ychange);
	  lstep = nstep;
	  //printf("  ychange at %d after (%.0f %.0f)  now b(%.0f %.0f) step=(%g %g)\n", y, p1[0], p1[1], lb, rb, lstep, rstep);
	} else if (y > p2[1]) {      // change the ratio of right boundary
	  nstep = (p1[1]-p2[1]==0 ? 0 : (p1[0]-p2[0])/(double)(p1[1]-p2[1]));
	  rb += (-rstep + nstep) * (y - ychange);
	  rstep = nstep;
	  //printf("  ychange at %d after (%.0f %.0f)  now b(%.0f %.0f) step=(%g %g)\n", y, p2[0], p2[1], lb, rb, lstep, rstep);
	}
	ychange = 1e6;
      }
      if (y >= 0) {
	uchar_t rgba[4];
	for (x = (int)(lb<0 ? 0 : ROUND(lb)); x <= rb && x < w; x++) {
	  imgp->getPixel(x, y, rgba);
	  if (r >= 0) rgba[0] = r;  if (g >= 0) rgba[1] = g;
	  if (b >= 0) rgba[2] = b;  if (a >= 0) rgba[3] = a;
	  imgp->setPixel(x, y, rgba);
	}
      }
      lb += lstep;  rb += rstep;  y++;  // update left/right boundary
    }
  }

  // -----------------------------------------------------------------
  // Rectangle
  // -----------------------------------------------------------------

  void drawRect(T_t x0, T_t y0, T_t x1, T_t y1,
		int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    drawLine( x0, y0,  x0, y1,  r, g, b,  check_bd );
    drawLine( x0, y1,  x1, y1,  r, g, b,  check_bd );
    drawLine( x1, y1,  x1, y0,  r, g, b,  check_bd );
    drawLine( x1, y0,  x0, y0,  r, g, b,  check_bd );
  }

  // -----------------------------------------------------------------
  // Quadrilateral
  // -----------------------------------------------------------------

  void drawQuad(T_t p0[2], T_t p1[2], T_t p2[2], T_t p3[2],
		int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    drawQuad( p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], r, g, b, check_bd );
  }
  void drawQuad(T_t x0, T_t y0, T_t x1, T_t y1, T_t x2, T_t y2, T_t x3, T_t y3,
		int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    drawLine( x0, y0,  x1, y1,  r, g, b,  check_bd );
    drawLine( x1, y1,  x2, y2,  r, g, b,  check_bd );
    drawLine( x2, y2,  x3, y3,  r, g, b,  check_bd );
    drawLine( x3, y3,  x0, y0,  r, g, b,  check_bd );
  }
  void drawFilledQuad(T_t p0[2], T_t p1[2], T_t p2[2], T_t p3[2], void *pvalue) {
    // Fill the quadrilateral given by four points with pixel value 'pixval[]',
    //   assumming points are in CCW order.  (for any pixel types)
    drawFilledTriangle( p0, p1, p2, pvalue );
    drawFilledTriangle( p0, p2, p3, pvalue );
  }

  void drawFilledQuad(T_t p0[2], T_t p1[2], T_t p2[2], T_t p3[2], int r, int g, int b, int a=-1) {
    // Fill the quadrilateral given by four points with pixel value 'pixval[]',
    //   assumming points are in CCW order.  (for any pixel types)
    drawFilledTriangle( p0, p1, p2, r, g, b, a );
    drawFilledTriangle( p0, p2, p3, r, g, b, a );
  }

  // -----------------------------------------------------------------
  // Arrow
  // -----------------------------------------------------------------

  void drawArrow(T_t x, T_t y, T_t len, T_t ori,
		 int r=-1, int g=-1, int b=-1, bool check_bd=true, bool with_head=true) {
    // Note that 'ori' is the angle from X axis in radian, in CCW direction.
    double stt[2], end[2], h1[2], h2[2], le2, or1, or2;
    le2 = len * 0.85;
    or1 = ori + M_PI/18.0;  or2 = ori - M_PI/18.0;  // orientation in radian
    IMGH2V_SET( stt, ROUNDI(x), ROUNDI(y) );
    IMGH2V_SET( end, ROUNDI( len*cos(ori)) + stt[0], ROUNDI(len*sin(-ori)) + stt[1] );
    IMGH2V_SET( h1,  ROUNDI( le2*cos(or1)) + stt[0], ROUNDI(le2*sin(-or1)) + stt[1] );
    IMGH2V_SET( h2,  ROUNDI( le2*cos(or2)) + stt[0], ROUNDI(le2*sin(-or2)) + stt[1] );
    drawLine( stt[0], stt[1], end[0], end[1], r, g, b,  check_bd );
    if (with_head) {
      drawLine( end[0], end[1],  h1[0], h1[1],  r, g, b,  check_bd );
      drawLine( end[0], end[1],  h2[0], h2[1],  r, g, b,  check_bd );
    }
  }

  void drawFilledArrow(T_t x, T_t y, T_t len, T_t ori,
		       int r=-1, int g=-1, int b=-1, bool check_bd=true) {
    // Note that 'ori' is the angle from X axis in radian, in CCW direction.
    double stt[2], end[2], nck[2], ort[2], le2;
    double st1[2], st2[2], ne1[2], ne2[2], hd1[2], hd2[2];
    le2 = len * 0.85;//ïœçXÇµÇΩå≥ílÇÕlen * 0.85
    IMGH2V_SET( stt, ROUNDI(x), ROUNDI(y) );
    IMGH2V_SET( end, ROUNDI( len*cos(ori)) + stt[0], ROUNDI(len*sin(-ori)) + stt[1] );
    IMGH2V_SET( nck, ROUNDI( le2*cos(ori)) + stt[0], ROUNDI(le2*sin(-ori)) + stt[1] );
    IMGH2V_SET( ort, sin(ori) * len * 0.02, +cos(ori) * len * 0.02 );
    IMGH2V_ADD( st1, stt, ort );IMGH2V_SUB( st2, stt, ort );
    IMGH2V_ADD( ne1, nck, ort );  IMGH2V_SUB( ne2, nck, ort );
    IMGH2V_SET( ort, sin(ori) * len * 0.06, +cos(ori) * len * 0.06 );
    IMGH2V_ADD( hd1, nck, ort );  IMGH2V_SUB( hd2, nck, ort );
    drawFilledQuad( st2, st1, ne1, ne2, r, g, b );
    drawFilledTriangle( hd2, hd1, end, r, g, b );
  }

};


}	// namespace IMGH


#endif	// IMGH_IMAGE_DRAWING_HPP
