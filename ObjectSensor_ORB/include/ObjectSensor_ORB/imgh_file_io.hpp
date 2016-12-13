//
// IMGH::FileIO
//
// Jaeil Choi
// last modified in June, 2007
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
// IMGH::FileIO is the only class in the IMGH library, which depends
//   on specific libraries of each Operating System (Window/Linux/Mac).
//   Therefore, you need to set appriate compiler options when you
//   use IMGH::FileIO in your code.
//
// To compile on Windows,
//   - use Visual Studio 8.0 or higher (for GDI+)
//   - add 'src' directory to Project->Properties->C/C++->General->AdditionalIncludeDirectories
//   - add '_CRT_SECURE_NO_DEPRECATE' to Project->Properties->C/C++->Preprocessor->PreprocessorDefinitions
//   - add 'gdiplus.lib' to Project->Properties->Linker->Input->AddtionalDependencies
// To compile on Apple OS X,
//   - use g++ compiler flags : '-framework Carbon -framework AGL'
//

#ifndef IMGH_FILE_IO_HPP
#define IMGH_FILE_IO_HPP

#include <stdio.h>
#include <iostream>
#include <cmath>
#include "imgh_common.hpp"
#include "imgh_converter.hpp"


// ===================================================================
#ifdef WIN32  		// Windows version
// ===================================================================


#include <windows.h>
#include <windowsx.h>
#include <gdiplus.h>

#ifndef GDIPLUS_DATA
#define GDIPLUS_DATA
static ULONG_PTR			gdiplusToken;
static Gdiplus::GdiplusStartupInput	gdiplusStartupInput;
static bool				gdiplusStarted = false;
static bool getEncoderClsid(char* fname, CLSID *pClsid)
{
	if (fname == NULL) return false;
	int len = (int)strlen(fname);
	if (len < 4 || fname[len-4] != '.') return false;
	wchar_t *format;
	char *ext = fname + strlen(fname)-3;
	if      (strcmp(ext, "png")==0 || strcmp(ext, "PNG")==0) format = L"image/png";
	else if (strcmp(ext, "jpg")==0 || strcmp(ext, "JPG")==0) format = L"image/jpg";
	else if (strcmp(ext, "bmp")==0 || strcmp(ext, "BMP")==0) format = L"image/bmp";
	else return false;
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes
	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;
	Gdiplus::GetImageEncodersSize(&num, &size);
	if(size == 0) return false;
	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
	if(pImageCodecInfo == NULL) return false;
	GetImageEncoders(num, size, pImageCodecInfo);
	for(UINT j = 0; j < num; ++j)
	  if( wcscmp(pImageCodecInfo[j].MimeType, format) == 0 ) {
		 *pClsid = pImageCodecInfo[j].Clsid;
		 free(pImageCodecInfo);  return true;
	  }
	free(pImageCodecInfo);
	return false;
}
#endif //GDIPLUS_DATA


namespace IMGH {

class FileIO
{
public:
  FileIO() {}
  ~FileIO() {}

public:
  Image* readFile(const char *fname, Image *img=NULL, pixel_t type=PIXEL_UNKNOWN) {
    if (!gdiplusStarted) {
      Gdiplus::GdiplusStartup( &gdiplusToken, &gdiplusStartupInput, NULL );
      gdiplusStarted = true;
    }
    if (!fname) return false;
    wchar_t wfname[512];
    mbstowcs( wfname, fname, (int)strlen(fname)+1 );
    Gdiplus::Bitmap bitmap(wfname);
    if (bitmap.GetWidth() <= 0) return NULL;
    if (img == NULL) img = new IMGH::Image;
    img->setImage( bitmap.GetWidth(), bitmap.GetHeight(), IMGH::PIXEL_RGB );
    Gdiplus::Rect rect(0, 0, bitmap.GetWidth(), bitmap.GetHeight());
    Gdiplus::BitmapData bitmapData;
    bitmap.LockBits( &rect, Gdiplus::ImageLockModeRead, PixelFormat24bppRGB, &bitmapData );
    char *src = (char*)bitmapData.Scan0, *dst = (char*)img->data;
    for (int row = 0; row < img->h; row++) {
      memcpy( dst, src, img->row_size );
      src += bitmapData.Stride;  dst += img->row_size;
    }
    if (type != PIXEL_UNKNOWN && type != PIXEL_RGB) {
      Image tmp( img->w, img->h, type );
      ImageConverter conv;
      conv.convertImage( img->data, tmp.data, img->w, img->h, PIXEL_RGB, type );
      img->swapImage( &tmp );
    }
    bitmap.UnlockBits( &bitmapData );
    return img;
  }

  bool writeFile(const char *fname, Image *img, int opt=0) {
    if (!fname || !img || !img->data) return false;
    if (!gdiplusStarted) {
      Gdiplus::GdiplusStartup( &gdiplusToken, &gdiplusStartupInput, NULL );
      gdiplusStarted = true;
    }
    if (img->w % 4 != 0) {
      std::cerr << "Error: failed to save image. Its width must be a multiple of four." << std::endl;
      std::cerr << "       This is a restriction given by Windows GDI+ library." << std::endl;
      return false;
    }
    wchar_t wfname[512];
    mbstowcs( wfname, fname, (int)strlen(fname)+1 );
    CLSID encoderClsid;  if (!getEncoderClsid( (char*)fname, &encoderClsid )) return false;
    Gdiplus::Status stat;
    switch (img->type) {
    case PIXEL_RGB: {
      Gdiplus::Bitmap bitmap(img->w, img->h, img->row_size, PixelFormat24bppRGB, img->data);
      stat = bitmap.Save( wfname, &encoderClsid, NULL );
    } break;
    default: return false;
    }
    return (stat==0 ? true : false);
  }

  char* getFileExtension(char *filename) {
    int  i;  // find the beginning of the file extension of 'filename'
    for (i=(int)strlen(filename); i>0 && filename[i]!='.'; i--);
    return (i>0 ? filename+i+1 : NULL);
  }
  char* getFileBaseName(const char *filename, char *out, bool without_path=true) {
    int  i=0, j, k, len = (int)strlen(filename);
    if (without_path) { for (i=len-1; i>=0; i--) if (filename[i]=='/') break;  i++; }
    for (j=len; j>i && filename[j]!='.'; j--);
    j = (j>i ? j : len);
    if (out) { for (k=i; k<j; k++) out[k-i] = filename[k]; out[k-i] = '\0'; }
    return out;
  }
  char* getFilePath(const char *filename, char *out) {
    int  i=0, j, len = (int)strlen(filename);
    for (i=len-1; i>=0; i--) if (filename[i]=='/') break;
    for (j=0; j <= i; j++) out[j] = filename[j];  out[j] = '\0';
    return out;
  }

};

}	// namespace IMGH


// ===================================================================
#elif defined(__APPLE__) || defined(MACOSX)	// OS X version
// ===================================================================

#include <iostream>
#include <Carbon/Carbon.h>
#include <ApplicationServices/ApplicationServices.h>
#include "imgh_converter.hpp"

namespace IMGH {

class FileIO
{
public:
  FileIO() {}
  ~FileIO() {}

public:
  Image* readFile(const char *fname, Image *img=NULL, pixel_t type=PIXEL_UNKNOWN) {
    // set an image source using a URL of the pathname, and
    // create an CGImage from the first item in the image source.
    //   Note that a CGImage is an abstract object and its bits may not be readily available.
    const char *ext = getFileExtension((const char*)fname);
    if (ext && (strncmp(ext,"ppm",3)==0 || strncmp(ext,"pgm",3)==0))
      return readPXMFile(fopen(fname, "rb"), img);
    CFURLRef url = CFURLCreateFromFileSystemRepresentation( NULL, (const UInt8*)fname, strlen(fname), false );
    if (!url) return NULL;
    CGImageSourceRef isrc = CGImageSourceCreateWithURL(url, NULL);
    CFRelease( url );    if (isrc == NULL) return NULL;
    CGImageRef imgref = CGImageSourceCreateImageAtIndex(isrc, 0, NULL);
    CFRelease(isrc);  if (imgref == NULL) return NULL;
    int  bpp = CGImageGetBitsPerPixel(imgref);
    int  bpc = CGImageGetBitsPerComponent(imgref);
    bool gray  = (bpp/bpc == 1 || bpp/bpc == 2);
    bool alpha = (CGImageGetAlphaInfo(imgref) != kCGImageAlphaNone);
    // create a bitmap image using CGBitmapContext
    //   for possible pixel format, refer to http://developer.apple.com/qa/qa2001/qa1037.html
    Image tmp( CGImageGetWidth(imgref), CGImageGetHeight(imgref), PIXEL_RGBA );
    CGColorSpaceRef cs = CGColorSpaceCreateWithName( kCGColorSpaceGenericRGB );
    CGContextRef    bc = CGBitmapContextCreate( tmp.data, tmp.w, tmp.h, 8, 4*tmp.w, cs, kCGImageAlphaNoneSkipLast);
    CGRect rect = CGRectMake( 0, 0, tmp.w, tmp.h );
    CGContextDrawImage( bc, rect, imgref );  // create RGBA bitmap data (without alpha)
    CGContextRelease( bc );
    // get the alpha channel, if any
    if (alpha) {  // Note that Carbon API does not provide a method to extract alpha values only.
      Image tma( CGImageGetWidth(imgref), CGImageGetHeight(imgref), PIXEL_RGBA );
      CGContextRef    bc = CGBitmapContextCreate( tma.data, tma.w, tma.h, 8, 4*tma.w, cs, kCGImageAlphaPremultipliedLast);
      CGContextDrawImage( bc, rect, imgref );  // create RGBA bitmap data (with alpha premultiplied)
      CGContextRelease( bc );
      for (int i=0,total=tmp.w*tmp.h; i<total; i++) {
	uchar_t *op = (uchar_t*)tmp.getPixel(i);  uchar_t *mp = (uchar_t*)tma.getPixel(i);
	op[3] = mp[3];
      }
    }
    CGColorSpaceRelease( cs );
    CGImageRelease( imgref );
    //tmp.printInfo();
    if (type == PIXEL_UNKNOWN) {
      if      ( gray  && !alpha) type = PIXEL_GRAY;
      else if ( gray  &&  alpha) type = PIXEL_GRAYA;
      else if (!gray  && !alpha) type = PIXEL_RGB;
      else if (!gray  &&  alpha) type = PIXEL_RGBA;
      else                       type = PIXEL_RGB;
    }
    // convert the bitmap image to the final image with the desired pixel format
    if (!img) img = new Image( tmp.w, tmp.h, type );
    else      img->setImage  ( tmp.w, tmp.h, type );
    if (img->type != tmp.type) {
      ImageConverter conv;
      conv.convertImage( tmp.data, img->data, tmp.w, tmp.h, tmp.type, img->type );
    } else img->swapImage( &tmp );
    return img;
  }

  bool writeFile(const char *fname, Image *img, int opt=0) {
    if (!fname || !img || !img->data) return false;
    // set an CGImageRef from input image
    int  pxsize = img->getPixelSize(img->type);
    CGColorSpaceRef cs=NULL;  bool alpha=false;
    switch (img->type) {
    case PIXEL_GRAY:  cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericGray); break;
    case PIXEL_GRAYA: cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericGray); alpha=true; break;
    case PIXEL_RGB:   cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericRGB);  break;
    case PIXEL_INT: case PIXEL_FLOAT:
    case PIXEL_RGBA:  cs = CGColorSpaceCreateWithName(kCGColorSpaceGenericRGB);  alpha=true; break;
    default : return false;
    }
    CGDataProviderRef dp = CGDataProviderCreateWithData( NULL, img->data, pxsize*img->w*img->h, NULL );
    CGImageRef imgref = CGImageCreate( img->w, img->h, 8, 8*pxsize, pxsize*img->w, cs,
				       (alpha ? kCGImageAlphaLast : kCGImageAlphaNone),
				       dp, NULL, false, kCGRenderingIntentDefault );
    CGColorSpaceRelease( cs );  CGDataProviderRelease( dp );
    // set options (http://developer.apple.com/documentation/GraphicsImaging/Reference/CGImageProperties_Reference/Reference/reference.html)
    float compression = (opt == 0 ? 1.0 : (opt/100.0));
    CFStringRef  myKeys[] = { kCGImagePropertyHasAlpha,
			      kCGImageDestinationLossyCompressionQuality };
    CFTypeRef    myVals[] = { (alpha ? kCFBooleanTrue : kCFBooleanFalse),
			      CFNumberCreate( NULL, kCFNumberFloatType, &compression ) };
    CFDictionaryRef opts = CFDictionaryCreate( NULL, (const void**)myKeys, (const void**)myVals,
					       2, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks );
    // save the image using CGImageDestination
    // (http://developer.apple.com/documentation/Carbon/Conceptual/understanding_utis/understand_utis_intro/chapter_1_section_1.html)
    CFStringRef type;  const char *ext = getFileExtension( fname );
    if      (strcmp(ext, "png")==0 || strcmp(ext, "PNG")==0)  type = kUTTypePNG;
    else if (strcmp(ext, "jpg")==0 || strcmp(ext, "JPG")==0)  type = kUTTypeJPEG;
    else if (strcmp(ext, "bmp")==0 || strcmp(ext, "BMP")==0)  type = kUTTypeBMP;
    else if (strcmp(ext, "gif")==0 || strcmp(ext, "GIF")==0)  type = kUTTypeGIF;
    else if (strcmp(ext, "tif")==0 || strcmp(ext, "tiff")==0) type = kUTTypeTIFF;
    else { CFRelease( opts ); CGImageRelease( imgref );  return false; }
    CFURLRef url = CFURLCreateFromFileSystemRepresentation( NULL, (const UInt8*)fname, strlen(fname), false );
    CGImageDestinationRef idst = CGImageDestinationCreateWithURL( url, type, 1, NULL );
    CFRelease( url );    if (idst == NULL) return false;
    CGImageDestinationAddImage( idst, imgref, opts );
    bool status = CGImageDestinationFinalize( idst );
    CFRelease( opts );
    CGImageRelease( imgref );
    return status;
  }

  const char* getFileExtension(const char *filename) {
    int  i;  // find the beginning of the file extension of 'filename'
    for (i=(int)strlen(filename); i>0 && filename[i]!='.'; i--);
    return (i>0 ? filename+i+1 : NULL);
  }
  char* getFileBaseName(const char *filename, char *out, bool without_path=true) {
    int  i=0, j, k, len = (int)strlen(filename);
    if (without_path) { for (i=len-1; i>=0; i--) if (filename[i]=='/') break;  i++; }
    for (j=len; j>i && filename[j]!='.'; j--);
    j = (j>i ? j : len);
    if (out) { for (k=i; k<j; k++) out[k-i] = filename[k]; out[k-i] = '\0'; }
    return out;
  }
  char* getFilePath(const char *filename, char *out) {
    int  i=0, j, len = (int)strlen(filename);
    for (i=len-1; i>=0; i--) if (filename[i]=='/') break;
    for (j=0; j <= i; j++) out[j] = filename[j];  out[j] = '\0';
    return out;
  }

  // -----------------------------------------------------------------
  // PPM / PGM
  // -----------------------------------------------------------------
private:

  Image* readPXMFile(FILE *fp, Image *img) {
    char type[10];  int w, h, maxv;
    if (!fp || fscanf(fp, "%s %d %d %d\n", type, &w, &h, &maxv)!=4 ||
	type[0] != 'P' || type[1] < '0' || type[1] > '6' || w==0 || h==0) {
      std::cerr << "Error (IMGH::FileIO::readPXMFile): invalid PXM file " << std::endl;
      return NULL;
    }
    int  i, total, v;
    IMGH::pixel_t pt;
    if (type[1]=='3'||type[1]=='6') { pt = IMGH::PIXEL_RGB; total = w*h*3; }
    else                            { pt = IMGH::PIXEL_GRAY; total = w*h;  }
    if (img) { if (!img->sameSize(w, h, pt)) img->setImage(w, h, pt); }
    else     { img = new IMGH::Image(w, h, pt); }
    switch (type[1]) {
    case '2': for (i=0; i<total; i++) { fscanf(fp, "%d", &v); img->data[i] = v; }  break;
    case '3': for (i=0; i<total; i++) { fscanf(fp, "%d", &v); img->data[i] = v; }  break;
    case '5': fread( img->data, sizeof(unsigned char), total, fp );  break;
    case '6': fread( img->data, sizeof(unsigned char), total, fp );  break;
    default: return NULL;
    }
    return img;
  }

  bool writePPMFile(FILE *fp, Image *img) {
    if (img->type != PIXEL_RGB) {
      std::cerr << "Error (IMGH::FileIO::writePPMFile): It's not PIXEL_RGB." << std::endl;
      return false;
    }
    char header[80];
    sprintf( header, "P6 %d %d %d\n", img->w, img->h, 255 );
    fwrite ( header, sizeof(char), (int)strlen(header), fp );
    fwrite ( img->data, sizeof(unsigned char), img->w * img->h * 3, fp );
    return true;
  }

  bool writePGMFile(FILE *fp, Image *img) {
    char header[80];
    sprintf( header, "P5 %d %d %d\n", img->w, img->h, 255 );
    fwrite ( header, sizeof(char), (int)strlen(header), fp );
    if (img->type == PIXEL_GRAY) {
      fwrite ( img->data, sizeof(unsigned char), img->w * img->h, fp );
    } else if (img->type == PIXEL_RGB) {
      unsigned char *src = img->data, value;
      int  i, total = img->w * img->h;
      for (i = 0; i < total; i++, src+=3) {
	value = (unsigned char)((6969*(int)src[0] + 23434*(int)src[1] + 2365*(int)src[2])/32768);
	fwrite ( &value, sizeof(unsigned char), 1, fp );
      }
    } else {
      std::cerr << "Error (IMGH::FileIO::writePPMFile): It neither PIXEL_RGB nor PIXEL_GRAY." << std::endl;
      return false;
    }
    return true;
  }
};

}	// namespace IMGH

// ===================================================================
#else			// Linux version
// ===================================================================


extern "C" {
#include <jpeglib.h>	// libjpeg (included in standard linux installation)
#include <jerror.h>
#include <tiffio.h>	// libtiff (included in standard linux installation)
#include <png.h>	// libpng  (included in standard linux installation)
}

namespace IMGH {


class FileIO
{
public:
  FileIO() {}
  ~FileIO() {}

  // -----------------------------------------------------------------
  // public function
  // -----------------------------------------------------------------
public:
  Image* readFile(const char *filename, Image *img=NULL, pixel_t type=PIXEL_UNKNOWN) {
    FILE *fp = fopen(filename, "rb");
    if (fp == (FILE*)NULL) {
//       std::cerr << "Error (IMGH::FileIO::readFile): cannot open file '" << filename << "'" << std::endl;
      return NULL;
    }
    int  i, len = (int)strlen(filename);   char *ep, ext[10];
    for (i = len-1; i >= 0; i--) if (filename[i] == '.') break;  ep = (char*)filename+i+1;
    for (i=0; i<4 && ep[i]; i++) ext[i] = (ep[i]>='A' && ep[i]<='Z' ? (ep[i]-'A'+'a') : ep[i]);
    Image *ret;
    if      (strncmp(ext,"jpg",3)==0) ret = readJPEGFile(fp, img);
    else if (strncmp(ext,"tif",3)==0) ret = readTIFFFile(fp, img);
    else if (strncmp(ext,"png",3)==0) ret = readPNGFile(fp, img);
    else if (strncmp(ext,"ppm",3)==0) ret = readPXMFile(fp, img);
    else if (strncmp(ext,"pgm",3)==0) ret = readPXMFile(fp, img);
    else {
//       std::cerr << "Error (IMGH::FileIO::readFile): invalid file type" << std::endl;
      fclose(fp); return NULL;
    }
    fclose(fp);
    if (type != PIXEL_UNKNOWN && type != ret->type) {	// conversion, if necessary
      IMGH::Image  dst( ret->w, ret->h, type );
      IMGH::ImageConverter conv;
      if (!conv.convertImage( ret, &dst )) return NULL;
      ret->copyFrom( &dst );
    }
    return ret;
  }

  bool writeFile(const char *filename, Image *img, int opt=0) {
    if (!filename || !img || !img->data) return false;
    FILE *fp = fopen(filename, "w+");
    if (fp == (FILE*)NULL) return false;
    int  i, len = (int)strlen(filename);   char *ep, ext[10];
    for (i = len-1; i >= 0; i--) if (filename[i] == '.') break;  ep = (char*)filename+i+1;
    for (i=0; i<4 && ep[i]; i++) ext[i] = (ep[i]>='A' && ep[i]<='Z' ? (ep[i]-'A'+'a') : ep[i]);
    bool ret;
    if      (strncmp(ext,"jpg",3)==0) ret = writeJPEGFile(fp, img, (opt ? opt : 90));
    else if (strncmp(ext,"tif",3)==0) ret = writeTIFFFile(fp, img);
    else if (strncmp(ext,"png",3)==0) ret = writePNGFile(fp, img);
    else if (strncmp(ext,"ppm",3)==0) ret = writePPMFile(fp, img);
    else if (strncmp(ext,"pgm",3)==0) ret = writePGMFile(fp, img);
    else { fclose(fp);  return false; }
    fclose(fp);
    return ret;
  }

  char* getFileExtension(char *filename) {
    int  i;  // find the beginning of the file extension of 'filename'
    for (i=(int)strlen(filename); i>0 && filename[i]!='.'; i--);
    return (i>0 ? filename+i+1 : NULL);
  }
  char* getFileBaseName(const char *filename, char *out, bool without_path=true) {
    int  i=0, j, k, len = (int)strlen(filename);
    if (without_path) { for (i=len-1; i>=0; i--) if (filename[i]=='/') break;  i++; }
    for (j=len; j>i && filename[j]!='.'; j--);
    j = (j>i ? j : len);
    if (out) { for (k=i; k<j; k++) out[k-i] = filename[k]; out[k-i] = '\0'; }
    return out;
  }
  char* getFilePath(const char *filename, char *out) {
    int  i=0, j, len = (int)strlen(filename);
    for (i=len-1; i>=0; i--) if (filename[i]=='/') break;
    for (j=0; j <= i; j++) out[j] = filename[j];  out[j] = '\0';
    return out;
  }

  // -----------------------------------------------------------------
  // JPEG
  // -----------------------------------------------------------------
private:

  Image* readJPEGFile(FILE *fp, Image *img) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);	// set up the normal JPEG error routines
    jpeg_create_decompress(&cinfo);	// initialize the JPEG decompression object.
    jpeg_stdio_src(&cinfo, fp);		// specify data source (eg, a file)
    jpeg_read_header(&cinfo, TRUE);	// read file parameters with jpeg_read_header()
    jpeg_start_decompress(&cinfo);	// start decompressor (header information available from now on)
    IMGH::pixel_t ptype;
    if      (cinfo.output_components == 1) ptype = PIXEL_GRAY;
    else if (cinfo.output_components == 3) ptype = PIXEL_RGB;
    else if (cinfo.output_components == 4) { std::cerr << "Error (IMGH::FileIO::readJPEGFile): ALPHA channel ? " << std::endl; return NULL; }
    else return NULL;
    if  (img)    img->setImage(cinfo.output_width, cinfo.output_height, ptype);
    else img = new IMGH::Image(cinfo.output_width, cinfo.output_height, ptype);
    int row_stride = cinfo.output_width * cinfo.output_components;
    JSAMPROW row_pointer[1];
    while (cinfo.output_scanline < cinfo.output_height) {
      row_pointer[0] = img->data + cinfo.output_scanline * row_stride;
      jpeg_read_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_decompress(&cinfo);	// finish decompression
    jpeg_destroy_decompress(&cinfo);	// release JPEG decompression object
    return img;
  }

  bool writeJPEGFile(FILE *fp, Image *img, int quality=90) {
    // 'quality' is integer value of quality percentage
    if (img == NULL || img->w == 0 || img->h == 0) return false;
    struct jpeg_compress_struct cinfo;	// JPEG compression struct (parameters and pointers to working space)
    struct jpeg_error_mgr jerr;		// This struct represents a JPEG error handler.
    cinfo.err = jpeg_std_error(&jerr);	// set up the normal JPEG error routines
    jpeg_create_compress(&cinfo);	// initialize the JPEG compression object
    jpeg_stdio_dest(&cinfo, fp);	// specify data destination (eg, a file)
    cinfo.image_width = img->w;		// set parameters for compression
    cinfo.image_height = img->h;
    switch (img->type) {
    case PIXEL_RGB:  cinfo.input_components=3; cinfo.in_color_space=JCS_RGB;  break;
    case PIXEL_GRAY: cinfo.input_components=1; cinfo.in_color_space=JCS_GRAYSCALE;  break;
    default: std::cerr << "Error (IMGH::FileIO::writeJPEGFile): invalid pixel type" << std::endl; return false;
    }
    jpeg_set_defaults(&cinfo);		// set default compression parameters.
    jpeg_set_quality(&cinfo, quality, TRUE);	// quality (quantization table) scaling:
    jpeg_start_compress(&cinfo, TRUE);	// start compressor (TRUE for interchange-JPEG file)
    int row_stride = img->row_size;	// write every scanline (JSAMPLEs per row in image data)
    JSAMPROW row_pointer[1];		// pointer to JSAMPLE row[s]
    while (cinfo.next_scanline < cinfo.image_height) {
      row_pointer[0] = img->data + cinfo.next_scanline * row_stride;
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);	// finish compression
    jpeg_destroy_compress(&cinfo);	// release JPEG compression object
    return true;
  }

  // -----------------------------------------------------------------
  // TIFF
  // -----------------------------------------------------------------
private:

  Image* readTIFFFile(FILE *fp, Image *img) {
    TIFF* tiffp = TIFFFdOpen( (const int)fp, "unknown.tif", "r" );
    if (tiffp == NULL) {
      std::cerr << "Error (IMGH::FileIO::readTIFFFile): cannot open the file " << std::endl;
      return NULL;
    }
    TIFFRGBAImage tiff;
    char emsg[1024];
    if (!TIFFRGBAImageBegin(&tiff, tiffp, 0, emsg)) {
      TIFFError("unknown.tif", emsg);
      TIFFClose(tiffp);
      return NULL;
    }
    int    i, j, pos;
    size_t npixels;
    uint32* raster;
    if (img) {
      if (!img->sameSize(tiff.width, tiff.height, PIXEL_RGB))
	img->setImage(tiff.width, tiff.height, PIXEL_RGB);
    } else img = new IMGH::Image(tiff.width, tiff.height, PIXEL_RGB);
    npixels = tiff.width * tiff.height;
    raster = (uint32*) _TIFFmalloc(npixels * sizeof (uint32));
    if (raster != NULL) {
      if (TIFFRGBAImageGet(&tiff, raster, tiff.width, tiff.height)) {
	for (i = pos = 0; i < (int)tiff.height; i++) {
	  for (j = 0; j < (int)tiff.width; j++, pos++) {
	    uchar_t *p = (uchar_t*)(raster + pos);
// 	    img->setRGBA(j, tiff.height-1-i, p[0], p[1], p[2], p[3]);
	    img->setPixel(j, tiff.height-1-i, p);
	  }
	}
      }
      _TIFFfree(raster);
    }
    TIFFRGBAImageEnd(&tiff);
    TIFFClose(tiffp);
    return img;
  }

  bool writeTIFFFile(FILE *fp, Image *img) {
    std::cerr << "Error (IMGH::FileIO::writeTIFFFile): not implemented yet " << std::endl;
    return false;
  }

  // -----------------------------------------------------------------
  // PNG
  // -----------------------------------------------------------------
private:

  Image* readPNGFile(FILE *fp, Image *img) {
    uchar_t signature[8];
    png_uint_32   w, h, row ;
    png_structp   pngp = NULL;
    png_infop     info = NULL;
    int           color_type, bit_depth;

    fread(signature, 1, 8, fp);				// check PNG signature
    if (!png_check_sig(signature, 8)) return NULL;
    pngp = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!pngp) return NULL;
    info = png_create_info_struct(pngp);
    if (!info) { png_destroy_read_struct(&pngp, NULL, NULL);  return NULL; }
    if (setjmp(png_jmpbuf(pngp))) {
      png_destroy_read_struct(&pngp, &info, NULL);
      return NULL;
    }
    png_init_io(pngp, fp);
    png_set_sig_bytes(pngp, 8);

    png_read_info(pngp, info); 				// read PNG info
    png_get_IHDR(pngp, info, &w, &h, &bit_depth, &color_type, NULL, NULL, NULL);
    if (img == NULL) img = new IMGH::Image;
    if      (color_type == PNG_COLOR_TYPE_GRAY_ALPHA) img->setImage( w, h, PIXEL_GRAYA );
    else if (color_type == PNG_COLOR_TYPE_RGB_ALPHA)  img->setImage( w, h, PIXEL_RGBA );
    else if (color_type == PNG_COLOR_TYPE_GRAY)       img->setImage( w, h, PIXEL_GRAY );
    else                                              img->setImage( w, h, PIXEL_RGB );

    // expand palette images to RGB, low-bit-depth grayscale images to 8 bits,
    // transparency chunks to full alpha channel; strip 16-bit-per-sample
    // images to 8 bits per sample; and convert grayscale to RGB[A]
    if (color_type == PNG_COLOR_TYPE_PALETTE)  png_set_expand(pngp);
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)  png_set_expand(pngp);
    if (png_get_valid(pngp, info, PNG_INFO_tRNS))  png_set_expand(pngp);
    if (bit_depth == 16)  png_set_strip_16(pngp);
    //if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)  png_set_gray_to_rgb(pngp);

    png_read_update_info(pngp, info);			// update info data

    png_bytepp    row_pointers = NULL;			// set the individual row_pointers
    if ((row_pointers = (png_bytepp)malloc(h*sizeof(png_bytep))) == NULL) {
      png_destroy_read_struct(&pngp, &info, NULL);
      return NULL;
    }
    for (row=0; row<h; row++) row_pointers[row] = img->data + row * img->row_size;

    png_read_image(pngp, row_pointers);			// read the whole image
    png_read_end(pngp, NULL);

    free(row_pointers);					// clean up memory
    png_destroy_read_struct(&pngp, &info, NULL);
    return img;
  }

  bool writePNGFile(FILE *fp, Image *img) {
    if (img == NULL || img->w == 0 || img->h == 0) return false;
    png_structp pngp = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!pngp) return false;
    png_infop   info = png_create_info_struct(pngp);
    if (!info) { png_destroy_write_struct(&pngp, NULL);  return false; }
    if (setjmp(png_jmpbuf(pngp))) {
      png_destroy_write_struct(&pngp, &info);
      return false;
    }
    png_init_io(pngp, fp);
    png_set_compression_level(pngp, Z_BEST_COMPRESSION);	// set the compression level (default : filtering ON, 32K zlib window)

    int interlace_type = PNG_INTERLACE_NONE;
    int color_type, bit_depth = 8, row;
    pixel_t oldtype = PIXEL_UNKNOWN;
    switch (img->type) {
    case PIXEL_RGB  : color_type = PNG_COLOR_TYPE_RGB; break;
    case PIXEL_RGBA : color_type = PNG_COLOR_TYPE_RGB_ALPHA; break;
    case PIXEL_GRAY : color_type = PNG_COLOR_TYPE_GRAY; break;
    //case PIXEL_GRAYA: color_type = PNG_COLOR_TYPE_GRAY_ALPHA; break;
    case PIXEL_INT:
    case PIXEL_FLOAT:
    case PIXEL_VOIDP:
      oldtype = img->type;  img->type = PIXEL_RGB;
      color_type = PNG_COLOR_TYPE_RGB_ALPHA;
      break;
    default :
      std::cerr << "Error (IMGH::FileIO::writePNGFile): invalid pixel type" << std::endl;
      png_destroy_write_struct(&pngp, &info);
      return false;
    }
    png_set_IHDR(pngp, info, img->w, img->h,
		 bit_depth, color_type, interlace_type,
		 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_bytepp    row_pointers = NULL;				// set the individual row_pointers
    if ((row_pointers = (png_bytepp)malloc(img->h *sizeof(png_bytep))) == NULL) {
      png_destroy_write_struct(&pngp, &info);
      return false;
    }
    for (row = 0;  row < img->h; row++)
      row_pointers[row] = img->data + row * img->row_size;
    png_set_rows( pngp, info, row_pointers );

    png_write_png(pngp, info, PNG_TRANSFORM_IDENTITY, NULL);	// write PNG image file

    free(row_pointers);						// clean up memory
    png_destroy_write_struct(&pngp, &info);
    if (oldtype != PIXEL_UNKNOWN) img->type = oldtype;
    return true;
  }

  // -----------------------------------------------------------------
  // PPM / PGM
  // -----------------------------------------------------------------
private:

  Image* readPXMFile(FILE *fp, Image *img) {
    char type[10];  int w, h, maxv;
    if (fscanf(fp, "%s %d %d %d\n", type, &w, &h, &maxv)!=4 ||
	type[0] != 'P' || type[1] < '0' || type[1] > '6' || w==0 || h==0) {
      std::cerr << "Error (IMGH::FileIO::readPXMFile): invalid PXM file " << std::endl;
      return NULL;
    }
    int  i, total, v;
    IMGH::pixel_t pt;
    if (type[1]=='3'||type[1]=='6') { pt = IMGH::PIXEL_RGB; total = w*h*3; }
    else                            { pt = IMGH::PIXEL_GRAY; total = w*h;  }
    if (img) { if (!img->sameSize(w, h, pt)) img->setImage(w, h, pt); }
    else     { img = new IMGH::Image(w, h, pt); }
    switch (type[1]) {
    case '2': for (i=0; i<total; i++) { fscanf(fp, "%d", &v); img->data[i] = v; }  break;
    case '3': for (i=0; i<total; i++) { fscanf(fp, "%d", &v); img->data[i] = v; }  break;
    case '5': fread( img->data, sizeof(unsigned char), total, fp );  break;
    case '6': fread( img->data, sizeof(unsigned char), total, fp );  break;
    default: return NULL;
    }
    return img;
  }

  bool writePPMFile(FILE *fp, Image *img) {
    if (img->type != PIXEL_RGB) {
      std::cerr << "Error (IMGH::FileIO::writePPMFile): It's not PIXEL_RGB." << std::endl;
      return false;
    }
    char header[80];
    sprintf( header, "P6 %d %d %d\n", img->w, img->h, 255 );
    fwrite ( header, sizeof(char), (int)strlen(header), fp );
    fwrite ( img->data, sizeof(unsigned char), img->w * img->h * 3, fp );
    return true;
  }

  bool writePGMFile(FILE *fp, Image *img) {
    char header[80];
    sprintf( header, "P5 %d %d %d\n", img->w, img->h, 255 );
    fwrite ( header, sizeof(char), (int)strlen(header), fp );
    if (img->type == PIXEL_GRAY) {
      fwrite ( img->data, sizeof(unsigned char), img->w * img->h, fp );
    } else if (img->type == PIXEL_RGB) {
      unsigned char *src = img->data, value;
      int  i, total = img->w * img->h;
      for (i = 0; i < total; i++, src+=3) {
	value = (unsigned char)((6969*(int)src[0] + 23434*(int)src[1] + 2365*(int)src[2])/32768);
	fwrite ( &value, sizeof(unsigned char), 1, fp );
      }
    } else {
      std::cerr << "Error (IMGH::FileIO::writePPMFile): It neither PIXEL_RGB nor PIXEL_GRAY." << std::endl;
      return false;
    }
    return true;
  }

};


}	// namespace IMGH


// ===================================================================
// ===================================================================
#endif


#endif	// IMGH_FILE_IO_HPP

