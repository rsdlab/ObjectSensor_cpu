

#ifndef CUDA_IMAGE_H
#define CUDA_IMAGE_H

#include <iostream>

// ===================================================================
// Image structure on Device memory
// ===================================================================

class DevImage {
public:
  int	w, h;
  float	*data;
public:
  DevImage() : w(0), h(0), data(NULL) {}
  DevImage(int ww, int hh) : w(0), h(0), data(NULL) { setImage(ww,hh); }
  ~DevImage() { clear(); }
  void clear(void);
  bool setImage(int w, int h);
  bool downloadFrom(float *src_buffer);
  bool uploadTo    (float *dst_buffer);
  void printInfo(char *cmmt=NULL);
};

// ===================================================================
// Image structure on Host memory
// ===================================================================

class HstImage {
public:
  int   w, h, hmax;
  float *data;
public:
  HstImage() : w(0), h(0), hmax(0), data(NULL) {}
  HstImage(int ww, int hh) : w(0), h(0), hmax(0), data(NULL) { setImage(ww,hh); }
  ~HstImage() { clear(); }
  void clear(void);
  bool setImage(int w, int h);
  bool reallocImage(int w, int hmax);
  bool clearImage(float v=0);
  bool downloadTo(DevImage *cimg);
  bool uploadFrom(DevImage *cimg);
  float* yieldData(void);
  void printInfo(char *cmmt=NULL, char *fmt="%12g ", int px=0, int py=0, int pw=0, int ph=0);
  inline float* getRow(int row) { return data + row*w; }
};


// ===================================================================
// Texture structure on Device memory
// ===================================================================

class DevTexture {
public:
  int	w, h;
  void *array;		// pointer to 'cudaArray'
public:
  DevTexture() : w(0), h(0), array(NULL) {}
  DevTexture(int ww, int hh) : w(0), h(0), array(NULL) { setTexture(ww,hh); }
  ~DevTexture() { clear(); }
  void clear(void);
  bool setTexture(int w, int h);
  bool copyFrom(DevImage *cimg);
  bool copyFrom(HstImage *cimg);
};

// ===================================================================

bool   cuda_init(void);
void   timer_start(unsigned int *hTimer);
double timer_stop (unsigned int hTimer);
inline int iDivUp(int a, int b) { return (a % b != 0) ? (a / b + 1) : (a / b); }
inline int iDivDown(int a, int b) { return a / b; }

#endif // CUDA_IMAGE_H
