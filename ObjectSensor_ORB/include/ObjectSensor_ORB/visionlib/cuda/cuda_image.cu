
#include <iostream>
#include "cutil.h"
#include "cuda_image.h"

// ===================================================================
// ===================================================================

void DevImage::clear(void)
{
  if (this->data) cudaFree( this->data );
  this->data = NULL;
  this->w = this->h = 0;
}

bool DevImage::setImage(int w, int h)
{
  if (w <= 0 || h <= 0) { clear(); return true; }
  if (this->w != w || this->h != h || this->data == NULL) {
    clear();
    int total = w * h * sizeof(float);
    cudaMalloc((void**)&this->data, total);
    if (this->data == NULL) return false;
    this->w = w;  this->h = h;
  }
  return true;
}

bool DevImage::downloadFrom(float *src_buffer)
{
  if (!src_buffer || !this->data) return false;
  int total = this->w * this->h * sizeof(float);
  cudaMemcpy( this->data, src_buffer, total, cudaMemcpyHostToDevice );
  return true;
}

bool DevImage::uploadTo(float *dst_buffer)
{
  if (!dst_buffer || !this->data) return false;
  int total = this->w * this->h * sizeof(float);
  cudaMemcpy( dst_buffer, this->data, total, cudaMemcpyDeviceToHost );
  return true;
}

void DevImage::printInfo(char *cmmt)
{
  printf("%s (%4d x %4d) Device Image\n", (cmmt ? cmmt : "CUDAIMG"), this->w, this->h);
}

// ===================================================================
// ===================================================================

void HstImage::clear(void)
{
  if (this->data) free( this->data );
  this->data = NULL;
  this->w = this->h = this->hmax = 0;
}

bool HstImage::setImage(int w, int h)
{
  if (w <= 0) { clear(); return true; }
  int tn = w * h, to = this->w * this->hmax;
  if (tn > to) {
    if (!reallocImage( w, h )) return false;
  }
  this->w = w;  this->h = h;
  return true;
}

bool HstImage::reallocImage(int w, int hmax)
{
  if (w <= 0 || hmax <= 0) { clear(); return true; }
  int total = w * hmax * sizeof(float);
  this->data = (float*)realloc( this->data, total );
  if (this->data) { 
    this->w = w;  this->hmax = hmax;
    return true;
  } else {
    this->w = this->h = this->hmax = 0;
    return false;
  }
}

bool HstImage::clearImage(float v)
{
  int  i, total = w * h;
  if (data) for (i=0; i<total; i++) data[i] = v;
  return true;
}

bool HstImage::downloadTo(DevImage *cimg)
{
  if (!this->data || !cimg || !cimg->data) return false;
  int total = this->w * this->h * sizeof(float);
  cudaMemcpy( cimg->data, this->data, total, cudaMemcpyHostToDevice );
  return true;
}

bool HstImage::uploadFrom(DevImage *cimg)
{
  if (!this->data || !cimg || !cimg->data) return false;
  int total = this->w * this->h * sizeof(float);
  cudaMemcpy( this->data, cimg->data, total, cudaMemcpyDeviceToHost );
  return true;
}

float* HstImage::yieldData(void)
{
  float *dbuffer = this->data;
  this->data = NULL;
  this->w = this->h = this->hmax = 0;
  return dbuffer;
}

void HstImage::printInfo(char *cmmt, char *fmt, int px, int py, int pw, int ph)
{
  int k, nsp=0, len=(cmmt!=NULL?strlen(cmmt):0);  char spaces[41];
  if (cmmt!=NULL) { for (k=0; k<40&&k<len; k++) if (cmmt[k]==' ') nsp++; }
  for (k=0; k<nsp; k++) spaces[k] = ' ';  spaces[k] = '\0';
  if (pw * ph <= 0) {
    printf("%s (%4d x %4d / %d)\n", (cmmt!=NULL ? cmmt : "HOSTImage"), w, h, hmax);
  } else {
    printf("%s (%4d x %4d / %d) (%d,%d:%dx%d)\n", (cmmt ? cmmt : "HOSTImage"), w, h, hmax, px, py, pw, ph);
    if (fmt  ==NULL) fmt = (char*)"%12g ";
    for (int j=0; j < this->h; j++) {
      if (j < py) continue; if (j >= py+ph) break;
      printf("%s  ", spaces);
      for (int i=0; i < this->w; i++)
	if (i >= px && i < px+pw) printf(fmt, this->data[j*this->w+i]);
      printf("\n");
    }
  }
}

// ===================================================================
// ===================================================================

void DevTexture::clear(void)
{
  if (this->array) cudaFreeArray( (cudaArray*)this->array ); 
  this->array = NULL;
  this->w = this->h = 0;
}

bool DevTexture::setTexture(int w, int h)
{
  cudaChannelFormatDesc t_desc = cudaCreateChannelDesc<float>(); 
  cudaMallocArray((cudaArray **)&this->array, &t_desc, w, h);
  if (this->array == NULL) { this->w = this->h = 0;  return false; }
  this->w = w;  this->h = h;
  return true;
}

bool DevTexture::copyFrom(DevImage *cimg)
{
  if (!this->array || !cimg || !cimg->data) return false;
  cudaMemcpyToArray((cudaArray*)this->array, 0, 0, 
		    cimg->data, sizeof(float)*cimg->w*cimg->h, 
		    cudaMemcpyDeviceToDevice);
  cudaThreadSynchronize();
  return true;
}

bool DevTexture::copyFrom(HstImage *himg)
{
  if (!this->array || !himg || !himg->data) return false;
  cudaMemcpyToArray((cudaArray*)this->array, 0, 0, 
		    himg->data, sizeof(float)*himg->w*himg->h, 
		    cudaMemcpyHostToDevice);
  cudaThreadSynchronize();
  return true;
}

// ===================================================================
// ===================================================================

bool cuda_init(void)
{
  static bool first_time = true;
  if (first_time) {
//     char  *argv[]={"--quiet"};
//     CUT_DEVICE_INIT( 1, argv );  //// initialize the data structures
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);
    if (deviceCount == 0) {
        fprintf(stderr, "cutil error: no devices supporting CUDA.\n");
        return false;
    }
    cudaDeviceProp deviceProp;
    cudaGetDeviceProperties(&deviceProp, 0);
    if (deviceProp.major < 1) {
        fprintf(stderr, "cutil error: device does not support CUDA.\n");
        return false;
    }
    cudaSetDevice(0);
  }
  first_time = false;
  return true;
}

void timer_start(unsigned int *hTimer)
{
  cutCreateTimer(hTimer);
  cutResetTimer(*hTimer);
  cutStartTimer(*hTimer);
}

double timer_stop(unsigned int hTimer)
{
  cutStopTimer(hTimer);
  double gpuTime = cutGetTimerValue(hTimer);
  cutDeleteTimer(hTimer);
  return gpuTime;
}

