
#include <stdio.h>
#include <cutil.h>
#include "cuda_image_edit.h"

__device__ __constant__ float edit_const[2]; 

// ===================================================================
// 
// ===================================================================

// __global__ void th_sample_up(float *dst, float *src, int sw, int sh)
// {
//   // Upsample the 'src' and set the pixels of 'dst', which is two times bigger.
//   //   'sw' and 'sh' is the size of 'src', not 'dst', and 
//   //   the thread blocks(16x16) must divide up the size of 'dst'.
//   const int dw = __mul24(2,sw), dh = __mul24(2,sh);
//   const int dx = __mul24(16, blockIdx.x) + threadIdx.x;
//   const int dy = __mul24(16, blockIdx.y) + threadIdx.y;
//   if (dx >= dw || dy >= dh) return;
//   int   sx = dx>>1, sy = dy>>1;
//   int spos = __mul24(sy, sw) + sx;
//   dst[ __mul24(dy, dw) + dx ] = src[spos];
//   __syncthreads();
// }

__global__ void th_sample_up(float *dst, float *src, int dw, int dh)
{
  // Upsample the 'src' and set the pixels of 'dst', which is two times bigger.
  //   'dw' and 'dh' is the size of 'dst', not 'src', and 
  //   the thread blocks(16x16) must divide up the size of 'dst'.
//   if (blockIdx.x != 1 || blockIdx.y != 1) return;
  __shared__ float data[9][9];
  int tx = threadIdx.x;
  int ty = threadIdx.y;
  int dx = __mul24(16, blockIdx.x) + threadIdx.x;
  int dy = __mul24(16, blockIdx.y) + threadIdx.y;
  int sstt = __mul24(dy>>1, dw>>1) + __mul24(8, blockIdx.x);
  int tx2 = tx>>1;
  int ty2 = ty>>1;
  int didx = __mul24(dy, dw) + dx;
  if (dx >= 200 || dy >= 200) return;
  if (tx < 9 && ty < 9 && dx < dw && dy < dh) data[ty][tx] = src[sstt + tx];
#if 1		//// Why is it failing ?
  if (tx&0x1 == 0) {
    if (ty&0x1 == 0) dst[didx] = (data[ty2][tx2]);
    else             dst[didx] = (data[ty2][tx2] + data[ty2+1][tx2]) / 2;
  } else {
    if (ty&0x1 == 0) dst[didx] = (data[ty2][tx2] + data[ty2][tx2+1]) / 2;
    else             dst[didx] = (data[ty2][tx2] + data[ty2][tx2+1] + data[ty2+1][tx2] + data[ty2+1][tx2+1]) / 4;
  }
#else		//// ... while this code is OK ?
  dst[__mul24(dy, dw) + dx] = src[ (dy>>1)*(dw>>1) + (dx>>1) ];
#endif
  __syncthreads();
}

double cimg_scale_up(DevImage *res, DevImage *src)
{
  int w = src->w;
  int h = src->h;
  if (res->data==NULL || src->data==NULL) {
    // printf("cimg_scale_up: missing data\n");
    return 0.0;
  }
  unsigned int hTimer; 
  timer_start(&hTimer);
  int  dw = w*2,  dh = h*2;
#if 0	//// This code is not working at all. Don't know why...
  dim3 blocks(iDivUp(dw,16), iDivUp(dh,16));
  dim3 threads(16,16);
  th_sample_up<<<blocks, threads>>>(res->data, src->data, dw, dh); 
  CUT_CHECK_ERROR("cimg_scale_up() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
#else
  HstImage hsrc(w, h+1);
  HstImage hres(dw, dh);
  src->uploadTo( hsrc.data );
  int   x, y, spos, dpos, dnxt;
  // duplicate the last row
  float *py = hsrc.data + (h-1)*w, *pz = hsrc.data + h*w;
  for (x = 0; x < w; x++) pz[x] = py[x];  
  // 
  for (y = 0; y < h-1; y++) {
    for (x = 0; x < w; x++) {
      spos =   y *  w +   x;
      dpos = 2*y * dw + 2*x;
      dnxt = (2*y+1) * dw + 2*x;
      hres.data[dpos]   = (hsrc.data[spos]);
      hres.data[dpos+1] = (hsrc.data[spos] + hsrc.data[spos+1])/2;
      hres.data[dnxt]   = (hsrc.data[spos] + hsrc.data[spos+w])/2;
      hres.data[dnxt+1] = (hsrc.data[spos] + hsrc.data[spos+1] + hsrc.data[spos+w] + hsrc.data[spos+w+1])/4;
    }
  }
  // duplicate the last column
  pz = hres.data + dw-1;
  for (y = 0; y < dh-1; y++, pz+=dw) pz[0] = pz[-1];
  // 
  res->downloadFrom( hres.data );
#endif

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("cimg_scale_up time =              %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// 
// ===================================================================

__global__ void th_sample_down(float *dst, float *src, int width, int height)
{
  // Downsample the 'src' and set the pixels of 'dst', which is half of the size.
  //   'width' and 'height' is the size of 'src', not 'dst', and 
  //   the thread blocks(16x16) must divide up the size of 'dst'.
  const int hw = width/2, hh = height/2;
  const int x = __mul24(16, blockIdx.x) + threadIdx.x;
  const int y = __mul24(16, blockIdx.y) + threadIdx.y;
  if (x >= hw || y >= hh) return;
  int dpos = __mul24(2, __mul24(y, width)) + __mul24(2, x);
  float sum = (src[dpos] + src[dpos+1] + src[dpos+width] + src[dpos+width+1])/4;
  dst[ __mul24(y, hw) + x ] = sum;
  __syncthreads();
}

double cimg_scale_down(DevImage *res, DevImage *src, float variance)
{
  int w = src->w;
  int h = src->h;
  if (res->data==NULL || src->data==NULL) {
    // printf("cimg_scale_down: missing data\n");
    return 0.0;
  }
  unsigned int hTimer; 
  timer_start(&hTimer);
  int  hw = w/2,  hh = h/2;
  dim3 blocks(iDivUp(hw,16), iDivUp(hh,16));
  dim3 threads(16,16);
  th_sample_down<<<blocks, threads>>>(res->data, src->data, w, h); 
  CUT_CHECK_ERROR("cimg_scale_down() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("cimg_scale_down time =              %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// 
// ===================================================================

__global__ void th_copy(float *d_dst, float *d_src, int width, int height)
{
  const int x = __mul24(blockIdx.x, 16) + threadIdx.x;
  const int y = __mul24(blockIdx.y, 16) + threadIdx.y;
  int p = __mul24(y, width) + x;
  if (x<width && y<height) d_dst[p] = d_src[p];
  __syncthreads();
}

double cimg_copy(DevImage *dst, DevImage *src)
{
  int w = dst->w;
  int h = dst->h;
  if (dst->data==NULL || src->data==NULL) {
    // printf("cimg_copy: missing data\n");
    return 0.0;
  }
  unsigned int hTimer;
  timer_start(&hTimer);
  dim3 blocks(iDivUp(w, 16), iDivUp(h, 16));
  dim3 threads(16, 16);
  th_copy<<<blocks, threads>>>(dst->data, src->data, w, h);
  CUT_CHECK_ERROR("cimg_copy() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("cimg_copy time =               %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// 
// ===================================================================

__global__ void th_subtract(float *d_Result, float *d_Data1, float *d_Data2, int width, int height)
{
  const int x = __mul24(blockIdx.x, 16) + threadIdx.x;
  const int y = __mul24(blockIdx.y, 16) + threadIdx.y;
  int p = __mul24(y, width) + x;
  if (x<width && y<height) d_Result[p] = d_Data1[p] - d_Data2[p];
  __syncthreads();
}

double cimg_subtract(DevImage *res, DevImage *dataA, DevImage *dataB)
{    
  int w = res->w;
  int h = res->h;
  unsigned int hTimer;
  timer_start(&hTimer);
  if (res->data==NULL || dataA->data==NULL || dataB->data==NULL) {
    // printf("cimg_subtract: missing data\n");
    return 0.0;
  }
  dim3 blocks(iDivUp(w, 16), iDivUp(h, 16));
  dim3 threads(16, 16);
  th_subtract<<<blocks, threads>>>(res->data, dataA->data, dataB->data, 
				w, h);
  CUT_CHECK_ERROR("cimg_subtract() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("cimg_subtract time =               %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// 
// ===================================================================

__global__ void th_multiply_add(float *d_Result, float *d_Data, int width, int height)
{
  const int x = __mul24(blockIdx.x, 16) + threadIdx.x;
  const int y = __mul24(blockIdx.y, 16) + threadIdx.y;
  int p = __mul24(y, width) + x;
  if (x<width && y<height) d_Result[p] = edit_const[0]*d_Data[p] + edit_const[1];
  __syncthreads();
}

double cimg_multiply_add(DevImage *res, DevImage *data, float constA, float constB)
{
  int w = res->w;
  int h = res->h;
  if (res->data==NULL || data->data==NULL) {
    // printf("cimg_multiply_add: missing data\n");
    return 0.0;
  }
  unsigned int hTimer;
  float values[2] = { constA, constB };
  timer_start(&hTimer);
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(edit_const, &values, 2*sizeof(float)));

  dim3 blocks(iDivUp(w, 16), iDivUp(h, 16));
  dim3 threads(16, 16);
  th_multiply_add<<<blocks, threads>>>(res->data, data->data, w, h);
  CUT_CHECK_ERROR("cimg_multiply_add() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("cimg_multiply_add time =            %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}
