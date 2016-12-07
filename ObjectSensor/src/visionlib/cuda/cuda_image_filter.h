
#ifndef CUDA_FILTER_H
#define CUDA_FILTER_H

#define WARP_SIZE     16

#define ROW_TILE_W    160
#define COLUMN_TILE_W 16
#define COLUMN_TILE_H 48

__device__ __constant__ float d_Kernel[17]; // NOTE: Maximum radius 

///////////////////////////////////////////////////////////////////////////////
// Loop unrolling templates, needed for best performance
///////////////////////////////////////////////////////////////////////////////
template<int i> 
__device__ float convRow(float *data)
{
    return data[i]*d_Kernel[i] + convRow<i-1>(data);
}

template<> 
__device__ float convRow<-1>(float *data)
{
    return 0;
}

template<int i> 
__device__ float convCol(float *data)
{
    return data[i*COLUMN_TILE_W]*d_Kernel[i] + convCol<i-1>(data);
}

template<> 
__device__ float convCol<-1>(float *data)
{
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Row convolution filter
///////////////////////////////////////////////////////////////////////////////
template<int RADIUS>
__global__ void convRowGPU(float *d_Result, float *d_Data,
  int width, int height)
{
  //Data cache
  __shared__ float data[RADIUS+ROW_TILE_W+RADIUS+1];
  //Current tile and apron limits, relative to row start
  const int tileStart = __mul24(blockIdx.x, ROW_TILE_W);

  //Row start index in d_Data[]
  const int rowStart = __mul24(blockIdx.y, width);
  const int rowEnd = rowStart + width - 1;
  const int loadPos = threadIdx.x - WARP_SIZE + tileStart;
  const int smemPos =  threadIdx.x - WARP_SIZE + RADIUS;

  //Set the entire data cache contents
  if (smemPos>=0) {
    if (loadPos<0)
      data[smemPos] = d_Data[rowStart];
    else if (loadPos>=width) 
      data[smemPos] = d_Data[rowEnd];
    else
      data[smemPos] = d_Data[rowStart + loadPos];
  }
  __syncthreads();
  
  //Clamp tile and apron limits by image borders
  const int tileEnd = tileStart + ROW_TILE_W - 1;
  const int tileEndClamped = min(tileEnd, width - 1);
  const int writePos = tileStart + threadIdx.x;
  
  if (writePos <= tileEndClamped){ 
    const int smemPos = threadIdx.x + RADIUS;
    d_Result[rowStart + writePos] = 
      convRow<2 * RADIUS>(data + smemPos - RADIUS);;
  }
  __syncthreads();
}

///////////////////////////////////////////////////////////////////////////////
// Column convolution filter
///////////////////////////////////////////////////////////////////////////////
template<int RADIUS>
__global__ void convColGPU(float *d_Result, float *d_Data, int width,
  int height, int smemStride, int gmemStride)
{
  // Data cache
  __shared__ float data[COLUMN_TILE_W*(RADIUS + COLUMN_TILE_H + RADIUS+1)];

  // Current tile and apron limits, in rows
  const int tileStart = __mul24(blockIdx.y, COLUMN_TILE_H);
  const int tileEnd = tileStart + COLUMN_TILE_H - 1;
  const int apronStart = tileStart - RADIUS;
  const int apronEnd = tileEnd + RADIUS;
  
  // Current column index
  const int columnStart = __mul24(blockIdx.x, COLUMN_TILE_W) + threadIdx.x;
  const int columnEnd = columnStart + __mul24(height-1, width);
    
  if (columnStart<width) {
    // Shared and global memory indices for current column
    int smemPos = __mul24(threadIdx.y, COLUMN_TILE_W) + threadIdx.x;
    int gmemPos = __mul24(apronStart + threadIdx.y, width) + columnStart;
    // Cycle through the entire data cache
    for (int y = apronStart + threadIdx.y; y <= apronEnd; y += blockDim.y){
      if (y<0) 
	data[smemPos] = d_Data[columnStart];
      else if (y>=height) 
	data[smemPos] = d_Data[columnEnd];
      else 
	data[smemPos] = d_Data[gmemPos];  
      smemPos += smemStride;
      gmemPos += gmemStride;
    }
  }
  __syncthreads();

  if (columnStart<width) {
    // Shared and global memory indices for current column
    // Clamp tile and apron limits by image borders
    const int tileEndClamped = min(tileEnd, height - 1);
    int smemPos = __mul24(threadIdx.y + RADIUS, COLUMN_TILE_W) + threadIdx.x;
    int gmemPos = __mul24(tileStart + threadIdx.y , width) + columnStart;
    // Cycle through the tile body, clamped by image borders
    // Calculate and output the results
    for (int y=tileStart+threadIdx.y;y<=tileEndClamped;y+=blockDim.y){
      d_Result[gmemPos] = 
	convCol<2*RADIUS>(data + smemPos - RADIUS*COLUMN_TILE_W);;
      smemPos += smemStride;
      gmemPos += gmemStride;
    }
  }
  __syncthreads();
}


///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
#  define CUT_CHECK_ERROR2(errorMessage) do {                                 \
    cudaError_t err = cudaGetLastError();                                    \
    if( cudaSuccess != err) {                                                \
        fprintf(stderr, "CHECK_ERROR: %s in file '%s' in line %i : %s.\n",    \
                errorMessage, __FILE__, __LINE__, cudaGetErrorString( err) );\
    }                                                                        \
    err = cudaThreadSynchronize();                                           \
    } while (0)

template<int RADIUS>
double SeparableFilter(DevImage *dataA, DevImage *dataB, DevImage *temp, float *h_Kernel)
{
  unsigned int width = dataA->w;
  unsigned int height = dataA->h;
  float *d_DataA = dataA->data;
  float *d_DataB = dataB->data;
  float *d_Temp = temp->data;
  if (d_DataA==NULL || d_DataB==NULL || d_Temp==NULL) {
    printf("SeparableFilter: missing data\n");
    return 0.0;
  }
  unsigned int hTimer;
  CUT_SAFE_CALL(cutCreateTimer(&hTimer));
  const unsigned int kernelSize = (2*RADIUS+1)*sizeof(float);
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(d_Kernel, h_Kernel, kernelSize));
  CUT_SAFE_CALL(cutResetTimer(hTimer));
  CUT_SAFE_CALL(cutStartTimer(hTimer));
        
#if 1
  dim3 blockGridRows(iDivUp(width, ROW_TILE_W), height);
  dim3 threadBlockRows(WARP_SIZE + ROW_TILE_W + RADIUS);
  convRowGPU<RADIUS><<<blockGridRows, threadBlockRows>>>
    (d_Temp, d_DataA, width, height); //%%%%
  CUT_CHECK_ERROR("convRowGPU() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
#endif
#if 1
  dim3 blockGridColumns(iDivUp(width, COLUMN_TILE_W), iDivUp(height, COLUMN_TILE_H));
  dim3 threadBlockColumns(COLUMN_TILE_W, 8);
  convColGPU<RADIUS><<<blockGridColumns, threadBlockColumns>>>
    (d_DataB, d_Temp, width, height, COLUMN_TILE_W*8, width*8); 
  CUT_CHECK_ERROR("convColGPU() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
#endif
  CUT_SAFE_CALL(cutStopTimer(hTimer));
  double gpuTime = cutGetTimerValue(hTimer);
#ifdef VERBOSE
  printf("SeparableFilter time =        %.2f msec\n", gpuTime);
#endif
  CUT_SAFE_CALL(cutDeleteTimer(hTimer));
  return gpuTime;
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
template<int RADIUS>
double cimg_lowpass(DevImage *dst, DevImage *src, DevImage *temp, double var)
{
  // Gaussian smoothing
  if (src->data == NULL || dst->data == NULL || temp->data == NULL) {
    printf("cimg_lowpass<>: missing data\n");
    return 0;
  }
  float kernel[2*RADIUS+1];
  float kernelSum = 0.0f;
  // calculate Gaussian filter
  for (int j=-RADIUS;j<=RADIUS;j++) {
    kernel[j+RADIUS] = (float)expf(-(double)j*j/2.0/var);
    kernelSum += kernel[j+RADIUS];
  }
  for (int j=-RADIUS;j<=RADIUS;j++) kernel[j+RADIUS] /= kernelSum;  
  // 
  return SeparableFilter<RADIUS>(src, dst, temp, kernel); 
}

#endif  // CUDA_FILTER_H
