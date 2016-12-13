
#include <stdio.h>
#include "cutil.h"
#include "../common/vm_macros.h"
#include "cuda_image_edit.h"
#include "cuda_image_filter.h"
#include "cuda_sift.h"

#define Pi 3.1415926536f
__device__ __constant__ float sift_const[4];
static bool cds_verbose = false;

void check_img(DevImage *cimg, char *cmmt, int oct, int lvl, bool print_on, bool save_on);
void check_sift(DevImage *sift, int cnt, int *ptrs=NULL, int iw=0, int ih=0);
int  csift_extract_recur(HstImage *flist, DevImage *img, int nOctaves, int nLevels,
			 float sigma, float ethreshold, float subsampling, int octave, int xywh[4]);

// ===================================================================
// ===================================================================

bool cuda_sift_extract(DevImage *img, HstImage *flist, int nOctaves, int nLevels,
		       float sigma, float ethreshold, int xywh[4], bool verbose)
{
  // Extract SIFT features from input image 'img', and save them in 'flist'.
  //   'img'  : FLOAT image on CUDA device memory with arbitrary width and height.
  //   'flist': FLOAT image on host memory with size 140 x N, 
  //            each row representing a SIFT feature, totaling N features.
  //            ( x,y,s,sh,  edge,ori,sco,amb,  m,mx,my,me,  desc[128] )
  if (!img || img->w <= 0 || img->h <= 0 || img->data == NULL) {
    printf("SIFT ERROR: invalid input image\n");
    return false;
  }
  int    w = img->w,  h = img->h;
  cds_verbose = verbose;
  // initialize the data structure for SIFT features
  // Note that memory buffer of 'flist' is preserved.
  flist->setImage( 140, 0 );
  
  // decide the number of octaves
  bool upsampling = (nOctaves < 0 ? true : false);
  if (nOctaves < 1) {
    // optionally autoselect the number of octaves. (downsample up to 8x8 patches)
    nOctaves = int(std::floor(log((double)(w<h ? w:h))/log(2.0)) + (upsampling?1:0) -3);
    if (nOctaves < 1) nOctaves = 1;
  }
  // set the original image
  DevImage cimg;
  // set the input image
  int   omin,  xywh2[4];
  float subsampling = 1.0f;
  if (upsampling) {		// input image : up-sampling
    if (cds_verbose) printf("SIFT options: nOct=%d nLev=%d sigma=%.2f eth=%.2f with upsampling\n", nOctaves, nLevels, sigma, ethreshold);
    DevImage timg, bimg;
    cimg.setImage( w*2, h*2 );
    bimg.setImage( w*2, h*2 );
    timg.setImage( w*2, h*2 );
    if (timg.data==NULL) {
      printf("Error (cuda_sift_extract): failed to allocate CUDA device memory\n");
      return false;
    }
    double tt = cimg_scale_up( &bimg, img );
    double ret = cimg_lowpass<7>(&cimg, &bimg, &timg, 1.23*1.23) ;
    omin = -1;  subsampling = 0.5f;
    if (xywh) { 
      xywh2[0]=2*xywh[0]; xywh2[1]=2*xywh[1]; xywh2[2]=2*xywh[2]; xywh2[3]=2*xywh[3];
      xywh = xywh2;
    }
  } else if (sigma > 0) {	// input image : smoothing
    if (cds_verbose) printf("SIFT options: nOct=%d nLev=%d sigma=%.2f eth=%.2f with smoothing\n", nOctaves, nLevels, sigma, ethreshold);
    DevImage timg;
    cimg.setImage( w, h );
    timg.setImage( w, h );
    if (cimg.data==NULL || timg.data==NULL) {
      printf("Error (cuda_sift_extract): failed to allocate CUDA device memory\n");
      return false;
    }
    double ret = cimg_lowpass<7>(&cimg, img, &timg, sigma*sigma) ;
    omin = 0;  subsampling = 1.0f;
    ////CUT_CHECK_ERROR2("cuda_sift_extract 0");
  } else {			// input image : copying
    if (cds_verbose) printf("SIFT options: nOct=%d nLev=%d sigma=%.2f eth=%.2f with copying\n", nOctaves, nLevels, sigma, ethreshold);
    cimg.setImage( w, h );
    if (cimg.data==NULL) {
      printf("Error (cuda_sift_extract): failed to allocate CUDA device memory\n");
      return false;
    }
    cimg_copy( &cimg, img );
  }
  if (cimg.data == NULL) return false;
  // extract the SIFT features
  ////printf("nO=%d nL=%d sig=%g ethresh=%g\n", nOctaves, nLevels, sigma, ethreshold);
  int cnt = csift_extract_recur( flist, &cimg, nOctaves, nLevels,
				 sigma, ethreshold, subsampling, omin, xywh);
  //   HstImage 'flist' is 140 x N float image, with a SIFT feature stored in each row.
  //   Each SIFT feature consists of 140 float values, including x,y,s,o and 128 descriptor.
  //   ( SIFTFeature[140]:  x,y,s,sh,  edge,ori,sco,amb,  desc[128],  dummy[4] )
  if (cds_verbose) { printf("Total number of SIFT features : %d \n", cnt); }
  
  return true;
}


// ===================================================================
// ===================================================================

int csift_extract_octave(HstImage *flist, DevImage *img, int nOctave, int nLevels, 
			 float sigma, float ethreshold, float subsampling, int octave, int xywh[4]);
double csift_find_3d_min_max(DevImage *minmax, DevImage *data1, DevImage *data2, 
			     DevImage *data3, float thresh, int maxPts);
double csift_unpack_pointers(DevImage *minmax, int maxPts, int *ptrs, int *numPts, int xywh[4]);
double csift_compute_positions(DevImage *data1, DevImage *data2, DevImage *data3,
			       int *h_ptrs, DevImage *sift, int numPts, int maxPts,
			       float sigma0, int oct, int lvl, int nLevels);
double csift_remove_edge_points(DevImage *sift, int *initNumPts, int maxPts, float ethreshold, int w, int h, float subsampling);
double csift_compute_orientations(DevImage *img, DevImage *sift, int numPts, int maxPts, float subsampling);
double csift_second_orientations(DevImage *sift, int *initNumPts, int maxPts);
double csift_extract_descriptors(DevTexture *timg, DevImage *sift,
				 DevImage *desc, int numPts, int maxPts, float subsampling);
double csift_add_sift_data(HstImage *flist, float *d_sift, float *d_desc,
			   int numPts, int maxPts, float subsampling);


int csift_extract_recur(HstImage *flist, DevImage *img, int nOctaves, int nLevels,
			float sigma, float ethreshold, float subsampling, int octave, int xywh[4])
{
  	if (octave >= nOctaves) return 0;
	int w=img->w, h=img->h, cnt=0;
  	// extract SIFT feature
  	cnt = csift_extract_octave( flist, img, nOctaves, nLevels,
			      sigma, ethreshold, subsampling, octave, xywh );
  	// call itself recursively, to process another octave
  	if (cds_verbose) { printf("octave:%d, nO:%d\n",octave,nOctaves); }
  	if (octave < nOctaves-1) {
		DevImage subImg;
    	bool ret = subImg.setImage( w/2, h/2 );
    	if (ret) {
      		cimg_scale_down(&subImg, img, 0.2f);
      		if (xywh) {
				int xywh2[4] = { xywh[0]/2, xywh[1]/2, xywh[2]/2, xywh[3]/2 };
				cnt += csift_extract_recur( flist, &subImg, nOctaves, nLevels,
					    sigma, ethreshold, subsampling*2.0f, octave+1, xywh2 );
  			} else {
				cnt += csift_extract_recur( flist, &subImg, nOctaves, nLevels,
				    	sigma, ethreshold, subsampling*2.0f, octave+1, NULL );
      		}
    	} else {
      		CUT_CHECK_ERROR2("csift_extract_recur 0");
      		printf("CUDA Error(csift_extract_recur): failed to allocate device memory oct=%d (%dx%d)\n", octave+1, img->w/2, img->h/2);
    	}
  	}
 	return cnt;
}

int csift_extract_octave(HstImage *flist, DevImage *img, int nOctave, int nLevels, 
			 float sigma, float ethreshold, float subsampling, int octave, int xywh[4])
{
	const int maxPts = 1024*3;
  	int w = img->w; 
  	int h = img->h;
  	DevImage blurImg[3], *bimg1, *bimg0;
  	DevImage diffImg[3];
  	blurImg[0].setImage(w,h);  blurImg[1].setImage(w,h);  blurImg[2].setImage(w,h);
  	diffImg[0].setImage(w,h);  diffImg[1].setImage(w,h);  diffImg[2].setImage(w,h);
  	DevImage tmpImg(w,h);
  	DevTexture textImg(w,h);
  	DevImage minmax(w, iDivUp(h,32));
  	DevImage sift( maxPts, 7 ); // { xpos, ypos, scale, value, edge, orient1, orient2 };
  	DevImage desc( 128, maxPts );
  	if (desc.data == NULL) {
    	printf("CUDA Error(csift_extract_octave): failed to allocate device memory oct=%d (%dx%d)\n", octave, img->w, img->h);
    	return 0;
  	}
  	// calculate the variance of the Gaussian filter for each level
  	float sii[10], sig[10]={0,0,0,0,0, 0,0,0,0,0};
  	if (true) {
    	float sk = powf( 2.0f, 1.0f/nLevels );
    	float sq_k2_1 = sqrtf( sk*sk - 1 );
    	for (int i=0; i<nLevels+3; i++) {
      		sig[i] = sigma * powf( sk, i );		// sigma at each level
      		sii[i] = (i==0 ? 0 : sq_k2_1 * sig[i-1]);	// sigma for incremental filtering
    	}
  	}
  	CUT_CHECK_ERROR2("csift_extract_octave 0");
  	CUDA_SAFE_CALL(cudaThreadSynchronize());
  	float  threshold = 0.04f / nLevels / 2.0f;
  	double etime[]={0,0,0,0,0, 0,0,0,0,0, 0,0};
  	int totPts = 0;
  	int *ptrs = (int *)malloc(sizeof(int)*maxPts);
  	cudaError_t err;
  	
  	for (int i=0; i<nLevels+3; i++) {
    	if(i == 0) {
	      	if (cds_verbose) { printf("O=%d/%d L=%d/%d ", octave, nOctave, i, nLevels); check_img(img, "zcBlur", octave, i, true, false); }
    	} else if (i >  0) { 
      		bimg1 = (i==1 ? img : &blurImg[(i-1)%3]);
      		bimg0 = &blurImg[i%3];
      		// blur the next level image
      		etime[0] += cimg_lowpass<7>( bimg0, bimg1, &tmpImg, sii[i]*sii[i] );
			if (cds_verbose) { printf("      L=%d/%d s=%.4f ", i, nLevels, sii[i]); check_img(bimg0, "zcBlur", octave, i, false, false); }
      		// get the difference image
      		etime[1] += cimg_subtract(&diffImg[(i)%3], bimg0, bimg1 );
      		if (cds_verbose) {
      			printf("Diff(%d) ", i%3);
      		}
      		//check_img( &diffImg[(i)%3], "zcDiff", octave, i, false, false );
      		err = cudaGetLastError();
      		if (err != cudaSuccess) {
      			if (cds_verbose) printf("=> error\n");
  			} else if (i < 3) {
  				if (cds_verbose) printf("\n");
			} else {
				// find the local maxima
				if (cds_verbose) { printf("SIFT(%d,%d,%d) ", (i)%3, (i-1)%3, (i-2)%3); }
				etime[2] += csift_find_3d_min_max(&minmax, &diffImg[(i)%3], &diffImg[(i-1)%3], &diffImg[(i-2)%3], threshold, maxPts);
				int numPts = 0;
				etime[3] += csift_unpack_pointers(&minmax, maxPts, ptrs, &numPts, xywh);
				if (numPts<=0) {
					if (cds_verbose) printf("\n");
				} else {
	  				etime[5] += csift_compute_positions(&diffImg[(i)%3], &diffImg[(i-1)%3], &diffImg[(i-2)%3], 
					      ptrs, &sift, numPts, maxPts, sig[0], octave, i-2, nLevels);
	  				CUT_CHECK_ERROR2("csift_extract_octave 5");
	  				//check_sift( &sift, numPts, ptrs, bimg1->w, bimg1->h );
	  				etime[4] += textImg.copyFrom( &blurImg[(i-2)%3] );
	  				etime[6] += csift_remove_edge_points(&sift, &numPts, maxPts, ethreshold, bimg0->w, bimg0->h, subsampling);
	  				if (cds_verbose) printf("  numPts = %d / %d   scale (%d,%d/%d)\n", numPts, maxPts, octave, (i-2), nLevels);
	  				if (numPts <= 0) {
	  					if (cds_verbose) printf("\n"); continue;
  					}
	  				etime[7] += csift_compute_orientations(&blurImg[(i-2)%3], &sift, numPts, maxPts, subsampling);
	  				etime[8] += csift_second_orientations(&sift, &numPts, maxPts);
	  				etime[9] += csift_extract_descriptors(&textImg, &sift, &desc, numPts, maxPts, subsampling);
	  				etime[10] += csift_add_sift_data(flist, sift.data, desc.data, numPts, maxPts, subsampling);
	  				err = cudaGetLastError();
	  				if (err != cudaSuccess) numPts = 0;  // CUDA error
	  				if (cds_verbose) {
	    				if (err != cudaSuccess) printf("=> error\n");  // CUDA error
	    				else printf("=> %d features\n", numPts);
	  				}
	  				totPts += numPts;
				}
      		}
    	}
    	// copy the blurred image at the top-2 level to the input image,
    	//   so that it could be used as the starting point for the next octave.
    	if (i==nLevels) cimg_copy( img, bimg0 );
  	}
	if (cds_verbose) {
	    printf("  Number of features at this octave: %d\n", totPts);
    	//flist->printInfo("  flist", "%.2f ", 0, flist->h-totPts, 2, totPts);
    	for (int i=0; i<11; i++) etime[11] += etime[i];
    	printf("   Filt  Subt  MiMx  Unpk  Text  Posi  Edge  Orin  Ori2  Desc  Addf   Total \n");
    	printf("  %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f  %6.2f \n",
	   	etime[0], etime[1], etime[2], etime[3], etime[4], etime[5], etime[6], etime[7], etime[8], etime[9], etime[10], etime[11]);
	}
  	CUT_CHECK_ERROR2("csift_extract_octave 9");
  	CUDA_SAFE_CALL(cudaThreadSynchronize());
  	free(ptrs);
  	
  	return totPts;
}

// ===================================================================
// ===================================================================
#define MINMAX_SIZE   128

__global__ void th_find_3d_min_max(int *d_Result, float *d_Data1, float *d_Data2, 
				   float *d_Data3, int width, int height)
{
  //Data cache
  __shared__ float data1[3*(MINMAX_SIZE + 2)];
  __shared__ float data2[3*(MINMAX_SIZE + 2)];
  __shared__ float data3[3*(MINMAX_SIZE + 2)];
  __shared__ float ymin1[(MINMAX_SIZE + 2)];
  __shared__ float ymin2[(MINMAX_SIZE + 2)];
  __shared__ float ymin3[(MINMAX_SIZE + 2)];
  __shared__ float ymax1[(MINMAX_SIZE + 2)];
  __shared__ float ymax2[(MINMAX_SIZE + 2)];
  __shared__ float ymax3[(MINMAX_SIZE + 2)];

  //Current tile and apron limits, relative to row start
  const int tx = threadIdx.x;
  const int xStart = __mul24(blockIdx.x, MINMAX_SIZE);
  const int xEnd = xStart + MINMAX_SIZE - 1;
  const int xReadPos = xStart + tx - WARP_SIZE;
  const int xWritePos = xStart + tx;
  const int xEndClamped = min(xEnd, width - 1);
  int memWid = MINMAX_SIZE + 2;

  int memPos0 = (tx - WARP_SIZE + 1);
  int memPos1 = (tx - WARP_SIZE + 1);
  int yq = 0;
  unsigned int output = 0;
  for (int y=0;y<34;y++) {

    output >>= 1;
    int memPos =  yq*memWid + (tx - WARP_SIZE + 1);
    int yp = 32*blockIdx.y + y - 1;
    yp = max(yp, 0);
    yp = min(yp, height-1);
    int readStart = __mul24(yp, width);

    //Set the entire data cache contents
    if (tx>=(WARP_SIZE-1)) {
      if (xReadPos<0) {
	data1[memPos] = 0;
	data2[memPos] = 0;
	data3[memPos] = 0;
      } else if (xReadPos>=width) {
	data1[memPos] = 0;
	data2[memPos] = 0;
	data3[memPos] = 0;
      } else {
	data1[memPos] = d_Data1[readStart + xReadPos];
	data2[memPos] = d_Data2[readStart + xReadPos];
	data3[memPos] = d_Data3[readStart + xReadPos];
	//if ((readStart + xReadPos)<0 || (readStart + xReadPos)>=width*height)
	//  printf("csift_find_3d_min_max: read error\n");
      }
    }
    __syncthreads();
  
    int memPos2 = yq*memWid + tx;
    if (y>1) {
      if (tx<memWid) {
	float min1 = fminf(fminf(data1[memPos0], data1[memPos1]), data1[memPos2]);
	float min2 = fminf(fminf(data2[memPos0], data2[memPos1]), data2[memPos2]);
	float min3 = fminf(fminf(data3[memPos0], data3[memPos1]), data3[memPos2]);
	float max1 = fmaxf(fmaxf(data1[memPos0], data1[memPos1]), data1[memPos2]);
	float max2 = fmaxf(fmaxf(data2[memPos0], data2[memPos1]), data2[memPos2]);
	float max3 = fmaxf(fmaxf(data3[memPos0], data3[memPos1]), data3[memPos2]);
	ymin1[tx] = min1;
	ymin2[tx] = fminf(fminf(min1, min2), min3);
	ymin3[tx] = min3;
	ymax1[tx] = max1;
	ymax2[tx] = fmaxf(fmaxf(max1, max2), max3);
	ymax3[tx] = max3;
      }
    }
    __syncthreads();

    if (y>1) {
      if (tx<MINMAX_SIZE) {
	if (xWritePos<=xEndClamped) {
	  float minv = fminf(fminf(fminf(fminf(fminf(ymin2[tx], ymin2[tx+2]), 
	    ymin1[tx+1]), ymin3[tx+1]), data2[memPos0+1]), data2[memPos2+1]);
	  minv = fminf(minv, sift_const[1]);
	  float maxv = fmaxf(fmaxf(fmaxf(fmaxf(fmaxf(ymax2[tx], ymax2[tx+2]),
	    ymax1[tx+1]), ymax3[tx+1]), data2[memPos0+1]), data2[memPos2+1]);
	  maxv = fmaxf(maxv, sift_const[0]);
	  if (data2[memPos1+1]<minv || data2[memPos1+1]>maxv)
	    output |= 0x80000000;
	}
      }
    }
    __syncthreads();

    memPos0 = memPos1;
    memPos1 = memPos2;
    yq = (yq<2 ? yq+1 : 0);
  }
  if (tx<MINMAX_SIZE && xWritePos<width) {
    int writeStart = __mul24(blockIdx.y, width) + xWritePos;
    d_Result[writeStart] = output;
    //if (writeStart<0 || writeStart>=width*iDivUp(height,32))
    //  printf("csift_find_3d_min_max: write error\n");
  }
}

double csift_find_3d_min_max(DevImage *minmax, DevImage *data1, DevImage *data2, 
			     DevImage *data3, float thresh, int maxPts)
{
  int *d_res = (int *)minmax->data;
  if (data1->data==NULL || data2->data==NULL || data3->data==NULL || d_res==NULL) {
    printf("Find3DMinMax: missing data %08x %08x %08x %08x\n", 
	   data1->data, data2->data, data3->data, d_res);
    return 0.0;
  }
  int w = data1->w;
  int h = data1->h;
  unsigned int hTimer;
  timer_start(&hTimer);
  float threshs[2] = { +thresh*0.8f, -thresh*0.8f };   ////
  //float threshs[2] = { +0.03f, -0.03f }; ////
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(sift_const, &threshs, 2*sizeof(float)));

  dim3 blocks(iDivUp(w, MINMAX_SIZE), iDivUp(h,32));
  dim3 threads(WARP_SIZE + MINMAX_SIZE + 1);
  th_find_3d_min_max<<<blocks, threads>>>(d_res, data1->data, data2->data, data3->data, w, h); 
  CUT_CHECK_ERROR("Find3DMinMax() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("Find3DMinMax time =           %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// ===================================================================

// __global__ void th_unpack_pointers(int *minmax, int *ptrs, int w, int h, int maxPts)
// {
//   const int tx = threadIdx.x;
//   int numPts = 0;
//   for (int y=0;y<h/32;y++) {
//     for (int x=0;x<w;x+=16) {
//       unsigned int val = minmax[y*w+x+tx];
//       if (val) {
// 	for (int k=0;k<32;k++) {
// 	  if (val&0x1 && numPts<maxPts) {
// 	    ptrs[16*numPts+tx] = (y*32+k)*w + x+tx;
// 	    numPts++;
// 	  }
// 	  val >>= 1;
// 	}
//       }
//     }
//   } 
// }

double csift_unpack_pointers(DevImage *minmax, int maxPts, int *ptrs, int *numPts, int xywh[4])
{
  HstImage himg;
  himg.setImage( minmax->w, minmax->h );
  himg.uploadFrom( minmax );
  unsigned int *minmax_data = (unsigned int *)himg.data;
  if (minmax_data==NULL || ptrs==NULL) {
    printf("UnpackPointers: missing data %08x %08x\n", minmax_data, ptrs);
    return 0.0;
  }
  int w = minmax->w;
  int h = 32*minmax->h;
  unsigned int timer = 0;
  timer_start(&timer);
  int num = 0, xymm[4];
  if (xywh) {
    xymm[0]=xywh[0]; xymm[1]=xywh[1]; xymm[2]=xywh[0]+xywh[2]; xymm[3]=xywh[1]+xywh[3]; 
    if (w < xymm[2]) xymm[2] = w;
    if (h < xymm[3]) xymm[3] = h;
  } else { xymm[0]=0; xymm[1]=0; xymm[2]=w; xymm[3]=h; }
  for (int yy=xymm[1]/32;yy<h/32;yy++) {
    if (yy > xymm[3]/32) break;
    for (int x=xymm[0];x<xymm[2];x++) {
      if (xymm[0]==0) { if (x==0 || x == xymm[2]-1) continue; }  // on boundary
      unsigned int val = minmax_data[yy*w+x];
      if (val) {
	//printf("%d %d %08x\n", x, yy, val);
	for (int k=0;k<32;k++) {
	  if (val&0x1 && num<maxPts) {
	    int y = yy * 32 + k;
	    if (xymm[1]==0) { if (y==0 || y == xymm[3]-1) continue; }  // on boundary
	    if (y >= xymm[1] && y < xymm[3]) ptrs[num++] = y*w + x;
	  }
	  val >>= 1;
	}
      }
    }
  }
  *numPts = num;
  double gpuTime = timer_stop(timer);
#ifdef VERBOSE
  printf("UnpackPointers time =         %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// Compute precise positions in xpos, ypos and scale
// ===================================================================
#define POSBLK_SIZE   32

__global__ void th_compute_positions(float *g_Data1, float *g_Data2, float *g_Data3, 
				     int *d_Ptrs, float *d_Sift, int numPts, int maxPts, int w, int h)
{
  // Calculate exact position of the keypoint on 'g_Data2' and save it in 'd_Sift', 
  //   by fitting 3D quadratic surface to the values in the neighborhood 
  //   including 'g_Data1' (one level above) and 'g_Data3' (below).
  int i = __mul24(blockIdx.x, POSBLK_SIZE) + threadIdx.x;
  if (i>=numPts) return;
  int p = d_Ptrs[i];
  float val[7];
  val[0] = g_Data2[p];
  val[1] = g_Data2[p-1];
  val[2] = g_Data2[p+1];
  float dx = 0.5f*(val[2] - val[1]);
  float dxx = 2.0f*val[0] - val[1] - val[2];
  val[3] = g_Data2[p-w];
  val[4] = g_Data2[p+w];
  float dy = 0.5f*(val[4] - val[3]); 
  float dyy = 2.0f*val[0] - val[3] - val[4];
  val[5] = g_Data3[p];
  val[6] = g_Data1[p];
  float ds = 0.5f*(val[6] - val[5]); 
  float dss = 2.0f*val[0] - val[5] - val[6];
  //// Note this part be hung up if 'p' is invalid, and may return 'unspecified launch failure'
  float dxy = 0.25f*(g_Data2[p+w+1] + g_Data2[p-w-1] - g_Data2[p-w+1] - g_Data2[p+w-1]);
  float dxs = 0.25f*(g_Data3[p+1] + g_Data1[p-1] - g_Data1[p+1] - g_Data3[p-1]);
  float dys = 0.25f*(g_Data3[p+w] + g_Data1[p-w] - g_Data3[p-w] - g_Data1[p+w]);
  float idxx = dyy*dss - dys*dys;
  float idxy = dys*dxs - dxy*dss;  
  float idxs = dxy*dys - dyy*dxs;
  float idyy = dxx*dss - dxs*dxs;
  float idys = dxy*dxs - dxx*dys;
  float idss = dxx*dyy - dxy*dxy;
  float det = idxx*dxx + idxy*dxy + idxs*dxs;
  float idet = 1.0f / det;
  float pdx = idet*(idxx*dx + idxy*dy + idxs*ds);
  float pdy = idet*(idxy*dx + idyy*dy + idys*ds);
  float pds = idet*(idxs*dx + idys*dy + idss*ds);
  if (pdx<-0.5f || pdx>0.5f || pdy<-0.5f || pdy>0.5f || pds<-0.5f || pds>0.5f){
    pdx = __fdividef(dx, dxx);
    pdy = __fdividef(dy, dyy);
    pds = __fdividef(ds, dss);
  }
  // sift_const[] = { sigma0, oct, lvl, nLevels };
  float dval = 0.5f*(dx*pdx + dy*pdy + ds*pds);
  float refined_level = sift_const[2] + pds;
  float oct = sift_const[1] + refined_level / sift_const[3];	// refined octave
  d_Sift[i+0*maxPts] = (p%w)+0.5f + pdx;			// xpos (on subsampled image)
  d_Sift[i+1*maxPts] = (p/w)+0.5f + pdy;			// ypos (on subsampled image)
  d_Sift[i+2*maxPts] = sift_const[0] * exp2f(oct);		// scale (in original scale)
  d_Sift[i+3*maxPts] = val[0] + dval;				// value
  d_Sift[i+5*maxPts] = 0;					// orientation
  float tra = dxx + dyy;
  det = dxx*dyy - dxy*dxy;
  d_Sift[i+4*maxPts] = __fdividef(tra*tra, det);		// edge strength
  // check if the refined position is valid
  if (pdx > +1.5f || pdx < -1.5f || pdy > +1.5f || pdy < -1.5f) d_Sift[i+2*maxPts] = -99;
  // check if the refined level is valid
  if ( refined_level < sift_const[2]-0.5f || 
       refined_level > sift_const[2]+1.5f ) d_Sift[i+2*maxPts] = -99;
}

double csift_compute_positions(DevImage *diff1, DevImage *diff2, DevImage *diff3,
			       int *h_ptrs, DevImage *sift, int numPts, int maxPts,
			       float sigma0, int oct, int lvl, int nLevels)
{
  // Compute exact position of the keypoints, which are specified by 'h_ptrs'
  //   as pixel indices in the image.
  int w = diff1->w;
  int h = diff1->h;
  unsigned int hTimer;
  timer_start(&hTimer);
  float sparam[4] = { sigma0, oct, lvl, nLevels };
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(sift_const, &sparam, 4*sizeof(float)));
  // copy keypoints indices from 'h_ptrs' to 'd_ptrs'
  int *d_ptrs = 0;
  CUDA_SAFE_CALL(cudaMalloc((void **)&d_ptrs, sizeof(int)*numPts));
  CUDA_SAFE_CALL(cudaMemcpy(d_ptrs, h_ptrs, sizeof(int)*numPts, cudaMemcpyHostToDevice));
  
  // calculate sub-pixel location of the keypoints, and save them in 'sift'
  dim3 blocks(iDivUp(numPts, POSBLK_SIZE));
  dim3 threads(POSBLK_SIZE);
  th_compute_positions<<<blocks, threads>>>(diff1->data, diff2->data, diff3->data,
					    d_ptrs, sift->data, numPts, maxPts, w, h);
  
  //CUT_CHECK_ERROR2("ComputePositions() 1 failed");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
  //CUT_CHECK_ERROR2("ComputePositions() 2 failed");
  
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("ComputePositions time =       %.2f msec\n", gpuTime);
#endif
  CUDA_SAFE_CALL(cudaFree(d_ptrs));
  return gpuTime;
}

// ===================================================================
// 
// ===================================================================

double csift_remove_edge_points(DevImage *sift, int *initNumPts, int maxPts, float ethreshold, int w, int h, float subsampling)
{
  unsigned int hTimer;
  timer_start(&hTimer);
  int numPts = *initNumPts;
  int bw = sizeof(float)*numPts;
  float *h_sift = (float *)malloc(5*bw);
  CUDA_SAFE_CALL(cudaMemcpy2D(h_sift, bw, sift->data, sizeof(float)*maxPts,  
			      bw, 5, cudaMemcpyDeviceToHost));
  float edgeLimit = (ethreshold+1)*(ethreshold+1)/ethreshold; 
  int num = 0;
  for (int i=0;i<numPts;i++) {
    float xx = h_sift[0*numPts+i];                   // x on subsampled image
    float yy = h_sift[1*numPts+i];                   // y on subsampled image
    float scale = h_sift[2*numPts+i] / subsampling;  // scale on subsampled image
    float edgev = h_sift[4*numPts+i];                // edge strength
    //printf("f%d : xx=%.2f yy=%.2f scale=%.2f edgev=%.2f\n", i, xx, yy, scale, edgev);
    if (edgev < edgeLimit && scale > 0 && 
	xx > scale && yy > scale && xx < w-scale && yy < h-scale) {
      for (int j=0;j<5;j++) h_sift[j*numPts+num] = h_sift[j*numPts+i];
      num ++;
    }
  }
  CUDA_SAFE_CALL(cudaMemcpy2D(sift->data, sizeof(float)*maxPts, h_sift, bw,  
			      bw, 5, cudaMemcpyHostToDevice));
  free(h_sift);
  *initNumPts = num;
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("RemoveEdgePoints time =       %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// Compute two dominating orientations in xpos and ypos
// ===================================================================
#define OWSZ 16
__global__ void th_compute_orientations(float *g_Data, float *d_finfo, int maxPts, int w, int h)
{
  __shared__ float data[OWSZ*OWSZ];	// OWSZxOWSZ neighbor pixels around the keypoint
  __shared__ float wsum[OWSZ*32];	// 32 x OWSZ weight sum
  const int tx = threadIdx.x;		// OWSZ threads for each keypoint
  const int bx = blockIdx.x;		// bx is just the index of the keypoint
  float xx = d_finfo[0*maxPts + bx];	// refined x position (on subsampled image)
  float yy = d_finfo[1*maxPts + bx];	// refined y position (on subsampled image)
  int yp = ((int)yy) - OWSZ/2;	// Y position of the keypoint - 7
  int xp = ((int)xx) - OWSZ/2;	// X position of the keypoint - 7
  
  // setup shared data (collect pixel values in the 16x16 neighborhood of keypoint (xp+7,yp+7))
  for (int y=0;y<OWSZ;y++) {
    int memPos = OWSZ*y + tx;
    int xi = xp + tx;
    int yi = yp + y;
    if (xi<0) xi = 0;
    if (xi>=w) xi = w-1;
    if (yi<0) yi = 0;
    if (yi>=h) yi = h-1;
    data[memPos] = g_Data[yi*w+xi];
  }
//   { for (int k=0;k<OWSZ;k++) dbuf[k*OWSZ+tx] = 0; }
  __syncthreads();
  
  for (int i=0;i<32;i++) wsum[i*OWSZ+tx] = 0.0f;
  __syncthreads();
  
  {	// calculate the weighted sum of orientations along Y axis
    float sigma = d_finfo[ 2 * maxPts + bx ] / sift_const[0];  // scale (on subsampled image)
    float sigdenom = ( 2 * 1.5f * 1.5f * sigma * sigma );
    //if (bx==0) { dbuf[0] = xp;  dbuf[1] = yp;  dbuf[3] = xx;  dbuf[4] =yy; }
    for (int y=1;y<OWSZ-1;y++) {
      int memPos = y*OWSZ + tx;
      float yoff  = yy - (yp+y+0.5f);  yoff = yoff * yoff;
      if (tx>=1 && tx<OWSZ-1) {
	float dy = data[memPos+OWSZ] - data[memPos-OWSZ];
	float dx = data[memPos+1]  - data[memPos-1];
	float aa = atan2f(dy, dx);   if (aa<0) aa += 2*Pi;
	int bin = (int)(aa*32/(2*Pi)) % (32-1);	// 32 orientations
	float grad = sqrtf(dx*dx + dy*dy);	// gradient magnitude * 2
	// calculate the Grassian falloff
	float xoff = xx - (xp+tx+0.5f);
	float ddist = yoff + xoff * xoff;
	float weight = 0;
	if (ddist <= (OWSZ/2)*(OWSZ/2)) { 
	  weight = exp( -ddist / sigdenom );
	  wsum[bin*OWSZ + tx] += grad * weight;
	}
	//if (bx==0) dbuf[y*OWSZ+tx] = bin;
      }
    }
    //if (bx==0) for (int y=0; y<32; y++) dbuf[y*OWSZ+tx] = wsum[y*OWSZ+tx];
  }
  __syncthreads();
  __shared__ float hist[32*2];
  if (tx < 32/2) {
    // calculate the weighted sum of orientations along X axis
    int k, pos = tx * OWSZ; //__mul24(tx,16);
// #pragma unroll OWSZ-2
    for (k=1; k<=OWSZ-2; k++) wsum[pos] += wsum[ pos + k ];
    hist[tx] = wsum[pos];		// first  half of 32 bins of orientation
    pos = (tx+16) * OWSZ; //__mul24(tx+16,16);
// #pragma unroll OWSZ-2
    for (k=1; k<=OWSZ-2; k++) wsum[pos] += wsum[ pos + k ];
    hist[tx+16] = wsum[pos];	// second half of 32 bins of orientation
    __syncthreads();
    //if (bx==0) { dbuf[tx+0] = hist[tx]; dbuf[tx+16] = hist[tx+16]; }
    {	// smooth the histogram in hist[32 + x]
      float pprv, prev, next, nnxt;
      pprv = hist[ (tx - 2 + 32) & 31 ];	// first half of hist[]
      prev = hist[ (tx - 1 + 32) & 31 ];
      next = hist[ (tx + 1) & 31 ];
      nnxt = hist[ (tx + 2) & 31 ];
      hist[tx + 32] = 6 * hist[tx] + 4 * (prev + next) + (pprv + nnxt);
      pprv = hist[ tx+16 - 2 ];			// second half of hist[]
      prev = hist[ tx+16 - 1 ];
      next = hist[ (tx+16 + 1) & 31 ];
      nnxt = hist[ (tx+16 + 2) & 31 ];
      hist[tx+16 + 32] = 6 * hist[tx+16] + 4 * (prev + next) + (pprv + nnxt);
      __syncthreads();
    }
    {	// find the local maximum in hist[x]
      float v = hist[32+tx];	// first  half of hist[]
      hist[tx] = (v>hist[32+(tx+1)] && v>=hist[32+((tx+31)&31)] ? v : 0.0f);
      v = hist[32+tx+16];	// second half of hist[]
      hist[tx+16] = (v>hist[32+((tx+16+1)&31)] && v>=hist[32+(tx+16-1)] ? v : 0.0f);
    }
  }
  //if (bx==0) { dbuf[tx+0] = hist[tx]; dbuf[tx+16] = hist[tx+16]; }
  __syncthreads();
  // 
  if (tx==0) {
    float *d_Orient = d_finfo + 5*maxPts;  // orientation of the keypoint (degree; 0~360)
    float maxval1 = 0.0;
    float maxval2 = 0.0;
    int i1 = -1;
    int i2 = -1;
    for (int i=0;i<32;i++) {
      float v = hist[i];
      if (v>maxval1) {
	maxval2 = maxval1;
	maxval1 = v;
	i2 = i1;
	i1 = i;
      } else if (v>maxval2) {
	maxval2 = v;
	i2 = i;
      }
    }
    float val1 = hist[32+((i1+1)&31)];
    float val2 = hist[32+((i1+31)&31)];
    float peak = i1 + 0.5f + 0.5f*(val1-val2) / (2.0f*maxval1-val1-val2);
    peak = peak * 0.19634954f;
    if (peak <     0) peak += 2*Pi;
    if (peak >= 2*Pi) peak -= 2*Pi;
    d_Orient[bx] = peak;  // [ 0 ~ 2PI ]
    if (maxval2<0.8f*maxval1)  i2 = -1;
//     if (bx==1) {dbuf[0] = i1;  dbuf[1] = hist[i1];  dbuf[2] = val1;  dbuf[3] = val2;  dbuf[4] = peak; }
    if (i2>=0) {
      float val1 = hist[32+((i2+1)&31)];
      float val2 = hist[32+((i2+31)&31)];
      float peak = i2 + 0.5f + 0.5f*(val1-val2) / (2.0f*maxval2-val1-val2);
      peak = peak * 0.1963495f;
      if (peak <     0) peak += 2*Pi;
      if (peak >= 2*Pi) peak -= 2*Pi;
      d_Orient[bx+maxPts] = peak; // [ 0 ~ 2PI]
//       if (bx==1) {dbuf[8] = i2;  dbuf[9] = hist[i2];  dbuf[10] = val1;  dbuf[11] = val2;  dbuf[12] = peak; }
    } else 
      d_Orient[bx+maxPts] = i2;
  }
} 

double csift_compute_orientations(DevImage *img, DevImage *sift, int numPts, int maxPts, float subsampling)
{
  int w = img->w;
  int h = img->h;
  unsigned int hTimer;
  timer_start(&hTimer);
  float *d_finfo = sift->data;
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(sift_const, &subsampling, 1*sizeof(float)));
#define OWIDTH  19
#define OHEIGHT 32
//   float h_buf[OWIDTH*OHEIGHT], *dbuf;
//   CUDA_SAFE_CALL(cudaMalloc((void **)&dbuf, sizeof(float)*OWIDTH*OHEIGHT));

  dim3 blocks(numPts);
  dim3 threads(OWIDTH);
  th_compute_orientations<<<blocks, threads>>>(img->data, d_finfo, maxPts, w, h);
  CUT_CHECK_ERROR("ComputeOrientations() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
  
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("ComputeOrientations time =    %.2f msec\n", gpuTime);
#endif
//   CUDA_SAFE_CALL(cudaMemcpy(h_buf, dbuf, sizeof(float)*OWIDTH*OHEIGHT, cudaMemcpyDeviceToHost));
//   cudaFree(dbuf);
//   PRINT_ARRAY2(h_buf, OWIDTH, OHEIGHT, "hist", "%5.2f ");
//   PRINT_ARRAY2(h_buf, 8, 4, "hist", "%5.2f ");
  
  return gpuTime;
}

double csift_second_orientations(DevImage *sift, int *initNumPts, int maxPts) 
{
  unsigned int hTimer;
  timer_start(&hTimer);
  int numPts = *initNumPts;
  int numPts2 = 2*numPts;
  float *d_sift = sift->data;
  int bw = sizeof(float)*numPts2;
  float *h_sift = (float *)malloc(7*bw);
  CUDA_SAFE_CALL(cudaMemcpy2D(h_sift, bw, d_sift, sizeof(float)*maxPts,  
			      sizeof(float)*numPts, 7, cudaMemcpyDeviceToHost));
  int num = numPts;
  for (int i=0;i<numPts;i++) {
    int o2pos = 6 * numPts2 + i;
    if (h_sift[o2pos]>=0.0f && num<maxPts) {
      float *pnew=h_sift+num, *pold=h_sift+i;
      for (int j=0;j<5;j++,pnew+=numPts2,pold+=numPts2) *pnew = *pold;
      h_sift[5*numPts2+num] = h_sift[o2pos];
      h_sift[6*numPts2+num] = -1.0f;
      num ++;
    }
  }
  CUDA_SAFE_CALL(cudaMemcpy2D(&d_sift[numPts], sizeof(float)*maxPts, 
			      &h_sift[numPts], bw, sizeof(float)*(num-numPts), 7, cudaMemcpyHostToDevice));
  free(h_sift);
  *initNumPts = num;
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("SecondOrientations time =     %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// Extract Sift descriptors
// ===================================================================

texture<float, 2, cudaReadModeElementType> tex;

__global__ void th_extract_descriptors(float *d_finfo, float *d_desc, int maxPts)
{
  const int bx = blockIdx.x;		// bx is just the index of the keypoint
  const int tx = threadIdx.x;		// 16x16 threads for each keypoint
  const int ty = threadIdx.y;		// 16x16 threads for each keypoint
  const int tidx = ty * 16 + tx;
  // get the weighted gradient and orientation for each sample (16x16)
  __shared__ float cont[16*16][8];
//  memset( &cont[tidx][0], 0, 8 * sizeof(float) );
  memset( &cont[tidx][0], 0, 16*16*8 * sizeof(float) );
  {
    float kx = d_finfo[0*maxPts + bx];	// refined x position (on subsampled image)
    float ky = d_finfo[1*maxPts + bx];	// refined y position (on subsampled image)
    float ks = d_finfo[2*maxPts + bx]/sift_const[0];	// refined scale (on subsampled image)
    float aa = d_finfo[5*maxPts + bx];  // orientation in radian
    float radius = ks * 6;		// radius of the coverage of 16 x 16 samples
    float xu[2]={radius/16,0}, yu[2]={0,radius/16};
    float R[4], rot=-Pi/2-aa, rxu[2], ryu[2];
    G2M_SET( R, +cosf(rot), -sinf(rot), +sinf(rot), +cosf(rot) );  // CCW
    G2M_MUL_MV( rxu, R, xu );		// unit vector in X for each sample
    G2M_MUL_MV( ryu, R, yu );		// unit vector in Y for each sample
    float xo = (+tx+ 0.5f - 8);		// [-7.5 +7.5]
    float yo = (-ty+15.5f - 8);		// [-7.5 +7.5]
    float sxy[2];			// position of the sample
    rxu[1] *= -1;  ryu[1] *= -1;
    G2V_SET( sxy, kx, ky );
    G2V_SCALED_ADD( sxy, sxy, 2*xo, rxu );
    G2V_SCALED_ADD( sxy, sxy, 2*yo, ryu );
    float Gx = tex2D(tex, sxy[0]+rxu[0], sxy[1]+rxu[1]) - tex2D(tex, sxy[0]-rxu[0], sxy[1]-rxu[1]);
    float Gy = tex2D(tex, sxy[0]+ryu[0], sxy[1]+ryu[1]) - tex2D(tex, sxy[0]-ryu[0], sxy[1]-ryu[1]);
    float mag = sqrtf(Gx*Gx + Gy*Gy);			// gradient magnitude
    float wgt = exp( - (xo*xo+yo*yo) / (2*8*8) );	// gaussian weight
    float ang = atan2f(Gy,Gx);  if (ang<0) ang += 2*3.1416f;  // gradient orientation [ 0 ~ PI ]
    float anf  = ang * 4.0f / 3.1415926536f;         // orientation bin index [ 0 ~ 8 ]
    int   ani = (int)floor( anf );  if (ani>7) ani -= 8;
    int   an[2];  float anw;
    if (ani+0.5f < anf) { an[0]=ani;   an[1]=ani+1; if (an[1]>7) an[1]=0;  anw=1-fabs(ani+0.5f-anf); }
    else                { an[0]=ani-1; an[1]=ani;   if (an[0]<0) an[0]=7;  anw=  fabs(ani+0.5f-anf); }
    cont[tidx][an[0]] = mag * wgt * anw;
    cont[tidx][an[1]] = mag * wgt * (1-anw);
//     if (bx==0) dbuf[tidx] = Gy;
//     if (bx==0 && tx==0 && ty==0) { 
//       dbuf[0] = kx,     dbuf[1] = ky,     dbuf[2] = ks,     dbuf[3] = aa*180/3.141592f;
//       dbuf[4] = rxu[0]; dbuf[5] = rxu[1]; dbuf[6] = ryu[0]; dbuf[7] = ryu[1];
//       dbuf[8] = sxy[0]; dbuf[9] = sxy[1]; dbuf[16] = xo; dbuf[17] = yo;
//     }
//     if (bx==0 && tx==15 && ty==0)  { dbuf[10] = sxy[0]; dbuf[11] = sxy[1]; dbuf[18] = xo; dbuf[19] = yo; }
//     if (bx==0 && tx== 0 && ty==15) { dbuf[12] = sxy[0]; dbuf[13] = sxy[1]; dbuf[20] = xo; dbuf[21] = yo; }
//     if (bx==0 && tx==15 && ty==15) { dbuf[14] = sxy[0]; dbuf[15] = sxy[1]; dbuf[22] = xo; dbuf[23] = yo; }
//     for (int k=0; k<8; k++) dbuf[tidx] = mag;  ////
//     for (int k=0; k<8; k++) dbuf[tidx*8+k] = (k== an[0] || k==an[1] ? 1 : 0);  // ori
  }
  // initialize the descriptor buffer [ 4 x 4 x 8 ]
  __shared__ float desc[4*4*8];
  if (ty<8) desc[tidx] = 0.0f;
  __syncthreads();
  // add up all contributions from all the samples, for each bin
  if (ty < 8) {
    int bx = tx & 3;	// bx = tx % 4
    int by = tx >> 2;	// by = tx / 4
    int bo = ty;	// bo = ty;
    int bx4=bx*4, by4=by*4, bidx = tx*8+bo, zx, zy, zidx;
//#define ADDS00(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx;                     desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS10(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx>=0)          desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS01(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zy>=0)          desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS20(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx<16)          desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS02(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zy<16)          desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS11(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx>=0 && zy>=0) desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS12(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx>=0 && zy<16) desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS21(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx<16 && zy>=0) desc[bidx] += cont[zidx][bo]*ww; } while(0)
//#define ADDS22(xx,yy,ww) do { zx=bx4+xx; zy=by4+yy; zidx=zy*16+zx; if (zx<16 && zy<16) desc[bidx] += cont[zidx][bo]*ww; } while(0)
//    ADDS11(-1,-1, 0.0625f);  ADDS10(-1, 0, 0.1875f);  ADDS10(-1, 1, 0.2500f);
//    ADDS10(-1, 2, 0.2500f);  ADDS10(-1, 3, 0.1875f);  ADDS12(-1, 4, 0.0625f);  // col-1
//    ADDS01( 0,-1, 0.1875f);  ADDS00( 0, 0, 0.5625f);  ADDS00( 0, 1, 0.7500f);
//    ADDS00( 0, 2, 0.7500f);  ADDS00( 0, 3, 0.5625f);  ADDS02( 0, 4, 0.1875f);  // col 0
//    ADDS01( 1,-1, 0.7500f);  ADDS00( 1, 0, 1.0000f);  ADDS00( 1, 1, 1.0000f);
//    ADDS00( 1, 2, 1.0000f);  ADDS00( 1, 3, 1.0000f);  ADDS02( 1, 4, 0.7500f);  // col 1
//    ADDS01( 2,-1, 0.7500f);  ADDS00( 2, 0, 1.0000f);  ADDS00( 2, 1, 1.0000f);
//    ADDS00( 2, 2, 1.0000f);  ADDS00( 2, 3, 1.0000f);  ADDS02( 2, 4, 0.7500f);  // col 2
//    ADDS01( 3,-1, 0.1875f);  ADDS00( 3, 0, 0.5625f);  ADDS00( 3, 1, 0.7500f);
//    ADDS00( 3, 2, 0.7500f);  ADDS00( 3, 3, 0.5625f);  ADDS02( 3, 4, 0.1875f);  // col 3
//    ADDS21( 4,-1, 0.0625f);  ADDS20( 4, 0, 0.1875f);  ADDS20( 4, 1, 0.2500f);
//    ADDS20( 4, 2, 0.2500f);  ADDS20( 4, 3, 0.1875f);  ADDS22( 4, 4, 0.0625f);  // col 4
	///// 
//	zx=bx4-1; zy=by4-1; if (zx>=0 && zy>=0) { zidx=zy*16+zx; desc[bidx] += cont[zidx][bo]*0.0625f; } 			//ADDS11(-1,-1, 0.0625f);
//	zx=bx4-1; if (zx>=0) { zy=by4;   zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS10(-1, 0, 0.1875f);
//	zx=bx4-1; if (zx>=0) { zy=by4+1; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS10(-1, 1, 0.2500f);
//	//
//	zx=bx4-1; if (zx>=0) { zy=by4+2; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS10(-1, 2, 0.2500f);
//	zx=bx4-1; if (zx>=0) { zy=by4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS10(-1, 3, 0.1875f);
//	zx=bx4-1; zy=by4+4; if (zx>=0 && zy<16) {zidx=zy*16+zx;  desc[bidx] += cont[zidx][bo]*0.0625f; } // col-1	//ADDS12(-1, 4, 0.0625f);
//	////
//	zy=by4-1; if (zy>=0) { zx=bx4;  zidx=zy*16+zx;           desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS01( 0,-1, 0.1875f);
//	zx=bx4; zy=by4; zidx=zy*16+zx;                           desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 0, 0, 0.5625f);
//	zx=bx4; zy=by4+1; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 0, 1, 0.7500f);
//	//
//	zx=bx4; zy=by4+2; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 0, 2, 0.7500f);
//	zx=bx4; zy=by4+3; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 0, 3, 0.5625f);
//	zy=by4+4; if (zy<16) { zx=bx4; zidx=zy*16+zx;            desc[bidx] += cont[zidx][bo]*0.1875f; } // col 0	//ADDS02( 0, 4, 0.1875f);
//	////
//	zy=by4-1; if (zy>=0) { zx=bx4+1; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.7500f; }			//ADDS01( 1,-1, 0.7500f);
//	zx=bx4+1; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 0, 1.0000f);
//	zx=bx4+1; zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 1, 1.0000f);
//	//
//	zx=bx4+1; zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 2, 1.0000f);
//	zx=bx4+1; zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 3, 1.0000f);
//	zy=by4+4; if (zy<16) {zx=bx4+1; zidx=zy*16+zx;           desc[bidx] += cont[zidx][bo]*0.7500f; } // col 1	//ADDS02( 1, 4, 0.7500f);
//	////
//	zy=by4-1; if (zy>=0) {zx=bx4+2; zidx=zy*16+zx;           desc[bidx] += cont[zidx][bo]*0.7500f; }			//ADDS01( 2,-1, 0.7500f);
//	zx=bx4+2; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 0, 1.0000f);
//	zx=bx4+2; zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 1, 1.0000f);
//	//
//	zx=bx4+2; zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 2, 1.0000f);
//	zx=bx4+2; zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 3, 1.0000f);
//	zy=by4+4; if (zy<16) { zx=bx4+2; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.7500f; } // col 2	//ADDS02( 2, 4, 0.7500f);
//	////
//	zy=by4-1; if (zy>=0) { zx=bx4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS01( 3,-1, 0.1875f);
//	zx=bx4+3; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 3, 0, 0.5625f);
//	zx=bx4+3; zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 3, 1, 0.7500f);
//	//
//	zx=bx4+3; zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 3, 2, 0.7500f);
//	zx=bx4+3; zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 3, 3, 0.5625f);
//	zy=by4+4; if (zy<16) { zx=bx4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; } // col 3	//ADDS02( 3, 4, 0.1875f);  
//	//
//	zx=bx4+4; zy=by4-1; if (zx<16 && zy>=0) {zidx=zy*16+zx;  desc[bidx] += cont[zidx][bo]*0.0625f; }			//ADDS21( 4,-1, 0.0625f);
//	zx=bx4+4; if (zx<16) { zy=by4; zidx=zy*16+zx;            desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS20( 4, 0, 0.1875f);
//	zx=bx4+4; if (zx<16) { zy=by4+1; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS20( 4, 1, 0.2500f);
//	//
//	zx=bx4+4; if (zx<16) { zy=by4+2; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS20( 4, 2, 0.2500f);	
//	zx=bx4+4; if (zx<16) { zy=by4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS20( 4, 3, 0.1875f);	
//	zx=bx4+4; zy=by4+4; if (zx<16 && zy<16) { zidx=zy*16+zx; desc[bidx] += cont[zidx][bo]*0.0625f; } //col 4    //ADDS22( 4, 4, 0.0625f);
	/////////
	zx=bx4-1; zy=by4-1; if (zx>=0 && zy>=0) { zidx=zy*16+zx; desc[bidx] += cont[zidx][bo]*0.0625f; } 			//ADDS11(-1,-1, 0.0625f);
	          if (zx>=0) { zy=by4;   zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f;				//ADDS10(-1, 0, 0.1875f);
	                       zy=by4+2; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f;				//ADDS10(-1, 2, 0.2500f);
	                       zy=by4+1; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f;				//ADDS10(-1, 1, 0.2500f);
	                       zy=by4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS10(-1, 3, 0.1875f);
	          zy=by4+4; if (zx>=0 && zy<16) {zidx=zy*16+zx;  desc[bidx] += cont[zidx][bo]*0.0625f; } // col-1	//ADDS12(-1, 4, 0.0625f);
	////
	zx=bx4; zy=by4; zidx=zy*16+zx;                           desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 0, 0, 0.5625f);
	zy=by4-1; if (zy>=0) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS01( 0,-1, 0.1875f);
	        zy=by4+1; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 0, 1, 0.7500f);
	//
	        zy=by4+2; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 0, 2, 0.7500f);
	        zy=by4+3; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 0, 3, 0.5625f);
	zy=by4+4; if (zy<16) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.1875f; } // col 0	//ADDS02( 0, 4, 0.1875f);
	////
	zx=bx4+1; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 0, 1.0000f);
	zy=by4-1; if (zy>=0) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.7500f; }			//ADDS01( 1,-1, 0.7500f);
	          zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 1, 1.0000f);
	//
	          zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 2, 1.0000f);
	          zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 1, 3, 1.0000f);
	zy=by4+4; if (zy<16) {zidx=zy*16+zx;                     desc[bidx] += cont[zidx][bo]*0.7500f; } // col 1	//ADDS02( 1, 4, 0.7500f);
	////
	zx=bx4+2; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 0, 1.0000f);
	zy=by4-1; if (zy>=0) {zidx=zy*16+zx;                     desc[bidx] += cont[zidx][bo]*0.7500f; }			//ADDS01( 2,-1, 0.7500f);
	          zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 1, 1.0000f);
	//
	          zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 2, 1.0000f);
	          zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo];						//ADDS00( 2, 3, 1.0000f);
	zy=by4+4; if (zy<16) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.7500f; } // col 2	//ADDS02( 2, 4, 0.7500f);
	////
	zx=bx4+3; zy=by4; zidx=zy*16+zx;                         desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 3, 0, 0.5625f);
	zy=by4-1; if (zy>=0) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS01( 3,-1, 0.1875f);
	          zy=by4+1; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 3, 1, 0.7500f);
	//
	          zy=by4+2; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.7500f;				//ADDS00( 3, 2, 0.7500f);
	          zy=by4+3; zidx=zy*16+zx;                       desc[bidx] += cont[zidx][bo]*0.5625f;				//ADDS00( 3, 3, 0.5625f);
	zy=by4+4; if (zy<16) { zidx=zy*16+zx;                    desc[bidx] += cont[zidx][bo]*0.1875f; } // col 3	//ADDS02( 3, 4, 0.1875f);  
	//
	zx=bx4+4; zy=by4-1; if (zx<16 && zy>=0) {zidx=zy*16+zx;  desc[bidx] += cont[zidx][bo]*0.0625f; }			//ADDS21( 4,-1, 0.0625f);
	          if (zx<16) { zy=by4; zidx=zy*16+zx;            desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS20( 4, 0, 0.1875f);
	          if (zx<16) { zy=by4+1; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS20( 4, 1, 0.2500f);
	//
	          if (zx<16) { zy=by4+2; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.2500f; }			//ADDS20( 4, 2, 0.2500f);	
	          if (zx<16) { zy=by4+3; zidx=zy*16+zx;          desc[bidx] += cont[zidx][bo]*0.1875f; }			//ADDS20( 4, 3, 0.1875f);	
	          zy=by4+4; if (zx<16 && zy<16) { zidx=zy*16+zx; desc[bidx] += cont[zidx][bo]*0.0625f; } //col 4    //ADDS22( 4, 4, 0.0625f);
  }
  __syncthreads();
  
  if (true) {	// normalize the description vector  ////
    __shared__ float sum[4*16];  // 4*16 == 4*4*8/2
    // calculate the sum for normalization
    if (ty < 4) {
    	sum[tidx] = desc[tidx] + desc[tidx + 4*16];
	}
	__syncthreads();
    if (ty < 2) {
    	sum[tidx] += sum[tidx + 2*16];
	}
	__syncthreads();
    if (ty < 1) sum[0] = ( sum[0] + sum[1] + sum[2] + sum[3] + sum[4] + sum[5] + 
			   sum[6] + sum[7] + sum[8] + sum[9] + sum[10] + sum[11] + 
			   sum[12] + sum[13] + sum[14] + sum[15] );
    __syncthreads();
    // normalize the descriptor
    if (ty < 8) { 
      float val = desc[tidx] / sum[0]; 		// normalize the descriptor
      desc[tidx] = (val > 0.2f ? 0.2f : val);	// cut off the values bigger than 0.2;
    }
    // calculate the sum for normalization
    if (ty < 4) sum[tidx] = desc[tidx] + desc[tidx + 4*16];
    __syncthreads();
    if (ty < 2) sum[tidx] += sum[tidx + 2*16];
    __syncthreads();
    if (ty==0 && tx==0) sum[0] = ( sum[0] + sum[1] + sum[2] + sum[3] + sum[4] + sum[5] + 
				   sum[6] + sum[7] + sum[8] + sum[9] + sum[10] + sum[11] + 
				   sum[12] + sum[13] + sum[14] + sum[15] );
    __syncthreads();
    // normalize the descriptor
    if (ty < 8) {
    	desc[tidx] /= sum[0]; 		// normalize the descriptor
   	}
  }
  
  // copy the descriptor
  if (ty < 8) {
    float *g_desc = d_desc + 128 * bx;
    g_desc[tidx] = desc[tidx];
  }
}

void save_fbuf(float *data, int w, int h, char *fname);

double csift_extract_descriptors(DevTexture *timg, DevImage *sift,
				 DevImage *desc, int numPts, int maxPts, float subsampling)
{
  unsigned int hTimer;
  timer_start(&hTimer);
  float *d_sift = sift->data, *d_desc = desc->data;
  
  tex.addressMode[0] = cudaAddressModeClamp;
  tex.addressMode[1] = cudaAddressModeClamp;
  tex.filterMode = cudaFilterModeLinear; 
  tex.normalized = false;
  float values[]={ subsampling, timg->w, timg->h };
  CUDA_SAFE_CALL(cudaBindTextureToArray(tex, (cudaArray*)timg->array));
  CUDA_SAFE_CALL(cudaMemcpyToSymbol(sift_const, values, 3*sizeof(float)));
  
  dim3 blocks(numPts); 
  dim3 threads(16, 16);
//   float *dbuf=NULL;							// for debugging
//   CUDA_SAFE_CALL(cudaMalloc((void**)&dbuf, 16*16*8*sizeof(float)));	// for debugging

//printf("maxPts = %d\n", maxPts);
  th_extract_descriptors<<<blocks, threads>>>(d_sift, d_desc, maxPts);
  
//   // test texture
//   int w = timg->w, h = timg->h;
//   float *d_img=NULL;  cudaMalloc((void**)&d_img, w*h*sizeof(float));
//   dim3 blk(iDivUp(w, 16), iDivUp(h,16)), thr(16,16);
//   TestTexture <<< blk, thr >>> (d_img, w, h, 3.14f/4);
//   float *h_img = (float*)malloc(w*h*sizeof(float));
//   cudaMemcpy( h_img, d_img, w*h*sizeof(float), cudaMemcpyDeviceToHost);
//   save_fbuf( h_img, w, h, "output.png" );
 
  CUT_CHECK_ERROR("ExtractSiftDescriptors() execution failed\n");
  CUDA_SAFE_CALL(cudaThreadSynchronize());
  CUDA_SAFE_CALL(cudaUnbindTexture(tex));

  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("ExtractSiftDescriptors time = %.2f msec\n", gpuTime);
#endif
//   float hbuf[16*16*8], hdesc[128];
//   CUDA_SAFE_CALL(cudaMemcpy(hbuf,  dbuf, 16*16*8*sizeof(float), cudaMemcpyDeviceToHost));
//   CUDA_SAFE_CALL(cudaMemcpy(hdesc, d_desc, 128*sizeof(float), cudaMemcpyDeviceToHost));
//   CUDA_SAFE_CALL(cudaFree(dbuf));
//   PRINT_ARRAY2(hbuf, 8, 3, "dbuf", "%7.4f ");
//   PRINT_ARRAY2(hbuf, 16, 16, "dbuf", "%7.4f ");
//   PRINT_ARRAY3(hbuf, 16, 16, 8, "dbuf", "%.0f");
//   PRINT_ARRAY2(hdesc, 8, 16, "desc", "%7.4f ");
  
  return gpuTime; 
}

// ===================================================================
// ===================================================================

double csift_add_sift_data(HstImage *flist, float *d_sift, float *d_desc,
			   int numPts, int maxPts, float subsampling)
{
  unsigned int hTimer;
  timer_start(&hTimer);
  // scale back XY position of the features (Not the scale)
  float *buffer = (float *)malloc(sizeof(float)*numPts*6);
  int bwidth = sizeof(float)*numPts;
  CUDA_SAFE_CALL(cudaMemcpy2D(buffer, bwidth, d_sift, sizeof(float)*maxPts, bwidth, 6, cudaMemcpyDeviceToHost));
  for (int i=0;i<2*numPts;i++) buffer[i] *= subsampling;
  CUDA_SAFE_CALL(cudaThreadSynchronize());
  // increase the size of the host/device memories, if necessary
  int newNum = flist->h + numPts;
  if (newNum > flist->hmax) flist->reallocImage( 140, newNum*2 );
  // copy all the features
  //   HstImage 'flist' is 140 x N float image, with a SIFT feature in each row.
  //   Each SIFT feature consists of 140 float values, including 128 descriptor.
  //   ( x,y,s,sh,  edge,ori,sco,amb,  desc[128], dummy[4] )
  float *ptr   = flist->data + flist->h * 140;
  // copy each row of 'd_sift' to a column of 'flist'
  for (int j=0; j<numPts; j++) {	// each keypoint
    float *fp = ptr + j * 140;
    for (int i=0; i<6; i++) fp[i] = buffer[ i * numPts + j ];
    fp[6] = fp[7] = 0.0;
    // note that orientation is in radian !
  }
  // copy each row of 'd_desc' to row of 'flist'
  CUDA_SAFE_CALL(cudaMemcpy2D(&ptr[8], 140*sizeof(float),
			      d_desc, 128*sizeof(float), 128*sizeof(float), numPts, cudaMemcpyDeviceToHost));
  flist->setImage( 140, newNum );
  free(buffer);
  double gpuTime = timer_stop(hTimer);
#ifdef VERBOSE
  printf("AddSiftData time =            %.2f msec\n", gpuTime);
#endif
  return gpuTime;
}

// ===================================================================
// ===================================================================

void save_fbuf(float *data, int w, int h, char *fname);

void check_img(DevImage *cimg, char *cmmt, int oct, int lvl, bool print_on, bool save_on)
{
  int   i, w=cimg->w, h=cimg->h;
  int   total = w*h, cnt_zero=0, cnt_nega=0, cnt_posi=0;
  HstImage himg;
  himg.setImage( cimg->w, cimg->h );
  himg.uploadFrom( cimg );
  float *imd = himg.data;
  float maxv=-1e6, minv=+1e6;
  for (i = 0; i < total; i++) {
    if (imd[i] < minv) minv = imd[i];
    if (imd[i] > maxv) maxv = imd[i];
    if      (imd[i] == 0) cnt_zero++;
    else if (imd[i] > +0.0001) cnt_posi++;
    else if (imd[i] < -0.0001) cnt_nega++;
  }
  if (print_on) printf("%s (%3d x %3d)  minv=%7.4f, maxv=%7.4f  (zero:%4.1f%%  pos:%4.1f%%  neg:%4.1f%%)\n",
		       (cmmt ? cmmt:"BUF"), w, h, minv, maxv, cnt_zero*100.0f/total, cnt_posi*100.0f/total, cnt_nega*100.0f/total);
  if (save_on) {
    char fname[80];  sprintf( fname, "%s_o%d_l%d.png", cmmt, oct, lvl );
    save_fbuf( imd, w, h, fname );
  }
}

void check_sift(DevImage *sift, int cnt, int *ptrs, int iw, int ih)
{
  int   i, w=sift->w;
  HstImage himg;
  himg.setImage( sift->w, sift->h );
  himg.uploadFrom( sift );
  float *imd = himg.data;
  printf("  keys = %d   width=%d   image=(%dx%d)\n", cnt, w, iw, ih);
  for (i = 0; i < cnt; i++) {
    if (!ptrs) {
      printf("  key xy=(%.2f %.2f) s=%.2f val=%.4f score=%.4f o=%.0f\n", imd[i+0*w], imd[i+1*w], imd[i+2*w], imd[i+3*w], imd[i+4*w], imd[i+5*w]*180/3.141592f);
    } else {
      printf("  key xy=(%.2f %.2f) s=%.2f val=%.4f score=%.4f o=%.0f  ptrs[i]=%d(%d,%d)\n", imd[i+0*w], imd[i+1*w], imd[i+2*w], imd[i+3*w], imd[i+4*w], imd[i+5*w]*180/3.141592f, ptrs[i], ptrs[i]%iw, ptrs[i]/iw);
    }
  }
}


