
#ifndef CUDA_SIFT_H
#define CUDA_SIFT_H

#include <iostream>
#include "cuda_image.h"

// ===================================================================
// 
// ===================================================================

bool cuda_sift_extract(DevImage *img, HstImage *feat, 
		       int oct, int lvl, float sigma, float edge, 
		       int xywh[4]=NULL, bool verbose=false);

#endif // CUDA_SIFT_H
