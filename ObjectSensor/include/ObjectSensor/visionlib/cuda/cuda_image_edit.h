
#include "cuda_image.h"

double cimg_scale_up(DevImage *res, DevImage *data);
double cimg_scale_down(DevImage *res, DevImage *data, float variance);
double cimg_copy(DevImage *dst, DevImage *src);
double cimg_subtract(DevImage *res, DevImage *dataA, DevImage *dataB);
double cimg_multiply_add(DevImage *res, DevImage *data, float constA, float constB);
