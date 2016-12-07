
#ifndef SIFTPP_DEFS_H
#define SIFTPP_DEFS_H


typedef struct { int x, y, width, height; }	sRect;
typedef struct { double x; double y; }		sPoint2D64f;
#ifndef ABS
#define ABS(x) ( ( x < 0 )? -x : x )
#endif

// ===================================================================
// The feature
// ===================================================================

#define SIFT_SIZE 128
typedef struct sift_feat {
  float  xy[2];                 // position on the image
  float  scl;                   // scale of a feature
  float  ori;                   // orientation of a feature [ 0, 2*PI ]
  float  descr[SIFT_SIZE];      // descriptor
  struct sift_feat* match;      // matching feature
  struct sift_feat* next;
} sift_feature;


// ===================================================================
// funtions from util.c
// ===================================================================

extern double descr_dist_sq( sift_feature* f1, sift_feature* f2 );
extern int  array_double( void** array, int n, int size );
extern void fatal_error(char* format, ...);
extern double dist_sq_2D( sPoint2D64f p1, sPoint2D64f p2 );

#endif //SIFTPP_DEFS_H
