/*
  This file contains definitions for functions to compute transforms from
  image feature correspondences

  Copyright (C) 2006  Rob Hess <hess@eecs.oregonstate.edu>

  @version 1.1.1-20070119
*/

#include <time.h>
#include <math.h>
#include <float.h>
#include "vm_macros.h"
#include "mtx_matrix.hpp"
#include "mtx_matrix_solver.hpp"

#include "siftpp_defs.h"
#include "siftpp_xform.h"

#include <gsl/gsl_sf.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

/************************* Local Function Prototypes *************************/

int get_matched_features( sift_feature*, int, int, sift_feature*** );
int calc_min_inliers( int, int, double, double );
sift_feature** draw_ransac_sample( sift_feature**, int, int, gsl_rng* );
void extract_corresp_pts( sift_feature**, int, int, sPoint2D64f**,
			  sPoint2D64f** );
int find_consensus( sift_feature**, int, int, double homography[9], ransac_err_fn,
		    double, sift_feature*** );
static inline void release_mem( sPoint2D64f*, sPoint2D64f*,
				sift_feature** );

/********************** Functions prototyped in model.h **********************/


/*
  Calculates a best-fit image transform from image feature correspondences
  using RANSAC.

  For more information refer to:

  Fischler, M. A. and Bolles, R. C.  Random sample consensus: a paradigm for
  model fitting with applications to image analysis and automated cartography.
  <EM>Communications of the ACM, 24</EM>, 6 (1981), pp. 381--395.

  @param features an array of features; only features with a non-NULL match
    of type mtype are used in homography computation
  @param n number of features in feat
  @param xform_fn pointer to the function used to compute the desired
    transformation from feature correspondences
  @param m minimum number of correspondences necessary to instantiate the
    model computed by xform_fn
  @param p_badxform desired probability that the final transformation
    returned by RANSAC is corrupted by outliers (i.e. the probability that
    no samples of all inliers were drawn)
  @param err_fn pointer to the function used to compute a measure of error
    between putative correspondences and a computed model
  @param err_tol correspondences within this distance of a computed model are
    considered as inliers
  @param inliers if not NULL, output as an array of pointers to the final
    set of inliers
  @param n_in if not NULL and \a inliers is not NULL, output as the final
    number of inliers

  @return Returns a transformation matrix computed using RANSAC or NULL
    on error or if an acceptable transform could not be computed.
*/
bool ransac_xform( sift_feature* features, int n, int mtype,
		   ransac_xform_fn xform_fn, int m, double p_badxform,
		   ransac_err_fn err_fn, double err_tol,
		   sift_feature*** inliers, int* n_in, double homography[9])
{
  sift_feature** matched, ** sample, ** consensus, ** consensus_max = NULL;
  struct ransac_data* rdata;
  sPoint2D64f* pts, * mpts;
  gsl_rng* rng;
  double p, in_frac = RANSAC_INLIER_FRAC_EST;
  int i, nm, in, in_min, in_max = 0, k = 0;

  nm = get_matched_features( features, n, mtype, &matched );
  if( nm < m ) {
    //fprintf( stderr, "Warning: not enough matches to compute xform, %s line %d\n", __FILE__, __LINE__ );
    goto end;
  }
  //cout << nm << " matches found " << endl;

  /* initialize random number generator */
  rng = gsl_rng_alloc( gsl_rng_mt19937 );
  gsl_rng_set( rng, time(NULL) );

  in_min = calc_min_inliers( nm, m, RANSAC_PROB_BAD_SUPP, p_badxform );
  p = pow( 1.0 - pow( in_frac, m ), k );
  while( p > p_badxform ) {
    sample = draw_ransac_sample( matched, nm, m, rng );
    extract_corresp_pts( sample, m, mtype, &pts, &mpts );
    if (! xform_fn( pts, mpts, m, homography )) goto iteration_end;
    in = find_consensus( matched, nm, mtype, homography, err_fn, err_tol, &consensus);
    if( in > in_max )	{
      if( consensus_max ) free( consensus_max );
      consensus_max = consensus;
      in_max = in;
      in_frac = (double)in_max / nm;
    } else free( consensus );

  iteration_end:
    release_mem( pts, mpts, sample );
    p = pow( 1.0 - pow( in_frac, m ), ++k );
  }
  //cout << G3M_COUT(homography) << " Homography" << endl;

  /* calculate final transform based on best consensus set */
  if( in_max >= in_min ) {
    extract_corresp_pts( consensus_max, in_max, mtype, &pts, &mpts );
    xform_fn( pts, mpts, in_max, homography );
    in = find_consensus( matched, nm, mtype, homography, err_fn, err_tol, &consensus);
    release_mem( pts, mpts, consensus_max );
    extract_corresp_pts( consensus, in, mtype, &pts, &mpts );
    xform_fn( pts, mpts, in, homography );
    if( inliers ) {
	*inliers = consensus;
	consensus = NULL;
      }
    if( n_in )  *n_in = in;
    release_mem( pts, mpts, consensus );
  } else if( consensus_max ) {
    if( inliers )  *inliers = NULL;
    if( n_in )     *n_in = 0;
    free( consensus_max );
  }

  gsl_rng_free( rng );
 end:
  for( i = 0; i < nm; i++ ) {
    rdata = feat_ransac_data( matched[i] );
    matched[i]->feature_data = rdata->orig_feat_data;
    free( rdata );
  }
  free( matched );
  //cout << "nm = " << nm << "   m = " << m << endl;
  return (nm >= m);
}


bool lsq_homography( sPoint2D64f* pts, sPoint2D64f* mpts, int n, double homography[9] )
{
  if (n<4) {
    fprintf( stderr, "Warning: too few points in lsq_homog(), %s line %d\n",  __FILE__, __LINE__ );
    return false;
  }
  // set the equation for H
  // s [ bu ] = [   h1t  ] [ au ]  =>  s * bu = (h1t a)
  //   [ bv ]   [   h2t  ] [ av ]      s * bv = (h2t a)
  //   [ 1  ]   [   h3t  ] [ 1  ]      s      = (h3t a)
  // => (h1t a)         - (h3t a) bu = e1 =>  [  at    0 0 0  -bu(at) ] [ h1 ] = [ 0 ]
  //            (h2t a) - (h3t a) bv = e2     [ 0 0 0    at   -bv(at) ] [ h2 ]   [ 0 ]
  //                                                                    [ h3 ]   [ 0 ]
  //    since the last element of H (h3[2]) is 1, move the last column, and make it 'Ax = b'.
  double *ap, *bp, au, av, bu, bv;  int i;
  MTX::Matrix<double> A( 2*n, 8 ), b( 2*n, 1 ), x( 8, 1, homography );
  for (i = 0, ap = A.data, bp = b.data; i < n; i++) {
    au = pts[i].x;   bu = mpts[i].x;
    av = pts[i].y;   bv = mpts[i].y;
    G8V_SET( ap,   au,  av, 1.0,  0.0, 0.0, 0.0,  -bu*au, -bu*av ); ap += 8; *bp++ = bu;
    G8V_SET( ap,  0.0, 0.0, 0.0,   au,  av, 1.0,  -bv*au, -bv*av ); ap += 8; *bp++ = bv;
  }
  // solve A * x = b using SVD  (numerically stable solution)
  MTX::MatrixSolver<double> solver;
  solver.solveBySVD( A, x, b );
  homography[8] = 1.0;

  return true;
}



/*
  Calculates the transfer error between a point and its correspondence for
  a given homography, i.e. for a point x, it's correspondence x', and
  homography H, computes d(x', Hx)^2.

  @param pt a point
  @param mpt pt's correspondence
  @param H a homography matrix

  @return Returns the transfer error between pt and mpt given H
*/
double homog_xfer_err( sPoint2D64f pt, sPoint2D64f mpt, double homography[9] )
{
  // Performs a perspective transformation on a single point.  That is, for a
  // point (x, y) and a 3 x 3 matrix T this function returns the point (u, v), where
  // [x' y' w']^T = T * [x y 1]^T,  and  (u, v) = (x'/w', y'/w').
  double xy[3] = { pt.x, pt.y, 1.0 }, uv[3];
  G3M_MUL_MV( uv, homography, xy );
  sPoint2D64f xpt;
  xpt.x = uv[0]/uv[2];
  xpt.y = uv[1]/uv[2];
  return sqrt( dist_sq_2D( xpt, mpt ) );
}



/************************ Local funciton definitions *************************/

/*
  Finds all features with a match of a specified type and stores pointers
  to them in an array.  Additionally initializes each matched feature's
  feature_data field with a ransac_data structure.

  @param features array of features
  @param n number of features in features
  @param mtype match type, one of FEATURE_{FWD,BCK,MDL}_MATCH
  @param matched output as an array of pointers to features with a match of
    the specified type

  @return Returns the number of features output in matched.
*/
int get_matched_features( sift_feature* features, int n, int mtype,
			  sift_feature*** matched )
{
  sift_feature** _matched;
  struct ransac_data* rdata;
  int i, m = 0;

  _matched = (sift_feature**) calloc( n, sizeof( sift_feature* ) );
  for( i = 0; i < n; i++ )
    if( features[i].match ) {
      rdata = (struct ransac_data*) malloc( sizeof( struct ransac_data ) );
      memset( rdata, 0, sizeof( struct ransac_data ) );
      rdata->orig_feat_data = features[i].feature_data;
      _matched[m] = features + i;
      _matched[m]->feature_data = rdata;
      m++;
    }
  *matched = _matched;
  return m;
}



/*
  Calculates the minimum number of inliers as a function of the number of
  putative correspondences.  Based on equation (7) in

  Chum, O. and Matas, J.  Matching with PROSAC -- Progressive Sample Consensus.
  In <EM>Conference on Computer Vision and Pattern Recognition (CVPR)</EM>,
  (2005), pp. 220--226.

  @param n number of putative correspondences
  @param m min number of correspondences to compute the model in question
  @param p_badsupp prob. that a bad model is supported by a correspondence
  @param p_badxform desired prob. that the final transformation returned is bad

  @return Returns the minimum number of inliers required to guarantee, based
    on p_badsupp, that the probability that the final transformation returned
    by RANSAC is less than p_badxform
*/
int calc_min_inliers( int n, int m, double p_badsupp, double p_badxform )
{
  double pi, sum;
  int i, j;

  for( j = m+1; j <= n; j++ )
    {
      sum = 0;
      for( i = j; i <= n; i++ )
	{
	  pi = pow( p_badsupp, i - m ) * pow( 1.0 - p_badsupp, n - i + m ) *
	    gsl_sf_choose( n - m, i - m );
	  sum += pi;
	}
      if( sum < p_badxform )
	break;
    }
  return j;
}



/*
  Draws a RANSAC sample from a set of features.

  @param features array of pointers to features from which to sample
  @param n number of features in features
  @param m size of the sample
  @param rng random number generator used to sample

  @return Returns an array of pointers to the sampled features; the sampled
    field of each sampled feature's ransac_data is set to 1
*/
sift_feature** draw_ransac_sample( sift_feature** features, int n,
				     int m, gsl_rng* rng )
{
  sift_feature** sample, * feat;
  struct ransac_data* rdata;
  int i, x;

  for( i = 0; i < n; i++ )
    {
      rdata = feat_ransac_data( features[i] );
      rdata->sampled = 0;
    }

  sample = (sift_feature**) calloc( m, sizeof( sift_feature* ) );
  for( i = 0; i < m; i++ )
    {
      do
	{
	  x = gsl_rng_uniform_int( rng, n );
	  feat = features[x];
	  rdata = feat_ransac_data( feat );
	}
      while( rdata->sampled );
      sample[i] = feat;
      rdata->sampled = 1;
    }

  return sample;
}



/*
  Extrancs raw point correspondence locations from a set of features

  @param features array of features from which to extract points and match
    points; each of these is assumed to have a match of type mtype
  @param n number of features
  @param pts output as an array of raw point locations from features
  @param mpts output as an array of raw point locations from features' matches
*/
void extract_corresp_pts( sift_feature** features, int n, int mtype,
			  sPoint2D64f** pts, sPoint2D64f** mpts )
{
  sift_feature* match;
  sPoint2D64f* _pts, * _mpts;
  int i;

  _pts = (sPoint2D64f*) calloc( n, sizeof( sPoint2D64f ) );
  _mpts = (sPoint2D64f*) calloc( n, sizeof( sPoint2D64f ) );

  for( i = 0; i < n; i++ ) {
      match = features[i]->match;
      if( ! match )
	fprintf(stderr, "feature does not have match of type %d, %s line %d",
		     mtype, __FILE__, __LINE__ );
      _pts[i].x = features[i]->xy[0];
      _pts[i].y = features[i]->xy[1];
      _mpts[i].x = match->xy[0];
      _mpts[i].y = match->xy[1];
    }

  *pts = _pts;
  *mpts = _mpts;
}



/*
  For a given model and error function, finds a consensus from a set of
  feature correspondences.

  @param features set of pointers to features; every feature is assumed to
    have a match of type mtype
  @param n number of features in features
  @param M model for which a consensus set is being found
  @param err_fn error function used to measure distance from M
  @param err_tol correspondences within this distance of M are added to the
    consensus set
  @param consensus output as an array of pointers to features in the
    consensus set

  @return Returns the number of points in the consensus set
*/
int find_consensus( sift_feature** features, int n, int mtype,
		    double homography[9], ransac_err_fn err_fn, double err_tol,
		    sift_feature*** consensus )
{
  sift_feature** _consensus;
  sift_feature* match;
  sPoint2D64f pt, mpt;
  double err;
  int i, in = 0;

  _consensus = (sift_feature**) calloc( n, sizeof( sift_feature* ) );

  for( i = 0; i < n; i++ ) {
      match = features[i]->match;
      if( ! match )
	fprintf(stderr, "feature does not have match of type %d, %s line %d", mtype, __FILE__, __LINE__ );
      pt.x = features[i]->xy[0];
      pt.y = features[i]->xy[1];
      mpt.x = match->xy[0];
      mpt.y = match->xy[1];
      err = err_fn( pt, mpt, homography );
      if( err <= err_tol ) {
	_consensus[in++] = features[i];
	features[i]->match_confirmed = true;
      } else {
	features[i]->match_confirmed = false;
      }
    }
  *consensus = _consensus;
  return in;
}



/*
  Releases memory and reduces code size above

  @param pts1 an array of points
  @param pts2 an array of points
  @param features an array of pointers to features; can be NULL
*/
static inline void release_mem( sPoint2D64f* pts1, sPoint2D64f* pts2,
				sift_feature** features )
{
  free( pts1 );
  free( pts2 );
  if( features ) free( features );
}
