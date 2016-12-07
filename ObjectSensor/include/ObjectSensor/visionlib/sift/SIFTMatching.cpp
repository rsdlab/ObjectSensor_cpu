
//
// SIFTMatching:
//
// Jaeil Choi
// last modified in Feb, 2009
//

#include <iostream>
#include <float.h>
#include <math.h>
#include "vm_macros.h"
#include "mtx_matrix.hpp"
#include "mtx_matrix_solver.hpp"
#include "imgh_common.hpp"
#include "imgh_drawer.hpp"
#include "imgh_converter.hpp"
#include "imgh_file_io.hpp"
#include "guih_config.hpp"
#include "mtx_matrix_solver.hpp"
#include "mth_homography.hpp"
#include "util_timer.hpp"
#include "SIFTMatching.hpp"
#include "siftpp_core.hpp"

#include "../cuda/cuda_image_edit.h"
using namespace std;

// ===================================================================
// 
// ===================================================================

double AffXform::getScale(void) { return sqrt(R[0] * R[3] - R[1] * R[2]); }
double AffXform::getRotat(void) { 
  double U[4], S[4], T[4];
  MTX::MatrixSolver<double> solver;
  solver.SVD( 2, 2, R, U, S, T, true );
  double aU = G2M_ROTATION_ANGLE( U );	// note that G2V_ROTATION_ANGLE returns
  double aT = G2M_ROTATION_ANGLE( T );	// the rotation angle of FRAME AXES, not points.
  return -(aU - aT);
}
void AffXform::getTransformedPoint(int xywh[4], double px, double py, double np[2]) {
  double c[2], v[2];
  G2V_SET( c, xywh[0] + xywh[2]/2, xywh[1] + xywh[3]/2 );
  G2V_SET( v, px - c[0], py - c[1] );
  np[0] = c[0] + (R[0] * v[0] + R[1] * v[1]) + T[0];
  np[1] = c[1] + (R[2] * v[0] + R[3] * v[1]) + T[1];
}
char* AffXform::getInfo(char buf[]) {
  if (buf) sprintf(buf, "[%.0f %.0f s=%.1f a=%.0f](%2.0f, %.2f)", 
		   T[0], T[1], getScale(), getRotat()*180/M_PI, etc[0], etc[1]);
  return buf;
}


// ===================================================================
// 
// ===================================================================

void SIFTMatching::clear(void) 
{
  xform.clear();
  G6V_SET( timer_log, 0, 0, 0, 0, 0, 0 );
  matched_count = 0;
  if (matches) { free(matches); matches=NULL; }
}

// ===================================================================
// Matching features
// ===================================================================

bool SIFTMatching::findSIFTPattern(SIFT *refer, SIFT *scene) 
{
	// Match its SIFT features against the 'scene' SIFT features,
	//   calculate the homography in 'homography[9]',
	//   and then estimate the 3D pose of the pattern in the camera frame.
	//   The 3D pose is saved in 'scene->Rco[9]' and 'scene->Tco[3]', only if 'cp != NULL'.
	if (!refer || !scene || refer->fimg.h <= 0 || scene->fimg.h <= 0) return false;
	this->refer = refer;
	this->scene = scene;
	if (verbose) printf("Matching SIFT features \n");
	if (use_timer) timer.start();
	matched_count = 0;
	xform.clear();  homography[8] = 0.0;
  
	if (use_simple_match) {
		findMatchesBrutalSearch( 0.75f );
		return true;
	}
  
	// register all the features on 'fmap', based on their locations
	// 'fmap' can be used for local search, as in the 'findClosestFeature()'.
	if  (fmap.sameSize(&scene->iimg)) {
		fmap.clearImage(0);
	}
	else {
		fmap.setImage( scene->iimg.w, scene->iimg.h, IMGH::PIXEL_VOIDP, true );
	}
	int  i, idx;
	if (verbose) printf("h:%d\n",scene->fimg.h);
	for (i=0, idx=0; i < scene->fimg.h; i++) {
		SIFTFeature *sfp = (SIFTFeature*)scene->fimg.getRow(i);
		idx = fmap.getIndex( (int)sfp->f.x, (int)sfp->f.y );
		sfp->f.next = (SIFTFeature*)fmap.getPointer( idx );
		fmap.setPointer( idx, sfp );
	}
	if (verbose) fmap.printInfo("  image of feature pointers : ");
	if (use_timer) timer_log[0] = timer.checkTime();
	// find some good matches to use as the initial candidates
	matched_count = findBestMatchesBrutalSearch( 10, 0.90 );
	if (verbose) printf("mcount:%d\n",matched_count);
	if (matched_count < 10) {
	if (verbose) printf("  initial matching failed (%d matched)\n", matched_count);
		return false;
	} else {
		if (verbose) printf("  initial %d matches found\n", matched_count);
	}
  //scene->checkMatches();
  if (use_timer) timer_log[1] = timer.checkTime();
  
#if 1
  int       nx = matched_count,  max_matched=0, bestx=0;
  AffXform *xlist = (AffXform*)malloc(nx*sizeof(AffXform));
  for (i = 0; i < nx; i++) 
    guessAffineTransformation( matches[i].rfp, matches[i].sfp, xlist+i );
  for (i = 0; i < nx; i++) {
    // find all the correspondences based on 'this->xform'
    findMatchesWithTransformation( xlist+i );
    finalizeTransformation( xlist+i );
    if (matched_count > max_matched) { max_matched = matched_count; bestx = i; }
  }
  if (max_matched < 4) {
	  matched_count = 0;
	  return false;
  }
  else xform = xlist[bestx];
  free(xlist);
#else
  // run a particle filter to find the best pose (affine transformation) 'this->xform'
  ALGH::ParticleFilter<AffXform> pfilter;
  int  np, np_max;
  if      (matched_count <  10) { np = matched_count*3; np_max = np; }
  else if (matched_count <  50) { np = matched_count*2; np_max = np; }
  else if (matched_count < 100) { np = matched_count*1; np_max = np; }
  else                          { np = 100;             np_max = np; }
  pfilter.setupParticleFilter( this, np_max, h_init, h_eval, h_copy, NULL, NULL, h_info );
  pfilter.initializeParticles( np );
  if (verbose) pfilter.printInfo("init", true);
  pfilter.updateParticles( 3, true, true );
  if (verbose) pfilter.printInfo("upda", true);
  if (verbose) { char buf[320]; printf("  best pose found at %s\n", pfilter.est.getInfo(buf)); }
  //printf("  pfilter best (%.0f %.1f)\n", pfilter.est.etc[0], pfilter.est.etc[1]);
  if (pfilter.est.etc[0] < 4 || pfilter.est.etc[1] > 20) { 
    matched_count = 0;  return false;
  } else xform = pfilter.est;
#endif
  if (use_timer) timer_log[2] = timer.checkTime();
  
  // find all the correspondences based on 'this->xform'
  findMatchesWithTransformation( &this->xform );
  finalizeTransformation( &this->xform );
  if (verbose) { char buf[80]; printf("  final %d matches found at %s\n", matched_count, this->xform.getInfo(buf)); }
  //scene->checkMatches();
  if (use_timer) timer_log[3] = timer.checkTime();
  
  // calculate the homography
  double scale = this->xform.getScale();
  int  min_matches = (int)(scale * 10);  if (min_matches<5) min_matches=5;
  bool ret = estimateHomography( min_matches );
  if (use_timer) timer_log[4] = timer.checkTime();
  if (use_timer) timer_log[5] = timer.stop();
  return ret;
}

int SIFTMatching::findMatchesBrutalSearch(double max_ratio)
{
  // Find the best match in the scene, for each SIFT feature.
  //   'max_ratio' : the max ratio of the closest and the second-closest distance
  //   'uvsa[6]'  : additional restriction in region (u,v), scale (s), and orientation (a)
  if (this->matches) { free(this->matches); this->matches=NULL; }
  this->matches = (sift_match*)malloc( refer->fimg.h * sizeof(sift_match));
  matched_count = 0;
  SIFTFeature  *rfp, *sfp, *c0, *c1;
  int    i, j, k;
  for (i = 0; i < refer->fimg.h; i++) ((SIFTFeature*)refer->fimg.getRow(i))->f.match = NULL;
  for (j = 0; j < scene->fimg.h; j++) ((SIFTFeature*)scene->fimg.getRow(j))->f.match = NULL;
  for( i = matched_count = 0; i < refer->fimg.h; i++ ) {
    rfp = (SIFTFeature*)refer->fimg.getRow(i);
    // find the first and the second closest neighbor
    float  d0=1e6, d1=1e6, dist;
    for (j = 0, c0 = c1 = NULL; j < scene->fimg.h; j++) {
      sfp = (SIFTFeature*)scene->fimg.getRow(j);
      dist = (float)getFeatureDistance( rfp, sfp );
      //if (dist > 0.3*0.3) continue;	// 
      if (dist < d0 ) {			// update the best candidate
	d1 = d0;   c1 = c0;
	d0 = dist; c0 = sfp;
      } else if (dist < d1) {		// update the second candidate
	d1 = dist; c1 = sfp;
      }
    }
    sfp = (c0 && d0 < d1*max_ratio ? c0 : NULL);
    // check if the result conflicts with prior matches
    if (sfp) {				// found the best match
      if (sfp->f.match) {		// there is a match with 'sfp' already
	if (d0 < getFeatureDistance( sfp->f.match, sfp )) {	// better than the previous match
	  sfp->f.match->f.match = NULL;  sfp->f.match = NULL;
	  // remove the old match from the list 'this->matches[]'
	  for (k = 0; k < matched_count; k++) if (this->matches[k].sfp==sfp) break;
	  if  (k < matched_count-1)
	    this->matches[k] = this->matches[matched_count-1];
	  matched_count--;
	} else {			// worse than previous match => cancelled
	  sfp = NULL;
	}
      }
      if (sfp) {			// if it is still the best match
	rfp->f.match = sfp;  sfp->f.match = rfp;
	this->matches[matched_count].rfp = rfp;
	this->matches[matched_count].sfp = sfp;
	matched_count++;
      } else rfp->f.match = NULL; 
    } else rfp->f.match = NULL;
  }
  return matched_count;
}

int SIFTMatching::findBestMatchesBrutalSearch(int min_matches, double max_ratio)
{
	// Find the best match in the scene, for each SIFT feature.
	//   'min_matches' : minimum number of matches
	//   'max_ratio'   : the max ratio of the closest and the second-closest distance
	if (this->matches) { free(this->matches); this->matches=NULL; }
	this->matches = (sift_match*)malloc( refer->fimg.h * sizeof(sift_match));
	SIFTFeature  *rfp, *sfp, *c0, *c1;
	int    i, j, k, cnt[10]={0,0,0,0,0,0,0,0,0,0};
	for (i = 0; i < refer->fimg.h; i++) ((SIFTFeature*)refer->fimg.getRow(i))->f.match = NULL;
	for (j = 0; j < scene->fimg.h; j++) ((SIFTFeature*)scene->fimg.getRow(j))->f.match = NULL;
	/////
	for( i = 0; i < refer->fimg.h; i++ ) {
		rfp = (SIFTFeature*)refer->fimg.getRow(i);
		// find the first and the second closest neighbor
		float  d0=1e6, d1=1e6, mratio, dist;
		for (j = 0, c0 = c1 = NULL; j < scene->fimg.h; j++) {
			sfp = (SIFTFeature*)scene->fimg.getRow(j);
			dist = (float)getFeatureDistance( rfp, sfp );
			//if (dist > 0.3*0.3) continue;	//
			if (dist < d0 ) {			// update the best candidate
				d1 = d0;   c1 = c0;
				d0 = dist; c0 = sfp;
			} else if (dist < d1) {		// update the second candidate
				d1 = dist; c1 = sfp;
			}
		}
		mratio = d0 / d1;
		if( mratio < max_ratio ) {
			sfp = c0;
		} else {
			sfp = NULL;
		}
		// check if the result conflicts with prior matches
		if (sfp) {				// found the best match
			if (sfp->f.match) {		// there is a match with 'sfp' already
				if (d0 < getFeatureDistance( sfp->f.match, sfp )) {	// better than the previous match
					sfp->f.match->f.match = NULL;  sfp->f.match = NULL;
				} else {			// worse than previous match => cancelled
					sfp = NULL;
				}
			}
			if (sfp) {			// if it is still the best match
				rfp->f.match = sfp;  sfp->f.match = rfp;
				rfp->f.mratio = mratio;
				cnt[(int)(mratio*10)]++;
			} else rfp->f.match = NULL;
		} else rfp->f.match = NULL;
	}
	// determine the ratio limt for the given number of matches
	int   cntsum;
	float rlimit=0.7f;
	for (k = cntsum = 0; k < 9; k++) {
		cntsum += cnt[k];
		if (cntsum > min_matches) { rlimit = (k+1) / 10.0f; break; }
	}
	if (k == 9) return 0;
	// select the matches that have the ratio below the limit
	for( i = matched_count = 0; i < refer->fimg.h; i++ ) {
		rfp = (SIFTFeature*)refer->fimg.getRow(i);
		sfp = rfp->f.match;
		if (!sfp) continue;
		else if (rfp->f.mratio < rlimit) {
			matches[matched_count].rfp = rfp;
			matches[matched_count].sfp = sfp;
			matched_count++;
		} else {
			rfp->f.match = NULL;
			sfp->f.match = NULL;
		}
	}
	return matched_count;
}

double SIFTMatching::getFeatureDistance( SIFTFeature* f1, SIFTFeature* f2 )
{
	double diff, dsq = 0;
	float *descr1 = f1->f.desc;
	float *descr2 = f2->f.desc;
	for (int i = 0; i < SIFT_SIZE; i++ ) {
		if( _isnan(descr1[i])!=0 || _isnan(descr2[i])!=0 ) {
			return FLT_MAX;
		}
		diff = descr1[i] - descr2[i];
		dsq += diff*diff;
	}
	return dsq;
}

int SIFTMatching::findMatchesWithTransformation(AffXform *xf, bool eval_only, double result_dist[3])
{
  // Find matches with given affine transformation 'xf', by looking for
  //   the best match in the local region according to the transformation.
  int i, j, mcount;
  // calculate the new pattern pose
  for (i = 0; i < refer->fimg.h; i++) { SIFTFeature *rfp = (SIFTFeature*)refer->fimg.getRow(i); rfp->f.match = NULL; }
  for (j = 0; j < scene->fimg.h; j++) { SIFTFeature *sfp = (SIFTFeature*)scene->fimg.getRow(j); sfp->f.match = NULL; }
  if (!eval_only) {
    if (this->matches) { free(this->matches); this->matches=NULL; }
    this->matches = (sift_match*)malloc( refer->fimg.h * sizeof(sift_match));
    matched_count = 0;
  }
  double rc[2], sxy[2], fdist=0;
  double scale = xf->getScale();
  double rotat = xf->getRotat();
  SIFTFeature *rfp=NULL, *sfp=NULL;
  G2V_SET( rc, refer->xywh[0] + refer->xywh[2]/2, refer->xywh[1] + refer->xywh[3]/2 );
  // find the correspondences
  for (i=0; i<refer->fimg.h; i++) {
    rfp = (SIFTFeature*)refer->fimg.getRow(i);
    xf->getTransformedPoint( refer->xywh, rfp->f.x, rfp->f.y, sxy );
    sfp = findClosestFeature( rfp, sxy[0], sxy[1], 30,
			      rfp->f.o - rotat, M_PI/4,
			      rfp->f.s * scale, 0.5, &fdist );
    if (!sfp) continue;
    if (sfp->f.match) {	// check old match, if any (in order to make it 1:1)
      SIFTFeature *ofp = sfp->f.match;
      double odist = getFeatureDistance( ofp, sfp );
      if (fdist >= odist) sfp = NULL;
      else ofp->f.match = sfp->f.match = NULL; 
    }
    if (sfp) { rfp->f.match = sfp;  sfp->f.match = rfp; }
  }
  if (result_dist) memset(result_dist, 0, 3*sizeof(double));
  // evaluate the correspondences
  for (i=mcount=0; i<refer->fimg.h; i++) {
    rfp = (SIFTFeature*)refer->fimg.getRow(i);
    sfp = rfp->f.match;
    if (!sfp) continue;
    xf->getTransformedPoint( refer->xywh, rfp->f.x, rfp->f.y, sxy );
    fdist = getFeatureDistance( rfp, sfp );
    if (result_dist) {
      result_dist[0] += fdist;				// sum of squared feature distance
      result_dist[1] += fdist * rfp->f.s/2;		// ssd weighted by feature scale
      result_dist[2] += G2V_DISTANCE( sxy, sfp->data );	// sum of image distance
    }
    if (!eval_only) {
      this->matches[ mcount ].rfp = rfp;
      this->matches[ mcount ].sfp = sfp;
    }
    mcount++;
  }
  if (!eval_only) matched_count = mcount;
  return mcount;
}

void SIFTMatching::finalizeTransformation(AffXform *f)
{
  // Calcualte the optimal affine transformation 'f', by solving least-square.
  // This function updates the affine transformation 'f', by updating it after outlier elimination.
  int     i, M=2*matched_count, removed=0;
  double *A = (double*)malloc( M*6*sizeof(double) );
  double *b = (double*)malloc( M * sizeof(double) );
  if (!A || !b) return;
  do {
    if (matched_count < 4) return;
    M = 2 * matched_count;
    double  x[6], *aa;
    // Note the transformation is in local coordinates, not the image coordinates.
    //   [ ra, rb, tx ] [ rx-cx ] = [ sx-cx ]
    //   [ rc, rd, ty ] [ ry-cy ]   [ sy-cx ]
    //   [  0,  0,  1 ] [   1   ]   [   1   ]
    double cx = refer->xywh[0] + refer->xywh[2]/2;
    double cy = refer->xywh[1] + refer->xywh[3]/2;
    for (i=0; i<matched_count; i++) {
      SIFTFeature *rfp = this->matches[ i ].rfp;
      SIFTFeature *sfp = this->matches[ i ].sfp;
      aa = A + 6 * (2*i);
      G6V_SET( aa, (rfp->f.x-cx), (rfp->f.y-cy), 0, 0, 1, 0 );  b[2*i+0] = sfp->f.x - cx;
      aa = A + 6 * (2*i+1);
      G6V_SET( aa, 0, 0, (rfp->f.x-cx), (rfp->f.y-cy), 0, 1 );  b[2*i+1] = sfp->f.y - cy;
    }
    MTX::MatrixSolver<double> solver;
    bool ret = solver.solveBySVD( M, 6, A, x, b );
    if (f && ret) {
      G2M_SET( f->R, x[0], x[1], x[2], x[3] );
      G2V_SET( f->T, x[4], x[5] );
    }
    // eliminate features with extreme geometric error
    double ixy[2], idist, idsum=0, idavg;
    for (i=0; i<matched_count; i++) {
      float *rxy = &(this->matches[ i ].rfp->f.x);
      float *sxy = &(this->matches[ i ].sfp->f.x);
      f->getTransformedPoint( refer->xywh, rxy[0], rxy[1], ixy );
      idist = G2V_DISTANCE( sxy, ixy );
      idsum += idist;
    }
    idavg = idsum / matched_count;
    for (i=0; i<matched_count; i++) {
      float *rxy = &(this->matches[ i ].rfp->f.x);
      float *sxy = &(this->matches[ i ].sfp->f.x);
      f->getTransformedPoint( refer->xywh, rxy[0], rxy[1], ixy );
      idist = G2V_DISTANCE( sxy, ixy );
      if (idist > idavg*2.5) matches[i].rfp = matches[i].sfp = NULL;
    }
    for (i=removed=0; i<matched_count; i++) {
      if (!matches[i].rfp || !matches[i].sfp) { removed++; continue; }
      if (removed > 0) matches[i - removed] = matches[i];
    }
    //printf("matched_count=%d  avgdist=%.4f  removed=%d\n", matched_count, idavg, removed);
    matched_count -= removed;
  } while (removed > 0);
  free(A); free(b);
}

void SIFTMatching::analyzeMatching(double mmv[3])
{
  // Analyze the distances between matched features.
  double dist, minv=0, maxv=0, sum=0;
  for (int i=0; i<matched_count; i++) {
    SIFTFeature *rfp = this->matches[ i ].rfp;
    SIFTFeature *sfp = this->matches[ i ].sfp;
    dist = sqrt(getFeatureDistance( rfp, sfp ));
    if (i==0 || dist < minv) minv = dist;
    if (i==0 || dist > maxv) maxv = dist;
    sum += dist;
  }
  if (mmv) G3V_SET( mmv, sum/matched_count, minv, maxv );
}

SIFTFeature* SIFTMatching::findClosestFeature(SIFTFeature *rfp, double x, double y, int max_dist, 
					      double ori, double ori_tol, double scl, double scl_tol,
					      double *fdist)
{
  static SIFTFeature *farray[50];
  int  i, j, k, max_count=30;
  // Find all the features in the rectangular window around (x,y) with size 'max_dist',
  //   using the image of feature pointers 'fmap', and save the results in 'array[]'.
  SIFTFeature *fp;
  int  u=(int)x, v=(int)y, total=fmap.w * fmap.h; 
  int  vst, cnt, idx, len, step, steps[4]={ +1, +fmap.w, -1, -fmap.w };
  if (max_dist <= 0) max_dist = 100;
  for (i = cnt = vst = 0; i < max_dist; i++) {		// for each distance
    idx = (v-i) * fmap.w + (u-i);
    for (j = 0; j < 4; j++) {		// for four sides of the rectangular boundary
      len  = 2 * i + 1;
      step = steps[j];
      for (k = 0; k < len; k++) {	// for each side
	if (idx < 0 || idx >= total) fp = NULL;
	else fp = (SIFTFeature*)fmap.getPointer( idx );
	for (; fp && cnt<max_count; fp = fp->f.next) farray[cnt++] = fp;
	idx += step;  vst++;
      }					// for each side
      if (i==0 || cnt>=max_count) break;
    }					// for four sides of the rectangular boundary
    if (cnt>=max_count) break;
  }					// for each distance
  // find the most similar feature among them
  SIFTFeature *sfp=NULL, *min_sfp=NULL, *nxt_sfp=NULL;
  double dist, min_dist=1e8, nxt_dist=1e8, odiff=0;
  for (i = 0; i < cnt; i++) {
    sfp = farray[i];
    bool conforming = true;
    if (ori_tol>0) {
      odiff = sfp->f.o - ori;
      while (odiff >  +M_PI) odiff -= 2*M_PI;
      while (odiff <= -M_PI) odiff += 2*M_PI;
      if (fabs(odiff) > ori_tol) conforming = false;
    }
    if (scl_tol>0) {
      if (sfp->f.s > (1.0 + scl_tol) * scl ||
	  sfp->f.s < (1.0 - scl_tol) * scl) conforming = false;
    }
    dist = getFeatureDistance( rfp, sfp );
    if (dist > 0.25*0.25) continue;  //// 
    // remember the best and the second-best candidates
    if        (dist < min_dist && conforming) { 
      if (min_dist < nxt_dist) { nxt_sfp = min_sfp;  nxt_dist = min_dist; }
      min_sfp = sfp;      min_dist = dist; 
    } else if (dist < nxt_dist) {
      nxt_sfp = sfp;      nxt_dist = dist;
    }
  }
  if (fdist) *fdist = min_dist;
  if (!min_sfp) return NULL;
  return (min_dist < nxt_dist * 0.6 ? min_sfp : NULL);
}

void SIFTMatching::guessAffineTransformation(SIFTFeature *rfp, SIFTFeature *sfp, AffXform *xf)
{
  double adiff = sfp->f.o - rfp->f.o;  // in radian
  double sdiff = sfp->f.o / rfp->f.s;
  // calculate the center of the pattern on the scene
  double rc[2], sc[2], rdir[2], sdir[2], R[4], T[2];
  G2V_SET( rc, refer->xywh[0] + refer->xywh[2]/2, refer->xywh[1] + refer->xywh[3]/2 );
  G2V_SUB( rdir, rc, rfp->data );
  G2M_SET( R, cos(adiff), -sin(adiff), +sin(adiff), cos(adiff) );
  G2M_MUL_VALUE( R, sdiff );
  G2M_MUL_MV( sdir, R, rdir );
  G2V_ADD( sc, sfp->data, sdir );
  G2V_SUB( T, sc, rc );
  // initialize the hypothesis
  G2M_COPY( xf->R, R );
  G2V_COPY( xf->T, T );
  G2V_SET ( xf->etc, 0, 0 );
}

bool SIFTMatching::estimateHomography(int min_matches) 
{
  // Note that is function is called from the 'reference', not the 'scene'.
  homography[8] = 0;
  if (matched_count < min_matches) {
    if (verbose) printf("  estimating homography : %d matches are not enough\n", matched_count);
    return false;
  }
  if (verbose) printf("Estimating homography from %d matches\n", matched_count);
  MTH::Homography<double> hmg;
  MTX::Matrix<double> A(2, matched_count), B(2, matched_count);
  int  i, cnt;
  for (i=cnt=0; i < matched_count; i++) {
    SIFTFeature *rfp = this->matches[i].rfp;  if (!rfp) continue;
    SIFTFeature *sfp = this->matches[i].sfp;  if (!sfp) continue;
    A(0,i) = rfp->f.x;  B(0,i) = sfp->f.x;
    A(1,i) = rfp->f.y;  B(1,i) = sfp->f.y;
    cnt++;
  }
  bool ret = hmg.calculateHomography( A, B );
  ret = (ret && hmg.checkConvexity(true));
  // check the validity of homography
  if (ret) memcpy( homography, hmg.h, 9*sizeof(double) );
  else   { matched_count = -1; homography[8] = 0; }
  if (verbose) {
    if (ret) printf("  estimating homography : success\n");
    else     printf("  estimating homography : FAILED \n");
  }
  return ret;
}

// ===================================================================
// 
// ===================================================================

void SIFTMatching::printInfo(char *cmmt)
{
  printf("%s  %d\n", (cmmt ? cmmt : "SIFT Matching"), matched_count);
  printf("  Refer: %4d features in ", refer->fimg.h); refer->iimg.printInfo("");
  printf("  Scene: %4d features in ", scene->fimg.h); scene->iimg.printInfo("");
  printf("  AffineXform [ %9.4f %9.4f   %9.4f ]\n", xform.R[0], xform.R[1], xform.T[0]);
  printf("     [A  b]   [ %9.4f %9.4f   %9.4f ]\n", xform.R[2], xform.R[3], xform.T[1]);
  if (use_timer) 
    printf("  stopwatch: fmap:%6.3f brutal:%6.3f pf:%6.3f best:%6.3f homo:%6.3f  Total:%6.4f\n",
	   timer_log[0], timer_log[1], timer_log[2], timer_log[3], timer_log[4], timer_log[5]);
}

bool SIFTMatching::getResultPose(double corners[4][2], double center[2], double sr[2])
{
  // Get the 2D pose of the detected pattern.
  // Make sure this function is called only after 'findPattern()'.
  if (!refer || refer->fimg.h <= 0 || 
      !scene || scene->fimg.h <= 0 || xform.isInvalid()) return false;
  
  double x=refer->xywh[0], y=refer->xywh[1], w=refer->xywh[2], h=refer->xywh[3];
  double c0[2], c1[2], c2[2], c3[2], cc[2], ssr[2];
  if (homography[8] == 1.0) {	// if available, use homography first
    MTH::Homography<double> H(homography);
    H.transform( x  , y  , c0 );
    H.transform( x  , y+h, c1 );
    H.transform( x+w, y+h, c2 );
    H.transform( x+w, y  , c3 );
  } else {				// otherwise, use affine transformation
    xform.getTransformedPoint( refer->xywh, x  , y  , c0 );
    xform.getTransformedPoint( refer->xywh, x  , y+h, c1 );
    xform.getTransformedPoint( refer->xywh, x+w, y+h, c2 );
    xform.getTransformedPoint( refer->xywh, x+w, y  , c3 );
  }
  G2V_AVERAGE4( cc, c0, c1, c2, c3 );
  G2V_SET( ssr, xform.getScale(), xform.getRotat() );
  
  if (corners) {	// four corners of the detected pattern
    G2V_COPY( corners[0], c0 );
    G2V_COPY( corners[1], c1 );
    G2V_COPY( corners[2], c2 );
    G2V_COPY( corners[3], c3 );
  }
  if (center) G2V_COPY( center, cc);	// center of the detected pattern
  if (sr)     G2V_COPY( sr, ssr );	// scale and rotation (radian) of the detected pattern
  if (verbose) {
    printf("[Vsft] 2D Pose : c=(%.0f %.0f) s=%.1f r=%.0f  (%3.0f,%3.0f)(%3.0f,%3.0f)(%3.0f,%3.0f)(%3.0f,%3.0f)\n", 
	   cc[0], cc[1], ssr[0], ssr[1]*180/M_PI, c0[0], c0[1], c1[0], c1[1], c2[0], c2[1], c3[0], c3[1]);
  }
  return true;
}

void SIFTMatching::showMatches(IMGH::Image *img, int r_uoff, int r_voff, int uoff, int voff)
{
  // Visualize matched feature pairs using red lines, assumming 
  //   the target image was merged with the reference image by the offset (uoff, voff)
  // Note that this function is called by the 'scene', not the 'reference'.
  if (this->matches == NULL) return;
  IMGH::Drawer<double> drw(img);
  //drw.line_thickness = 2.0;
  int  i, rgb[3];
  for (i = 0; i < this->matched_count; i++) {
    SIFTFeature *rfp = this->matches[i].rfp;  if (!rfp) continue;
    SIFTFeature *sfp = this->matches[i].sfp;  if (!sfp) continue;
    G3V_SET( rgb,   0, 255,   0 );
    drw.drawLine( rfp->f.x + r_uoff, rfp->f.y + r_voff, 
		  sfp->f.x +   uoff, sfp->f.y +   voff, rgb[0], rgb[1], rgb[2], true );
  }
}

void SIFTMatching::showTransformation(IMGH::Image *img, int rxywh[4], int uoff, int voff)
{
  // Visualize the result of the 2D pose estimation on the image 'img'.
  // Note that this function is called by the 'scene', not the 'reference'.
  if (xform.isInvalid()) return;
  IMGH::Drawer<double> drw( img );
  double x=rxywh[0], y=rxywh[1], w=rxywh[2], h=rxywh[3];
  double c0[2], c1[2], c2[2], c3[2];
  // show the affine transformation
  xform.getTransformedPoint( rxywh, x  , y  , c0 );
  xform.getTransformedPoint( rxywh, x  , y+h, c1 );
  xform.getTransformedPoint( rxywh, x+w, y+h, c2 );
  xform.getTransformedPoint( rxywh, x+w, y  , c3 );
#if 0
  drw.drawQuad( c0[0]+uoff, c0[1]+voff,  c1[0]+uoff, c1[1]+voff,
		c2[0]+uoff, c2[1]+voff,  c3[0]+uoff, c3[1]+voff, 0, 0, 100, true );
#else
  drw.drawLine( c0[0]+uoff, c0[1]+voff,  c1[0]+uoff, c1[1]+voff, 255, 0, 0, true );
  drw.drawLine( c1[0]+uoff, c1[1]+voff,  c2[0]+uoff, c2[1]+voff, 0, 255, 0, true );
  drw.drawLine( c2[0]+uoff, c2[1]+voff,  c3[0]+uoff, c3[1]+voff, 0, 0, 255, true );
  drw.drawLine( c3[0]+uoff, c3[1]+voff,  c0[0]+uoff, c0[1]+voff, 80, 80, 80, true );
#endif
  //printf("(%.0f %.0f) (%.0f %.0f) (%.0f %.0f) (%.0f %.0f) \n", c0[0], c0[1], c1[0], c1[1], c2[0], c2[1], c3[0], c3[1]);
  
  // show the homography
  if (homography[8] != 1.0) return;
  MTH::Homography<double> H(homography);
  H.transform( x  , y  , c0 );
  H.transform( x  , y+h, c1 );
  H.transform( x+w, y+h, c2 );
  H.transform( x+w, y  , c3 );
  drw.line_thickness = 3.0;
#if 0
  drw.drawQuad( c0[0]+uoff, c0[1]+voff,  c1[0]+uoff, c1[1]+voff,
		c2[0]+uoff, c2[1]+voff,  c3[0]+uoff, c3[1]+voff, 0, 0, 255, true );
#else
  drw.drawLine( c0[0]+uoff, c0[1]+voff,  c1[0]+uoff, c1[1]+voff, 255, 0, 0, true );
  drw.drawLine( c1[0]+uoff, c1[1]+voff,  c2[0]+uoff, c2[1]+voff, 0, 255, 0, true );
  drw.drawLine( c2[0]+uoff, c2[1]+voff,  c3[0]+uoff, c3[1]+voff, 0, 0, 255, true );
  drw.drawLine( c3[0]+uoff, c3[1]+voff,  c0[0]+uoff, c0[1]+voff, 80, 80, 80, true );
#endif
}

