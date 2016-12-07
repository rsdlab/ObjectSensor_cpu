
//
// SIFTPatternFinder: 
//   3D pose estimation using SIFT feature matching
//
// Jaeil Choi
// last modified in Feb, 2009
//

#include <iostream>
#include "guih_config.hpp"
#include "imgh_file_io.hpp"
#include "imgh_drawer.hpp"
#include "mth_homography.hpp"
#include "mth_rotation.hpp"
#include "mtx_matrix_solver.hpp"
#include "geometry3d.hpp"
#include "minpack/CMinpack.hpp"
#include "SIFTPatternFinder.hpp"
using namespace std;

// ===================================================================
// setting reference pattern and its pose
// ===================================================================

bool SIFTPatternFinder::loadCameraParameters(char *cfg_file, char *cfg_section)
{
  // Load camera intrinsic and extrinsic parameters
  //   intrinsic : needed for 3D pose estimation
  if (!cfg_file || !cfg_section) return false;
  if (!camera.readConfigFile( cfg_file, cfg_section, true, false, false )) {
    if (verbose) printf("[sift] loading camera intrinsic parameters .. FAILED ('%s')\n", cfg_section);
    return false;
  }
  if (verbose) printf("[sift] loading camera intrinsic parameters .. done ('%s')\n", cfg_section);
  return true;
}

bool SIFTPatternFinder::loadReferencePattern(char *cfg_file, char *cfg_section, int left_right)
{
	// Load the SIFT reference image from a configuration file 'cfg_file'.
	// Example of the configuration:
	//  [ cup ]
	//   RefImgFile       = cup_ref_image.png
	//   RefImgRegion     = 150 : 100 : 300 : 300
	//   CameraExtrinsic0 = 1.0   0.00   0.00   0.00	# pose of the reference pattern
	//   CameraExtrinsic1 = 0.0  -1.00   0.00   0.00	# in camera frame [Rco Tco]
	//   CameraExtrinsic2 = 0.0   0.00  -1.00   0.25
	if (verbose) printf("[sift] Loading SIFT reference image from configuration file '%s' [%s]\n", cfg_file, cfg_section);
	char  ref_img[200], ref_region[200];
	GUIH::Config cfg;
	cfg.set( cfg_section, "RefImgFile",   CFG_STRING, ref_img );
	if (left_right<=0) cfg.set( cfg_section, "RefImgRegion" , CFG_STRING, ref_region );
	else               cfg.set( cfg_section, "RefImgRegion2", CFG_STRING, ref_region );
	if (!cfg.process( cfg_file, cfg_section, false, false )) {
		if (verbose) printf("[sift] Error (SIFTpp::loadFeatures): cannot read camera parameters\n");
		return false;
	}
	// set the region of the pattern on the image
	sscanf( ref_region, "%d : %d : %d : %d", refer.xywh+0, refer.xywh+1, refer.xywh+2, refer.xywh+3 );
	refer.xywh_given = true;
	// read the image file into 'refer.iimg'
	IMGH::FileIO ifile;
	IMGH::Image temp;
	if (!ifile.readFile( ref_img, &temp, IMGH::PIXEL_FLOAT )) {
		if (verbose) printf("[sift] Error (SIFTpp::loadFeatures): cannot open the image file '%s'\n", ref_img);
		return false;
	}
	if(left_right < 0) {		// use half of the image on the right
		refer.iimg.setImage( temp.w/2, temp.h, IMGH::PIXEL_FLOAT );
		temp.copySubregionTo( 0, 0, temp.w/2, temp.h, &refer.iimg, 0, 0 );
	} else if (left_right > 0) {		// use half of the image on the right
		refer.iimg.setImage( temp.w/2, temp.h, IMGH::PIXEL_FLOAT );
		temp.copySubregionTo( temp.w/2, 0, temp.w/2, temp.h, &refer.iimg, 0, 0 );
		refer.xywh[0] -= temp.w/2;
	} else {				// use entire image
		refer.iimg.swapImage( &temp );
	}
	// read the pose of the reference pattern [ Rcf Tcf ]
	Camera<double> cam;
	bool pose_ready = false;
	if (cam.readConfigFile( cfg_file, cfg_section, false, true, false )) {
		memcpy( Rcf, cam.Rcw, 9*sizeof(double) );	// extrinsic parameters
		memcpy( Tcf, cam.Tcw, 3*sizeof(double) );	// extrinsic parameters
		memcpy( camera.Rcw, Rcf, 9*sizeof(double) );  // added by iwane 2011.01.13
		memcpy( camera.Tcw, Tcf, 3*sizeof(double) );  // In order to print Extrinsc parameter
	                                             // when function[ pf.camera.printInfo() ] is used 
		pose_ready = true;
	}
	//
	refer.verbose = sift_verbose;
	if (refer.extractFeatures( sift_nOctaves, sift_nLevels, sift_sigma, sift_ethreshold, use_gpu ) < 1) {
		if (verbose) printf("[sift] Error (SIFTpp::loadFeatures): cannot extract SIFT feature at all\n");
		return false;
	}
	if (verbose) printf("[sift] loading ref. pattern '%s' .. done (%s, %d features)\n",
		      cfg_section, (pose_ready ? "with 3D pose" : "WITHOUT 3D pose"), refer.fimg.h);

	return true;
}

bool SIFTPatternFinder::loadReferencePattern(char *img_file, int xywh[4], double Rcf[9], double Tcf[3])
{
  // Read an image file as the reference image.
  IMGH::FileIO imgfile;
  if (! imgfile.readFile( img_file, &refer.iimg, IMGH::PIXEL_FLOAT )) { 
    if (verbose) printf("[sift] loading ref. pattern '%s' .. FAILED\n", img_file);
    return false;
  }
  refer.setupInputImage( &refer.iimg, xywh );
  refer.verbose = sift_verbose;
  refer.extractFeatures( sift_nOctaves, sift_nLevels, sift_sigma, sift_ethreshold, use_gpu );
  bool pose_ready = false;
  if (Rcf && Tcf) {
    memcpy( this->Rcf, Rcf, 9*sizeof(double) );
    memcpy( this->Tcf, Tcf, 3*sizeof(double) );
	memcpy( this->camera.Rcw, Rcf, 9*sizeof(double) );  // added by iwane 2011.01.13
	memcpy( this->camera.Tcw, Tcf, 3*sizeof(double) );  // In order to print Extrinsc parameter 
	                                             // when function[ pf.camera.printInfo() ] is used 

  }
  if (verbose) printf("[sift] loading ref. pattern '%s' .. done (%s, %d features)\n", img_file,
		      (pose_ready ? "with 3D pose" : "WITHOUT 3D pose"), refer.fimg.h);
  return true;
}

// ===================================================================
// setting the input scene
// ===================================================================

bool SIFTPatternFinder::setupInputImage(char *fname)
{
  // Set the input image.
  IMGH::FileIO ifile;
  ifile.readFile( fname, &scene.iimg, IMGH::PIXEL_FLOAT );
  scene.xywh_given = false;
  scene.clearFeatures();
  match.clear();
  return true;
}

bool SIFTPatternFinder::setupInputImage(IMGH::Image *img)
{
  // Set the input image. (create a new copy)
  IMGH::ImageConverter conv;
  conv.convertImage( img, &scene.iimg, IMGH::PIXEL_FLOAT );
  scene.xywh_given = false;
  scene.clearFeatures();
  match.clear();
  return true;
}

bool SIFTPatternFinder::setupInputImageWithFloatBuffer(int w, int h, float *buf)
{
  // Set the input image. (using the original buffer; no copy)
  scene.iimg.setImage( w, h, IMGH::PIXEL_FLOAT, buf );
  scene.xywh_given = false;
  scene.clearFeatures();
  match.clear();
  return true;
}

bool SIFTPatternFinder::setupSIFTFeaturesWithBuffer(int n, SIFTFeature *fbuf)
{
  return scene.setupSIFTFeaturesWithBuffer( n, fbuf );
  return true;
}

// ===================================================================
// finding the pattern 
// ===================================================================

bool SIFTPatternFinder::findSIFTPattern(double Rco[9], double Tco[3])
{
	memset( timer_sec, 0, sizeof(timer_sec) );
	memset( this->Tco, 0, 3*sizeof(double));
	found = false;
	UTIL::Timer timer;
	timer.start();
	scene.verbose = sift_verbose;
	if (scene.fimg.h <= 0) {	// extract features
		scene.extractFeatures( sift_nOctaves, sift_nLevels, sift_sigma, sift_ethreshold, use_gpu );
	}
	timer_sec[0] = timer.checkTime();
	if (refer.fimg.h <= 0 || scene.fimg.h <= 0) return false;
	// match features from the reference and the scene, and find optimal affine transformation
	if (0) match.use_timer = true;
	found = match.findSIFTPattern( &refer, &scene );	// match features
	timer_sec[1] = timer.checkTime();
	if (!found) return false;
	// calculate the 3D pose
	if (Rco || Tco) found = getResult3DPose( Rco, Tco );	// estimate pose
	timer_sec[2] = timer.checkTime();
	timer.stop();
	if (0) {
		match.printInfo();
		printf("SIFTPatternFinder::findSIFTPattern timer:  Ext:%.4f  2D:%.4f  3D:%.4f\n", timer_sec[0], timer_sec[1], timer_sec[2]);
	}
	return found;
}

// ===================================================================
// inquiry of the results
// ===================================================================

bool SIFTPatternFinder::getResult2DPose(double corners[4][2], double center[2], double sa[2])
{
  return match.getResultPose( corners, center, sa );
}

bool SIFTPatternFinder::getResult3DPose(double Rco[9], double Tco[3])
{
  // Estimate the pose of the found pattern in 'scn', and save
  //   the result in 'Rco[9]' and 'Tco[3]'.
  double *R = this->Rco, *T = this->Tco;
  if (Tco) memset(Tco, 0, 3*sizeof(double));
  if (refer.fimg.h <= 0) return false;
  
  if (camera.cc[0] < scene.iimg.w*0.20 || camera.cc[0] > scene.iimg.w*0.80) {
    if (verbose) printf("[sift] Pose estimation FAILED .. invalid camera int. parameters (%d x %d)\n", camera.wh[0], camera.wh[1]);
    return false;
  }
  if (match.homography[8] != 1.0) {
    if (verbose) printf("[sift] Pose estimation FAILED .. invalid homography\n");
    return false;
  }
  // Estimate the homography again, using rectified coordinates
  double rhomography[9];
  estimateHomographyRectified( rhomography );
  
  // Convert the homography to new 3D pose  (scene->Rco[9] and scene->Tco[3]),
  //   using the pose in the reference image (refp->Rcr[9] and  refp->Tcr[3]).
  MTX::Matrix<double> H(3,3,rhomography), K, Kinv(3,3), RT(3,4), RTnew;
  MTX::MatrixSolver<double> solver;
  // set camera intrinsic matrix K and its inverse
  K.assign(3,3, camera.fc[0], 0.0, camera.cc[0], 
	   0.0, camera.fc[1], camera.cc[1],  0.0, 0.0, 1.0);
  solver.inverseByGaussJordan( 3, K.data, Kinv.data );
  RT.copySubMatrixFrom(0,0, 3,3, Rcf); 
  RT.copySubMatrixFrom(0,3, 3,1, Tcf); 
  
  // calculate the new pose of the pattern in the scene
  //   K * [Rco' Tco'] = s * H * K * [Rcf Tcf]
  //       [Rco' Tco'] = s * K^ * H * K * [Rcf Tcf]
  RTnew.mult( Kinv, H, K, RT );
  // normalize RTnew
  double len0 = G3V_LENGTH( RTnew.data+0 );
  double len1 = G3V_LENGTH( RTnew.data+4 );
  double len2 = G3V_LENGTH( RTnew.data+8 );
  RTnew.divValue( (len0 + len1 + len2) / 3.0 );
  memcpy( R+0, RTnew.data+0, 3*sizeof(double) );  T[0] = RTnew.data[3];
  memcpy( R+3, RTnew.data+4, 3*sizeof(double) );  T[1] = RTnew.data[7];
  memcpy( R+6, RTnew.data+8, 3*sizeof(double) );  T[2] = RTnew.data[11];
  // make the rotation matrix orthogonal
  // Note that above linear equation is ill-conditioned, and 
  //   the singular values S[3] are in the range of [ 0.3 ~ 2.0 ].
  double U[9], S[3], V[9];  // R = U * S * Vt
  solver.SVD( 3, 3, R, U, S, V );
  G3M_MUL_MMt( R, U, V );
  // Here, we have decided 12 unknowns using 8 DOF information. Consequently, 
  //   the returned pose is not good enough. We optimize it with nonlinear LM method.
  //   printf("[sift] initial rotation singular values: %.4f %.4f %.4f\n", S[0], S[1], S[2]);
  optimizePoseForHomography( R, T );
  if (Rco) G3M_COPY( Rco, this->Rco );
  if (Tco) G3V_COPY( Tco, this->Tco );
  
  // check the result ( invalid if it's closer than 10cm or farther than 10m )
  double dist = G3V_LENGTH( this->Tco );
  bool   ret = (dist > 0.10 && dist < 10.0 && this->Tco[2] > 0.1);
  if (verbose) printf("[sift] 3D Pose : %4s  Rco[9] Tco[3]=(%.2f %.2f %.2f)\n", 
		      (ret ? "good" : "BAD"), this->Tco[0], this->Tco[1], this->Tco[2]);
  return ret;
}

bool SIFTPatternFinder::getResultImage(IMGH::Image *rimg, bool show_features, bool show_matches, bool show_pose)
{
  if (refer.iimg.w > 0 && scene.iimg.w > 0) {	// both images -------
    int  wh[2], rr[2]={0,0}, ss[2]={0,0};
    if ((refer.iimg.w == refer.xywh[2] || refer.xywh[3]==0) &&
	refer.iimg.w >= scene.iimg.w  && refer.iimg.h >= scene.iimg.h) {
      bool v = (refer.iimg.w > refer.iimg.h*4/3);
      G2V_SET( wh, refer.iimg.w * (v ? 1:2), refer.iimg.h * (v ? 2:1) );
      G2V_SET( ss, (v ? 0 : refer.iimg.w), (v ? refer.iimg.h : 0) );
    } else {
      G2V_SET( wh, scene.iimg.w, scene.iimg.h );
      G2V_SET( rr, -refer.xywh[0], -refer.xywh[1] );
    }
    rimg->setImage( wh[0], wh[1], IMGH::PIXEL_RGB );
    scene.showScene( rimg, ss[0], ss[1] );
    if (show_features) scene.showFeatures( rimg, ss[0], ss[1] );
    refer.showScene( rimg, rr[0], rr[1], refer.xywh );
    if (show_features) refer.showFeatures( rimg, rr[0], rr[1] );
    if (show_matches)  match.showMatches ( rimg, rr[0], rr[1], ss[0], ss[1] );
    if (show_matches)  match.showTransformation( rimg, refer.xywh, ss[0], ss[1] );
    // visualize the pose of the reference image
    if (show_pose) visualizePatternPose( rimg, Rcf, Tcf, 0.12, rr[0], rr[1] );
    if (show_pose) visualizePatternPose( rimg, Rco, Tco, 0.12, ss[0], ss[1] );
    //if (show_pose) visualizeObjBoundary( rimg, Rco, Tco, 0.074, 0.14, ss[0], ss[1] );
  } else if (refer.iimg.w > 0) {	// reference only ------------
    // visualize the result of SIFT extraction (single image)
    rimg->setImage( refer.iimg.w, refer.iimg.h, IMGH::PIXEL_RGB );
    refer.showScene( rimg );
    if (show_features) refer.showFeatures( rimg );
    if (show_pose) visualizePatternPose( rimg, Rcf, Tcf, 0.12, 0, 0 );
    //if (show_pose) visualizeObjBoundary( rimg, Rcf, Tcf, 0.074, 0.14 );
  } else if (scene.iimg.w > 0) {	// scene only ----------------
    // visualize the result of SIFT extraction (single image)
    rimg->setImage( scene.iimg.w, scene.iimg.h, IMGH::PIXEL_RGB );
    scene.showScene( rimg );
    if (show_features) scene.showFeatures( rimg );
  }
  return true;
}

void SIFTPatternFinder::visualizePatternPose(IMGH::Image *img, double R[9], double T[3], 
					     double size, int uoff, int voff)
{
  // Draw the three axes of object frame on the image.
  // Note that this function should be called after 'getResult3DPose()'.
  if (!img || !R || !T || T[2]<=0) return;
  // calculate the origin and the end points of the axes in 3D
  double xv[3], yv[3], zv[3], xe[3], ye[3], ze[3];
  G3V_SET( xv, R[0], R[3], R[6] );  G3V_SCALED_ADD( xe, T, size, xv );
  G3V_SET( yv, R[1], R[4], R[7] );  G3V_SCALED_ADD( ye, T, size, yv );
  G3V_SET( zv, R[2], R[5], R[8] );  G3V_SCALED_ADD( ze, T, size, zv );
  double ouv[2], xuv[2], yuv[2], zuv[2];
  // calculate the origin and the end points of the axes in 2D
  camera.Camera2Pixel( T, ouv );
  camera.Camera2Pixel(  xe, xuv );
  camera.Camera2Pixel(  ye, yuv );
  camera.Camera2Pixel(  ze, zuv );
  // draw the axes on the image
  IMGH::Drawer<double> drw(img);
#if 0
  drw.setColor(0,255,0); drw.drawLine(ouv[0]+uoff, ouv[1]+voff, yuv[0]+uoff, yuv[1]+voff); // Y
  drw.setColor(0,0,255); drw.drawLine(ouv[0]+uoff, ouv[1]+voff, zuv[0]+uoff, zuv[1]+voff); // Z
  drw.setColor(255,0,0); drw.drawLine(ouv[0]+uoff, ouv[1]+voff, xuv[0]+uoff, xuv[1]+voff); // X
#else
  double xlen = G2V_DISTANCE( ouv, xuv ), xduv[2];
  double zlen = G2V_DISTANCE( ouv, zuv ), zduv[2];
  //double ylen = G2V_DISTANCE( ouv, yuv ), yduv[2];  G2V_SUB( yduv, yuv, ouv );  G2V_NORMALIZE( yduv );
  G2V_SUB( xduv, xuv, ouv );  G2V_NORMALIZED( xduv );  xduv[1] *= -1;
  G2V_SUB( zduv, zuv, ouv );  G2V_NORMALIZED( zduv );  zduv[1] *= -1;
  double xang = G2V_ROTATION_ANGLE( xduv );
  double zang = G2V_ROTATION_ANGLE( zduv );
  //double yang = G2V_ROTATION_ANGLE( xduv );
  drw.drawFilledArrow(ouv[0]+uoff, ouv[1]+voff, xlen, xang, 255, 0, 0 );
  drw.drawFilledArrow(ouv[0]+uoff, ouv[1]+voff, zlen, zang, 0, 255, 0 );
  //printf("X: %.1f %.1f  Y: %.1f %.1f\n", xlen, xang*180/M_PI, zlen, zang*180/M_PI);
#endif
}

void SIFTPatternFinder::visualizeObjBoundary(IMGH::Image *img, double R[9], double T[3],
					     double width, double height, double uoff, double voff)
{
  // Draw the boundary of the object frame on the image.
  // Note that this function should be called after 'getResult3DPose()'.
  if (!img || !R || !T) return;
  // calculate the origin and the end points of the axes in 3D
  double xv[3], yv[3], zv[3], op0[3], op1[3], op2[3], op3[3];
  G3V_SET( xv, R[0], R[3], R[6] );
  G3V_SET( yv, R[1], R[4], R[7] );
  G3V_SET( zv, R[2], R[5], R[8] );
  G3V_SCALED_ADD( op0, T, +width/2, xv );
  G3V_SCALED_ADD( op3, T, -width/2, xv );
  G3V_SCALED_ADD( op1, op0, +height,  zv );
  G3V_SCALED_ADD( op2, op3, +height,  zv );
  double ip[4][2];
  // calculate the origin and the end points of the axes in 2D
  camera.Camera2Pixel( op0, ip[0] );
  camera.Camera2Pixel( op1, ip[1] );
  camera.Camera2Pixel( op2, ip[2] );
  camera.Camera2Pixel( op3, ip[3] );
  for (int i=0; i<4; i++) G2V_SET( ip[i], ip[i][0]+uoff, ip[i][1]+voff );
  // draw the boundary on the image
  IMGH::Drawer<double> drw(img);
  drw.drawQuad( ip[0], ip[1], ip[2], ip[3], 255, 0, 0 );
}
 
// ===================================================================
// 
// ===================================================================

bool SIFTPatternFinder::estimateHomographyRectified(double homog[9])
{
  // bool SIFTpp::estimateHomography(SIFTpp *scene)
  // Note that is function is called from the 'reference', not the 'scene'.
  homog[8] = 0;
  MTH::Homography<double> hmg;
  MTX::Matrix<double> A(2, match.matched_count), B(2, match.matched_count);
  int  i, cnt;  double rxy[2], sxy[2];
  for (i=cnt=0; i < match.matched_count; i++) {
    SIFTFeature *rfp = match.matches[i].rfp;  if (!rfp) continue;
    SIFTFeature *sfp = match.matches[i].sfp;  if (!sfp) continue;
    G2V_COPY( rxy, rfp->data );  camera.undistortUV( rxy, rxy );
    G2V_COPY( sxy, sfp->data );  camera.undistortUV( sxy, sxy );
    A(0,i) = rxy[0];  B(0,i) = sxy[0];
    A(1,i) = rxy[1];  B(1,i) = sxy[1];
    cnt++;
  }
  
  bool ret = hmg.calculateHomography( A, B );
  
  // check the validity of homography
  if (ret) memcpy( homog, hmg.h, 9*sizeof(double) );
  else   { match.matched_count = -1; homog[8] = 0; }
  return ret;
}

struct stLM { double wh[2], ip[4][2]; Camera<double> *cam; };

static void lm_eval(double *x, double *me, int nx, int nm, void *adata)
{
  struct stLM *pp = (struct stLM*)adata;
  MTH::Rotation<double> rot;
  double R[9], *T=(x+3), xv[3], zv[3], wp[4][3], uv[4][2];
  rot.Rv2R( x, R );
  G3V_SET( xv, R[0], R[3], R[6] );
  G3V_SET( zv, R[2], R[5], R[8] );
  G3V_SCALED_ADD( wp[0], T, +pp->wh[0]/2, xv );
  G3V_SCALED_ADD( wp[3], T, -pp->wh[0]/2, xv );
  G3V_SCALED_ADD( wp[1], wp[0], +pp->wh[1], zv );
  G3V_SCALED_ADD( wp[2], wp[3], +pp->wh[1], zv );
  for (int i=0; i<4; i++) {
    pp->cam->Camera2Pixel( wp[i], uv[i] );
    me[i*2+0] = fabs( uv[i][0] - pp->ip[i][0] );
    me[i*2+1] = fabs( uv[i][1] - pp->ip[i][1] );
  }
//   printf("%.3f %.3f %.3f  %.3f %.3f %.3f (%.0f %.0f) (%.0f %.0f) (%.0f %.0f) (%.0f %.0f)\n", 
// 	 x[0], x[1], x[2], x[3], x[4], x[5], uv[0][0], uv[0][1], uv[1][0], uv[1][1], uv[2][0], uv[2][1], uv[3][0], uv[3][1]);
}

void SIFTPatternFinder::optimizePoseForHomography(double Rco[9], double Tco[3])
{
  struct stLM lmdata;
  lmdata.cam = &camera;
  // get the four corners of the homography in the scene
  double x=refer.xywh[0], y=refer.xywh[1], w=refer.xywh[2], h=refer.xywh[3];
  double rp[4][2] = { {x+w,y+h}, {x+w,y}, {x,y}, {x,y+h} };
  MTH::Homography<double> H(match.homography);
  for (int i=0; i<4; i++) H.transform( rp[i][0], rp[i][1], lmdata.ip[i] );
  // get the four corners of the reference pattern in 3D
  getReferencePatternSize( lmdata.wh );
//   printf("wh=(%.3f %.3f)  (%.0f %.0f)  (%.0f %.0f)  (%.0f %.0f)  (%.0f %.0f)\n",
// 	 lmdata.wh[0], lmdata.wh[1], lmdata.ip[0][0], lmdata.ip[0][1], lmdata.ip[1][0], lmdata.ip[1][1], lmdata.ip[2][0], lmdata.ip[2][1], lmdata.ip[3][0], lmdata.ip[3][1]);
  // prepare LM
  double pose[6], me[8]={0,0,0,0,0,0,0,0};
  MTH::Rotation<double> rot;
  rot.R2Rv( Rco, pose );
  G3V_COPY( pose+3, Tco );
  // LM optimize
  CMinpack minpack;
  minpack.LM2( lm_eval, pose, me, 6, 8, 100, NULL, &lmdata );
//   printf("%.3f %.3f %.3f  %.3f %.3f %.3f (%.0f %.0f) (%.0f %.0f) (%.0f %.0f) (%.0f %.0f)\n", 
// 	 pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], me[0], me[1], me[2], me[3], me[4], me[5], me[6], me[7]);
  // update with the result pose
  rot.Rv2R( pose, Rco );
  G3V_COPY( Tco, pose+3 );
}

void SIFTPatternFinder::getReferencePatternSize(double wh[2], double objp[4][3])
{
  // find the equation of the XZ plane of the object coordinate system
  double xv[3], zv[3], xzPlane[4], o[3]={0,0,0}, dir[4][3], dummy[4][3];
  if (!objp) objp = dummy;
  G3V_SET( xv, Rcf[0], Rcf[3], Rcf[6] );  G3V_ADD( xv, Tcf, xv );
  G3V_SET( zv, Rcf[2], Rcf[5], Rcf[8] );  G3V_ADD( zv, Tcf, zv );
  Geometry3D<double> geom;
  geom.getPlane( xzPlane, Tcf, xv, zv );
  double ip[4][2];
  G2V_SET( ip[0], refer.xywh[0]+refer.xywh[2], refer.xywh[1]+refer.xywh[3] );
  G2V_SET( ip[1], refer.xywh[0]+refer.xywh[2], refer.xywh[1]               );
  G2V_SET( ip[2], refer.xywh[0]              , refer.xywh[1]               );
  G2V_SET( ip[3], refer.xywh[0]              , refer.xywh[1]+refer.xywh[3] );
  for (int i=0; i<4; i++) {
    camera.Pixel2Camera( ip[i], dir[i] );
    geom.intersectPlaneAndLineDir( xzPlane, o, dir[i], objp[i] );
  }
  wh[0] = (G3V_DISTANCE(objp[0],objp[3]) + G3V_DISTANCE(objp[1],objp[2])) / 2;
  wh[1] = (G3V_DISTANCE(objp[0],objp[1]) + G3V_DISTANCE(objp[2],objp[3])) / 2;
}

// ===================================================================
//
// ===================================================================

void SIFTPatternFinder::printInfo(char *cmmt)
{
  printf("%s  (O=%d,L=%d,s=%.1f,e=%.2f,GPU=%s)\n", 
	 (cmmt ? cmmt : "SIFT Pattern Finder"), sift_nOctaves, sift_nLevels, 
	 sift_sigma, sift_ethreshold, (use_gpu ? "YES":"NO"));
  printf("  CamIntrinsic: (%d x %d) fc=(%6.2f %6.2f) cc=(%6.2f %6.2f)\n", camera.wh[0], camera.wh[1], camera.fc[0], camera.fc[1], camera.cc[0], camera.cc[1]);
  if (refer.xywh_given)
    printf("  Refer: %4d features from (%3d x %3d) image in xywh(%3d %3d %3d %3d) \n",
	   refer.fimg.h, refer.iimg.w, refer.iimg.h, refer.xywh[0], refer.xywh[1], refer.xywh[2], refer.xywh[3] );
  else
    printf("  Refer: %4d features from (%3d x %3d) image\n", refer.fimg.h, refer.iimg.w, refer.iimg.h );
  printf("  Scene: %4d features from (%3d x %3d) image\n", 
	 scene.fimg.h, scene.iimg.w, scene.iimg.h );
  if (scene.fimg.h>0) {
    double smin=0, smax=0, mmv[3];
    for (int i=0; i < scene.fimg.h; i++) {
      SIFTFeature *fp = (SIFTFeature*)scene.fimg.getRow(i);
      if (i==0 || fp->f.s < smin) smin = fp->f.s;
      if (i==0 || fp->f.s > smax) smax = fp->f.s;
    }
    match.analyzeMatching(mmv);
    printf("  Match: %4d features  Method=%s  d(%.4f %.4f %.4f)  PatternFound=%d  FeatureScale(%.2f ~ %.2f)\n", 
	   match.matched_count, (match.use_simple_match ? "simple":"geometric"), 
	   mmv[0], mmv[1], mmv[2], found, smin, smax);
    if (match.xform.R[0] != 0 || match.xform.R[2] != 0) {
      double scale = match.xform.getScale();
      double rotat = match.xform.getRotat() * 180 / M_PI;
      printf("  AffineXform [ %9.4f %9.4f   %9.4f ]  scale=%.2f\n", match.xform.R[0], match.xform.R[1], match.xform.T[0], scale);
      printf("     [A  b]   [ %9.4f %9.4f   %9.4f ]  rotat=%.0f\n", match.xform.R[2], match.xform.R[3], match.xform.T[1], rotat);
    }
  }
  if (timer_sec[0]>0 || timer_sec[1]>0) {
    printf("  Timer: extraction=%.4f matching=%.4f pose:%.4f\n", timer_sec[0], timer_sec[1], timer_sec[2]);
  }
}

