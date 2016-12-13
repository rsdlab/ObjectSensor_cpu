// -*- C++ -*-
/*!
 * @file  ObjectSensor.cpp
 * @brief ObjectSensor
 * @date $Date$
 *
 * $Id$
 */

#include "ObjectSensor.h"
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

cv::Mat imageC;
cv::Mat imageO;
cv::Mat imageOC;
cv::Mat Rmodel;
cv::Mat Tmodel;
std::vector<cv::KeyPoint>keypoint1;//検出されたキーポイントを記録
cv::Ptr<cv::FeatureDetector> detector;
cv::Ptr<cv::FeatureDetector> extractor;
cv::Mat descriptor1;

// Module specification
// <rtc-template block="module_spec">
static const char* objectsensor_spec[] =
  {
    "implementation_id", "ObjectSensor",
    "type_name",         "ObjectSensor",
    "description",       "ObjectSensor",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.CfgName", "cfg/demo.cfg",
    "conf.default.SIFT_Eehreshold", "10",
    "conf.default.SIFT_Sigma", "1.6",
    "conf.default.SIFT_nLevels", "0",
    "conf.default.SIFT_nOctaves", "-1",
    "conf.default.Display", "on",
    "conf.default.ObjName", "None",
    // Widget
    "conf.__widget__.CfgName", "text",
    "conf.__widget__.SIFT_Eehreshold", "text",
    "conf.__widget__.SIFT_Sigma", "text",
    "conf.__widget__.SIFT_nLevels", "text",
    "conf.__widget__.SIFT_nOctaves", "text",
    "conf.__widget__.Display", "text",
    "conf.__widget__.ObjName", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ObjectSensor::ObjectSensor(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_SingleImageIn("SingleImage", m_SingleImage),
    m_ObjectPoseOut("ObjectPose", m_ObjectPose),
    m_ResultImgOut("ResultImg", m_ResultImg),
    m_ModelPort("Model")

    // </rtc-template>
{
	
}

/*!
 * @brief destructor
 */
ObjectSensor::~ObjectSensor()
{
	//ここから
	// Registration: InPort/OutPort/Service
	// <rtc-template block="registration">
	// Set InPort buffers
	registerInPort("SingleImage", m_SingleImageIn);

	// Set OutPort buffer
	registerOutPort("ObjectPose", m_ObjectPoseOut);
	registerOutPort("ResultImage", m_ResultImgOut);

	// Set service provider to Ports
	m_ModelPort.registerProvider("ModelAcceptor", "AcceptModelService",
		m_ModelAcceptor);

	// Set CORBA Service Ports
	registerPort(m_ModelPort);

	//ここまで
}



RTC::ReturnCode_t ObjectSensor::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("SingleImage", m_SingleImageIn);
  
  // Set OutPort buffer
  addOutPort("ObjectPose", m_ObjectPoseOut);
  addOutPort("ResultImg", m_ResultImgOut);
  
  // Set service provider to Ports
  m_ModelPort.registerProvider("AcceptModelService", "AcceptModelService", m_ModelAcceptor);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_ModelPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("CfgName", m_CfgName, "cfg/demo.cfg");
  bindParameter("SIFT_Eehreshold", m_SIFT_Eehreshold, "10");
  bindParameter("SIFT_Sigma", m_SIFT_Sigma, "1.6");
  bindParameter("SIFT_nLevels", m_SIFT_nLevels, "0");
  bindParameter("SIFT_nOctaves", m_SIFT_nOctaves, "-1");
  bindParameter("Display", m_Display, "on");
  bindParameter("ObjName", m_ObjName, "None");
  // </rtc-template>
  
  //ここから

  _sift.verbose = true;

  m_ModelAcceptor.addEventListener(this);

  std::cout << "### Object Sensor Component ###" << std::endl;

  std::cout << "setting initial SIFT parameter" << std::endl;
  // set parameter
  _sift.use_gpu = true;

  // print initial information
  std::cout << "  -SIFT_nOctaves : " << _sift.sift_nOctaves << std::endl;
  std::cout << "  -SIFT_nLevels : " << _sift.sift_nLevels << std::endl;
  std::cout << "  -SIFT_Sigma : " << _sift.sift_sigma << std::endl;
  std::cout << "  -SIFT_Ethreshold : " << _sift.sift_ethreshold << std::endl;

  // allocate memory buffer for pose　姿勢を出力する配列の初期化
  m_ObjectPose.data.length(12);
  for (int i = 0; i < 12; ++i)
	  m_ObjectPose.data[i] = 0.0;

  // add listener to model acceptor port
  m_ModelAcceptor.addEventListener(this);
  m_ModelAcceptor.lockSetModel();

  // initialize log file
  ::UTIL::Timer time;
  _logname.resize(18);
  time.getDateTimeString(const_cast<char*> (_logname.c_str()));
  _logname += "_log.txt";
  FILE *fp;
  if ((fp = fopen(_logname.c_str(), "a+")) == 0) {
	  ;
  }
  else {
	  fprintf(
		  fp,
		  "object %s : transfer[sec] downloadform[sec] SIFT extruction[sec] Match sift pattern[sec] Pose estimation[sec] : Rotation[0] Rotation[1] Rotation[2] Rotation[3] Rotation[4] Rotation[5] Rotation[6] Rotation[7] Rotation[8] translation[0] translation[1] translation[2]\n",
		  m_ObjName.c_str());
	  fclose(fp);
  }

  //ここまで

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ObjectSensor::onFinalize()
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ObjectSensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObjectSensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ObjectSensor::onActivated(RTC::UniqueId ec_id)
{
	//ここから

	// while ( !m_SingleImageIn.isNew() ){
	//   coil::sleep(1);
	//   std::cout<< "Waiting for recieving Image buffer" << std::endl;
	// }

	// std::cout << "Image read finish " << std::endl;

	extractor = cv::xfeatures2d::SIFT::create(m_SIFT_nLevels, 3, 0.04, m_SIFT_Eehreshold, m_SIFT_Sigma);//SIFT
	detector = cv::xfeatures2d::SIFT::create(m_SIFT_nLevels, 3, 0.04, m_SIFT_Eehreshold, m_SIFT_Sigma);

	//extractor = cv::xfeatures2d::SIFT::create();//SIFT
	//detector = cv::xfeatures2d::SIFT::create();
	//extractor = cv::AKAZE::create();
	//detector = cv::AKAZE::create();

	// check connection of service port
	ALTH::UTIL::CheckPortConnection cpc;
	cpc.addConnectorPropertyName("port.AcceptModelService.ModelAcceptor");
	_modelC = cpc(m_ModelPort);

	if (_modelC > 0) { // Port is connected
		std::cout << "ServicePort:Model  is connected " << std::endl;
		m_ObjName = "None";

	}
	else { // Port is not connected
		std::cout << "ServicePort:Model  is not connected " << std::endl;
	}

	// load reference image and it's pose
	if (m_ObjName == "None") {
		std::cout << "Model is None, so model image was not loaded"
			<< std::endl;
		m_ModelAcceptor.unlockSetModel();
		_readModelFlag = true;
		return RTC::RTC_OK;
	}

	//参照モデルの読み込み
	std::cout << "loading reference pattern[" << m_ObjName << "] from file"
		<< std::endl;

	if (!_sift.loadReferencePattern(const_cast<char*> (m_CfgName.c_str()),
		const_cast<char*> (m_ObjName.c_str()))) {
		std::cout << "...failed : reference image doesn't exist" << std::endl;
		return RTC::RTC_ERROR;
	}
	else {
		std::cout << "...done" << std::endl;
	}
	_readModelFlag = false;

	m_ModelAcceptor.unlockSetModel();

	// print current setting information
	std::cout << "\nthe setting of sorce image format is as follows."
		<< "\n  -pixel type: " << getLabel((ALTH::pixel_t) _srcPixel_tL)
		<< "\n  -resolution: " << _grayL.w << "x" << _grayL.h
		<< "\nthe setting of camera parameter is as follows." << std::endl;
	_sift.camera.printInfo();

	std::cout << "Number of sift feature of reference image :  "
		<< _sift.refer.fimg.h << std::endl;



	//ここまで

	//モデル画像読み込み
	int widthO = _sift.refer.iimg.w;
	int heightO = _sift.refer.iimg.h;
	int channelsO = _sift.refer.iimg.getPixelSize();
	int remnant = 0;

	imageO.create(heightO, widthO, CV_32FC1);

	//std::cout << "width : " << widthO << std::endl << "height : " << heightO << std::endl << "channel : " << channelsO << std::endl;

	for (int i = 0; i < heightO; i++)
	{
		memcpy(&imageO.data[i*imageO.step], &_sift.refer.iimg.data[i*widthO*channelsO], sizeof(float)*widthO);
	}

	cv::Rect roi(_sift.refer.xywh[0], _sift.refer.xywh[1], _sift.refer.xywh[2], _sift.refer.xywh[3]);
	imageOC = imageO(roi);
	imageOC.convertTo(imageOC, CV_8U, 255);

	//モデル画像
	detector->detect(imageOC, keypoint1);//img1の特徴点を検出
	extractor->compute(imageOC, keypoint1, descriptor1);
	//cv::drawKeypoints(imageOC, keypoint1, imageOC);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ObjectSensor::onDeactivated(RTC::UniqueId ec_id)
{
	//ここから
	_iv.closeWindow();
	m_ModelAcceptor.lockSetModel();
	cv::destroyAllWindows();
	//ここまで

  return RTC::RTC_OK;
}


RTC::ReturnCode_t ObjectSensor::onExecute(RTC::UniqueId ec_id)
{
	//ここから

	// if object key is 'None', do nothing.
	if (m_ObjName == "None") {
		return RTC::RTC_OK;
	}

	// model port is not connected, find target object
	/*
	if (_modelC < 0) {
	std::cout << "model port is not conected. finding object in onExecute : " << findObject()
	<< std::endl;
	}
	*/

	// model port is connected and set new model
	//if( ( _modelC>0 && _ServiceFlag ) ){ return RTC::RTC_OK;}

	// // when model port is not connected / model port is called, findObj() function is used.
	// if(!( (_modelC<0) || (_modelC>0 && _ServiceFlag) )){ return RTC::RTC_OK;}

	// // set new model
	// // _ServiceFlag == true : invokeEvent is called.
	// // _readModelFlag == true : read model
	// if(_readModelFlag && _ServiceFlag){
	//   std::cout << "set new model [" << m_ObjName << "] " << std::ends;
	//   if(!_sift.loadReferencePattern(const_cast<char*>(m_CfgName.c_str()), const_cast<char*>(m_ObjName.c_str()))){
	//     std::cout << "... failed." << std::endl;
	//     m_ObjName = "None";
	//   }
	//   _readModelFlag = false; // model is loaded
	// }

	// set new model
	if (_readModelFlag) {
		std::cout << "set new model [" << m_ObjName << "] " << std::ends;
		if (!_sift.loadReferencePattern(const_cast<char*> (m_CfgName.c_str()),
			const_cast<char*> (m_ObjName.c_str()))) {
			std::cout << "... failed." << std::endl;
			m_ObjName = "None";
		}
		else
		{
			//モデル画像読み込み
			int widthO = _sift.refer.iimg.w;
			int heightO = _sift.refer.iimg.h;
			int channelsO = _sift.refer.iimg.getPixelSize();
			int remnant = 0;


			imageO.create(heightO, widthO, CV_32FC1);

			for (int i = 0; i < heightO; i++)
			{
				memcpy(&imageO.data[i*imageO.step], &_sift.refer.iimg.data[i*widthO*channelsO], sizeof(float)*widthO);
			}

			cv::Rect roi(_sift.refer.xywh[0], _sift.refer.xywh[1], _sift.refer.xywh[2], _sift.refer.xywh[3]);
			imageOC = imageO(roi);
			imageOC.convertTo(imageOC, CV_8U, 255);

			//モデル画像
			//std::vector<cv::KeyPoint>keypoint1;//検出されたキーポイントを記録
			detector->detect(imageOC, keypoint1);//img1の特徴点を検出
			extractor->compute(imageOC, keypoint1, descriptor1);

		}
		_readModelFlag = false; // model is loaded
	}
	// find target object
	else {
		_onExeFlag = true;
		std::cout << "finding object in onExecute : " << findObject()
			<< std::endl;
		_onExeFlag = false;
	}

	// // find target object
	// if( ( !_onExeFlag && !_ServiceFlag) ){
	//   _onExeFlag = true;
	//   std::cout << "finding object in onExecute : " << findObject() << std::endl;
	//   _onExeFlag = false;
	// }

	//ここまで

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ObjectSensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ObjectSensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ObjectSensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObjectSensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObjectSensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
//ここから

bool ObjectSensor::findObject() {
	// variable number declaration
	double time_log[6] = { 0.0 }; // time log data : read, conversion, extract SIFT, pose estimation
	//	int time(0);                           // time for log
	::UTIL::Timer timer; // timer for log
	static RTC::Time pTm; // previous time
	RTC::Time cTm; // current time
	bool found_2d(false), found_3d(false); // flag: success/failed of detection and pose estimation
	double Ho[9];
	double Rco[9] = { 0.0 }, Tco[3] = { 0.0 };
	cv::Mat imgRes;


	// start timer
	timer.start();

	//single image
	if (m_SingleImageIn.isNew()) {
		//最新のデータまで読み飛ばし
		while (!m_SingleImageIn.isEmpty())
			m_SingleImageIn.read();



		// camera parameters
		double f[2], c[2], a, d[5];

		// read image size
		_grayL.w = m_SingleImage.data.image.width;
		_grayL.h = m_SingleImage.data.image.height;
		if (m_SingleImage.data.image.format == CF_GRAY)
			_grayL.type = IMGH::PIXEL_GRAY, _srcPixel_tL = IMGH::PIXEL_GRAY;
		else if (m_SingleImage.data.image.format == CF_RGB)
			_grayL.type = IMGH::PIXEL_RGB, _srcPixel_tL = IMGH::PIXEL_RGB;
		else
			_grayL.type = IMGH::PIXEL_UNKNOWN, _srcPixel_tL
			= IMGH::PIXEL_UNKNOWN;

		//_cgrayL.setImage(_grayL.w, _grayL.h);

		// read camera paramters
		f[0] = m_SingleImage.data.intrinsic.matrix_element[0];
		f[1] = m_SingleImage.data.intrinsic.matrix_element[2];
		c[0] = m_SingleImage.data.intrinsic.matrix_element[3];
		c[1] = m_SingleImage.data.intrinsic.matrix_element[4];
		a = m_SingleImage.data.intrinsic.matrix_element[1];

		for (int index = 0; index < 5; index++)
			d[index] = m_SingleImage.data.intrinsic.distortion_coefficient[index];

		// std::cout << "camera parameters" << std::endl;
		// std::cout << "intrinsic : " ; for(int i=0; i<5; ++i){ std::cout << m_SingleImage.data.intrinsic.matrix_element[i] << " " ;} std::cout << std::endl;
		// std::cout << "distortion : " ;for(int i=0; i<5; ++i){ std::cout << d[i] << " " ;} std::cout << std::endl;

		// set camera parameters
		_sift.camera.setIntrinsic(_grayL.w, _grayL.h, f, a, c, d);
		_sift.camera.updatePmatUsingParam();

		// set result image size
		//m_ResultImg.data.image.width = _grayL.w;
		//m_ResultImg.data.image.height = _grayL.h;
		//m_ResultImg.data.image.format = CF_RGB;
		// allocate buffer for result image
		//m_ResultImg.data.image.raw_data.length((_grayL.w + _grayR.w)* _grayL.h * 3);

		//std::cout << "single image is read" << std::endl;

		_grayL.type = _srcPixel_tL;

		// check trans port time
		cTm = getCurrentTimeStamp();
		RTC::Time eTm = cTm - m_SingleImage.tm;
		time_log[0] = toDouble(eTm);

		// check time of reading data
		timer.checkTime();

		// check elapsed time
		eTm = cTm - pTm;

		// set previous time
		pTm = cTm;
		//if(toDouble(eTm) > 1.0){
		// time is out
		//	std::cout << "current image is time out." << std::endl;
		//	return  false;
		//}

		// convert images format to float gray image
		if (_grayL.type != IMGH::PIXEL_FLOAT) {
			//std::cout << "converting image type" << std::ends;
			//IMGH::ImageConverter conv;
			//IMGH::Image imgL(_grayL.w, _grayL.h, _srcPixel_tL,(void*)&(m_SingleImage.data.image.raw_data[0]));
			//conv.convertImage(&imgL, &_grayL, IMGH::PIXEL_FLOAT);//グレースケールに変えてる？
			//std::cout << " ... done" << std::endl;
		}

		int width = m_SingleImage.data.image.width;
		int height = m_SingleImage.data.image.height;
		int channels = (m_SingleImage.data.image.format == CF_GRAY) ? 1 :
			(m_SingleImage.data.image.format == CF_RGB || m_SingleImage.data.image.format == CF_PNG || m_SingleImage.data.image.format == CF_JPEG) ? 3 :
			(m_SingleImage.data.image.raw_data.length() / width / height);
		if (channels == 3)
			imageC.create(height, width, CV_8UC3);
		else
			imageC.create(height, width, CV_8UC1);


		long data_length = m_SingleImage.data.image.raw_data.length();
		long image_size = width*height*channels;

		if (m_SingleImage.data.image.format == Img::CF_RGB)
		{
			for (int n = 0; n < height; n++)
			{

				memcpy(&imageC.data[n*imageC.step], &m_SingleImage.data.image.raw_data[n*width*channels], sizeof(unsigned char)*width*channels);

				//std::cout << "RGB" << std::endl;
			}
			if (channels == 3)
				cv::cvtColor(imageC, imageC, CV_BGR2RGB);
		}
		else if (m_SingleImage.data.image.format == Img::CF_JPEG || m_SingleImage.data.image.format == Img::CF_PNG)
		{
			std::vector<uchar> compressed_image = std::vector<uchar>(data_length);
			memcpy(&compressed_image[0], &m_SingleImage.data.image.raw_data[0], sizeof(unsigned char) * data_length);

			//Decode received compressed image
			cv::Mat decoded_image;
			if (channels == 3)
			{
				decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_COLOR);
				cv::cvtColor(decoded_image, imageC, CV_RGB2BGR);

			}
			else
			{
				decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_GRAYSCALE);
				imageC = decoded_image;
			}
		}


		// check time of conversion
		time_log[1] = timer.checkTime();
		time_log[2] = timer.checkTime();
		_sift.setupInputImage(&_grayL);//Siftに入力画像をセット？(カメラからの画像)

		// check time of SIFT extruction
		time_log[3] = timer.checkTime();

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//カメラ画像
		std::vector<cv::KeyPoint>keypoint2;
		detector->detect(imageC, keypoint2);//img2の特徴点を検出
		cv::Mat descriptor2;
		extractor->compute(imageC, keypoint2, descriptor2);
		//cv::drawKeypoints(imageC, keypoint2, imageC);

		cv::Ptr<cv::DescriptorMatcher> matcher;
		matcher = cv::DescriptorMatcher::create("BruteForce");
		std::vector<cv::DMatch> matches, matches12, matches21;
		matcher->match(descriptor1, descriptor2, matches12);
		matcher->match(descriptor2, descriptor1, matches21);

		for (size_t k = 0; k < matches12.size(); k++)
		{
			cv::DMatch forward = matches12[k];
			cv::DMatch backward = matches21[forward.trainIdx];
			if (backward.trainIdx == forward.queryIdx)
				matches.push_back(forward);
		}

		//最小距離//今回は必要ない？
		double min_dist = DBL_MAX;//
		for (int l = 0; l < (int)matches.size(); l++)
		{
			double dist = matches[l].distance;
			if (dist < min_dist)
			{
				min_dist = dist;
			}

		}
		//良いペアのみ残す
		std::vector<cv::DMatch>good_matches;
		for (int k = 0; k < (int)matches.size(); k++)
		{
			if (matches[k].distance < 2.0/*どこからきた？*/*min_dist)
			{
				good_matches.push_back(matches[k]);
			}
		}

		time_log[4] = timer.checkTime();

		//cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//十分な対応点がある
		if (good_matches.size() > 20)
		{
			std::vector<cv::Point2f>obj, scene, obj_h, scene_h;
			for (int j = 0; j < (int)good_matches.size(); j++)
			{
				obj.push_back(keypoint1[good_matches[j].queryIdx].pt);
				scene.push_back(keypoint2[good_matches[j].trainIdx].pt);
				//ホモグラフィ推定用

				obj_h.push_back(keypoint1[good_matches[j].queryIdx].pt + cv::Point2f(_sift.refer.xywh[0], _sift.refer.xywh[1]));
				scene_h.push_back(keypoint2[good_matches[j].trainIdx].pt);

			}


			//ホモグラフィー行列の導出、ランザック処理してる？
			cv::Mat masks;
			H = cv::findHomography(obj_h, scene, masks, cv::RANSAC, 1);
			H_v = cv::findHomography(obj, scene, masks, cv::RANSAC, 1);

			std::vector<cv::DMatch> inlinerMatch;
			for (size_t k = 0; k < masks.rows; ++k) {
				uchar *inliner = masks.ptr<uchar>(k);
				if (inliner[0] == 1) {
					inlinerMatch.push_back(good_matches[k]);
				}
			}



			//cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			//cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes);


			//行列が空ではない場合イフ文内に入る
			if (!H.empty())
			{
				cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 2);

				std::vector<cv::Point2d>obj_corners(4), scene_corners(4);
				//オブジェクトの大きさを指定
				obj_corners[0] = scene_corners[0] = cv::Point2d(0, 0);
				obj_corners[1] = scene_corners[1] = cv::Point2d(imageOC.cols, 0);
				obj_corners[2] = scene_corners[2] = cv::Point2d(imageOC.cols, imageOC.rows);
				obj_corners[3] = scene_corners[3] = cv::Point2d(0, imageOC.rows);

				//ホモグラフィ行列の推定
				cv::perspectiveTransform(obj_corners, scene_corners, H_v);


				std::cout << "線で囲む" << std::endl;
				//緑の線で囲む（開始点を元画像が左にあるので右にオフセット）
				cv::line(imgRes, scene_corners[0] + cv::Point2d(imageOC.cols, 0), scene_corners[1] + cv::Point2d(imageOC.cols, 0), cv::Scalar(0, 255, 0), 4);
				cv::line(imgRes, scene_corners[1] + cv::Point2d(imageOC.cols, 0), scene_corners[2] + cv::Point2d(imageOC.cols, 0), cv::Scalar(0, 255, 0), 4);
				cv::line(imgRes, scene_corners[2] + cv::Point2d(imageOC.cols, 0), scene_corners[3] + cv::Point2d(imageOC.cols, 0), cv::Scalar(0, 255, 0), 4);
				cv::line(imgRes, scene_corners[3] + cv::Point2d(imageOC.cols, 0), scene_corners[0] + cv::Point2d(imageOC.cols, 0), cv::Scalar(0, 255, 0), 4);


				//ホモグラフィ行列を配列に変換
				for (int y = 0; y < 3; y++)
				{
					for (int x = 0; x < 3; x++)
					{
						Ho[y * 3 + x] = H.at<double>(y, x);
					}
				}

				//Ho[8]が桁落ちして、１でなくなっているのを補正
				if (0.99999<Ho[8] && Ho[8]<1.000001)
				{
					Ho[8] = (double)1.0;
				}



				//SIFT特徴量の対応関係をチェック
				std::cout << "start to match sift feature" << std::endl;
				//found_2d = _sift.findSIFTPattern( Rco, Tco);
				found_2d = _sift.getResult3DPosewithH(Ho, Rco, Tco);
				//found_2d=_sift.getResult3DPose(Rco, Tco);
				std::cout << "OpenCV H" << std::endl;
				std::cout << H << std::endl;
				time_log[4] = timer.checkTime();
				std::cout << "   SIFT対応チェック:   " << time_log[4] << std::endl;
				/*
				if (found_2d) {
				//３次元姿勢の推定
				std::cout << "start to get Result 3D pose" << std::endl;
				found_3d = _sift.getResult3DPose(Rco, Tco);
				}
				*/

				// check time of pose estimation
				time_log[5] = timer.checkTime();
				std::cout << "三次元姿勢の推定:   " << time_log[5] << std::endl;



				/*
				RTcamera.create(3, 4, CV_64F);

				std::fstream ofs("test.log", std::ios::out | std::ios::app | std::ios::ate);
				if (!ofs.is_open())
				{
				std::cout << "ファイルが開けません" << std::endl;
				return -1;
				}
				*/

			}
			else
			{
				std::cout << "ホモグラフィ行列が空です" << std::endl;
				cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 2);
			}
		}
		else
		{
			std::cout << "マッチング点が少なすぎます" << std::endl;
			cv::drawMatches(imageOC, keypoint1, imageC, keypoint2, good_matches, imgRes, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), 2);
		}




		////////////////////////////////////////////////////////////////////////テストで挿入




		// find the reference pattern using 'B'oth ('L'eft and 'R'ight) pattern finders




		// display result
		if (m_Display == "on") {
			// set result image size
			m_ResultImg.data.image.width = imgRes.cols;
			m_ResultImg.data.image.height = imgRes.rows;
			m_ResultImg.data.image.format = CF_RGB;
			m_ResultImg.data.image.raw_data.length(imgRes.cols*imgRes.rows * 3);
			// allocate buffer for result image

			cv::Mat imgResBGR;
			imgRes.copyTo(imgResBGR);
			cv::cvtColor(imgResBGR, imgResBGR, CV_BGR2RGB);
			for (int j = 0; j < height; j++)
			{
				memcpy(&m_ResultImg.data.image.raw_data[j*imgResBGR.cols * channels], &imgResBGR.data[j*imgResBGR.step], imgResBGR.cols*channels);
			}
			m_ResultImg.tm = m_SingleImage.tm;
			m_ResultImgOut.write();
			cv::namedWindow("Result_Image", CV_WINDOW_AUTOSIZE);
			cv::imshow("Result_Image", imgRes);
			cv::waitKey(1);

		}
		else if (m_Display == "off") {
			//_iv.closeWindow();
			cv::destroyWindow("Result_Image");
		}
		std::cout << "Number of SIFT Feature Matching : "
			<< _sift.match.matched_count << std::endl;
		// push estimated pose from OutPort
		if (found_2d) {
			// push estimated pose
			memcpy(&(m_ObjectPose.data[0]), &Rco[0], 3 * sizeof(double));
			memcpy(&(m_ObjectPose.data[4]), &Rco[3], 3 * sizeof(double));
			memcpy(&(m_ObjectPose.data[8]), &Rco[6], 3 * sizeof(double));
			for (int i = 0; i < 3; i++) {
				m_ObjectPose.data[3 + 4 * i] = Tco[i];
			}
			//m_ObjectPose.data[12] = (double)found_2d;
			//m_ObjectPose.data[13] = (double)found_3d;
			m_ObjectPose.tm = m_SingleImage.tm;
			std::cout << m_ObjName << " is found." << std::endl;
			std::cout << "time stamp : " << m_ObjectPose.tm.sec << "[sec] "
				<< m_ObjectPose.tm.nsec << "[nsec]" << std::endl;
			std::cout << "pose is as follows." << std::endl;
			G3M_XFORM_PRINTF(Rco, Tco);

			if (!m_ObjectPoseOut.write()) {
				std::cout << "error: pose is not sent!" << std::endl;
			}

		}
		else {
			std::cout << m_ObjName << " is not found." << std::endl;
		}

	} //end of "if(m_SingleImageIn.isNew())": new image is pushed?

	timer.stop();

	// write time's log
	FILE *fp;
	if ((fp = fopen(_logname.c_str(), "a+")) == 0) {
		;
	}
	else{
		if (found_2d) {
			fprintf(fp, "%s : ", m_ObjName.c_str());
			for (int i = 0; i < 6; ++i)
				fprintf(fp, "%.6f ", time_log[i]);
			fprintf(fp, ": ");
			for (int i = 0; i < 12; ++i)
				fprintf(fp, "%.6lf ", m_ObjectPose.data[i]);
			fprintf(fp, "\n");
			fclose(fp);

			return true;
		}
		fclose(fp);
	}


	return false;
}

// when "m_ModelAcceptor->setModel()" is called by other components,
// this function is executed.
void ObjectSensor::invokeEvent(const std::string &ev) {
	std::cout << "invokeEvent is called." << std::endl;

	// if object ID is current ID, the object model is not updated.
	if (ev == m_ObjName) {
		std::cout << "Current model ID is same, so model was not changed"
			<< std::endl;
		return;
	}

	// set model ID
	m_ObjName = ev;

	// if object ID is "None", the object model is not updated.
	if (ev == "None") {
		std::cout << "Set model ID to None" << std::endl;
		return;
	}

	// _ServiceFlag = true;

	std::cout << "Waiting for finishing to execute function : findObject()"
		<< std::endl;
	while (_onExeFlag) {
		coil::usleep(1000);
	}
	_readModelFlag = true;

	std::cout << "Waiting for finishing to read model image" << std::endl;
	while (_readModelFlag) {
		coil::usleep(1000);
	}
	std::cout << "Change model to " << ev << std::endl;

	//_ServiceFlag = false;

	// while(1){
	//   if(!_onExeFlag){ // findObj() function is not
	//     std::cout << "findObj() is not used in onExecute." << std::endl;
	//     if(m_ObjName != ev){
	// 	m_ObjName = ev;
	// 	_readModelFlag = true; // onExecute function can read model.

	// 	while(1){
	// 	  if(!_readModelFlag){ // _readModelFlag :false.//model is loaded.
	// 	    break;
	// 	  }
	// 	}

	//     }
	//     _onExeFlag=true;    //
	//     break;
	//   }else{ // findObj() is used in onExecute
	//     std::cout << "findObj() is used in onExecute." << std::endl;
	//     if(m_ObjName == "None"){
	// 	m_ObjName = ev;
	// 	_readModelFlag = true;

	// 	while(1){
	// 	  if(!_readModelFlag){
	// 	    break;
	// 	  }
	// 	}
	// 	_onExeFlag = true;   // unlock findObj()
	// 	break;
	//     }
	//   }
	//   coil::usleep(100);
	// }

	// while(1){
	//   if(!_onExeFlag){
	//     _ServiceFlag=false;
	//     break;
	//   }
	//   coil::usleep(100);
	// }
}

//ここまで


extern "C"
{
 
  void ObjectSensorInit(RTC::Manager* manager)
  {
    coil::Properties profile(objectsensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<ObjectSensor>,
                             RTC::Delete<ObjectSensor>);
  }
  
};


