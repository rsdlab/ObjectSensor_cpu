// -*- C++ -*-
/*!
 * @file  ObjectSensor.h
 * @brief ObjectSensor
 * @date  $Date$
 *
 * $Id$
 */

#ifndef OBJECTSENSOR_H
#define OBJECTSENSOR_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include<Img.hh>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AcceptModelSVC_impl.h"

//ここから

// User's functions headers
// for vision
#include <SIFTPatternFinder.hpp>
#include "imgh_common.hpp"
#include "imgh_converter.hpp"
#include "visionlib/cuda/cuda_image.h"
#include "visionlib/cuda/cuda_sift.h"
// for util
#include "MultiCastEventListener.h"
#include "ImageViewer.h"
#include "TimeStampUtil_forRTC.h"
#include "ConnectionCheckUtil_forRTC.h"

#include "idl/ImgStub.h"

using namespace ALTH;
using namespace ALTH::UTIL;
//ここまで
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
using namespace Img;
// </rtc-template>

using namespace RTC;

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

/*!
 * @class ObjectSensor
 * @brief ObjectSensor
 *
 */
class ObjectSensor
	: public RTC::DataFlowComponentBase,
	public EventListener<std::string>
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  ObjectSensor(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~ObjectSensor();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 参照画像設定ファイル名及びパスを指定する
   * - Name:  CfgName
   * - DefaultValue: cfg/sample.cfg
   */
  std::string m_CfgName;
  /*!
   * カメラ画像に対するSIFT特徴量の設定項目
   * - Name:  SIFT_Eehreshold
   * - DefaultValue: 10
   */
  double m_SIFT_Eehreshold;
  /*!
   * カメラ画像に対するSIFT特徴量の設定項目
   * - Name:  SIFT_Sigma
   * - DefaultValue: 0.5
   */
  double m_SIFT_Sigma;
  /*!
   * カメラ画像に対するSIFT特徴量の設定項目
   * - Name:  SIFT_nLevels
   * - DefaultValue: 3
   */
  int m_SIFT_nLevels;
  /*!
   * カメラ画像に対するSIFT特徴量の設定項目
   * - Name:  SIFT_nOctaves
   * - DefaultValue: -1
   */
  int m_SIFT_nOctaves;
  /*!
   * 物体の検出及び位置推定結果を描画した画像の表示・非表示を設定す
   * る
   * "on":表示
   * "off":非表示
   * - Name:  Display
   * - DefaultValue: on
   */
  std::string m_Display;
  /*!
   * 物体名を記述することで，使用する参照画像を指定する
   * - Name:  ObjName
   * - DefaultValue: None
   */
  std::string m_ObjName;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  Img::TimedCameraImage m_SingleImage;
  /*!
   * カメラでキャプチャしたRGB画像を入力する.
   */
  InPort<Img::TimedCameraImage> m_SingleImageIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_ObjectPose;
  /*!
   * カメラ座標系における物体の位置・姿勢を同次変換行列により出力す
   * る.
   */
  OutPort<RTC::TimedDoubleSeq> m_ObjectPoseOut;
  Img::TimedCameraImage m_ResultImg;
  /*!
   * 検出および位置姿勢推定結果を描画したRGB画像を出力する.
   */
  OutPort<Img::TimedCameraImage> m_ResultImgOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_ModelPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  AcceptModelServiceSVC_impl m_ModelAcceptor;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">

	 //ここから
	 SIFTPatternFinder _sift;  // sift pattern finder
	 IMGH::Image _grayL, _grayR;	  // float images for CPU
	 IMGH::pixel_t _srcPixel_tL = (IMGH::pixel_t)PIXEL_UNKNOWN;
	 IMGH::pixel_t _srcPixel_tR = (IMGH::pixel_t)PIXEL_UNKNOWN;   // pixel type of src image

	 ImageViewer _iv; // image viewer
	 //ALTH::UTIL::Mutex _model_sync;


	 // log name
	 std::string _logname;

	 // read model 
	 bool _onExeFlag;
	 bool _ServiceFlag;
	 bool _readModelFlag;

	 // connection check
	 int _modelC;

	 // main function for object recognition
	 bool findObject();

	 //カメラパラメータ
	 cv::Mat CCM;

	 //モデル座標変換行列
	 cv::Mat RTmodel = cv::Mat(3, 4, CV_64F);

	 //対象の座標変換行列
	 cv::Mat RTcamera;

	 //ホモグラフィ行列
	 cv::Mat H;



 public:
	 // when "m_ModelAcceptor->setModel()" is called by other components,
	 // this function is executed.
	 void invokeEvent(const std::string &ev);

	 //ここまで
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void ObjectSensorInit(RTC::Manager* manager);
};

#endif // OBJECTSENSOR_H
