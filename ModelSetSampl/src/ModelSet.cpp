// -*- C++ -*-
/*!
 * @file  ModelSet.cpp
 * @brief ModelSet
 * @date $Date$
 *
 * $Id$
 */

#include "ModelSet.h"

// Module specification
// <rtc-template block="module_spec">
static const char* modelset_spec[] =
  {
    "implementation_id", "ModelSet",
    "type_name",         "ModelSet",
    "description",       "ModelSet",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ModelSet::ModelSet(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_ModelPortPort("ModelPort")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ModelSet::~ModelSet()
{
}



RTC::ReturnCode_t ModelSet::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_ModelPortPort.registerConsumer("AcceptModelService", "AcceptModelService", m_AcceptModelService);
  
  // Set CORBA Service Ports
  addPort(m_ModelPortPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ModelSet::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ModelSet::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ModelSet::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t ModelSet::onExecute(RTC::UniqueId ec_id)
{
	std::cout << "モデル名を入力してください"<< std::endl;
	std::string model_name;
	std::cin >> model_name;
	std::cout << model_name << "をセットします" << std::endl;
	/*
	std::string model_data;
	for (int i = 0; i < model_name.length(); i++)
	{

		model_data.push_back(model_name[i]);

	}


	AcceptModelService::OctetSeq model_id;
	std::cout <<"モデル名の長さを設定"<< std::endl;
	model_id.length(model_data.length());


	std::cout << "メモリコピー" << std::endl;
	memcpy(&model_id[0], model_data.c_str(), sizeof(char)*model_data.length());
	std::cout << "ポートに出力中" << std::endl;
	m_AcceptModelService->setModel(model_id);
	std::cout << "出力完了"<< std::endl;
	*/
	AcceptModelService::OctetSeq model_id;
	std::cout << "モデル名の長さを設定" << std::endl;
	model_id.length(model_name.length());


	std::cout << "メモリコピー" << std::endl;
	memcpy(&model_id[0], model_name.c_str(), sizeof(char)*model_name.length());
	std::cout << "ポートに出力中" << std::endl;
	m_AcceptModelService->setModel(model_id);
	std::cout << "出力完了" << std::endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ModelSet::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ModelSet::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ModelSetInit(RTC::Manager* manager)
  {
    coil::Properties profile(modelset_spec);
    manager->registerFactory(profile,
                             RTC::Create<ModelSet>,
                             RTC::Delete<ModelSet>);
  }
  
};


