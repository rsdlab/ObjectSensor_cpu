// -*-C++-*-
/*!
 * @file  AcceptModelSVC_impl.cpp
 * @brief Service implementation code of AcceptModel.idl
 *
 */
#include<iostream>
#include "AcceptModelSVC_impl.h"

/*
 * Example implementational code for IDL interface AcceptModelService
 */
AcceptModelServiceSVC_impl::AcceptModelServiceSVC_impl()
{
  // Please add extra constructor code here.
}


AcceptModelServiceSVC_impl::~AcceptModelServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void AcceptModelServiceSVC_impl::setModel(const AcceptModelService::OctetSeq& model)
{
  // Please insert your code here and remove the following warning pragma
	while(1){
		
		if(!_setModelFlag){
			break;
}
  }
	
	std::string cmodel((char*)&model[0], model.length());
	std::cout << "AcceptModelServiceSVC_impl::setModel() is called.:" << cmodel << std::endl;

	notifyEvent(cmodel);
#ifndef WIN32
  #warning "Code missing in function <void AcceptModelServiceSVC_impl::setModel(const AcceptModelService::OctetSeq& model)>"
#endif
}



// End of example implementational code



