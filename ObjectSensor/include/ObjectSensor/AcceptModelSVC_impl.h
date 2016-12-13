// -*-C++-*-
/*!
 * @file  AcceptModelSVC_impl.h
 * @brief Service implementation header of AcceptModel.idl
 *
 */

#include "AcceptModelSkel.h"

#ifndef ACCEPTMODELSVC_IMPL_H
#define ACCEPTMODELSVC_IMPL_H
 

#include<string>
#include<iostream>
#include"MultiCastEventListener.h"
using namespace ALTH;



/*!
 * @class AcceptModelServiceSVC_impl
 * Example class implementing IDL interface AcceptModelService
 */
class AcceptModelServiceSVC_impl
 : public virtual POA_AcceptModelService,
 public virtual PortableServer::RefCountServantBase,
 public EventSource<std::string>
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~AcceptModelServiceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   AcceptModelServiceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~AcceptModelServiceSVC_impl();

   // attributes and operations
   void setModel(const AcceptModelService::OctetSeq& model);
   //‚±‚±‚©‚ç

private:
	bool _setModelFlag;
public:
	inline void lockSetModel(){ _setModelFlag = true; }
	inline void unlockSetModel(){ _setModelFlag = false; }


	//‚±‚±‚Ü‚Å

};



#endif // ACCEPTMODELSVC_IMPL_H


