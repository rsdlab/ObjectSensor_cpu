#ifndef _CONNECTION_CHECK_UTIL_FORRTC_H_
#define _CONNECTION_CHECK_UTIL_FORRTC_H_

//
// ConnectionCheckUtil_forRTC.h
//
// @brief this header file provides some utility.
//
// @author H.Takahashi, Osaka Univ.
//
// @last modefied in Oct., 2009
//


#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <list>
#include <string>
#include <cmath>

namespace ALTH{
	namespace UTIL{

		//
		// CheckPortConnection class
		// @breif this class can check of port connection
		//
		class CheckPortConnection{
		private:
			std::list<std::string> _listCPName; //list of connector profile's name
		public:
			bool verbose; // flag of comment
		public:
			// constructor
			CheckPortConnection();

			// destructor
			~CheckPortConnection();

			// add connector property's name to list
			// @pram name: connector property's name
			// Note: connector property name is as follows.
			//       "port.XXX.YYY"
			//       XXX : interface name
			//       YYY : instance name
			void addConnectorPropertyName(std::string name);

			// delete connector porperty's name from list
			// @pram name: connector property's name
			void deleteConnectorPropertyName(std::string name);

			// clear all connector porperty's name from list
			void clearConnectorPropertyName();

			// operator() over load
			// this operator checks connection of indicated port.
			// @param port: port
			// @return result of connection check
			//         if port has not been connected with other port, the numerical value of less than 0 is returned.
			//         if port has been connected with other port, the numerical value will be returned.
			//         bit of the value is correspond with sequential number of connector property's name as follows.
			//         the first connector property's name is connected  : 0000.....0001
			//         the second connector property's name is connected : 0000.....0010
			//         the both connector property's name is connected   : 0000.....0011
			int operator()(RTC::PortBase &port);
		}; //class CheckPortConnection

	}; //namespace UTIL
}; //namespace ALTH

#endif //_UTILITY_FORRTC_H_
