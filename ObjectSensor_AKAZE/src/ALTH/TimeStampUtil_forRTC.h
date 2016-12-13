#ifndef _TIME_STAMP_UTIL_H_
#define _TIME_STAMP_UTIL_H_

//
// TimeStampUtil.h
//
// @brief This header file defines "time stamp" function.
//
// @author H.Takahashi, Osaka Univ.
// @last modified in Sep., 2009
//

#include "rtm/idl/BasicDataTypeStub.h"

namespace ALTH{
	namespace UTIL{
		// get time stamp
		RTC::Time getCurrentTimeStamp();

		// translate time stamp into double
		double toDouble(RTC::Time tm);
	}; //namespace UTIL
}; //namespace ALTH

// operator overload
::RTC::Time operator+(::RTC::Time& _tm, const ::RTC::Time& tm);
::RTC::Time operator-(::RTC::Time& _tm, const ::RTC::Time& tm);

#endif //_TIME_STAMP_UTIL_H_
