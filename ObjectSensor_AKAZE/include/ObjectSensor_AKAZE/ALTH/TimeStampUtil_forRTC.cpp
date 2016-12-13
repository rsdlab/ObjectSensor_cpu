#include "TimeStampUtil_forRTC.h"

#include <rtm/RTC.h> // include to check version of RTM


// Version of RTM
#ifndef COIL_PROPERTIES_H
// 0.4.x ver
#include <ace/Time_Value.h>
#include <ace/OS_NS_sys_time.h>
#include <rtm/TimeValue.h>

namespace ALTH{
	namespace UTIL{
		// get time stamp
		RTC::Time getCurrentTimeStamp(){
			ACE_Time_Value time_val = ACE_OS::gettimeofday();

			RTC::Time retTime;
			retTime.sec = static_cast<CORBA::ULong>(time_val.sec());
			retTime.nsec = static_cast<CORBA::ULong>(time_val.usec()*1000);
			return retTime;
		}

		// translate time stamp into double
		double toDouble(RTC::Time tm){
			TimeValue tv(tm.sec, tm.nsec/1000);
			return tv.toDouble();
		}

	}; //namespace UTIL
}; // namespace ALTH


// operator overload
::RTC::Time operator+(::RTC::Time& _tm, const ::RTC::Time& tm){
	TimeValue _tv(_tm.sec, _tm.nsec/1000);
	TimeValue tv(tm.sec, tm.nsec/1000);
	TimeValue dtm = _tv + tv;
	::RTC::Time dst;
	dst.sec = dtm.tv_sec; dst.nsec = dtm.tv_usec*1000;
	return dst;
}

::RTC::Time operator-(::RTC::Time& _tm, const ::RTC::Time& tm){
	TimeValue _tv(_tm.sec, _tm.nsec/1000);
	TimeValue tv(tm.sec, tm.nsec/1000);
	TimeValue dtm = _tv - tv;
	::RTC::Time dst;
	dst.sec = dtm.tv_sec; dst.nsec = dtm.tv_usec*1000;
	return dst;
}


#else
// 1.0.x ver
#include <coil/Time.h>
#include <coil/TimeValue.h>
using namespace coil;

namespace ALTH{
	namespace UTIL{
		// get time stamp
		RTC::Time getCurrentTimeStamp(){
			coil::TimeValue tm(coil::gettimeofday());
			RTC::Time retTime;
			retTime.sec = static_cast<CORBA::ULong>(tm.sec());
			retTime.nsec = static_cast<CORBA::ULong>(tm.usec()*1000);

			return retTime;
		}

		// translate time stamp into double
		double toDouble(RTC::Time tm){
			TimeValue tv(tm.sec, tm.nsec/1000);
			return (double)tv;
		}

	}; //namespace UTIL
}; // namespace ALTH



// operator overload
::RTC::Time operator+(::RTC::Time& _tm, const ::RTC::Time& tm){
	TimeValue _tv(_tm.sec, _tm.nsec/1000);
	TimeValue tv(tm.sec, tm.nsec/1000);
	TimeValue dtm = _tv + tv;
	::RTC::Time dst;
	dst.sec = dtm.sec(); dst.nsec = dtm.usec()*1000;
	return dst;
}

::RTC::Time operator-(::RTC::Time& _tm, const ::RTC::Time& tm){
	TimeValue _tv(_tm.sec, _tm.nsec/1000);
	TimeValue tv(tm.sec, tm.nsec/1000);
	TimeValue dtm = _tv - tv;
	::RTC::Time dst;
	dst.sec = dtm.sec(); dst.nsec = dtm.usec()*1000;
	return dst;
}


#endif



//======================== test code =========================================
#if 0
#include <iostream>
#undef main;
int main(){
	RTC::Time stm1={1, 1000}, stm2={2, 3000};
	RTC::Time dtm = stm1 + stm2;

	RTC::Time ptm = ALTH::UTIL::getCurrentTimeStamp();
	RTC::Time ctm, tm;
	for (int i=0; i<1000; ++i){
		ctm = ALTH::UTIL::getCurrentTimeStamp();
		tm = ctm-ptm;
		std::cout << (unsigned long)tm.sec << "[sec] " << (unsigned long)tm.nsec << "[nsec]"
			<< "->" << toDouble(tm) << "[sec] \r" << std::ends;
	}
	std::cout << "\n<Push any key>" << std::ends;
	getchar();
	return 0;
}
#endif
//======================= end of test code ===================================
