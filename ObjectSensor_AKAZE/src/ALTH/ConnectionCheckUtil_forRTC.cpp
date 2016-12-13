#include "ConnectionCheckUtil_forRTC.h"

namespace ALTH{
	namespace UTIL{

//============= implementation of CheckPortConnection class ============================
		// constructor
		CheckPortConnection::CheckPortConnection():verbose(true){
		}
		// destructor
		CheckPortConnection::~CheckPortConnection(){
		}

		// add connector property's name to list
		// @pram name: connector property's name
		void CheckPortConnection::addConnectorPropertyName(std::string name){
			_listCPName.push_back(name);
		}
		// delete connector porperty's name from list
		// @pram name: connector property's name
		void CheckPortConnection::deleteConnectorPropertyName(std::string name){
			_listCPName.remove(name);
		}
		// clear all connector porperty's name from list
		void CheckPortConnection::clearConnectorPropertyName(){
			_listCPName.clear();
		}

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
		int CheckPortConnection::operator()(RTC::PortBase &port){
			RTC::ConnectorProfileList* cpl = port.get_connector_profiles();
			int ret(0);

			for(unsigned int i=0; i<cpl->length(); ++i){
			for(unsigned int j=0; j<cpl[i].length(); ++j){
			for(unsigned int k=0; k<cpl[i][j].properties.length(); ++k){
			  std::string name = (std::string) cpl[i][j].properties[k].name;
				std::list<std::string>::iterator itr=_listCPName.begin();
				int index=0;
				while(itr != _listCPName.end()){
				if(*itr == name){
					if(verbose){std::cout << *itr << " has been connected." << std::endl;}
						ret|=static_cast<int>(pow(2.0, index));
					}
					++index;
					++itr;
				}
			}
			}
			}
			if(ret == 0){
				ret = -1;
			}
			return ret;
		}

	}; //namespace UTIL
}; //namespace ALTH

//=========================== test code ===============================
// use case
//
// ALTH::UTIL::CheckPortConnection cpc;
// cpc.addConnectorPropertyName("Port.InterfaceName.InstanceName")
// int pcheck=cpc(port);
// if(pchec>0){
// 	// port is connected with someting. do something.
// 	 ;
// }
//
//======================== end of test code ===========================
