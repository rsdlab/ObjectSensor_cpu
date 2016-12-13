#ifndef _MULTICAST_EVENTLISTENER_H_
#define _MULTICAST_EVENTLISTENER_H_

//
// MultiCastEventListener.h
//
// @breif This header provides template class for "MultiCast Pattern" in Pattern Hatching[Vlissides99].
//        Please refer http://www.objectclub.jp/technicaldoc/pattern/eDWP in Japanese.
//
// @author H.Takahashi, Osaka Univ.
//
// @last modified in Oct., 2009
//
#include <list>

namespace ALTH{

	// Previous declaration
	// abstract class of EventListener
	template<class EventType> class EventListener;
	// super class of EventSource
	template<class EventType> class EventSource;

	//
	// abstract class of EventListener
	// definition
	template<class EventType>
	class EventListener
	{
	public:
		virtual ~EventListener(){}

		// callback function
		// @param ev: event informed from 'EventSource'
		virtual void invokeEvent(const EventType &ev)=0;
	};

	//
	// super class of EventSource
	// definition
	template<class EventType>
	class EventSource
	{
	public:
		// type of EventListener class
		typedef EventListener<EventType> ListenerType;

	protected:
		// list of EventListener
		::std::list<ListenerType*> _listenerList;

	public:
		virtual ~EventSource(){}

		// add listener of event
		// @param el: event listener
		void addEventListener(ListenerType* el){_listenerList.push_back(el);}

		// delete specific listener from listener list
		// @param el: event listener
		void deleteEventListener(ListenerType* el){_listenerList.remove(el);}

		// delete all listener from listener list
		void deleteEventListener(){_listenerList.clear();}

		// notify event to 'EventListener'
		// @param ev: event which is informed
		void notifyEvent(const EventType& ev){
			typename ::std::list<ListenerType*>::iterator it = _listenerList.begin();
			while(it != _listenerList.end()){
				(*it)->invokeEvent(ev);
				++it;
			}
		}
	};

}; //namespace ALTH

#endif //_MULTICAST_EVENTLISTENER_H_
