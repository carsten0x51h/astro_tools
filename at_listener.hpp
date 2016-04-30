/*****************************************************************************
 *
 *  AstroTools
 *
 *  Copyright(C) 2015 Carsten Schmitt <c.schmitt51h@gmail.com>
 *
 *  This program is free software ; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation ; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY ; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program ; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 ****************************************************************************/

#ifndef _AT_LISTENER_H_
#define _AT_LISTENER_H_ _AT_LISTENER_H

#include <boost/signals2.hpp>

// TODO: Remove code duplication by macro magic!! Even possible??
//#include <boost/preprocessor/punctuation/comma.hpp>

using namespace boost;


#define DEFINE_PROP_LISTENER7(__name__,__type1__,__type2__,__type3__,__type4__,__type5__,__type6__) \
public:								        \
 typedef signals2::signal<void (__type1__,__type2__,__type3__,__type4__,__type5__,__type6__)> __name__##ListenersT; \
protected:								\
 void call##__name__##Listener(__type1__ inData1,__type2__ inData2,__type3__ inData3,__type4__ inData4,__type5__ inData5,__type6__ inData6) { m##__name__##Listeners(inData1,inData2,inData3,inData4,inData5,inData6); } \
public:								        \
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\


#define DEFINE_PROP_LISTENER6(__name__,__type1__,__type2__,__type3__,__type4__,__type5__) \
public:								        \
 typedef signals2::signal<void (__type1__,__type2__,__type3__,__type4__,__type5__)> __name__##ListenersT; \
protected:								\
 void call##__name__##Listener(__type1__ inData1,__type2__ inData2,__type3__ inData3,__type4__ inData4,__type4__ inData5) { m##__name__##Listeners(inData1,inData2,inData3,inData4,inData5); } \
public:								        \
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\


#define DEFINE_PROP_LISTENER5(__name__,__type1__,__type2__,__type3__,__type4__)	\
public:								        \
 typedef signals2::signal<void (__type1__,__type2__,__type3__,__type4__)> __name__##ListenersT; \
protected:								\
 void call##__name__##Listener(__type1__ inData1,__type2__ inData2,__type3__ inData3,__type4__ inData4) { m##__name__##Listeners(inData1,inData2,inData3,inData4); } \
public:								        \
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\



#define DEFINE_PROP_LISTENER4(__name__,__type1__,__type2__,__type3__)	\
public:								        \
 typedef signals2::signal<void (__type1__,__type2__,__type3__)> __name__##ListenersT; \
protected:								\
 void call##__name__##Listener(__type1__ inData1,__type2__ inData2,__type3__ inData3) { m##__name__##Listeners(inData1,inData2,inData3); } \
public:								        \
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\



#define DEFINE_PROP_LISTENER3(__name__,__type1__,__type2__)		\
  public:								\
  typedef signals2::signal<void (__type1__,__type2__)> __name__##ListenersT; \
protected:								\
 void call##__name__##Listener(__type1__ inData1,__type2__ inData2) { m##__name__##Listeners(inData1,inData2); } \
public:								        \
 signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
   return m##__name__##Listeners.connect(inCallBack);			\
 }									\
 template <class T>							\
 void unregister##__name__##Listener(const T & inCallBack) {		\
   inCallBack.disconnect();						\
 }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\
 


#define DEFINE_PROP_LISTENER2(__name__,__type__)			\
public:								        \
  typedef signals2::signal<void (__type__)> __name__##ListenersT;	\
protected:								\
  void call##__name__##Listener(__type__ inData) { m##__name__##Listeners(inData); }  \
public:								        \
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\


 
// TODO: Need a DEFINE_PROP_LISTENER wuth only a name -no param!!!
// See http://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
#define GET_MACRO(_1,_2,_3,_4,_5,_6,_7,NAME,...) NAME
#define DEFINE_PROP_LISTENER(...) GET_MACRO(__VA_ARGS__,DEFINE_PROP_LISTENER7,DEFINE_PROP_LISTENER6,DEFINE_PROP_LISTENER5,DEFINE_PROP_LISTENER4,DEFINE_PROP_LISTENER3,DEFINE_PROP_LISTENER2)(__VA_ARGS__)

#endif /* _AT_LISTENER_H_ */
