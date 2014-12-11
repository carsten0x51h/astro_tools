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

#ifndef _INDI_LISTENER_H_
#define _INDI_LISTENER_H_ _INDI_LISTENER_H

#include <boost/signals2.hpp>

using namespace boost;

#define DEFINE_PROP_LISTENER(__name__, __type__)			\
  public:								\
  typedef signals2::signal<void (__type__)> __name__##ListenersT;	\
  signals2::connection register##__name__##Listener(const __name__##ListenersT::slot_type & inCallBack) { \
    return m##__name__##Listeners.connect(inCallBack);			\
  }									\
  template <class T>							\
  void unregister##__name__##Listener(const T & inCallBack) {		\
    inCallBack.disconnect();						\
  }									\
protected:								\
  void call##__name__##Listener(__type__ inData) {			\
    m##__name__##Listeners(inData);					\
  }									\
private:								\
 __name__##ListenersT m##__name__##Listeners;				\
 


#define DEFINE_INDI_SLOT(__methodname__, __name__, __type__)		\
  protected:								\
  virtual void __methodname__(__type__ p) {				\
   if (p) {								\
     m##__name__##Listeners(p);						\
   }									\
 }									\

#endif /* _INDI_LISTENER_H_ */
