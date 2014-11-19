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

#ifndef _AT_EXCEPTION_H_
#define _AT_EXCEPTION_H_ _AT_EXCEPTION_H_

#include <sstream>
#include <string>

using namespace std;

#define AT_THROW_CLASS(type, ctor) \
  { \
    type __exn__ ctor; \
    throw __exn__; \
  }

#define AT_THROW(x,m) { \
  ostringstream __oss__; \
  __oss__.flush(); \
  __oss__ << m; \
  AT_THROW_CLASS (x##ExceptionT, (__oss__.str())); \
}

#define AT_ASSERT(x,a,m) { if (! (a)) { AT_THROW(x,m); } }

class BaseExceptionT : public std::exception {
public:
  BaseExceptionT(const string & inName = "", const string & inMsg = "") : mName(inName), mMsg(inMsg) { }
  ~BaseExceptionT() throw() {}
  const char * what() const throw() { return mMsg.c_str(); }

private:
  string mName;
  string mMsg;
};


#define DEF_Exception(XYZ) \
  class XYZ##ExceptionT : public BaseExceptionT {			\
  public:								\
  inline XYZ##ExceptionT(const string & inMsg = "") : BaseExceptionT(#XYZ"Exception", inMsg) {} \
  }

DEF_Exception(Generic);


#endif /* _AT_EXCEPTION_H_ */
