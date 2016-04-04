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

#ifndef _AT_TASK_HPP_
#define _AT_TASK_HPP_ _AT_TASK_HPP_

#include <boost/thread.hpp>

namespace AT {

  class MtTaskPolicyT {
  private:
    static boost::mutex mMutex;    

  public:
    static void lock() {
      boost::lock_guard<boost::mutex> lock(mMutex);
    }
  };

  class NonMtTaskPolicyT {
  public:
    static void lock() { } // Simply do no locking
  };


  template <typename ST, typename MTS = NonMtTaskPolicyT>
  class TaskT {
  private:
    ST mStatus;

  protected:
    void setStatus(const ST & inStatus) {
      MTS::lock();
      mStatus = inStatus;
    }
  
  public:
    const ST & getStatus() const {
      MTS::lock();
      return mStatus;
    }
    virtual void execute() = 0; // TODO: Do we need this?
  };
  
}; // end namespace AT

#endif // _AT_TASK_HPP_
