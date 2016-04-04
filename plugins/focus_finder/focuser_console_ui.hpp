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

#ifndef _FOCUSER_CONSOLE_UI_HPP_
#define _FOCUSER_CONSOLE_UI_HPP_ _FOCUSER_CONSOLE_UI_HPP_

#include <ncurses.h>

#include "at_plugin.hpp"
#include "at_exception.hpp"
#include "limited_queue.hpp"

#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"
#include "indi/indi_filter_wheel.hpp"

#include "focus_finder_parabel_fit_impl.hpp"

using namespace std;

namespace AT {
  
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);
  
  template <typename... Args> void
  mv_print(int x, int y, const char * c, Args&&... args) {
    move(y, x);
    clrtoeol();
    mvprintw(y, x, c, std::forward<Args>(args)...);
  }

  // TODO: Create ManualConsoleFocusCntl class or similar...??!
  void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec, BinningT inBinning, bool inFollowStar = true);
  
}; // end AT

#endif /* _FOCUSER_CONSOLE_UI_HPP_ */
