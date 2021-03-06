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


#ifndef _FOCUS_FINDER_HPP_
#define _FOCUS_FINDER_HPP_ _FOCUS_FINDER_HPP_

#include "at_plugin.hpp"
#include "at_exception.hpp"

using namespace std;

namespace AT {

  DEF_Exception(FocusFinder);
    
  class FocusFinderPluginT : public PluginT {
  public:
    FocusFinderPluginT();
  };
  
  static FocusFinderPluginT sInstance;
};

EXPORT_INSTANCE();

#endif // _FOCUS_FINDER_HPP_
