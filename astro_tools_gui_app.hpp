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

#ifndef _ASTRO_TOOLS_GUI_APP_HPP_
#define _ASTRO_TOOLS_GUI_APP_HPP_ _ASTRO_TOOLS_GUI_APP_HPP_

#include "astro_tools_app.hpp"

// Static polimorphism since decision can be made at compile time.
namespace AT {
  class AstroToolsGuiAppT : public AstroToolsAppT<AstroToolsGuiAppT> {
  public:
    // ...
    static void init(int argc, char **argv) {
      AstroToolsAppT::init(argc, argv);
    }

    static void execute(int argc, char **argv);

    static bool evaluateCmdLineArguments(int argc, char **argv, po::variables_map * outCmdLineOptionsMap) {
      return true;
    }

    static const po::options_description getExtendedGeneralOptions() {
      po::options_description additionalGeneralOptions;
      return additionalGeneralOptions;
    }

  };
}; // end AT


#endif
