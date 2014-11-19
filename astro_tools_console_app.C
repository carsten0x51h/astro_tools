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

#include "astro_tools_app.hpp"
#include "astro_tools_console_app.hpp"

int main(int argc, char * argv[]) {
  static int returnCode = 0;
  
  try {
    AT::AstroToolsConsoleAppT::init(argc, argv);
    AT::AstroToolsConsoleAppT::execute(argc, argv);
    AT::AstroToolsConsoleAppT::destroy();
  } catch(std::exception & exc) {
    LOG(fatal) << "Catched exception in main: " << exc.what() << endl;
  }

  return returnCode;
}
