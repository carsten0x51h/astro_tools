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

#include <Qt/qapplication.h>
#include "main_window.hpp"

// #include "focus_finder_panel.hpp"

// #include <iostream>
// #include <stdio.h>
// #include <stdarg.h>
// #include <math.h>
// #include <unistd.h>
// #include <time.h>
// #include <memory>
// #include <sys/types.h>
// #include <sys/stat.h>

// #include "qt_util.hpp"

#include "astro_tools_gui_app.hpp"

namespace AT {
  void AstroToolsGuiAppT::execute(int argc, char **argv) {
    QApplication app(argc, argv);
    MainWindow mainWindow(0);
    mainWindow.setGeometry(0,0, 300, 300);
    mainWindow.show();
    //FocusFinderPanelT focusFinderPanel;
    //fitWindow(mainWindow, & focusFinderPanel);
    app.exec();
  }
};

int main(int argc, char *argv[]) {
  static int returnCode = 0;

  try {
    AT::AstroToolsGuiAppT::init(argc, argv);
    AT::AstroToolsGuiAppT::execute(argc, argv);
    AT::AstroToolsGuiAppT::destroy();
  } catch(std::exception & exc) {
    LOG(fatal) << "Catched exception in main: " << exc.what() << endl;
  }

  return returnCode;
}
