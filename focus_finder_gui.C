#include <Qt/qapplication.h>
#include "focus_finder_panel.hpp"
#include "main_window.hpp"

#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include "qt_util.hpp"
#include "astro_focuser_app.hpp"

using namespace std;

int main(int argc, char *argv[]) {
  AstroFocuserAppT::init(argc, argv);

  QApplication app(argc, argv);
  MainWindow mainWindow(0);
  mainWindow.setGeometry(0,0, 300, 300);
  mainWindow.show();

  FocusFinderPanelT focusFinderPanel;
  fitWindow(mainWindow, & focusFinderPanel);

  app.exec();
  return 0;
}
