// Qt Stuff
#include <Qt/qapplication.h>
#include <Qt/qmainwindow.h>

#include "focuser_cntl_panel.hpp"

#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include "baseclient.h"
#include "basedevice.h"
#include "indiproperty.h"

/* INDI Common Library Routines */
#include "indicom.h"
#include "../indi/indi_focuser_client.hpp"


using namespace std;


// Qt helper function
void fitWindow(QMainWindow & stretchWindow, QWidget * stretchWidgetL) {
  stretchWindow.setCentralWidget(stretchWidgetL);
  
  QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy1.setHorizontalStretch(100);
  sizePolicy1.setVerticalStretch(100);
  sizePolicy1.setHeightForWidth(true);
  stretchWidgetL->setSizePolicy(sizePolicy1);
  
  stretchWindow.setMinimumSize(stretchWidgetL->width(), stretchWidgetL->height());
  stretchWindow.setGeometry(0,0, stretchWidgetL->width(), stretchWidgetL->height());
}


/* Our client auto pointer */
//auto_ptr<MoonLiteFocuserClientT> focusClient(0);

IndiFocuserClientT focusClient("MoonLite");

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QMainWindow mainWindow;
  mainWindow.setGeometry(0,0, 300, 300);
  mainWindow.show();


  // cerr << "Wainting some time until props are populated (later do this on event...)" << endl;
  // sleep(3);
  // cerr << "Ok, waited enough! Moving!" << endl;
  

  //bool res = focusClient->move(5, 123);
  //cerr << "res: " << res << endl;

  // Now do an exposure for 3 seconds...
  //focusClient->takeExposure();

  FocuserCntlPanel focuserCntlPanel;
  focuserCntlPanel.setFocusClient(& focusClient);
  fitWindow(mainWindow, & focuserCntlPanel); // Add the focuser panel

  app.exec();
  return 0;
}
