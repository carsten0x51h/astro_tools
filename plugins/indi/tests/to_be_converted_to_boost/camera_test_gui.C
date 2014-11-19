// Qt Stuff
#include <Qt/qapplication.h>
#include <Qt/qmainwindow.h>

#include "camera_cntl_panel.hpp"

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
#include "../indi/indi_camera_client.hpp"


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
IndiCameraClientT cameraClient("ATIK CCD Atik 383L+");

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QMainWindow mainWindow;
  mainWindow.setGeometry(0,0, 300, 300);
  mainWindow.show();
  
  CameraCntlPanel cameraCntlPanel;
  cameraCntlPanel.setCameraClient(& cameraClient);
  fitWindow(mainWindow, & cameraCntlPanel); // Add the camera cntl panel

  app.exec();
  return 0;
}
