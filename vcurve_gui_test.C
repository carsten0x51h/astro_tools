// Qt Stuff
#include <Qt/qapplication.h>
#include <Qt/qmainwindow.h>

#include "vcurve_viewer_panel.hpp"

#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include "vcurve.hpp"

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


int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  QMainWindow mainWindow;
  mainWindow.setGeometry(0,0, 600, 300);
  mainWindow.show();

  // Init the VCurve...
  VCurveT<int, double> vcurve1, vcurve2, vcurveSum;

  // Add values to curves
  vcurve1[-4] = 0.59;
  vcurve1[-3] = 0.5;
  vcurve1[-2] = 0.35;
  vcurve1[-1] = 0.2;
  vcurve1[0] = 0.1;
  vcurve1[1] = 0.22;
  vcurve1[2] = 0.4;
  vcurve1[3] = 0.55;
  vcurve1[4] = 0.61;

  // vcurve2[-1] = 4.4;
  // vcurve2[0] = 0.05;
  // vcurve2[1] = 5.5;

  // cerr << vcurve1;
  // cerr << vcurve2;

  // vcurveSum = vcurve1 + vcurve2;
  
  // cerr << "SUM" << flush;
  // cerr << vcurveSum;

  // //cerr << (vcurveSum / 2.0);
  // //cerr << (vcurveSum * 4.0);

  // VCurveT<int, double> fourTimesVCurve = 4.0 * vcurveSum;// * 4.0;
  // VCurveT<int, double> threeTimesVCurve = vcurveSum * 3.0;
  // VCurveT<int, double> diffVCurve = fourTimesVCurve - threeTimesVCurve;

  // cerr << "DIFF" << flush;
  // cerr << diffVCurve;

  // cerr << "EQUAL? " << (diffVCurve == vcurveSum) << endl;

  VCurveViewerPanel vcurveViewerPanel;
  vcurveViewerPanel.setVCurve(& vcurve1);
  fitWindow(mainWindow, & vcurveViewerPanel);

  app.exec();
  return 0;
}
