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

#include "qt_util.hpp"

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

void fitWindow(QDialog & stretchWindow, QWidget * stretchWidgetL) {
  QGridLayout * layout = (QGridLayout*)stretchWindow.layout();
  layout->addWidget(stretchWidgetL, 0, 0);
  
  QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy1.setHorizontalStretch(100);
  sizePolicy1.setVerticalStretch(100);
  sizePolicy1.setHeightForWidth(true);
  stretchWidgetL->setSizePolicy(sizePolicy1);
  
  stretchWindow.setMinimumSize(stretchWidgetL->width(), stretchWidgetL->height());
  stretchWindow.setGeometry(0,0, stretchWidgetL->width(), stretchWidgetL->height());
}
