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

#ifndef _MAIN_WINDOW_HPP_
#define _MAIN_WINDOW_HPP_ _MAIN_WINDOW_HPP_

#include <iostream>

#include "gen/ui_main_window.hpp"

#include "qt_util.hpp"

#include "about_dialog.hpp"
//#include "script_editor_panel.hpp"

class MainWindow : public QMainWindow, private Ui_MainWindow {
Q_OBJECT
public:
  MainWindow(QWidget * inParent) : QMainWindow(inParent) { 
    setupUi(this);
  }
  ~MainWindow() {
    // Delete all windows...
    // TODO: FIXME! Memory leak!

  };
signals:
    
private slots:
  void on_actionAbout_triggered() {
    AboutDialog aboutDlg;
    aboutDlg.exec();
  }

  void on_actionScriptEditor_triggered() {
    // FIXME!!! MEMORY LEAK!
    cerr << "on_actionScriptEditor_triggered..." << endl;

    // TODO: Read about AT window mgmt!!! Ther should be a mechanism!
    // FIXME! This way we have a memory leak!
    QMainWindow * mainWindow = new QMainWindow(this);
    mainWindow->setGeometry(0,0, 300, 300);
    mainWindow->show();
    
    //ScriptEditorPanel * sep = new ScriptEditorPanel(mainWindow);
    //fitWindow(*mainWindow, sep);

    // TODO: FIXME! Memory leak!!
    //PythonInterpreterT * pi = new PythonInterpreterT();
    //sep->setScriptInterpreter(pi);
  }

private:
  // TODO: Read about AT window mgmt!!! Ther should be a mechanism!
  // typedef map<string, pair<QMainWindow*, bool> > WindowContainerT;
  // WindowContainerT mWindows;
};


#endif // _MAIN_WINDOW_HPP_
