/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include <cassert>
#include <cfenv>
#include <iostream>
#include <QApplication>
#include <QFileInfo>
#include <QSettings>
#include "mainwindow.h"
#include <QLocale>
#ifdef _WIN32
#  include <windows.h>
#else
#  include "qt-unix-signals/sigwatch.h"
#endif

using namespace std;
using namespace MBSimGUI;

int main(int argc, char *argv[]) {
#ifndef _WIN32
//MISSING Qt seems to generate some FPE, hence disabled  assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1);
#endif

  // check for errors during ObjectFactory
  string errorMsg(OpenMBV::ObjectFactory::getAndClearErrorMsg());
  if(!errorMsg.empty()) {
    cerr<<"The following errors occured during the pre-main code of the OpenMBVC++Interface object factory:"<<endl;
    cerr<<errorMsg;
    cerr<<"Exiting now."<<endl;
    return 1;
  }

  // environment variables
  // Disable COIN VBO per default (see --help)
  static char COIN_VBO[11];
  if(getenv("COIN_VBO")==nullptr) putenv(strcpy(COIN_VBO, "COIN_VBO=0"));

  QStringList arg;
  for(int i=1; i<argc; i++)
    arg << argv[i];

  // help
  if(arg.contains("-h") || arg.contains("--help")) {
    cout<<"MBSimGUI - A Graphical User Interface for MBSim"<<endl;
    cout    <<""<<endl;
    cout    <<"Copyright (C) 2013 Martin Foerg <martin.o.foerg@googlemail.com"<<endl;
    cout    <<"This is free software; see the source for copying conditions. There is NO"<<endl;
    cout    <<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl;
    cout    <<""<<endl;
    cout    <<"Licensed under the GNU General Public License (GPL)"<<endl;
    cout    <<""<<endl;
    cout    <<"Usage: mbsimgui [-h|--help]"<<endl;
    cout    <<"                [<dir>] [<mbsimfile>]"<<endl;
    cout    <<""<<endl;
    cout    <<"-h|--help          Shows this help"<<endl;
    cout    <<"--fullscreen       Start in full screen mode"<<endl;
    cout    <<"<dir>              Open first *.mbsx file in dir"<<endl;
    cout    <<"<mbsimfile>        Open <mbsimfile> (*.mbsx)"<<endl;
    cout    <<""<<endl;
    return 0;
  }

  char moduleName[2048];
#ifdef _WIN32
  GetModuleFileName(nullptr, moduleName, sizeof(moduleName));
#else
  size_t s=readlink("/proc/self/exe", moduleName, sizeof(moduleName));
  moduleName[s]=0; // null terminate
#endif
  QCoreApplication::setLibraryPaths(QStringList(QFileInfo(moduleName).absolutePath())); // do not load plugins from buildin defaults

  QApplication app(argc, argv);
#ifndef _WIN32
  UnixSignalWatcher sigwatch;
  sigwatch.watchForSignal(SIGINT);
  sigwatch.watchForSignal(SIGTERM);
  QObject::connect(&sigwatch, &UnixSignalWatcher::unixSignal, &app, &QApplication::quit);
#endif

  // regenerate arg: QApplication removes all arguments known by Qt
  arg.clear();
  for (int i=1; i<argc; i++)
    arg.push_back(argv[i]);

  app.setOrganizationName("mbsim-env");
  app.setApplicationName("mbsimgui");
  app.setOrganizationDomain("www.mbsim-env.de");
  QSettings::setDefaultFormat(QSettings::IniFormat);
  QLocale::setDefault(QLocale::C);
  setlocale(LC_ALL, "C");
  MainWindow mainwindow(arg);
  mainwindow.show();
  if(arg.contains("--fullscreen")) mainwindow.showFullScreen(); // must be done after mainwindow.show()
  if(int ret=app.exec(); ret!=0) return ret;
  if(!mainwindow.getExitOK()) return 1;
  return 0;
}
