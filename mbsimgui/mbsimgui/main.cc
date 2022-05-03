/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
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
#include <QLibrary>
#include <boost/dll.hpp>
#include <mbxmlutilshelper/shared_library.h>
#ifdef _WIN32
#  include <windows.h>
#else
#  include "qt-unix-signals/sigwatch.h"
#endif

using namespace std;
using namespace MBSimGUI;

namespace {
  int loadPlugins(const QStringList &arg);
}

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

  // check for errors during ObjectFactory
  string errorMsg2(ObjectFactory::getInstance().getAndClearErrorMsg());
  if(!errorMsg2.empty()) {
    cerr<<"The following errors occured during the pre-main code of the MBSimGUI object factory:"<<endl;
    cerr<<errorMsg2;
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
    cout    <<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl;
    cout    <<""<<endl;
    cout    <<"Usage: mbsimgui [-h|--help]"<<endl;
    cout    <<"                [<dir>] [<mbsimfile>]"<<endl;
    cout    <<""<<endl;
    cout    <<"-h|--help          Shows this help"<<endl;
    cout    <<"--fullscreen       Start in full screen mode"<<endl;
    cout    <<"--searchPath=<dir> Directory used to search plugsin. Can be specified multiple times"<<endl;
    cout    <<"                   Searches for libmbsimgui-plugin-*.[so|dll] files"<<endl;
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

  if(loadPlugins(arg)!=0)
    return 1;

  {
    MainWindow mainwindow(arg);
    mainwindow.show();
    if(arg.contains("--fullscreen")) mainwindow.showFullScreen(); // must be done after mainwindow.show()
    if(int ret=app.exec(); ret!=0) return ret;
  }
  if(!MainWindow::getExitOK()) return 1;
  return 0;
}

namespace {

int loadPlugins(const QStringList &arg)
{
#ifndef _WIN32
  const static string libDir="lib";
  const static QString libSuffix=".so";
#else
  const static string libDir="bin";
  const static QString libSuffix=".dll";
#endif

  QStringList allSearchDirs;
  // command arguments
  for(auto &a : arg)
    if(a.startsWith("--searchPath="))
      allSearchDirs.append(a.mid(QString("--searchPath=").length()));
  // install directory (local/[lib|bin])
  allSearchDirs.append((boost::dll::program_location().parent_path().parent_path()/libDir).string().c_str());
  // current dir
  allSearchDirs.append(QDir::currentPath());
  // dirs from config file
  QSettings settings;
  auto pathStr=settings.value("mainwindow/options/plugins", QString()).toString();
  auto path=pathStr.isEmpty() ? QStringList() : pathStr.split("\n");
  allSearchDirs.append(path);

  // load mbsimgui plugins
  for(auto &dir : allSearchDirs) {
    cout<<"Searching for MBSimGUI plugins in directory: "<<dir.toStdString()<<endl;
    auto qdir=QDir(dir);
    for(auto &f : qdir.entryList(QDir::Files)) {
      if(!f.startsWith("libmbsimgui-plugin-") || !f.endsWith(libSuffix))
        continue;
      cout<<" - load: "<<f.toStdString()<<endl;
      MBXMLUtils::SharedLibrary::load(qdir.absoluteFilePath(f).toStdString());
    }
  }

  // check for errors during ObjectFactory
  string errorMsg3(ObjectFactory::getInstance().getAndClearErrorMsg());
  if(!errorMsg3.empty()) {
    cerr<<"The following errors occured during loading of MBSimGUI plugin object factory:"<<endl;
    cerr<<errorMsg3;
    cerr<<"Exiting now."<<endl;
    return 1;
  }
  return 0;
}

}
