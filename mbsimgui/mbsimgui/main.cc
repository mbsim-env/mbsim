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

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif
#include <config.h>
#include <clocale>
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
#include <mbxmlutilshelper/utils.h>
#include "../../mbsimxml/mbsimxml/set_current_path.h"
#ifndef _WIN32
#  include "qt-unix-signals/sigwatch.h"
#endif

using namespace std;
using namespace MBSimGUI;

namespace {
  int loadPlugins(const QStringList &arg);
  #ifndef NDEBUG
    void myQtMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg);
  #endif
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  SetConsoleCP(CP_UTF8);
  SetConsoleOutputCP(CP_UTF8);
#endif
  MBXMLUtils::handleFPE();
  setlocale(LC_ALL, "C");
  QLocale::setDefault(QLocale::C);

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
    cout    <<"-C <dir/file>      Change current to dir to <dir>/dir of <file> first."<<endl;
    cout    <<"                   All arguments are still relative to the original current dir."<<endl;
    cout    <<"--CC               Change current dir to dir of <mbsimprjfile> first."<<endl;
    cout    <<"                   All arguments are still relative to the original current dir."<<endl;
    cout    <<"--searchPath=<dir> Directory used to search plugsin. Can be specified multiple times"<<endl;
    cout    <<"                   Searches for libmbsimgui-plugin-*.[so|dll] files"<<endl;
    cout    <<"<dir>              Open first *.mbsx file in dir"<<endl;
    cout    <<"                   <dir> must be the last argument."<<endl;
    cout    <<"<mbsimfile>        Open <mbsimfile> (*.mbsx)"<<endl;
    cout    <<"                   <mbsimfile> must be the last argument."<<endl;
    cout    <<""<<endl;
    return 0;
  }

  // current directory and adapt paths
  boost::filesystem::path dirFile;
  if(!arg.empty())
    dirFile=(--arg.end())->toStdString();
  boost::filesystem::path newCurrentPath;
  if(auto i=std::find(arg.begin(), arg.end(), "--CC"); !dirFile.empty() && i!=arg.end()) {
    if(boost::filesystem::is_directory(dirFile))
      newCurrentPath=dirFile;
    else
      newCurrentPath=dirFile.parent_path();
    arg.erase(i);
  }
  if(auto i=std::find(arg.begin(), arg.end(), "-C"); i!=arg.end()) {
    auto i2=i; i2++;
    if(boost::filesystem::is_directory(i2->toStdString()))
      newCurrentPath=i2->toStdString();
    else
      newCurrentPath=boost::filesystem::path(i2->toStdString()).parent_path();
    arg.erase(i);
    arg.erase(i2);
  }
  SetCurrentPath currentPath(newCurrentPath);
  for(auto a : {"--searchPath"})
    if(auto i=std::find(arg.begin(), arg.end(), a); i!=arg.end()) {
      auto i2=i; i2++;
      *i2=currentPath.adaptPath(i2->toStdString()).string().c_str();
    }
  for(auto i=arg.rbegin(); i!=arg.rend(); ++i)
    if(currentPath.existsInOrg(i->toStdString()))
      *i=currentPath.adaptPath(i->toStdString()).string().c_str();

  char moduleName[2048];
#ifdef _WIN32
  GetModuleFileName(nullptr, moduleName, sizeof(moduleName));
#else
  size_t s=readlink("/proc/self/exe", moduleName, sizeof(moduleName));
  moduleName[s]=0; // null terminate
#endif
  QCoreApplication::setLibraryPaths(QStringList(QFileInfo(moduleName).absolutePath())); // do not load plugins from buildin defaults

  auto argSaved=arg; // save arguments (QApplication removes all arguments known by Qt)
  QApplication app(argc, argv);
#ifndef NDEBUG
  qInstallMessageHandler(myQtMessageHandler);
#endif
  arg=argSaved; // restore arguments
#ifndef _WIN32
  UnixSignalWatcher sigwatch;
  sigwatch.watchForSignal(SIGHUP);
  sigwatch.watchForSignal(SIGINT);
  sigwatch.watchForSignal(SIGTERM);
  QObject::connect(&sigwatch, &UnixSignalWatcher::unixSignal, &app, &QApplication::quit);
#endif

  app.setOrganizationName("mbsim-env");
  app.setApplicationName("mbsimgui");
  app.setOrganizationDomain("www.mbsim-env.de");
  QSettings::setDefaultFormat(QSettings::IniFormat);

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

namespace {
#ifndef NDEBUG
  void myQtMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
    static map<QtMsgType, string> typeStr {
      {QtDebugMsg,    "Debug"},
      {QtWarningMsg,  "Warning"},
      {QtCriticalMsg, "Critical"},
      {QtFatalMsg,    "Fatal"},
      {QtInfoMsg,     "Info"},
    };
    string category(context.category?context.category:"<nocategory>");
    cerr<<(context.file?context.file:"<nofile>")<<":"<<context.line<<": "<<(context.function?context.function:"<nofunc>")<<": "<<category
        <<": "<<typeStr[type]<<": "<<msg.toStdString()<<endl;
    cerr.flush();
    if(category=="qt.accessibility.atspi" || category=="qt.qpa.xcb")
      return;
    switch(type) {
      case QtDebugMsg:
      case QtInfoMsg:
        break;
      case QtWarningMsg:
      case QtCriticalMsg:
      case QtFatalMsg:
        std::abort();
        break;
    }
  }
#endif
}
