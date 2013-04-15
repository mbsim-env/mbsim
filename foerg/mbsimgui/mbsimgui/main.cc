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
#include <QApplication>
#include "mainwindow.h"
#include "octaveutils.h"
#include <mbxmlutils/utils.h>
#include <H5Cpp.h>
#include <QLocale>

int main(int argc, char *argv[]) {
  // environment variables
  // Disalbe COIN VBO per default (see --help)
  char COIN_VBO[strlen("COIN_VBO=0")+1];
  if(getenv("COIN_VBO")==NULL) putenv(strcpy(COIN_VBO, "COIN_VBO=0"));

  // disalbe HDF5 error message print
  H5::Exception::dontPrint();

  initializeOctave();
  QApplication app(argc, argv);
  QLocale::setDefault(QLocale::C);
  setlocale(LC_ALL, "C");
  MainWindow *mainwindow = new MainWindow;
  mainwindow->show();
  //mainwindow->showMaximized();
  //mainwindow->resize(1400, 900);
  mainwindow->resize(1100, 700);
  int ret=app.exec();
  MBXMLUtils::OctaveEvaluator::terminate();
  delete mainwindow;
  return ret;
}
