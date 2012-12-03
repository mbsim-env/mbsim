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
#include <QLocale>

int main(int argc, char *argv[]) {
  initializeOctave();
  QApplication app(argc, argv);
  QLocale::setDefault(QLocale::C);
  setlocale(LC_ALL, "C");
  MainWindow *mainwindow = new MainWindow;
  mainwindow->show();
  mainwindow->resize(1024, 768);
  int ret=app.exec();
  do_octave_atexit(); // do_octave_atexit must be called before leaving (to prevent crashed in atexit())
  return ret;
}
