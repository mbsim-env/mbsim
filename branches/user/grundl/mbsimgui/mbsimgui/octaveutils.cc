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
#include "octaveutils.h"
#include <string>
#include <mbxmlutilstinyxml/getinstallpath.h>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "mainwindow.h"
#include <mbxmlutils/utils.h>
using namespace std;
using namespace MBXMLUtils;

void initializeOctave() {
  MBXMLUtils::OctaveEvaluator::initialize();

  // preserve whitespace and newline in TiXmlText nodes
  TiXmlBase::SetCondenseWhiteSpace(false);
}

string evalOctaveExpression(const string &str) {
  string ret;
  if(str!="") {
    bool error = false;
    try{
      MainWindow::octEval->octaveEvalRet(str, 0, false);
      ret = MainWindow::octEval->octaveGetRet();
    }
    catch (string e) {
      cout << "An exception occurred in evalOctaveExpression: " << e << endl;
    }
  }
  return ret;
}

