/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "evaluator.h"
#include "mainwindow.h"
#include "octave_highlighter.h"
#include "python_highlighter.h"
#include <QTextEdit>
#include <QTextDocument>
#include <QPlainTextEdit>
#include <boost/format.hpp>

using namespace std;

namespace MBSimGUI {

extern MainWindow *mw;

const string Evaluator::defaultEvaluator = "octave";

const vector<string> Evaluator::evaluators {
  "octave", // must be the first one
  "python",
  "flatxml",
};

pair<string, string> Evaluator::getFalseTrueStr(const std::string &evaluator) {
  if(evaluator=="octave")
    return {"false", "true"};
  else if(evaluator=="python")
    return {"False", "True"};
  else if(evaluator=="flatxml")
    return {"false", "true"};
  return {"0", "1"};
}

vector<tuple<string, bool, string>> Evaluator::getImportActions() {
  if(mw->eval->getName()=="octave") {
    return {{"", false, ""}};
  }
  if(mw->eval->getName()=="python") {
    return {
      {"addNewVarsToInstance", true, "add new variables globally (deprecated)"},
      {"addAllVarsAsParams"  , false, "add all variables locally"},
    };
  }
  return {};
}

int Evaluator::getImportActionDefaultIdx() {
  if(mw->eval->getName()=="python")
    return 1;
  return 0;
}

int Evaluator::getImportActionOnlyOneNoneDepr() {
  int count=0;
  int idx=-1;
  int oneDeprIdx=0;
  for(auto &x : getImportActions()) {
    idx++;
    if(get<1>(x)==true)
      continue;
    count++;
    oneDeprIdx=idx;
  }
  if(count!=1)
    return -1;
  else
    return oneDeprIdx;
}

template<class T1, class T2>
void Evaluator::installSyntaxHighlighter(T1 *t1, T2 *t2) {
  if(mw->eval->getName()=="octave")
    new OctaveHighlighter(t1);
  else if(mw->eval->getName()=="python")
    new PythonHighlighter(t1);
  else
    cerr<<"No syntax hightlighter for current evaluator "+mw->eval->getName()+" available."<<endl;
  static const QFont fixedFont=QFontDatabase::systemFont(QFontDatabase::FixedFont);
  t2->setFont(fixedFont);
  t2->setLineWrapMode(T2::NoWrap);
}

template void Evaluator::installSyntaxHighlighter<QTextEdit, QTextEdit>(QTextEdit *t1, QTextEdit *t2);
template void Evaluator::installSyntaxHighlighter<QTextDocument, QPlainTextEdit>(QTextDocument *t1, QPlainTextEdit *t2);

pair<string, string> Evaluator::getInitCode() {
  if(mw->eval->getName()=="python")
    return {"addAllVarsAsParams", (boost::format(R"(
def _local():
  import sys
  sys.path.append(r"%1$s/share/mbsimgui/python")
_local()
del _local
)")%mw->getInstallPath().string()).str()};
  return {};
}

string Evaluator::getElementObjCode(Element *e) {
  if(mw->eval->getName()=="python")
    return (boost::format(R"(
import mbsimgui
ret=mbsimgui._Element(%1$x)
)")%e).str();
  return {};
}

string Evaluator::getParameterObjCode(Parameter *p) {
  if(mw->eval->getName()=="python")
    return (boost::format(R"(
import mbsimgui
ret=mbsimgui._Parameter(%1$x)
)")%p).str();
  return {};
}

}
