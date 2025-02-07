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

#ifndef MBSIMGUI_EVALUATOR_H_
#define MBSIMGUI_EVALUATOR_H_

#include <string>
#include <vector>

class QObject;
class QTextEdit;
class QTextDocument;
class QPlainTextEdit;

namespace MBSimGUI {

class Element;

// Everything in mbsimgui which depends on the current evaluator should be added to this class.
// This ensure that only a simple place needs to be adapted if new evaluators are added or something is changed.
class Evaluator {
  public:
    static const std::string defaultEvaluator;

    static const std::vector<std::string> evaluators;

    static std::pair<std::string, std::string> getFalseTrueStr(const std::string &evaluator);

    static std::vector<std::tuple<std::string, bool, std::string>> getImportActions();
    static int getImportActionDefaultIdx();
    static int getImportActionOnlyOneNoneDepr();

    template<class T1, class T2> static void installSyntaxHighlighter(T1 *t1, T2 *t2);

    static std::pair<std::string, std::string> getInitCode();
    static std::string getElementObjCode(Element *);
};

extern template void Evaluator::installSyntaxHighlighter<QTextEdit, QTextEdit>(QTextEdit *t1, QTextEdit *t2);
extern template void Evaluator::installSyntaxHighlighter<QTextDocument, QPlainTextEdit>(QTextDocument *t1, QPlainTextEdit *t2);

}

#endif
