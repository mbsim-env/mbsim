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
#include "parameter_context_menu.h"
#include "parameter_view.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "mainwindow.h"
#include "utils.h"
#include "evaluator/evaluator.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterContextMenu::ParameterContextMenu(Parameter *parameter_, QWidget *parent) : QMenu(parent), parameter(parameter_) {
    QAction *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    action->setShortcut(QKeySequence("Ctrl+E"));
    connect(action,&QAction::triggered,this,[=](){ mw->openParameterEditor(); });
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-copy"), "Copy", this);
    action->setShortcut(QKeySequence::Copy);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("edit-cut"), "Cut", this);
    action->setShortcut(QKeySequence::Cut);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(true); });
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(true); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)>0);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(false); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
    action->setShortcut(QKeySequence::Delete);
    connect(action,&QAction::triggered,mw,QOverload<>::of(&MainWindow::removeParameter));
    addAction(action);

    // add the context actions MFMF void code duplication with element_context_menu.cc
    if(parameter->getXMLElement()) {
      auto contextAction=getMBSimGUIContextActions(parameter->getXMLElement()); // read from XML
      if(!contextAction.empty()) {
        addSeparator();
        for(auto &ca : contextAction) {
          addAction(ca.first.c_str(), [ca,parameter_](){
            // this code is run when the action is triggered
            mw->clearEchoView();
            try {
              auto parameterLevels = mw->updateParameters(parameter_->getParent(), true);
              auto [counterName, values]=MainWindow::evaluateForAllArrayPattern(parameterLevels, ca.second,
                parameter_->getXMLElement(), true, true, false, true, [parameter_,ca](const std::vector<std::string>& counterNames, const std::vector<int> &counts) {
                  fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<std::endl<<"Running context action '"<<ca.first<<"' with ";
                  for(size_t i=0; i<counterNames.size(); ++i)
                    fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<(i!=0?", ":"")<<counterNames[i]<<"="<<counts[i];
                  fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<":"<<std::endl;
                  mw->updateEchoView("");
                  auto code=Evaluator::getParameterObjCode(parameter_);
                  if(!code.empty())
                    mw->eval->addParam("mbsimgui_parameter", mw->eval->stringToValue(code));
                }
              );
              mw->updateEchoView("");
            }
            catch(const MBXMLUtils::DOMEvalException &ex) {
              // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
              fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<std::flush<<std::skipws<<ex.what()<<std::flush<<std::noskipws<<std::endl;
              mw->updateEchoView("");
            }
            catch(const std::exception &ex) {
              fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<ex.what()<<std::endl;
              mw->updateEchoView("");
            }
            catch(...) {
              fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception"<<std::endl;
              mw->updateEchoView("");
            }
          });
        }
      }
    }
  }

}
