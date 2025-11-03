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

#ifndef _PROJECT__H_
#define _PROJECT__H_

#include "embed.h"
#include "embeditemdata.h"
#include "project_property_dialog.h"
#include "project_context_menu.h"
#include "namespace.h"
#include "evaluator/evaluator.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class DynamicSystemSolver;
  class Solver;

  class Project : public EmbedItemData {
    MBSIMGUI_OBJECTFACTORY_CLASS(Project, EmbedItemData, MBSIMXML%"MBSimProject", "MBSim project");
    public:
      Project();
      ~Project() override;
      virtual void removeXMLElements();
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      void create() override;
      ProjectPropertyDialog* createPropertyDialog() override { return new ProjectPropertyDialog(this); }
      QMenu* createContextMenu() override { return new ProjectContextMenu; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      void setDynamicSystemSolver(DynamicSystemSolver *dss_);
      void setSolver(Solver *solver_);
      DynamicSystemSolver* getDynamicSystemSolver() const { return dss; }
      Solver* getSolver() const { return solver; }
      xercesc::DOMElement* createEmbedXMLElement() override;
      void maybeRemoveEmbedXMLElement() override;
      void setEvaluator(const std::string &evaluator_);
      const std::string& getEvaluator() { return evaluator; }
      void setDefaultEvaluator(int index) { defaultEvaluator = index; }
      int getDefaultEvaluator() const { return defaultEvaluator; }
      QString getVarTrue();
      QString getVarFalse();
    private:
      DynamicSystemSolver *dss{nullptr};
      Solver *solver{nullptr};
      std::string evaluator{Evaluator::defaultEvaluator};
      int defaultEvaluator{0};
  };

}

#endif
