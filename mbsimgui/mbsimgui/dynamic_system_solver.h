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

#ifndef _DYNAMIC_SYSTEM_SOLVER__H_
#define _DYNAMIC_SYSTEM_SOLVER__H_

#include "group.h"
#include <string>

namespace MBSimGUI {

  class Project;

  class DynamicSystemSolver : public Group {
    MBSIMGUI_OBJECTFACTORY_CLASS(DynamicSystemSolver, Group, MBSIM%"DynamicSystemSolver", "Dynamic system solver");
    protected:
      xercesc::DOMElement *environments;
      Project *project{nullptr};
    public:
      DynamicSystemSolver();
      void setProject(Project* project_) { project = project_; }
      Project* getProject() { return project; }
      xercesc::DOMElement* getXMLEnvironments() { return environments; }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      void create() override;
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new DynamicSystemSolverPropertyDialog(this); }
      QMenu* createContextMenu() override { return new DynamicSystemSolverContextMenu(this); }
      EmbedItemData* getEmbedItemParent() override;
  };

}

#endif
