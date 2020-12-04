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

#ifndef _SOLVER__H_
#define _SOLVER__H_

#include "embeditemdata.h"
#include "solver_property_dialog.h"
#include "solver_context_menu.h"
#include "namespace.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class Project;

  class Solver : public EmbedItemData {
    protected:
      Project *project{nullptr};
    public:
      Solver();
      void setProject(Project* project_) { project = project_; }
      Project* getProject() { return project; }
      virtual void removeXMLElements();
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      QString getName() override { return "Solver"; }
      virtual MBXMLUtils::FQN getXMLType() const { return MBSIM%"Solver"; }
      PropertyDialog* createPropertyDialog() override { return new SolverPropertyDialog(this); }
      QMenu* createContextMenu() override { return new SolverContextMenu; }
      EmbedItemData* getEmbedItemParent() override;
  };

  class UnknownSolver : public Solver {
    public:
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

}

#endif
