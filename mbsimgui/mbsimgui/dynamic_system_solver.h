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

#ifndef _DYNAMIC_SYSTEM_SOLVER__H_
#define _DYNAMIC_SYSTEM_SOLVER__H_

#include "group.h"
#include <string>

namespace MBSimGUI {

  class Project;

  class Environment : public QObject {
    public:
      static Environment *getInstance() { return instance?instance:(instance=new Environment); }

    protected:
      Environment() = default;
      ~Environment() override = default;
      static Environment *instance;
  };

  class DynamicSystemSolver : public Group {
    protected:
      xercesc::DOMElement *environments;
      Project *project{nullptr};
    public:
      DynamicSystemSolver()  { config = true; }
      QString getType() const override { return "DynamicSystemSolver"; }
      void setProject(Project* project_) { project = project_; }
      Project* getProject() { return project; }
      xercesc::DOMElement* getXMLEnvironments() { return environments; }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      QString getFileExtension() const override { return ".mbsim.xml"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      ElementPropertyDialog* createPropertyDialog() override { return new DynamicSystemSolverPropertyDialog(this); }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() override { return new EmbeddingPropertyDialog(this,false); }
      QMenu* createContextMenu() override { return new ElementContextMenu(this,nullptr,false,true); }
      EmbedItemData* getEmbedItemParent() override;
  };

}

#endif
