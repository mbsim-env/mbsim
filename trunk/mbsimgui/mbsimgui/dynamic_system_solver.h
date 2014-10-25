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
#include "extended_properties.h"
#include <string>

namespace MBSimGUI {

  class Environment : public QObject {
    public:
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent);
      static Environment *getInstance() { return instance?instance:(instance=new Environment); }

    protected:
      Environment();
      virtual ~Environment();
      static Environment *instance;
  };

  class DynamicSystemSolver : public Group {
    friend class DynamicSystemSolverPropertyDialog;
    protected:
    ExtProperty environment, solverParameters, inverseKinetics, initialProjection;
    public:
    DynamicSystemSolver(const std::string &str, Element *parent);
    virtual Element* clone() const {return new DynamicSystemSolver(*this);}
    std::string getType() const { return "DynamicSystemSolver"; }
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getFileExtension() const { return ".mbsim.xml"; }

    static DynamicSystemSolver* readXMLFile(const std::string &filename, Element *parent=0);

    ElementPropertyDialog* createPropertyDialog() {return new DynamicSystemSolverPropertyDialog(this);}
    QMenu* createContextMenu() {return new DynamicSystemSolverContextMenu(this);}
  };

}

#endif
