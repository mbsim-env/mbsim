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

#include "group.h"
#include "extended_properties.h"
#include <string>

class Environment : public QObject {
  public:
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
    static Environment *getInstance() { return instance?instance:(instance=new Environment); }

  protected:
    Environment();
    virtual ~Environment();
    static Environment *instance;
};

class Solver : public Group {
  friend class SolverPropertyDialog;
  protected:
    ExtProperty environment, solverParameters, inverseKinetics;
  public:
    Solver(const std::string &str, Element *parent);
    std::string getType() const { return "DynamicSystemSolver"; }
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    std::string getFileExtension() const { return ".mbsim.xml"; }

    static Solver* readXMLFile(const std::string &filename);
    void writeXMLFile(const std::string &name);
    void writeXMLFile() { writeXMLFile(getName()); }

    ElementPropertyDialog* createPropertyDialog() {return new SolverPropertyDialog(this);}
    ElementContextMenu* createContextMenu() {return new SolverContextMenu;}
};

#endif
