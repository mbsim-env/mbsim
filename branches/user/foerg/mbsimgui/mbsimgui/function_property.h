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

#ifndef _FUNCTION_PROPERTY_H_
#define _FUNCTION_PROPERTY_H_

#include "property.h"
#include "function_widget.h"

class FunctionProperty : public Property {
  public:
    FunctionProperty(const std::string &name="") : Property(name) {}
    virtual int getArgSize(int i=0) const {return 0;}
    virtual std::string getType() const { return "Function"; }
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element) { return element; }
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
    void fromWidget(QWidget *widget) { }
    void toWidget(QWidget *widget) { }
 //   QMenu* createContextMenu() { return new FunctionChoiceContextMenu(this); }
};

class FunctionChoiceProperty : public Property {
  public:
    FunctionChoiceProperty(const std::string &name="") : Property(name) {}
    QMenu* createContextMenu() { return new FunctionChoiceContextMenu2(this); }
};

#endif

