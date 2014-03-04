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

#ifndef _PROPERTIES_H_
#define _PROPERTIES_H_

#include<string>
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

extern MBXMLUtils::NamespaceURI MBSIM;
extern MBXMLUtils::NamespaceURI PARAM;
extern MBXMLUtils::NamespaceURI OPENMBV;
extern MBXMLUtils::NamespaceURI MBSIMINT;
extern MBXMLUtils::NamespaceURI MBSIMCONTROL;
extern MBXMLUtils::NamespaceURI MBSIMXML;

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
}

class QWidget;

namespace MBSimGUI {

  class PropertyInterface {
    public:
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) = 0;
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element) = 0;
      virtual void fromWidget(QWidget *widget) = 0;
      virtual void toWidget(QWidget *widget) = 0;
      virtual void initialize() {}
      virtual void deinitialize() {}
      virtual std::string getType() const {return "";}
  };

  class Property : public PropertyInterface {
    public:
      Property() {}
      virtual ~Property() {}
      virtual Property* clone() const {return 0;}
  };

  class PropertyFactory {
    public:
      virtual Property* createProperty(int i=0) = 0;
      virtual MBXMLUtils::FQN getName(int i=0) const { return ""; }
      virtual int getSize() const { return 0; }
  };

}

#endif
