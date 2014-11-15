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

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
}

class QWidget;

namespace MBSimGUI {

  const MBXMLUtils::NamespaceURI MBSIM("http://mbsim.berlios.de/MBSim");
  const MBXMLUtils::NamespaceURI PARAM("http://openmbv.berlios.de/MBXMLUtils/physicalvariable");
  const MBXMLUtils::NamespaceURI OPENMBV("http://openmbv.berlios.de/OpenMBV");
  const MBXMLUtils::NamespaceURI MBSIMINT("http://mbsim.berlios.de/MBSimIntegrator");
  const MBXMLUtils::NamespaceURI MBSIMANALYSER("http://mbsim.berlios.de/MBSimAnalyser");
  const MBXMLUtils::NamespaceURI MBSIMCONTROL("http://mbsim.berlios.de/MBSimControl");
  const MBXMLUtils::NamespaceURI MBSIMPOWERTRAIN("http://mbsim.berlios.de/MBSimPowertrain");
  const MBXMLUtils::NamespaceURI MBSIMXML("http://mbsim.berlios.de/MBSimXML");

  class PropertyInterface {
    public:
      virtual ~PropertyInterface() {}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) {return 0;}
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element) {return 0;}
      virtual void fromWidget(QWidget *widget) {}
      virtual void toWidget(QWidget *widget) {}
      virtual void initialize() {}
      virtual void deinitialize() {}
      virtual std::string getType() const {return "";}
      virtual PropertyInterface* clone() const {return 0;}
  };

  class Property : public PropertyInterface {
    public:
      Property() {}
      virtual ~Property() {}
  };

  class PropertyFactory {
    public:
      virtual PropertyInterface* createProperty(int i=0) = 0;
      virtual MBXMLUtils::FQN getName(int i=0) const { return ""; }
      virtual int getSize() const { return 0; }
  };

}

#endif
