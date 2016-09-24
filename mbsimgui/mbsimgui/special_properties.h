/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _SPECIAL_PROPERTIES_H_
#define _SPECIAL_PROPERTIES_H_

#include <vector>
#include "property.h"
#include "extended_properties.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

class QWidget;

namespace MBSimGUI {

  class OneDimMatArrayProperty : public Property {
    protected:
      MBXMLUtils::FQN xmlName;
      std::vector<ExtProperty> ele;
      bool var;
    public:
      OneDimMatArrayProperty(int size=3, int m=3, int n=1, const MBXMLUtils::FQN &xmlName_="", bool var=false);
      virtual PropertyInterface* clone() const {return new OneDimMatArrayProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void resize_(int size, int m=3, int n=1);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget); 
  };

  class TwoDimMatArrayProperty : public Property {
    protected:
      MBXMLUtils::FQN xmlName;
      std::vector<std::vector<ExtProperty> > ele;
      bool var;
    public:
      TwoDimMatArrayProperty(int size=3, int m=1, int n=1, const MBXMLUtils::FQN &xmlName_="", bool var=false);
      virtual PropertyInterface* clone() const {return new TwoDimMatArrayProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void resize_(int size, int m=3, int n=1);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget); 
  };

}

#endif
