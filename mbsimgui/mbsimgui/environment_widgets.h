/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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

#ifndef _ENVIRONMENT_WIDGETS_H_
#define _ENVIRONMENT_WIDGETS_H_

#include "widget.h"
#include "namespace.h"

namespace MBSimGUI {

  class ExtWidget;

  class EnvironmentWidget : public Widget {
    MBSIMGUI_OBJECTFACTORY_CLASS(EnvironmentWidget, Widget, MBSIM%"Environment", "Environment");

    public:
      EnvironmentWidget() = default;
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) { return element; }
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
  };

  class MBSimEnvironmentWidget : public EnvironmentWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(MBSimEnvironmentWidget, EnvironmentWidget, MBSIM%"MBSimEnvironment", "MBSim environment");

    public:
      MBSimEnvironmentWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
    private:
      ExtWidget *accelerationOfGravity, *openMBVObject;
  };

}

#endif
