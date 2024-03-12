/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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

#ifndef _PHYSICS_PROPERTY_DIALOG_H_
#define _PHYSICS_PROPERTY_DIALOG_H_

#include "link_property_dialog.h"

namespace MBSimGUI {

  class UniversalGravitationPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      UniversalGravitationPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravitationalConstant, *enableOpenMBV;
  };

  class WeightPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      WeightPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravityFunction, *enableOpenMBV;
  };

  class BuoyancyPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      BuoyancyPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *displacedVolume, *densityFunction, *gravityFunction, *enableOpenMBV;
  };

  class DragPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DragPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dragFunction, *enableOpenMBV;
  };

  class AerodynamicsPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      AerodynamicsPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *densityFunction, *coefficientFunction, *referenceSurface, *windSpeed, *enableOpenMBV;
  };

}

#endif
