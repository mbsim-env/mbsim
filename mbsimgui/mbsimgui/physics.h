/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2019 Martin Förg

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

#ifndef _PHYSICS__H_
#define _PHYSICN__H_

#include "link_.h"

namespace MBSimGUI {

  class UniversalGravitation : public MechanicalLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(UniversalGravitation, MechanicalLink, MBSIMPHYSICS%"UniversalGravitation", "Universal gravitation");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new UniversalGravitationPropertyDialog(this); }
  };

  class Weight : public MechanicalLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(Weight, MechanicalLink, MBSIMPHYSICS%"Weight", "Weight");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new WeightPropertyDialog(this); }
  };

  class Buoyancy : public FloatingFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(Buoyancy, FloatingFrameLink, MBSIMPHYSICS%"Buoyancy", "Buoyancy");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new BuoyancyPropertyDialog(this); }
  };

  class Drag : public FloatingFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(Drag, FloatingFrameLink, MBSIMPHYSICS%"Drag", "Drag");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new DragPropertyDialog(this); }
  };

  class Aerodynamics : public FloatingFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(Aerodynamics, FloatingFrameLink, MBSIMPHYSICS%"Aerodynamics", "Aerodynamics");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new AerodynamicsPropertyDialog(this); }
  };

}

#endif
