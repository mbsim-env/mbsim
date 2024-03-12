/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _SPRING_DAMPER__H_
#define _SPRING_DAMPER__H_

#include "link_.h"
#include "link_property_dialog.h"

namespace MBSimGUI {

  class SpringDamper : public FixedFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpringDamper, FixedFrameLink, MBSIM%"SpringDamper", "Spring damper");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new SpringDamperPropertyDialog(this); }
  };

  class DirectionalSpringDamper : public FloatingFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(DirectionalSpringDamper, FloatingFrameLink, MBSIM%"DirectionalSpringDamper", "Directional spring damper");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new DirectionalSpringDamperPropertyDialog(this); }
  };

  class IsotropicRotationalSpringDamper : public FixedFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(IsotropicRotationalSpringDamper, FixedFrameLink, MBSIM%"IsotropicRotationalSpringDamper", "Isotropic rotational spring damper");
    public:
      PropertyDialog* createPropertyDialog() override { return new IsotropicRotationalSpringDamperPropertyDialog(this); }
  };

  class GeneralizedSpringDamper : public DualRigidBodyLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedSpringDamper, DualRigidBodyLink, MBSIM%"GeneralizedSpringDamper", "Generalized spring damper");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedSpringDamperPropertyDialog(this); }
  };

}

#endif
