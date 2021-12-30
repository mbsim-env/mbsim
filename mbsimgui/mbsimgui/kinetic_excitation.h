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

#ifndef _KINETIC_EXCITATION__H_
#define _KINETIC_EXCITATION__H_

#include "link_.h"

namespace MBSimGUI {

  class KineticExcitation : public FloatingFrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(KineticExcitation, FloatingFrameLink, MBSIM%"KineticExcitation", "Kinetic excitation");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new KineticExcitationPropertyDialog(this); }
  };

}

#endif
