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

#ifndef _GEAR__H_
#define _GEAR__H_

#include "link_.h"
#include "link_property_dialog.h"

namespace MBSimGUI {

  class GeneralizedGear : public RigidBodyLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedGear, RigidBodyLink, MBSIM%"GeneralizedGear", "Generalized gear");
    public:
      PropertyDialog* createPropertyDialog() override { return new GeneralizedGearPropertyDialog(this); }
  };

}

#endif
