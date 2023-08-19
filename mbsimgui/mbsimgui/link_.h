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

#ifndef _LINK__H_
#define _LINK__H_

#include "element.h"

namespace MBSimGUI {

  class Link : public Element {
    MBSIMGUI_OBJECTFACTORY_CLASS(Link, Element, MBSIM%"Link", "Link");
    public:
      Link();
      QMenu* createContextMenu() override { return new LinkContextMenu(this); }
  };

  class UnknownLink : public Link {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownLink, Link, MBSIM%"UnknownLink_dummy", "Unknown link");
    public:
      UnknownLink();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class MechanicalLink : public Link {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalLink, Link, MBSIM%"MechanicalLink", "Mechanical link");
  };

  class FrameLink : public MechanicalLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(FrameLink, MechanicalLink, MBSIM%"FrameLink", "Frame link");
  };

  class FixedFrameLink : public FrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(FixedFrameLink, FrameLink, MBSIM%"FixedFrameLink", "Fixed frame link");
  };

  class FloatingFrameLink : public FrameLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(FloatingFrameLink, FrameLink, MBSIM%"FloatingFrameLink", "Floating frame link");
  };

  class ContourLink : public Link {
    MBSIMGUI_OBJECTFACTORY_CLASS(ContourLink, Link, MBSIM%"ContourLink", "Contour link");
  };

  class RigidBodyLink : public MechanicalLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidBodyLink, MechanicalLink, MBSIM%"RigidBodyLink", "Rigidbody link");
  };

  class DualRigidBodyLink : public RigidBodyLink {
    MBSIMGUI_OBJECTFACTORY_CLASS(DualRigidBodyLink, RigidBodyLink, MBSIM%"DualRigidBodyLink", "Dual rigidbody link");
  };

}

#endif
