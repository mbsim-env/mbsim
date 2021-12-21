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
    public:
      Link();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Link"; }
      QMenu* createContextMenu() override { return new LinkContextMenu(this); }
  };

  class UnknownLink : public Link {
    public:
      QString getType() const override { return "Unknown link"; }
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class MechanicalLink : public Link {
  };

  class FrameLink : public MechanicalLink {
  };

  class FixedFrameLink : public FrameLink {
  };

  class FloatingFrameLink : public FrameLink {
  };

  class RigidBodyLink : public MechanicalLink {
  };

  class DualRigidBodyLink : public RigidBodyLink {
  };

}

#endif
