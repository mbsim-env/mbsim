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

#ifndef _LINK__H_
#define _LINK__H_

#include "element.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class Link : public Element {
    public:
      Link(const QString &str="");
      static Link* readXMLFile(const std::string &filename);
  };

  class MechanicalLink : public Link {
    public:
      MechanicalLink(const QString &str="") : Link(str) { }
  };

  class FrameLink : public MechanicalLink {
    public:
      FrameLink(const QString &str="");
  };

  class FixedFrameLink : public FrameLink {
    public:
      FixedFrameLink(const QString &str="") : FrameLink(str) { }
 };

  class FloatingFrameLink : public FrameLink {
    public:
      FloatingFrameLink(const QString &str="");
 };

  class RigidBodyLink : public MechanicalLink {
    public:
      RigidBodyLink(const QString &str="");
 };

  class DualRigidBodyLink : public RigidBodyLink {
    public:
      DualRigidBodyLink(const QString &str="");
 };

}

#endif
