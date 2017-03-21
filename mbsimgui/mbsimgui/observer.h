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

#ifndef _OBSERVER__H_
#define _OBSERVER__H_

#include "element.h"
#include "extended_properties.h"

namespace MBSimGUI {

  class Observer : public Element {
    public:
      Observer(const std::string &str="");
      static Observer* readXMLFile(const std::string &filename);
      virtual int getxSize() {return 0;}
  };

  class KinematicCoordinatesObserver : public Observer {
    public:
      KinematicCoordinatesObserver(const std::string &str="");
      std::string getType() const { return "KinematicCoordinatesObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new KinematicCoordinatesObserverPropertyDialog(this);}
  };

  class RelativeKinematicsObserver : public Observer {
    public:
      RelativeKinematicsObserver(const std::string &str="");
      std::string getType() const { return "RelativeKinematicsObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new RelativeKinematicsObserverPropertyDialog(this);}
  };

  class MechanicalLinkObserver : public Observer {
    public:
      MechanicalLinkObserver(const std::string &str="");
      std::string getType() const { return "MechanicalLinkObserver"; }
  };

  class MechanicalConstraintObserver : public Observer {
    public:
      MechanicalConstraintObserver(const std::string &str="");
      std::string getType() const { return "MechanicalConstraintObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new MechanicalConstraintObserverPropertyDialog(this);}
  };

  class ContactObserver : public Observer {
    public:
      ContactObserver(const std::string &str="");
      std::string getType() const { return "ContactObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new ContactObserverPropertyDialog(this);}
  };

  class FrameObserver : public Observer {
    public:
      FrameObserver(const std::string &str="");
      std::string getType() const { return "FrameObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new FrameObserverPropertyDialog(this);}
  };

  class RigidBodyObserver : public Observer {
    public:
      RigidBodyObserver(const std::string &str="");
      std::string getType() const { return "RigidBodyObserver"; }
      ElementPropertyDialog* createPropertyDialog() {return new RigidBodyObserverPropertyDialog(this);}
  };
}

#endif
