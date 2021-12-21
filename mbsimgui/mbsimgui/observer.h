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

#ifndef _OBSERVER__H_
#define _OBSERVER__H_

#include "element.h"

namespace MBSimGUI {

  class Observer : public Element {
    public:
      Observer();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Observer"; }
      QMenu* createContextMenu() override { return new ObserverContextMenu(this); }
  };

  class UnknownObserver : public Observer {
    public:
      QString getType() const override { return "Unknown observer"; }
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class MechanicalLinkObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"MechanicalLinkObserver"; }
      QString getType() const override { return "Mechanical link observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkObserverPropertyDialog(this); }
  };

  class MechanicalConstraintObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"MechanicalConstraintObserver"; }
      QString getType() const override { return "Mechanical constraint observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintObserverPropertyDialog(this); }
  };

  class ContactObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"ContactObserver"; }
      QString getType() const override { return "Contact observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new ContactObserverPropertyDialog(this); }
  };

  class FrameObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"FrameObserver"; }
      QString getType() const override { return "Frame observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new FrameObserverPropertyDialog(this); }
  };

  class RigidBodyObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"RigidBodyObserver"; }
      QString getType() const override { return "Rigid body observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new RigidBodyObserverPropertyDialog(this); }
  };

  class InverseKinematicsConstraintObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"InverseKinematicsConstraintObserver"; }
      QString getType() const override { return "Inverse kinematics constraint observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new InverseKinematicsConstraintObserverPropertyDialog(this); }
  };

  class SignalObserver : public Observer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"SignalObserver"; }
      QString getType() const override { return "Signal observer"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new SignalObserverPropertyDialog(this); }
  };

}

#endif
