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
#include "observer_property_dialog.h"

namespace MBSimGUI {

  class Observer : public Element {
    MBSIMGUI_OBJECTFACTORY_CLASS(Observer, Element, MBSIM%"Observer", "Observer");
    public:
      Observer();
      QMenu* createContextMenu() override { return new ObserverContextMenu(this); }
  };

  class UnknownObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownObserver, Observer, MBSIM%"UnknownObserver_dummy", "Unknown observer");
    public:
      UnknownObserver();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class MechanicalLinkObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalLinkObserver, Observer, MBSIM%"MechanicalLinkObserver", "Mechanical link observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkObserverPropertyDialog(this); }
  };

  class MechanicalConstraintObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalConstraintObserver, Observer, MBSIM%"MechanicalConstraintObserver", "Mechanical constraint observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintObserverPropertyDialog(this); }
  };

  class ContactObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(ContactObserver, Observer, MBSIM%"ContactObserver", "Contact observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new ContactObserverPropertyDialog(this); }
  };

  class TyreContactObserver : public MechanicalLinkObserver {
    MBSIMGUI_OBJECTFACTORY_CLASS(TyreContactObserver, Observer, MBSIM%"TyreContactObserver", "Tyre contact observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new TyreContactObserverPropertyDialog(this); }
  };

  class FrameObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(FrameObserver, Observer, MBSIM%"FrameObserver", "Frame observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new FrameObserverPropertyDialog(this); }
  };

  class RigidBodyObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidBodyObserver, Observer, MBSIM%"RigidBodyObserver", "Rigid body observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new RigidBodyObserverPropertyDialog(this); }
  };

  class InverseKinematicsConstraintObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(InverseKinematicsConstraintObserver, Observer, MBSIM%"InverseKinematicsConstraintObserver", "Inverse kinematics constraint observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new InverseKinematicsConstraintObserverPropertyDialog(this); }
  };

  class SignalObserver : public Observer {
    MBSIMGUI_OBJECTFACTORY_CLASS(SignalObserver, Observer, MBSIMCONTROL%"SignalObserver", "Signal observer");
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new SignalObserverPropertyDialog(this); }
  };

}

#endif
