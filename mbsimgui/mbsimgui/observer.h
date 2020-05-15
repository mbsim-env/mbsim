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

namespace MBSimGUI {

  class Observer : public Element {
    public:
      QString getType() const override { return "Observer"; }
      QMenu* createContextMenu() override { return new ObserverContextMenu(this); }
  };

  class UnknownObserver : public Observer {
    public:
      QString getType() const override { return "UnknownObserver"; }
      PropertyDialog* createPropertyDialog() override {return new UnknownItemPropertyDialog(this);}
  };

  class MechanicalLinkObserver : public Observer {
    public:
      QString getType() const override { return "MechanicalLinkObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new MechanicalLinkObserverPropertyDialog(this);}
  };

  class MechanicalConstraintObserver : public Observer {
    public:
      QString getType() const override { return "MechanicalConstraintObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new MechanicalConstraintObserverPropertyDialog(this);}
  };

  class ContactObserver : public Observer {
    public:
      QString getType() const override { return "ContactObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new ContactObserverPropertyDialog(this);}
  };

  class FrameObserver : public Observer {
    public:
      QString getType() const override { return "FrameObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new FrameObserverPropertyDialog(this);}
  };

  class RigidBodyObserver : public Observer {
    public:
      QString getType() const override { return "RigidBodyObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new RigidBodyObserverPropertyDialog(this);}
  };

  class InverseKinematicsConstraintObserver : public Observer {
    public:
      QString getType() const override { return "InverseKinematicsConstraintObserver"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new InverseKinematicsConstraintObserverPropertyDialog(this);}
  };

  class SignalObserver : public Observer {
    public:
      QString getType() const override { return "SignalObserver"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMCONTROL; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new SignalObserverPropertyDialog(this);}
  };

}

#endif
