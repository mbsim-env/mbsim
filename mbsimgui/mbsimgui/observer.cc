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

#include <config.h>
#include "observer.h"
#include "utils.h"
#include "mainwindow.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Observer::Observer() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"observer.svg").string()));
  }

  DOMElement* MechanicalLinkObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* MechanicalConstraintObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* ContactObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* FrameObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* RigidBodyObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* InverseKinematicsConstraintObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  DOMElement* SignalObserver::processIDAndHref(DOMElement *element) {
    Observer::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBV");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

}
