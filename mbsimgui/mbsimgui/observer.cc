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

#include <config.h>
#include "observer.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Observer::Observer(const QString &str) : Element(str) {
  }

  KinematicCoordinatesObserver::KinematicCoordinatesObserver(const QString &str) : Observer(str) {
//
//    frame.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"frame"));
//
//    frameOfReference.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"frameOfReference"));
//
//    position.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVPosition",getID(),true));
//
//    velocity.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVVelocity",getID(),true));
//
//    acceleration.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAcceleration",getID(),true));
  }

  DOMElement* KinematicCoordinatesObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  RelativeKinematicsObserver::RelativeKinematicsObserver(const QString &str) : Observer(str) {

//    frame.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"frame"));
//
//    refFrame.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"frameOfReference"));
//
//    position.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVPosition",getID(),true));
//
//    velocity.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVVelocity",getID(),true));
//
//    angularVelocity.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAngularVelocity",getID(),true));
//
//    acceleration.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAcceleration",getID(),true));
//
//    angularAcceleration.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAngularAcceleration",getID(),true));
  }

  DOMElement* RelativeKinematicsObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  MechanicalLinkObserver::MechanicalLinkObserver(const QString &str) : Observer(str) {
  }

  DOMElement* MechanicalLinkObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  MechanicalConstraintObserver::MechanicalConstraintObserver(const QString &str) : Observer(str) {
  }

  DOMElement* MechanicalConstraintObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  ContactObserver::ContactObserver(const QString &str) : Observer(str) {

//    link.setProperty(new LinkOfReferenceProperty("",this,MBSIM%"contact"));
//
//    forceArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVForce",getID()));
//
//    momentArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVMoment",getID()));
//
//    contactPoints.setProperty(new FrameMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVContactPoints",getID()));
//
//    normalForceArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVNormalForce",getID()));
//
//    frictionArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVTangentialForce",getID()));
  }

  DOMElement* ContactObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  FrameObserver::FrameObserver(const QString &str) : Observer(str) {

//    frame.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"frame"));
//
//    position.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVPosition",getID(),true));
//
//    velocity.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVVelocity",getID(),true));
//
//    angularVelocity.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAngularVelocity",getID(),true));
//
//    acceleration.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAcceleration",getID(),true));
//
//    angularAcceleration.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAngularAcceleration",getID(),true));
  }

  DOMElement* FrameObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularVelocity");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularAcceleration");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

  RigidBodyObserver::RigidBodyObserver(const QString &str) : Observer(str) {

//    body.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBody"));
//
//    weight.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVWeight",getID(),true));
//
//    jointForce.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVJointForce",getID(),true));
//
//    jointMoment.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVJointMoment",getID(),true));
//
//    axisOfRotation.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVAxisOfRotation",getID(),true));
  }

  DOMElement* RigidBodyObserver::processFileID(DOMElement *element) {
    Observer::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }
}
