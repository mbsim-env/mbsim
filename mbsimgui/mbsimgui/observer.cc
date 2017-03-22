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
#include "objectfactory.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

extern DOMLSParser *parser;

namespace MBSimGUI {

  Observer::Observer(const string &str) : Element(str) {
  }

  Observer* Observer::readXMLFile(const string &filename) {
    shared_ptr<DOMDocument> doc(parser->parseURI(X()%filename));
    DOMElement *e=doc->getDocumentElement();
    Observer *observer=Embed<Observer>::createAndInit(e);
    return observer;
  }

  KinematicCoordinatesObserver::KinematicCoordinatesObserver(const string &str) : Observer(str) {
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

  RelativeKinematicsObserver::RelativeKinematicsObserver(const string &str) : Observer(str) {

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

  MechanicalLinkObserver::MechanicalLinkObserver(const string &str) : Observer(str) {

//    link.setProperty(new LinkOfReferenceProperty("",this,MBSIM%"mechanicalLink"));
//
//    forceArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVForce",getID()));
//
//    momentArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVMoment",getID()));
  }

  MechanicalConstraintObserver::MechanicalConstraintObserver(const string &str) : Observer(str) {

//    constraint.setProperty(new ConstraintOfReferenceProperty("",this,MBSIM%"mechanicalConstraint"));
//
//    forceArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVForce",getID()));
//
//    momentArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVMoment",getID()));
  }

  ContactObserver::ContactObserver(const string &str) : Observer(str) {

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

  FrameObserver::FrameObserver(const string &str) : Observer(str) {

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

  RigidBodyObserver::RigidBodyObserver(const string &str) : Observer(str) {

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

}
