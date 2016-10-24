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
#include "rigid_body.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "basic_properties.h"
#include "kinematics_properties.h"
#include "ombv_properties.h"
#include "kinematic_functions_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  RigidBody::RigidBody(const string &str, Element *parent) : Body(str,parent), constrained(false), K(0,false), frameForInertiaTensor(0,false), translation(0,false), rotation(0,false), translationDependentRotation(0,false), coordinateTransformationForRotation(0,false), bodyFixedRepresentationOfAngularVelocity(0,false), ombvEditor(0,true), weightArrow(0,false), jointForceArrow(0,false), jointMomentArrow(0,false) {
    Frame *C = new Frame("C",this,true,vector<FQN>(1,MBSIM%"plotFeatureFrameC"));
    addFrame(C);

    K.setProperty(new LocalFrameOfReferenceProperty("Frame[C]",this,MBSIM%"frameForKinematics"));

    mass.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"mass",vector<string>(2,"kg")),"",4));

    inertia.setProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,3,"0.01","0"),MBSIM%"inertiaTensor",vector<string>(3,"kg*m^2")),"",4));

    frameForInertiaTensor.setProperty(new LocalFrameOfReferenceProperty("Frame[C]",this,MBSIM%"frameForInertiaTensor"));

    vector<Property*> property;

    translation.setProperty(new ChoiceProperty2(new TranslationPropertyFactory4(this),"",3)); 

    rotation.setProperty(new ChoiceProperty2(new RotationPropertyFactory4(this),"",3)); 

    translationDependentRotation.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIM%"translationDependentRotation",vector<string>(2,"")),"",4));

    coordinateTransformationForRotation.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"coordinateTransformationForRotation",vector<string>(2,"")),"",4));
    coordinateTransformationForRotation.setActive(false);

    bodyFixedRepresentationOfAngularVelocity.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIM%"bodyFixedRepresentationOfAngularVelocity",vector<string>(2,"")),"",4));

    ombvEditor.setProperty(new OMBVBodySelectionProperty(this));

    weightArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    weightArrow.setXMLName(MBSIM%"enableOpenMBVWeight",false);

    jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointForceArrow.setXMLName(MBSIM%"enableOpenMBVJointForce",false);

    jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    jointMomentArrow.setXMLName(MBSIM%"enableOpenMBVJointMoment",false);

  }

  int RigidBody::getqRelSize() const {
    int nqT=0, nqR=0;
    if(translation.isActive()) {
      const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(translation.getProperty())->getProperty());
      const ChoiceProperty2 *trans = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
      nqT = static_cast<Function*>(trans->getProperty())->getArg1Size();
    }
    if(rotation.isActive()) {
      const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(rotation.getProperty())->getProperty());
      const ChoiceProperty2 *rot = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
      nqR = static_cast<Function*>(rot->getProperty())->getArg1Size();
    }
    int nq = nqT + nqR;
    return nq;
  }

  int RigidBody::getuRelSize() const {
    return getqRelSize();
  }

  void RigidBody::initialize() {
    Body::initialize();

    for(size_t i=0; i<frame.size(); i++)
      frame[i]->initialize();
    for(size_t i=0; i<contour.size(); i++)
      contour[i]->initialize();
  }

  DOMElement* RigidBody::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    Frame *f;
    while(e) {
      f = Embed<Frame>::createAndInit(e,this);
      if(f) addFrame(f);
      e=e->getNextElementSibling();
    }

    // contours
    e=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    Contour *c;
    while(e) {
      c = Embed<Contour>::createAndInit(e,this);
      if(c) addContour(c);
      e=e->getNextElementSibling();
    }

    K.initializeUsingXML(element);

    mass.initializeUsingXML(element);
    inertia.initializeUsingXML(element);
    frameForInertiaTensor.initializeUsingXML(element);

    translation.initializeUsingXML(element);
    rotation.initializeUsingXML(element);
    translationDependentRotation.initializeUsingXML(element);
    coordinateTransformationForRotation.initializeUsingXML(element);
    bodyFixedRepresentationOfAngularVelocity.initializeUsingXML(element);

    ombvEditor.initializeUsingXML(element);

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameC");
    if(e)
      getFrame(0)->initializeUsingXML2(e);
    else
      getFrame(0)->setOpenMBVFrame(false);

    weightArrow.initializeUsingXML(element);

    jointForceArrow.initializeUsingXML(element);
    jointMomentArrow.initializeUsingXML(element);

    getFrame(0)->initializeUsingXML3(element);

    return element;
  }

  DOMElement* RigidBody::writeXMLFile(DOMNode *parent) {

    DOMElement *ele0 = Body::writeXMLFile(parent);
    DOMElement *ele1;

    K.writeXMLFile(ele0);

    mass.writeXMLFile(ele0);
    inertia.writeXMLFile(ele0);
    frameForInertiaTensor.writeXMLFile(ele0);

    translation.writeXMLFile(ele0);
    rotation.writeXMLFile(ele0);
    translationDependentRotation.writeXMLFile(ele0);
    coordinateTransformationForRotation.writeXMLFile(ele0);
    bodyFixedRepresentationOfAngularVelocity.writeXMLFile(ele0);

    DOMDocument *doc=ele0->getOwnerDocument();
    ele1 = D(doc)->createElement( MBSIM%"frames" );
    for(size_t i=1; i<frame.size(); i++)
      Embed<Frame>::writeXML(frame[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ele1 = D(doc)->createElement( MBSIM%"contours" );
    for(size_t i=0; i<contour.size(); i++)
      Embed<Contour>::writeXML(contour[i],ele1);
    ele0->insertBefore( ele1, NULL );

    ombvEditor.writeXMLFile(ele0);

    Frame *C = getFrame(0);
    if(C->openMBVFrame()) {
      ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameC" );
      C->writeXMLFile2(ele1);
      ele0->insertBefore(ele1, NULL);
    }

    weightArrow.writeXMLFile(ele0);

    jointForceArrow.writeXMLFile(ele0);
    jointMomentArrow.writeXMLFile(ele0);

    C->writeXMLFile3(ele0);

    return ele0;
  }

}
