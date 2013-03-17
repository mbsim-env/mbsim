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
#include "constraint.h"
#include "frame.h"
#include "rigidbody.h"
#include "string_widgets.h"
#include "function_widgets.h"
#include "kinetics_widgets.h"

using namespace std;

Constraint::Constraint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Object(str, parentItem, ind) {
}

Constraint::~Constraint() {
}

KinematicConstraint::KinematicConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Constraint(str, parentItem, ind), refBody(0) {

  setText(1,getType());

//  properties->addTab("Kinetics");

  dependentBody = new ExtXMLWidget("Dependent body",new RigidBodyOfReferenceWidget(MBSIMNS"dependentRigidBody",this,0));
  connect((RigidBodyOfReferenceWidget*)dependentBody->getWidget(),SIGNAL(bodyChanged()),this,SLOT(updateReferenceBody()));
  properties->addToTab("General", dependentBody);

  kinematicFunction = new ExtXMLWidget("Kinematic function",new Function1ChoiceWidget(MBSIMNS"kinematicFunction"),true);
  properties->addToTab("General", kinematicFunction);
  connect((Function1ChoiceWidget*)kinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  firstDerivativeOfKinematicFunction = new ExtXMLWidget("First derivative of kinematic function",new Function1ChoiceWidget(MBSIMNS"firstDerivativeOfKinematicFunction"),true);
  properties->addToTab("General", firstDerivativeOfKinematicFunction);
  connect((Function1ChoiceWidget*)firstDerivativeOfKinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  secondDerivativeOfKinematicFunction = new ExtXMLWidget("Second derivative of kinematic function",new Function1ChoiceWidget(MBSIMNS"secondDerivativeOfKinematicFunction"),true);
  properties->addToTab("General", secondDerivativeOfKinematicFunction);
  connect((Function1ChoiceWidget*)secondDerivativeOfKinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  properties->addStretch();
}

KinematicConstraint::~KinematicConstraint() {
}

void KinematicConstraint::resizeVariables() {
  int size = refBody?refBody->getUnconstrainedSize():0;
  ((Function1ChoiceWidget*)kinematicFunction->getWidget())->resize(size,1);
  ((Function1ChoiceWidget*)firstDerivativeOfKinematicFunction->getWidget())->resize(size,1);
  ((Function1ChoiceWidget*)secondDerivativeOfKinematicFunction->getWidget())->resize(size,1);
}

void KinematicConstraint::updateReferenceBody() {
  if(refBody) {
    refBody->setConstrained(false);
    refBody->resizeGeneralizedPosition();
    refBody->resizeGeneralizedVelocity();
  }
  refBody = ((RigidBodyOfReferenceWidget*)dependentBody->getWidget())->getBody();
  refBody->setConstrained(true);
  refBody->resizeGeneralizedPosition();
  refBody->resizeGeneralizedVelocity();
  connect(refBody,SIGNAL(sizeChanged()),this,SLOT(resizeVariables()));
  resizeVariables();
}

void KinematicConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e, *ee;
  blockSignals(true);
  Constraint::initializeUsingXML(element);
  dependentBody->initializeUsingXML(element);
  kinematicFunction->initializeUsingXML(element);
  firstDerivativeOfKinematicFunction->initializeUsingXML(element);
  secondDerivativeOfKinematicFunction->initializeUsingXML(element);
  blockSignals(false);
}

TiXmlElement* KinematicConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);

  dependentBody->writeXMLFile(ele0);
  kinematicFunction->writeXMLFile(ele0);
  firstDerivativeOfKinematicFunction->writeXMLFile(ele0);
  secondDerivativeOfKinematicFunction->writeXMLFile(ele0);

  return ele0;
}

JointConstraint::JointConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Constraint(str, parentItem, ind) {

  setText(1,getType());

  properties->addTab("Kinetics",1);

  independentBody = new ExtXMLWidget("Independent body",new RigidBodyOfReferenceWidget(MBSIMNS"independentRigidBody",this,0));
  properties->addToTab("General", independentBody);

  DependenciesWidget *dependentBodiesFirstSide_ = new DependenciesWidget(MBSIMNS"dependentRigidBodiesFirstSide",this);
  dependentBodiesFirstSide = new ExtXMLWidget("Dependendent bodies first side",dependentBodiesFirstSide_);
  properties->addToTab("General", dependentBodiesFirstSide);
  connect(dependentBodiesFirstSide_,SIGNAL(bodyChanged()),this,SLOT(resizeVariables()));

  DependenciesWidget *dependentBodiesSecondSide_ = new DependenciesWidget(MBSIMNS"dependentRigidBodiesSecondSide",this);
  dependentBodiesSecondSide = new ExtXMLWidget("Dependendent bodies second side",dependentBodiesSecondSide_);
  properties->addToTab("General", dependentBodiesSecondSide);
  connect(dependentBodiesSecondSide_,SIGNAL(bodyChanged()),this,SLOT(resizeVariables()));

  connections = new ExtXMLWidget("Connections",new ConnectFramesWidget(2,this));
  properties->addToTab("Kinetics", connections);

  force = new ExtXMLWidget("Force",new GeneralizedForceDirectionWidget(MBSIMNS"forceDirection"));
  properties->addToTab("Kinetics", force);

  moment = new ExtXMLWidget("Moment",new GeneralizedForceDirectionWidget(MBSIMNS"momentDirection"));
  properties->addToTab("Kinetics", moment);

  properties->addStretch();
}

JointConstraint::~JointConstraint() {
}

void JointConstraint::resizeGeneralizedPosition() {
  int size = 0;
  for(int i=0; i<((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getBody(i))
    size += ((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getBody(i)->getUnconstrainedSize();
  for(int i=0; i<((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getBody(i))
      size += ((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getBody(i)->getUnconstrainedSize();
  if(q0->size() != size)
    q0->resize(size);
}

void JointConstraint::initializeUsingXML(TiXmlElement *element) {
  blockSignals(true);
  Constraint::initializeUsingXML(element);
  dependentBodiesFirstSide->initializeUsingXML(element);
  dependentBodiesSecondSide->initializeUsingXML(element);
  independentBody->initializeUsingXML(element);

  force->initializeUsingXML(element);
  moment->initializeUsingXML(element);
  connections->initializeUsingXML(element);
  blockSignals(false);
}

TiXmlElement* JointConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);

  dependentBodiesFirstSide->writeXMLFile(ele0);
  dependentBodiesSecondSide->writeXMLFile(ele0);

  independentBody->writeXMLFile(ele0);

  force->writeXMLFile(ele0);
  moment->writeXMLFile(ele0);

  connections->writeXMLFile(ele0);

  return ele0;
}
