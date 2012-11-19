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

using namespace std;


Constraint::Constraint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Object(str, parentItem, ind) {
}

Constraint::~Constraint() {
}

JointConstraint::JointConstraint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Constraint(str, parentItem, ind) {

  setText(1,getType());

  properties->addTab("Kinetics");

  independentBody = new RigidBodyOfReferenceWidget("Independent body",MBSIMNS"independentRigidBody",this,0);
  properties->addToTab("General", independentBody);

  dependentBodiesFirstSide = new DependenciesWidget("Dependendent bodies first side",MBSIMNS"dependentRigidBodiesFirstSide",this);
  properties->addToTab("General", dependentBodiesFirstSide);
  connect(dependentBodiesFirstSide,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));

  dependentBodiesSecondSide = new DependenciesWidget("Dependendent bodies second side",MBSIMNS"dependentRigidBodiesSecondSide",this);
  properties->addToTab("General", dependentBodiesSecondSide);

  connect(dependentBodiesSecondSide,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));

  connections = new ConnectWidget("Connections",2,this);
  properties->addToTab("Kinetics", connections);

  force = new GeneralizedForceDirectionWidget("Force",MBSIMNS"forceDirection");
  properties->addToTab("Kinetics", force);

  moment = new GeneralizedForceDirectionWidget("Moment",MBSIMNS"momentDirection");
  properties->addToTab("Kinetics", moment);

  properties->addStretch();
}

JointConstraint::~JointConstraint() {
}

void JointConstraint::resizeGeneralizedPosition() {
  int size = 0;
  for(int i=0; i<((DependenciesWidget*)dependentBodiesFirstSide)->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesFirstSide)->getBody(i))
    size += ((DependenciesWidget*)dependentBodiesFirstSide)->getBody(i)->getUnconstrainedSize();
  for(int i=0; i<((DependenciesWidget*)dependentBodiesSecondSide)->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesSecondSide)->getBody(i))
      size += ((DependenciesWidget*)dependentBodiesSecondSide)->getBody(i)->getUnconstrainedSize();
  GeneralizedCoordinatesWidget *q0 = (GeneralizedCoordinatesWidget*)initialGeneralizedPosition;
  if(((SVecWidget*)q0->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->size() != size)
    ((SVecWidget*)q0->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->resize(size);
}

void JointConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e, *ee;
  Constraint::initializeUsingXML(element);
  dependentBodiesFirstSide->initializeUsingXML(element);
  dependentBodiesSecondSide->initializeUsingXML(element);
  independentBody->initializeUsingXML(element);

  force->initializeUsingXML(element);
  moment->initializeUsingXML(element);
  connections->initializeUsingXML(element);
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
