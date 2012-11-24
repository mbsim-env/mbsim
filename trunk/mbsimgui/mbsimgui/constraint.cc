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

  independentBody = new ExtXMLWidget("Independent body","",new RigidBodyOfReferenceWidget(MBSIMNS"independentRigidBody",this,0));
  properties->addToTab("General", independentBody);

  DependenciesWidget *dependentBodiesFirstSide_ = new DependenciesWidget(MBSIMNS"dependentRigidBodiesFirstSide",this);
  dependentBodiesFirstSide = new ExtXMLWidget("Dependendent bodies first side","",dependentBodiesFirstSide_);
  properties->addToTab("General", dependentBodiesFirstSide);
  connect(dependentBodiesFirstSide_,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));

  DependenciesWidget *dependentBodiesSecondSide_ = new DependenciesWidget(MBSIMNS"dependentRigidBodiesSecondSide",this);
  dependentBodiesSecondSide = new ExtXMLWidget("Dependendent bodies second side","",dependentBodiesSecondSide_);
  properties->addToTab("General", dependentBodiesSecondSide);
  connect(dependentBodiesSecondSide_,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));

  connections = new ExtXMLWidget("Connections","",new ConnectWidget(2,this));
  properties->addToTab("Kinetics", connections);

  force = new ExtXMLWidget("Force","",new GeneralizedForceDirectionWidget(MBSIMNS"forceDirection"));
  properties->addToTab("Kinetics", force);

  moment = new ExtXMLWidget("Moment","",new GeneralizedForceDirectionWidget(MBSIMNS"momentDirection"));
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
