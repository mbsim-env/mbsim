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
  independentBody=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Independent body", "General", new RigidBodyOfReferenceWidget(MBSIMNS"independentRigidBody",this,0));
  DependenciesWidget *widget = new DependenciesWidget(MBSIMNS"dependentRigidBodiesFirstSide",this);
  dependentBodiesFirstSide=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Dependendent bodies first side", "General", widget);
  connect(widget,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));
  widget = new DependenciesWidget(MBSIMNS"dependentRigidBodiesSecondSide",this);
  dependentBodiesSecondSide=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Dependendent bodies second side", "General", widget);
  connect(widget,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));
  connections = new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Connections", "Kinetics", new ConnectWidget(2,this));
  force=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Force", "Kinetics", new GeneralizedForceDirectionWidget(MBSIMNS"forceDirection"));
  moment=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Moment", "Kinetics", new GeneralizedForceDirectionWidget(MBSIMNS"momentDirection"));

  properties->addStretch();
}

JointConstraint::~JointConstraint() {
}

void JointConstraint::resizeGeneralizedPosition() {
  int size = 0;
  for(int i=0; i<((DependenciesWidget*)dependentBodiesFirstSide->getXMLWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesFirstSide->getXMLWidget())->getBody(i))
    size += ((DependenciesWidget*)dependentBodiesFirstSide->getXMLWidget())->getBody(i)->getUnconstrainedSize();
  for(int i=0; i<((DependenciesWidget*)dependentBodiesSecondSide->getXMLWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesSecondSide->getXMLWidget())->getBody(i))
      size += ((DependenciesWidget*)dependentBodiesSecondSide->getXMLWidget())->getBody(i)->getUnconstrainedSize();
  GeneralizedCoordinatesWidget *q0 = (GeneralizedCoordinatesWidget*)initialGeneralizedPosition->getXMLWidget();
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
