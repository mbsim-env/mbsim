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
  independentBody=new RigidBodyOfReferenceEditor(this, properties, Utils::QIconCached("lines.svg"), "Independent body", "General");
  dependentBodiesFirstSide=new DependenciesEditor(this, properties, Utils::QIconCached("lines.svg"), "Dependendent bodies first side", "General");
  connect(dependentBodiesFirstSide,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));
  dependentBodiesSecondSide=new DependenciesEditor(this, properties, Utils::QIconCached("lines.svg"), "Dependendent bodies second side", "General");
  connect(dependentBodiesSecondSide,SIGNAL(bodyChanged()),this,SLOT(resizeGeneralizedPosition()));
  connections = new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Connections", "Kinetics", new ConnectWidget(2,this));
  force=new GeneralizedForceDirectionEditor(properties, Utils::QIconCached("lines.svg"), true);
  moment=new GeneralizedForceDirectionEditor(properties, Utils::QIconCached("lines.svg"), false);

  properties->addStretch();
}

JointConstraint::~JointConstraint() {
}

void JointConstraint::resizeGeneralizedPosition() {
  int size = 0;
  for(int i=0; i<dependentBodiesFirstSide->getSize(); i++)
    if(dependentBodiesFirstSide->getBody(i))
    size += dependentBodiesFirstSide->getBody(i)->getUnconstrainedSize();
  for(int i=0; i<dependentBodiesSecondSide->getSize(); i++)
    if(dependentBodiesSecondSide->getBody(i))
      size += dependentBodiesSecondSide->getBody(i)->getUnconstrainedSize();
  if(((SVecWidget*)initialGeneralizedPosition->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->size() != size)
    ((SVecWidget*)initialGeneralizedPosition->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->resize(size);
}

void JointConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e, *ee;
  Constraint::initializeUsingXML(element);
  e=element->FirstChildElement(MBSIMNS"dependentRigidBodiesFirstSide");
  ee=e->FirstChildElement();
  while(ee) {
    e=element->FirstChildElement(MBSIMNS"frameOfReference");
    saved_RigidBodyFirstSide.push_back(ee->Attribute("ref"));
    ee=ee->NextSiblingElement();
  }
  e=element->FirstChildElement(MBSIMNS"dependentRigidBodiesSecondSide");
  ee=e->FirstChildElement();
  while(ee) {
    saved_RigidBodySecondSide.push_back(ee->Attribute("ref"));
    ee=ee->NextSiblingElement();
  }
  e=element->FirstChildElement(MBSIMNS"independentRigidBody");
  saved_IndependentBody=e->Attribute("ref");
  force->initializeUsingXML(element);
  moment->initializeUsingXML(element);

  connections->initializeUsingXML(element);
}

TiXmlElement* JointConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Constraint::writeXMLFile(parent);
  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"dependentRigidBodiesFirstSide" );
  for(int i=0; i<dependentBodiesFirstSide->getSize(); i++) {
    if(dependentBodiesFirstSide->getBody(i)) {
      TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"dependentRigidBody" );
      ele2->SetAttribute("ref", dependentBodiesFirstSide->getBody(i)->getXMLPath(this,true).toStdString()); // relative path
      ele1->LinkEndChild(ele2);
    }
  }
  ele0->LinkEndChild(ele1);
  ele1 = new TiXmlElement( MBSIMNS"dependentRigidBodiesSecondSide" );
  for(int i=0; i<dependentBodiesSecondSide->getSize(); i++) {
    if(dependentBodiesSecondSide->getBody(i)) {
      TiXmlElement *ele2 = new TiXmlElement( MBSIMNS"dependentRigidBody" );
      ele2->SetAttribute("ref", dependentBodiesSecondSide->getBody(i)->getXMLPath(this,true).toStdString()); // relative path
      ele1->LinkEndChild(ele2);
    }
  }
  ele0->LinkEndChild(ele1);

  ele1 = new TiXmlElement( MBSIMNS"independentRigidBody" );
  if(independentBody->getBody())
    ele1->SetAttribute("ref", independentBody->getBody()->getXMLPath(this,true).toStdString()); // relative path
  ele0->LinkEndChild(ele1);

  force->writeXMLFile(ele0);
  moment->writeXMLFile(ele0);

  connections->writeXMLFile(ele0);

  return ele0;
}

void JointConstraint::initialize() {
  Object::initialize();
  vector<RigidBody*> rigidBodies;
  if (saved_RigidBodyFirstSide.size()>0) {
    for (unsigned int i=0; i<saved_RigidBodyFirstSide.size(); i++)
      rigidBodies.push_back(getByPath<RigidBody>(saved_RigidBodyFirstSide[i]));
    dependentBodiesFirstSide->setBodies(rigidBodies);
  }
  rigidBodies.clear();
  if (saved_RigidBodySecondSide.size()>0) {
    for (unsigned int i=0; i<saved_RigidBodySecondSide.size(); i++)
      rigidBodies.push_back(getByPath<RigidBody>(saved_RigidBodySecondSide[i]));
    dependentBodiesSecondSide->setBodies(rigidBodies);
  }
  rigidBodies.clear();
  if (saved_IndependentBody!="")
    independentBody->setBody(getByPath<RigidBody>(saved_IndependentBody));
}
