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

#include "spring_damper.h"
#include "utils.h"
#include <QtGui/QMenu>
#include "frame.h"

using namespace std;


SpringDamper::SpringDamper(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind) {

  setText(1,getType());

  properties->addTab("Kinetics");
  //properties->addTab("Constitutive laws");

  //force=new ForceDirectionEditor(properties, Utils::QIconCached("lines.svg"), "Position");
  //MomentDirectionEditor *moment=new MomentDirectionEditor(properties, Utils::QIconCached("lines.svg"), "Position");
  connections=new ConnectEditor(2, this, properties, Utils::QIconCached("lines.svg"), "Connections", "Kinetics");
  force=new ForceLawEditor2(this, properties, Utils::QIconCached("lines.svg"));
  connect(connections->getLineEdit(0),SIGNAL(textChanged(const QString&)),this,SLOT(updateFrameOfReference()));
  //connect(connections->getLineEdit(1),SIGNAL(textChanged(const QString&)),this,SLOT(updateFrameOfReference()));

  properties->addStretch();
}

SpringDamper::~SpringDamper() {
}

void SpringDamper::updateFrameOfReference() {
  //if(frameOfReference->getFrame()==0)
    force->getFrameOfReference()->setFrame(connections->getFrame(0));
    force->getFrameOfReference()->update();
}

void SpringDamper::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Link::initializeUsingXML(element);
  force->initializeUsingXML(element);
  e=element->FirstChildElement(MBSIMNS"connect");
  saved_ref1=e->Attribute("ref1");
  saved_ref2=e->Attribute("ref2");
}

TiXmlElement* SpringDamper::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  force->writeXMLFile(ele0);
  TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"connect");
  if(connections->getFrame(0))
    ele1->SetAttribute("ref1", connections->getFrame(0)->getXMLPath(this,true).toStdString()); // relative path
  if(connections->getFrame(1))
    ele1->SetAttribute("ref2", connections->getFrame(1)->getXMLPath(this,true).toStdString()); // relative path
  ele0->LinkEndChild(ele1);
  return ele0;
}

void SpringDamper::initialize() {
  if(saved_ref1!="")
    connections->setFrame(0,getByPath<Frame>(saved_ref1));
  if(saved_ref2!="")
    connections->setFrame(1,getByPath<Frame>(saved_ref2));
  force->initialize();
}
