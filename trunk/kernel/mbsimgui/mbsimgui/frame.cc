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

#include "frame.h"
#include "utils.h"
#include <QtGui/QMenu>
#include "mainwindow.h"
#include "utils.h"
#include "rigidbody.h"
#include "objectfactory.h"

using namespace std;


Frame::Frame(const QString &str, QTreeWidgetItem *parentItem, int ind, bool grey) : Element(str, parentItem, ind, grey) {

  setText(1,getType());

  if(!grey) {
    QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
    connect(action,SIGNAL(triggered()),this,SLOT(remove()));
    contextMenu->addAction(action);
  }

  properties->addTab("Visualisation");
  visu=new FrameVisuEditor(this, properties, Utils::QIconCached("lines.svg"), "Visualisation");

  properties->addStretch();
}

Frame::~Frame() {
}

void Frame::resetAnimRange(int numOfRows, double dt) {
}

void Frame::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);

  TiXmlElement *ee;
  if((ee=element->FirstChildElement(MBSIMNS"enableOpenMBV"))) {
    visu->setOpenMBVFrame(true);
    visu->setSize(getDouble(ee->FirstChildElement(MBSIMNS"size")));
    visu->setOffset(getDouble(ee->FirstChildElement(MBSIMNS"offset")));
  }
}

TiXmlElement* Frame::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  if(visu->openMBVFrame()) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"enableOpenMBV" );
    addElementText(ele1,MBSIMNS"size",visu->getSize());
    addElementText(ele1,MBSIMNS"offset",visu->getOffset());
    ele0->LinkEndChild(ele1);
  }
  return ele0;
}

