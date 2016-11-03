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
#include "link.h"
#include "frame.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include "embed.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Link* Link::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Link *link=Embed<Link>::createAndInit(e,parent);
    if(link) link->initialize();
    return link;
  }

  FloatingFrameLink::FloatingFrameLink(const string &str, Element *parent) : Link(str, parent), refFrameID(0,false), forceArrow(0,false), momentArrow(0,false) {

    connections.setProperty(new ConnectFramesProperty(2,this,"../Frame[I]"));

    refFrameID.setProperty(new IntegerProperty(0,MBSIM%"frameOfReferenceID"));

    forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    momentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    momentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  void FloatingFrameLink::initialize() {
    Link::initialize();
    connections.initialize();
  }

  DOMElement* FloatingFrameLink::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    connections.initializeUsingXML(element);
    refFrameID.initializeUsingXML(element);
    forceArrow.initializeUsingXML(element);
    momentArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* FloatingFrameLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    connections.writeXMLFile(ele0);
    refFrameID.writeXMLFile(ele0);
    forceArrow.writeXMLFile(ele0);
    momentArrow.writeXMLFile(ele0);
    return ele0;
  }

  RigidBodyLink::RigidBodyLink(const string &str, Element *parent) : Link(str, parent), support(0,false) {
    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  void RigidBodyLink::initialize() {
    Link::initialize();
    support.initialize();
  }

  DOMElement* RigidBodyLink::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    support.initializeUsingXML(element);
    return element;
  }

  DOMElement* RigidBodyLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    support.writeXMLFile(ele0);
    return ele0;
  }

}
