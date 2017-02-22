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
#include "function_property_factory.h"

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

  FrameLink::FrameLink(const string &str, Element *parent) : MechanicalLink(str, parent) {
    connections.setProperty(new ConnectFramesProperty(2,this));
  }

  void FrameLink::initialize() {
    MechanicalLink::initialize();
    connections.initialize();
  }

  DOMElement* FrameLink::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    connections.initializeUsingXML(element);
    return element;
  }

  DOMElement* FrameLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalLink::writeXMLFile(parent);
    connections.writeXMLFile(ele0);
    return ele0;
  }

  FloatingFrameLink::FloatingFrameLink(const string &str, Element *parent) : FrameLink(str, parent), refFrameID(0,false) {

    refFrameID.setProperty(new IntegerProperty(1,MBSIM%"frameOfReferenceID"));
  }

  DOMElement* FloatingFrameLink::initializeUsingXML(DOMElement *element) {
    FrameLink::initializeUsingXML(element);
    refFrameID.initializeUsingXML(element);
    return element;
  }

  DOMElement* FloatingFrameLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FrameLink::writeXMLFile(parent);
    refFrameID.writeXMLFile(ele0);
    return ele0;
  }

  RigidBodyLink::RigidBodyLink(const string &str, Element *parent) : MechanicalLink(str, parent), support(0,false) {
    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  void RigidBodyLink::initialize() {
    MechanicalLink::initialize();
    support.initialize();
  }

  DOMElement* RigidBodyLink::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    support.initializeUsingXML(element);
    return element;
  }

  DOMElement* RigidBodyLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = MechanicalLink::writeXMLFile(parent);
    support.writeXMLFile(ele0);
    return ele0;
  }

  DualRigidBodyLink::DualRigidBodyLink(const string &str, Element *parent) : RigidBodyLink(str, parent) {
    connections.setProperty(new ChoiceProperty2(new ConnectRigidBodiesPropertyFactory(this),"",4));
  }

  void DualRigidBodyLink::initialize() {
    RigidBodyLink::initialize();
    connections.initialize();
  }

  DOMElement* DualRigidBodyLink::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    connections.initializeUsingXML(element);
    return element;
  }

  DOMElement* DualRigidBodyLink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = RigidBodyLink::writeXMLFile(parent);
    connections.writeXMLFile(ele0);
    return ele0;
  }

}
