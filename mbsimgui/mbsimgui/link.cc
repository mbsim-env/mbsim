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

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Link* Link::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
//    Link *link=ObjectFactory::getInstance()->createLink(e, parent);
    Link *link=Embed<Link>::createAndInit(e,parent);
    if(link) {
//      link->initializeUsingXML(e);
      link->initialize();
    }
    return link;
  }

  RigidBodyLink::RigidBodyLink(const string &str, Element *parent) : Link(str, parent) {
    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  void RigidBodyLink::initialize() {
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
