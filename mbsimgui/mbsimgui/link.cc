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
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

extern DOMLSParser *parser;

namespace MBSimGUI {

  Link::Link(const string &str) : Element(str) {
    addPlotFeature("generalizedRelativePosition");
    addPlotFeature("generalizedRelativeVelocity");
    addPlotFeature("generalizedForce");
    addPlotFeature("energy");
  }

  Link* Link::readXMLFile(const string &filename) {
    shared_ptr<DOMDocument> doc(parser->parseURI(X()%filename));
    DOMElement *e=doc->getDocumentElement();
    Link *link=Embed<Link>::createAndInit(e);
    return link;
  }

  FrameLink::FrameLink(const string &str) : MechanicalLink(str) {
//    connections.setProperty(new ConnectFramesProperty(2,this));
  }

  FloatingFrameLink::FloatingFrameLink(const string &str) : FrameLink(str) {

//    refFrameID.setProperty(new IntegerProperty(1,MBSIM%"frameOfReferenceID"));
  }

  RigidBodyLink::RigidBodyLink(const string &str) : MechanicalLink(str) {
//    support.setProperty(new FrameOfReferenceProperty("",this,MBSIM%"supportFrame"));
  }

  DualRigidBodyLink::DualRigidBodyLink(const string &str) : RigidBodyLink(str) {
//    connections.setProperty(new ChoiceProperty2(new ConnectRigidBodiesPropertyFactory(this),"",4));
  }

}
