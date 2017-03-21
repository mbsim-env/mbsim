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
#include "frame.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include "embed.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Frame::Frame(const string &str, Element *parent) : Element(str,parent) {

    addPlotFeature("position");
    addPlotFeature("angle");
    addPlotFeature("velocity");
    addPlotFeature("angularVelocity");
    addPlotFeature("acceleration");
    addPlotFeature("angularAcceleration");
  }

  Frame* Frame::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Frame *frame=Embed<Frame>::createAndInit(e,parent);
    return frame;
  }

  DOMElement* Frame::processFileID(DOMElement *element) {
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
      ELE->insertBefore(id, NULL);
    }
    return element;
  }

  void InternalFrame::removeXMLElements() {
    DOMElement *e = E(parent->getXMLElement())->getFirstElementChildNamed(getXMLFrameName());
    if(e) {
      if(X()%e->getPreviousSibling()->getNodeName()=="#text")
        parent->getXMLElement()->removeChild(e->getPreviousSibling());
      parent->getXMLElement()->removeChild(e);
    }
    e = E(parent->getXMLElement())->getFirstElementChildNamed(MBSIM%getPlotFeatureType());
    while (e and E(e)->getTagName()==MBSIM%getPlotFeatureType()) {
      DOMElement *en = e->getNextElementSibling();
      if(X()%e->getPreviousSibling()->getNodeName()=="#text")
        parent->getXMLElement()->removeChild(e->getPreviousSibling());
      parent->getXMLElement()->removeChild(e);
      e = en;
    }
  }

  FixedRelativeFrame::FixedRelativeFrame(const string &str, Element *parent) : Frame(str,parent) {
  }

  NodeFrame::NodeFrame(const string &str, Element *parent) : Frame(str,parent) {

    //nodeNumber.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIMFLEX%"nodeNumber",vector<string>(2,"-")),"",4));
  }

}
