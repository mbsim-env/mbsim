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
#include "rigid_body.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "embed.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  RigidBody::RigidBody(const QString &str) : Body(str), constrained(false) {
    InternalFrame *C = new InternalFrame("C",MBSIM%"enableOpenMBVFrameC","plotFeatureFrameC");
    addFrame(C);
  }

  void RigidBody::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    DOMElement *ombvFrame=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameC");
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (e != ombvFrame))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* RigidBody::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSIM%"frames" );
    ele0->insertBefore( frames, NULL );
    contours = D(doc)->createElement( MBSIM%"contours" );
    ele0->insertBefore( contours, NULL );

    DOMElement *ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameC" );
    ele0->insertBefore( ele1, NULL );

    for(size_t i=1; i<frame.size(); i++)
      frame[i]->createXMLElement(frames);
    for(size_t i=0; i<contour.size(); i++)
      contour[i]->createXMLElement(contours);
    return ele0;
  }

  DOMElement* RigidBody::processFileID(DOMElement *element) {
    Body::processFileID(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    for(size_t i=0; i<contour.size(); i++) {
      contour[i]->processFileID(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"openMBVRigidBody");
    if(ELE) {
      ELE = ELE->getFirstElementChild();
      if(ELE) {
        DOMDocument *doc=element->getOwnerDocument();
        DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
        ELE->insertBefore(id, NULL);
      }
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameC");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID().toStdString());
      ELE->insertBefore(id, NULL);
    }

    return element;
  }

//  DOMElement* RigidBody::processHref(DOMElement *element) {
//    Body::processHref(element);
//
//    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
//    for(size_t i=1; i<frame.size(); i++) {
//      frame[i]->processHref(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
//      ELE=ELE->getNextElementSibling();
//    }
//
//    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
//    for(size_t i=0; i<contour.size(); i++) {
//      contour[i]->processHref(E(ELE)->getTagName()==PV%"Embed"?ELE->getLastElementChild():ELE);
//      ELE=ELE->getNextElementSibling();
//    }
//
//    return element;
//  }

  DOMElement* RigidBody::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Body::initializeUsingXML(element);

    frames = E(element)->getFirstElementChildNamed(MBSIM%"frames");
    e=frames->getFirstElementChild();
    Frame *f;
    while(e) {
      f = Embed<Frame>::createAndInit(e);
      if(f) addFrame(f);
      e=e->getNextElementSibling();
    }

    contours = E(element)->getFirstElementChildNamed(MBSIM%"contours");
    e=contours->getFirstElementChild();
    Contour *c;
    while(e) {
      c = Embed<Contour>::createAndInit(e);
      if(c) addContour(c);
      e=e->getNextElementSibling();
    }

    return element;
  }

}
