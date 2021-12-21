/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
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

  RigidBody::RigidBody()  {
    InternalFrame *C = new InternalFrame("C",MBSIM%"enableOpenMBVFrameC","plotFeatureFrameC");
    addFrame(C);
  }

  void RigidBody::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (E(e)->getTagName() != MBSIM%"enableOpenMBVFrameC") and (E(e)->getTagName() != MBSIM%"plotFeatureFrameC"))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* RigidBody::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    xercesc::DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSIM%"frames" );
    ele0->insertBefore( frames, nullptr );
    contours = D(doc)->createElement( MBSIM%"contours" );
    ele0->insertBefore( contours, nullptr );
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"enableOpenMBVFrameC" );
    ele0->insertBefore( ele1, nullptr );
    return ele0;
  }

  DOMElement* RigidBody::processIDAndHref(DOMElement *element) {
    element = Body::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processIDAndHref(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    for(auto & i : contour) {
      i->processIDAndHref(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"openMBVRigidBody");
    if(ELE) {
      ELE = ELE->getFirstElementChild();
      if(ELE) {
        xercesc::DOMDocument *doc=element->getOwnerDocument();
        DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID());
        ELE->insertBefore(id, nullptr);
      }
    }

    ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameC");
    if(ELE) {
      xercesc::DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getFrame(0)->getID());
      ELE->insertBefore(id, nullptr);
    }

    return element;
  }

  void RigidBody::create() {
    Body::create();

    frames = E(element)->getFirstElementChildNamed(MBSIM%"frames");
    DOMElement *e=frames->getFirstElementChild();
    Frame *f;
    while(e) {
      f = Embed<Frame>::create(e,this);
      if(f) {
        addFrame(f);
        f->create();
      }
      e=e->getNextElementSibling();
    }

    contours = E(element)->getFirstElementChildNamed(MBSIM%"contours");
    e=contours->getFirstElementChild();
    Contour *c;
    while(e) {
      c = Embed<Contour>::create(e,this);
      if(c) {
        addContour(c);
        c->create();
      }
      e=e->getNextElementSibling();
    }
  }

  void RigidBody::clear() {
    for (auto it = frame.begin()+1; it != frame.end(); ++it)
      delete *it;
    for (auto it = contour.begin(); it != contour.end(); ++it)
      delete *it;
    frame.erase(frame.begin()+1,frame.end());
    contour.erase(contour.begin(),contour.end());
  }

  void RigidBody::setDedicatedFileItem(FileItemData* dedicatedFileItem) {
    Body::setDedicatedFileItem(dedicatedFileItem);
    frame[0]->setDedicatedFileItem(dedicatedFileItem);
  }

  void RigidBody::setDedicatedParameterFileItem(FileItemData* dedicatedParameterFileItem) {
    Body::setDedicatedParameterFileItem(dedicatedParameterFileItem);
    frame[0]->setDedicatedParameterFileItem(dedicatedFileItem);
  }

}
