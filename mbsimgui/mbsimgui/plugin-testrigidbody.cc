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

/*
 This file is just for testing the plugin mechanism!!!
 It just copies the RigidBody code from MBSimGUI and renames the class to TestRigidBody
 to test the plugin mechanism of MBSimGUI.
*/

#include <config.h>
#include "body.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUITestPlugin {

  const MBXMLUtils::NamespaceURI MBSIMGUITESTPLUGIN("http://www.mbsim-env.de/MBSimGUI/TestPlugin");

  class TestRigidBody : public MBSimGUI::Body {
    MBSIMGUI_OBJECTFACTORY_CLASS(TestRigidBody, Body, MBSIMGUITESTPLUGIN%"TestRigidBody", "Plugin test rigid body");
    public:
      TestRigidBody();
      xercesc::DOMElement* getXMLFrames() override { return frames; }
      xercesc::DOMElement* getXMLContours() override { return contours; }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      void create() override;
      void clear() override;
      void setDedicatedFileItem(MBSimGUI::FileItemData *dedicatedFileItem) override;
      void setDedicatedParameterFileItem(MBSimGUI::FileItemData *dedicatedFileItem) override;
      MBSimGUI::PropertyDialog* createPropertyDialog() override { return new MBSimGUI::RigidBodyPropertyDialog(this); }
      QMenu* createFrameContextMenu() override { return new MBSimGUI::FixedRelativeFramesContextMenu(this); }
    protected:
      xercesc::DOMElement *frames, *contours;
  };

  MBSIMGUI_REGOBJECTFACTORY(TestRigidBody);

  TestRigidBody::TestRigidBody()  {
    MBSimGUI::InternalFrame *C = new MBSimGUI::InternalFrame("C",MBSimGUI::MBSIM%"enableOpenMBVFrameC",MBSimGUI::MBSIM%"plotFeatureFrameC");
    addFrame(C);
  }

  void TestRigidBody::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      if((e != frames) and (e != contours) and (E(e)->getTagName() != MBSimGUI::MBSIM%"enableOpenMBVFrameC") and (E(e)->getTagName() != MBSimGUI::MBSIM%"plotFeatureFrameC"))
        element->removeChild(e);
      e = en;
    }
  }

  DOMElement* TestRigidBody::createXMLElement(DOMNode *parent) {
    DOMElement *ele0 = Element::createXMLElement(parent);
    DOMDocument *doc=ele0->getOwnerDocument();
    frames = D(doc)->createElement( MBSimGUI::MBSIM%"frames" );
    ele0->insertBefore( frames, nullptr );
    contours = D(doc)->createElement( MBSimGUI::MBSIM%"contours" );
    ele0->insertBefore( contours, nullptr );
    DOMElement *ele1 = D(doc)->createElement( MBSimGUI::MBSIM%"enableOpenMBVFrameC" );
    ele0->insertBefore( ele1, nullptr );
    return ele0;
  }

  DOMElement* TestRigidBody::processIDAndHref(DOMElement *element) {
    element = Body::processIDAndHref(element);

    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"frames")->getFirstElementChild();
    for(size_t i=1; i<frame.size(); i++) {
      frame[i]->processIDAndHref(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"contours")->getFirstElementChild();
    for(auto & i : contour) {
      i->processIDAndHref(ELE);
      ELE=ELE->getNextElementSibling();
    }

    ELE=E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"openMBVRigidBody");
    if(ELE) {
      ELE = ELE->getFirstElementChild();
      if(ELE)
        E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getID());
    }

    ELE=E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"enableOpenMBVFrameC");
    if(ELE)
      E(ELE)->addProcessingInstructionChildNamed("OPENMBV_ID", getFrame(0)->getID());

    return element;
  }

  void TestRigidBody::create() {
    Body::create();

    frames = E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"frames");
    DOMElement *e=frames->getFirstElementChild();
    MBSimGUI::Frame *f;
    while(e) {
      f = MBSimGUI::Embed<MBSimGUI::FixedRelativeFrame>::create(e,this);
      if(f) {
        addFrame(f);
        f->create();
      }
      e=e->getNextElementSibling();
    }

    contours = E(element)->getFirstElementChildNamed(MBSimGUI::MBSIM%"contours");
    e=contours->getFirstElementChild();
    MBSimGUI::Contour *c;
    while(e) {
      c = MBSimGUI::Embed<MBSimGUI::Contour>::create(e,this);
      if(c) {
        addContour(c);
        c->create();
      }
      e=e->getNextElementSibling();
    }
  }

  void TestRigidBody::clear() {
    for (auto it = frame.begin()+1; it != frame.end(); ++it)
      delete *it;
    for (auto it = contour.begin(); it != contour.end(); ++it)
      delete *it;
    frame.erase(frame.begin()+1,frame.end());
    contour.erase(contour.begin(),contour.end());
  }

  void TestRigidBody::setDedicatedFileItem(MBSimGUI::FileItemData* dedicatedFileItem) {
    Body::setDedicatedFileItem(dedicatedFileItem);
    frame[0]->setDedicatedFileItem(dedicatedFileItem);
  }

  void TestRigidBody::setDedicatedParameterFileItem(MBSimGUI::FileItemData* dedicatedParameterFileItem) {
    Body::setDedicatedParameterFileItem(dedicatedParameterFileItem);
    frame[0]->setDedicatedParameterFileItem(dedicatedFileItem);
  }

}
