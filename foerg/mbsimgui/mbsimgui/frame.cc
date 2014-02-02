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
#include <boost/shared_ptr.hpp>
#include "frame.h"
#include "ombv_properties.h"
#include "objectfactory.h"
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

#define ieO 0
#define irP 1
#define irO 2
#define ifo 3

Frame::Frame(const string &str, Element *parent, bool grey) : Element(str,parent) {

  type = "Frame";
 // properties->addTab("Plotting");
 // plotFeature.push_back(new ExtWidget("Plot global position", new PlotFeature("globalPosition"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global velocity", new PlotFeature("globalVelocity"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global acceleration", new PlotFeature("globalAcceleration"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);

//  visu.setProperty(new OMBVFrameProperty("NOTSET",grey?"":MBSIM%"enableOpenMBV",getID()));
  //property.push_back(new OMBVFrameProperty("enable OpenMBV",grey?"":MBSIM%"enableOpenMBV",getID()));
  property.push_back(new OMBVFrameProperty("enableOpenMBV",grey?"":"",getID()));
  property[ieO]->setDisabling(true);
  property[ieO]->setDisabled(false);
}

Frame::~Frame() {
}

Frame* Frame::readXMLFile(const string &filename, Element *parent) {
  shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
  DOMElement *e=doc->getDocumentElement();
  Frame *frame=ObjectFactory::getInstance()->createFrame(e, parent);
  if(frame) {
    frame->initializeUsingXML(e);
    frame->initialize();
  }
  return frame;
}

void Frame::initializeUsingXML(DOMElement *element) {
  Element::initializeUsingXML(element);
  DOMElement *e = E(element)->getFirstElementChildNamed( MBSIM%"enableOpenMBV" );
  if(e)
    property[ieO]->initializeUsingXML(e);
}

DOMElement* Frame::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Element::writeXMLFile(parent);
  if(not(property[ieO]->isDisabled())) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele1=D(doc)->createElement(MBSIM%"enableOpenMBV");
    property[ieO]->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  return ele0;
}

void Frame::initializeUsingXML2(DOMElement *element) {
  property[ieO]->initializeUsingXML(element);
}

DOMElement* Frame::writeXMLFile2(DOMNode *parent) {
  property[ieO]->writeXMLFile(parent);
  return 0;
}

Element *Frame::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  else if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  return NULL;
}

FixedRelativeFrame::FixedRelativeFrame(const string &str, Element *parent) : Frame(str,parent,false) {
  type = "FixedRelativeFrame";

  //property.push_back(new ChoiceProperty2("relativePosition",new VecPropertyFactory(3,LengthUnits()),"",4));
  property.push_back(new Vec_Property("relativePosition",LengthUnits()));
  property[irP]->setDisabling(true);
  property[irP]->setDisabled(true);

  //vector<Property*> p;
  //p.push_back(new MatProperty(getEye<string>(3,3,"1","0")));
  //p.push_back(new CardanProperty);
  //p.push_back(new OctaveExpressionProperty("","eye(3)"));
  //property.push_back(new VariableProperty("relativeOrientation",p,1));
  property.push_back(new RotMatProperty("relativeOrientation"));
  property[irO]->setDisabling(true);
  property[irO]->setDisabled(true);

  //refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
  property.push_back(new ParentFrameOfReferenceProperty("frameOfReference",getParent()->getFrame(0)->getXMLPath(this,true),this));
  property[ifo]->setDisabling(true);
  property[ifo]->setDisabled(true);
}

FixedRelativeFrame::~FixedRelativeFrame() {
}

void FixedRelativeFrame::initialize() {
  Frame::initialize();
  property[ifo]->initialize();
}

void FixedRelativeFrame::initializeUsingXML(DOMElement *element) {
  Frame::initializeUsingXML(element);
  DOMElement *ele1 = E(element)->getFirstElementChildNamed( MBSIM%"frameOfReference" );
  if(ele1) {
    property[ifo]->initializeUsingXML(ele1);
    property[ifo]->setDisabled(false);
  }
  ele1 = E(element)->getFirstElementChildNamed( MBSIM%"relativePosition" );
  if(ele1) {
    property[irP]->initializeUsingXML(ele1);
    property[irP]->setDisabled(false);
  }
  ele1 = E(element)->getFirstElementChildNamed( MBSIM%"relativeOrientation" );
  if(ele1) {
    property[irO]->initializeUsingXML(ele1);
    property[irO]->setDisabled(false);
  }
}

DOMElement* FixedRelativeFrame::writeXMLFile(DOMNode *parent) {

  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0 = Frame::writeXMLFile(parent);
  if(not(property[ifo]->isDisabled())) {
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"frameOfReference" );
    property[ifo]->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  if(not(property[irP]->isDisabled())) {
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"relativePosition" );
    property[irP]->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  if(not(property[irO]->isDisabled())) {
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"relativeOrientation" );
    property[irO]->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  return ele0;
}
