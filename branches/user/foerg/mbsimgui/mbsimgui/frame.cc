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
#include "ombv_properties.h"
#include "objectfactory.h"

using namespace std;
using namespace MBXMLUtils;

Frame::Frame(const string &str, Element *parent, bool grey) : Element(str,parent) {

 // properties->addTab("Plotting");
 // plotFeature.push_back(new ExtWidget("Plot global position", new PlotFeature("globalPosition"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global velocity", new PlotFeature("globalVelocity"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global acceleration", new PlotFeature("globalAcceleration"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);

//  visu.setProperty(new OMBVFrameProperty("NOTSET",grey?"":MBSIMNS"enableOpenMBV",getID()));
  //property.push_back(new OMBVFrameProperty("enable OpenMBV",grey?"":MBSIMNS"enableOpenMBV",getID()));
  property.push_back(new OMBVFrameProperty("enable OpenMBV",grey?"":"",getID()));
  property[1]->setDisabling(true);
  property[1]->setDisabled(false);
}

Frame::~Frame() {
}

Frame* Frame::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    Frame *frame=ObjectFactory::getInstance()->createFrame(e,parent);
    if(frame) {
      frame->initializeUsingXML(e);
      frame->initialize();
    }
    return frame;
  }
  return 0;
}

void Frame::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  TiXmlElement *e = element->FirstChildElement( MBSIMNS"enableOpenMBV" );
  if(e)
    property[1]->initializeUsingXML(e);
}

TiXmlElement* Frame::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  if(not(property[1]->isDisabled())) {
    TiXmlElement *ele1=new TiXmlElement(MBSIMNS"enableOpenMBV");
    property[1]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  return ele0;
}

void Frame::initializeUsingXML2(TiXmlElement *element) {
  property[1]->initializeUsingXML(element);
}

TiXmlElement* Frame::writeXMLFile2(TiXmlNode *parent) {
  property[1]->writeXMLFile(parent);
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

FixedRelativeFrame::FixedRelativeFrame(const string &str, Element *parent) : Frame(str,parent,false), refFrame(0,false), orientation(0,false) {

  property.push_back(new ChoiceProperty2("relative position",new VecPropertyFactory(3,"",LengthUnits()),"",4));
  property[2]->setDisabling(true);
  property[2]->setDisabled(true);

  orientation.setProperty(new ChoiceProperty2(new RotMatPropertyFactory(MBSIMNS"relativeOrientation"),"",4));

  refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIMNS"frameOfReference"));
}

FixedRelativeFrame::~FixedRelativeFrame() {
}

void FixedRelativeFrame::initialize() {
  Frame::initialize();
  refFrame.initialize();
}

void FixedRelativeFrame::initializeUsingXML(TiXmlElement *element) {
  Frame::initializeUsingXML(element);
  refFrame.initializeUsingXML(element);
  TiXmlElement* ele1 = element->FirstChildElement( MBSIMNS"relativePosition" );
  if(ele1) {
    property[2]->initializeUsingXML(ele1);
    property[2]->setDisabled(false);
  }
  orientation.initializeUsingXML(element);
}

TiXmlElement* FixedRelativeFrame::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Frame::writeXMLFile(parent);
  refFrame.writeXMLFile(ele0);
  if(not(property[2]->isDisabled())) {
    TiXmlElement* ele1 = new TiXmlElement( MBSIMNS"relativePosition" );
    property[2]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  orientation.writeXMLFile(ele0);
  return ele0;
}
