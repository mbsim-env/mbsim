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

Frame::Frame(const string &str, Element *parent, bool grey) : Element(str,parent), visu(0,true) {

 // properties->addTab("Plotting");
 // plotFeature.push_back(new ExtWidget("Plot global position", new PlotFeature("globalPosition"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global velocity", new PlotFeature("globalVelocity"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);
 // plotFeature.push_back(new ExtWidget("Plot global acceleration", new PlotFeature("globalAcceleration"),true));
 // properties->addToTab("Plotting",plotFeature[plotFeature.size()-1]);

  visu.setProperty(new OMBVFrameProperty("NOTSET",grey?"":MBSIMNS"enableOpenMBV"));
  ((OMBVFrameProperty*)visu.getProperty())->setID(getID());
}

Frame::~Frame() {
  //cout << "destroy frame" << endl;
}

Frame* Frame::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename);
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  map<string,string> dummy;
  incorporateNamespace(doc.FirstChildElement(), dummy);
  Frame *frame=ObjectFactory::getInstance()->createFrame(e,parent);
  if(frame)
    frame->initializeUsingXML(e);
  return frame;
}

void Frame::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  visu.initializeUsingXML(element);
}

TiXmlElement* Frame::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  visu.writeXMLFile(ele0);
  return ele0;
}

void Frame::initializeUsingXML2(TiXmlElement *element) {
  visu.initializeUsingXML(element);
}

TiXmlElement* Frame::writeXMLFile2(TiXmlNode *parent) {
  visu.writeXMLFile(parent);
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

FixedRelativeFrame::FixedRelativeFrame(const string &str, Element *parent) : Frame(str,parent,false), refFrame(0,false), position(0,false), orientation(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3), "m", MBSIMNS"relativePosition"));
  position.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new MatProperty(getEye<string>(3,3,"1","0")),"-",MBSIMNS"relativeOrientation"));
  orientation.setProperty(new ExtPhysicalVarProperty(input));

  refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0),this,MBSIMNS"frameOfReference"));
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
  position.initializeUsingXML(element);
  orientation.initializeUsingXML(element);
}

TiXmlElement* FixedRelativeFrame::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Frame::writeXMLFile(parent);
  refFrame.writeXMLFile(ele0);
  position.writeXMLFile(ele0);
  orientation.writeXMLFile(ele0);
  return ele0;
}

void FixedRelativeFrame::initializeUsingXML2(TiXmlElement *element) {
  refFrame.initializeUsingXML(element);
  string ref = ((ParentFrameOfReferenceProperty*)refFrame.getProperty())->getSavedFrameOfReference();
  if(ref[0]=='F')
    ((ParentFrameOfReferenceProperty*)refFrame.getProperty())->setSavedFrameOfReference(string("../")+ref);
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)position.getProperty())->getPhysicalVariableProperty(0))->setXmlName(MBSIMNS"position");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)position.getProperty())->getPhysicalVariableProperty(1))->setXmlName(MBSIMNS"position");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)orientation.getProperty())->getPhysicalVariableProperty(0))->setXmlName(MBSIMNS"orientation");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)orientation.getProperty())->getPhysicalVariableProperty(1))->setXmlName(MBSIMNS"orientation");
  position.initializeUsingXML(element);
  orientation.initializeUsingXML(element);
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)position.getProperty())->getPhysicalVariableProperty(0))->setXmlName(MBSIMNS"relativePosition");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)position.getProperty())->getPhysicalVariableProperty(1))->setXmlName(MBSIMNS"relativePosition");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)orientation.getProperty())->getPhysicalVariableProperty(0))->setXmlName(MBSIMNS"relativeOrientation");
  ((PhysicalVariableProperty*)((ExtPhysicalVarProperty*)orientation.getProperty())->getPhysicalVariableProperty(1))->setXmlName(MBSIMNS"relativeOrientation");
}

