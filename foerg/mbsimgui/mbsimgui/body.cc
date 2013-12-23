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
#include "body.h"
#include "frame.h"
#include "contour.h"
#include "basic_properties.h"

#define ifr 2

using namespace std;
using namespace MBXMLUtils;

Body::Body(const string &str, Element *parent) : Object(str,parent), q0(0,false), u0(0,false), R(0,false) {
//  q0.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIMNS"initialGeneralizedPosition"),"",4));

//  u0.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIMNS"initialGeneralizedVelocity"),"",4));

  //R.setProperty(new FrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIMNS"frameOfReference"));
  property.push_back(new FrameOfReferenceProperty("frameOfReference",getParent()->getFrame(0)->getXMLPath(this,true),this));
  property[ifr]->setDisabling(true);
  property[ifr]->setDisabled(true);

}

Body::Body(const Body &b) : Object(b), R(b.R) {
  for(unsigned int i=0; i<b.frame.size(); i++)
    frame.push_back(static_cast<Frame*>(b.frame[i]->clone()));;
  for(unsigned int i=0; i<b.contour.size(); i++)
    contour.push_back(static_cast<Contour*>(b.contour[i]->clone()));;
}

Body::~Body() {
  for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i) 
    delete *i;
  for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
    delete *i;
  for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
    delete *i;
}

Body& Body::operator=(const Body &b) {
  Object::operator=(b);
  for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
    delete *i;
  for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
    delete *i;
  for(vector<Element*>::iterator i = removedElement.begin(); i != removedElement.end(); ++i) 
    delete *i;
  frame.clear();
  contour.clear();
  removedElement.clear();
  R=b.R;
  for(unsigned int i=0; i<b.frame.size(); i++)
    frame.push_back(static_cast<Frame*>(b.frame[i]->clone()));;
  for(unsigned int i=0; i<b.contour.size(); i++)
    contour.push_back(static_cast<Contour*>(b.contour[i]->clone()));;
}

void Body::initialize() {
  property[ifr]->initialize();
}

void Body::addFrame(Frame* frame_) {
  frame.push_back(frame_);
}

void Body::addContour(Contour* contour_) {
  contour.push_back(contour_);
}

void Body::removeElement(Element* element) {
  if(dynamic_cast<Frame*>(element)) {
    for (vector<Frame*>::iterator it = frame.begin() ; it != frame.end(); ++it)
      if(*it==element) {
        //cout << "erase " << (*it)->getName() << endl;
        frame.erase(it);
        //delete (*it);
        break;
      }
  }
  else if(dynamic_cast<Contour*>(element)) {
    for (vector<Contour*>::iterator it = contour.begin() ; it != contour.end(); ++it)
      if(*it==element) {
        //cout << "erase " << (*it)->getName() << endl;
        contour.erase(it);
        //delete (*it);
        break;
      }
  }
  removedElement.push_back(element);
}

Frame* Body::getFrame(const string &name) {
  int i;
  for(i=0; i<frame.size(); i++) {
    if(frame[i]->getValue() == name)
      return frame[i];
  }
  return NULL;
}

Contour* Body::getContour(const string &name) {
  int i;
  for(i=0; i<contour.size(); i++) {
    if(contour[i]->getValue() == name)
      return contour[i];
  }
  return NULL;
}

void Body::initializeUsingXML(TiXmlElement *element) {
  Object::initializeUsingXML(element);
//  q0.initializeUsingXML(element);
//  u0.initializeUsingXML(element);
  TiXmlElement *ele1 = element->FirstChildElement( MBSIMNS"frameOfReference" );
  if(ele1) {
    property[ifr]->initializeUsingXML(ele1);
    property[ifr]->setDisabled(false);
  }
}

TiXmlElement* Body::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Object::writeXMLFile(parent);
 // q0.writeXMLFile(ele0);
 // u0.writeXMLFile(ele0);
  if(not(property[ifr]->isDisabled())) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
    property[ifr]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  return ele0;
}

Element * Body::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") // absolut path
    if(parent)
      return parent->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  else if (path.substr(0, 3)=="../") // relative path
    return parent->getByPathSearch(path.substr(3));
  else { // local path
    size_t pos0=path.find_first_of("[", 0);
    string container=path.substr(0, pos0);
    size_t pos1=path.find_first_of("]", pos0);
    string searched_name=path.substr(pos0+1, pos1-pos0-1);
    if (container=="Frame")
      return getFrame(searched_name);
    else if (container=="Contour")
      return getContour(searched_name);
  }
  return NULL;
}


