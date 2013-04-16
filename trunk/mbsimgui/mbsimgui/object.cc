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
#include "object.h"
#include <QtGui/QMenu>

using namespace std;

Object::Object(const string &str, Element *parent) : Element(str,parent), q0(0,false), u0(0,false) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMNS"initialGeneralizedPosition"));
  q0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMNS"initialGeneralizedVelocity"));
  u0.setProperty(new ExtPhysicalVarProperty(input));
}

Object::~Object() {
}

void Object::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  q0.initializeUsingXML(element);
  u0.initializeUsingXML(element);
}

TiXmlElement* Object::writeXMLFile(TiXmlNode *parent) {    
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  q0.writeXMLFile(ele0);
  u0.writeXMLFile(ele0);
  return ele0;
}

Element * Object::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  else if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  else  // local path
    throw MBSimError("Unknown identifier of container");
}

