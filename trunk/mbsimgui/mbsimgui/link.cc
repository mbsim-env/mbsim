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
#include "objectfactory.h"

using namespace std;
using namespace MBXMLUtils;

Link::Link(const string &str, Element *parent) : Element(str, parent) {
}

Link::~Link() {
}

Link* Link::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    Link *link=ObjectFactory::getInstance()->createLink(e,parent);
    if(link)
      link->initializeUsingXML(e);
    return link;
  }
  return 0;
}

Element * Link::getByPathSearch(string path) {
  if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  else // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  return NULL;
}

//void Link::initializeUsingXML(TiXmlElement *element) {
//}

//TiXmlElement* Link::writeXMLFile(TiXmlNode *parent) {    
//}
