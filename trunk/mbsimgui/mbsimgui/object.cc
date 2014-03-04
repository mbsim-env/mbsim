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
#include "objectfactory.h"
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

namespace MBSimGUI {

  Object::Object(const string &str, Element *parent) : Element(str,parent) {

  }

  Object* Object::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Object *object=ObjectFactory::getInstance()->createObject(e, parent);
    if(object) {
      object->initializeUsingXML(e);
      object->initialize();
    }
    return object;
  }

  void Object::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
  }

  DOMElement* Object::writeXMLFile(DOMNode *parent) {    
    DOMElement *ele0 = Element::writeXMLFile(parent);
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
    return NULL;
  }

}
