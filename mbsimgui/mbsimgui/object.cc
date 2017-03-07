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
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Object::Object(const string &str, Element *parent) : Element(str,parent) {
    addPlotFeature("generalizedPosition");
    addPlotFeature("generalizedVelocity");
    addPlotFeature("derivativeOfGeneralizedPosition");
    addPlotFeature("generalizedAcceleration");
    addPlotFeature("energy");
  }

  Object* Object::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Object *object=Embed<Object>::createAndInit(e,parent);
    if(object)
      object->initialize();
    return object;
  }

  DOMElement* Object::initializeUsingXML(DOMElement *element) {
    return Element::initializeUsingXML(element);
  }

  DOMElement* Object::writeXMLFile(DOMNode *parent) {    
    DOMElement *ele0 = Element::writeXMLFile(parent);
    return ele0;
  }

}
