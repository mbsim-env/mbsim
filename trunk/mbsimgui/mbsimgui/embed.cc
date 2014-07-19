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
#include "embed.h"
#include "dynamic_system_solver.h"
#include "parameter.h"
#include "objectfactory.h"

namespace MBSimGUI {

  template <>
    DynamicSystemSolver* Embed<DynamicSystemSolver>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<DynamicSystemSolver*>(ObjectFactory::getInstance()->createGroup(element,parent));
    }

  template <>
    Group* Embed<Group>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Group*>(ObjectFactory::getInstance()->createGroup(element,parent));
    }

  template <>
    Object* Embed<Object>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Object*>(ObjectFactory::getInstance()->createObject(element,parent));
    }

  template <>
    Link* Embed<Link>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Link*>(ObjectFactory::getInstance()->createLink(element,parent));
    }

  template <>
    Observer* Embed<Observer>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Observer*>(ObjectFactory::getInstance()->createObserver(element,parent));
    }

  template <>
    Contour* Embed<Contour>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Contour*>(ObjectFactory::getInstance()->createContour(element,parent));
    }

  template <>
    Frame* Embed<Frame>::create(xercesc::DOMElement *element, Element *parent) {
      return static_cast<Frame*>(ObjectFactory::getInstance()->createFrame(element,parent));
    }

}
