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

#include <config.h>
#include "embed.h"
#include "dynamic_system_solver.h"
#include "parameter.h"
#include "project.h"
#include "contour.h"
#include "link_.h"
#include "constraint.h"
#include "observer.h"
#include "solver.h"
#include "objectfactory.h"

namespace MBSimGUI {

  template <>
    DynamicSystemSolver* Embed<DynamicSystemSolver>::create(xercesc::DOMElement *element) {
      return new DynamicSystemSolver; // this object is very spezial -> do not use the ObjectFactory
    }

  template <>
    Group* Embed<Group>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Group>(element);
    }

  template <>
    Contour* Embed<Contour>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Contour>(element);
    }

  template <>
    FixedRelativeFrame* Embed<FixedRelativeFrame>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<FixedRelativeFrame>(element);
    }

  template <>
    NodeFrame* Embed<NodeFrame>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<NodeFrame>(element);
    }

  template <>
    Object* Embed<Object>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Object>(element);
    }

  template <>
    Link* Embed<Link>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Link>(element);
    }

  template <>
    Constraint* Embed<Constraint>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Constraint>(element);
    }

  template <>
    Observer* Embed<Observer>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Observer>(element);
    }

  template <>
    Solver* Embed<Solver>::create(xercesc::DOMElement *element) {
      return ObjectFactory::getInstance().create<Solver>(element);
    }

  template <>
    Project* Embed<Project>::create(xercesc::DOMElement *element) {
      return new Project;
    }

}
