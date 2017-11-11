/* Copyright (C) 2004-2014 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/group.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/objects/object.h"
#include "mbsim/links/link.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/observers/observer.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/xmlutils.h"

#include <openmbvcppinterface/frame.h>

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Group)

  Group::Group(const string &name) : DynamicSystem(name) {}

  Group::~Group() {}

  void Group::updateLLM() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLLM();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updateLLM();
  }

  void Group::updatedu() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatedu();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updatedu();
  }

  void Group::updatezd() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatezd();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->updateqd();
      (*i)->updateud();
    }

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (*i)->updatexd();

    for(vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (*i)->updatexd();
  }

  void Group::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);

    // search first element known by Group
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e)
      saved_frameOfReference=E(e)->getAttribute("ref");

    // frames
    e=E(element)->getFirstElementChildNamed(MBSIM%"frames");
    DOMElement *E=e->getFirstElementChild();
    while(E) {
      FixedRelativeFrame *f=new FixedRelativeFrame(MBXMLUtils::E(E)->getAttribute("name"));
      addFrame(f);
      f->initializeUsingXML(E);
      E=E->getNextElementSibling();
    }

    // contours
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    while(E) {
      RigidContour *c=ObjectFactory::createAndInit<RigidContour>(E);
      addContour(c);
      E=E->getNextElementSibling();
    }

    // groups
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    Group *g;
    while(E) {
      g=ObjectFactory::createAndInit<Group>(E);
      addGroup(g);
      E=E->getNextElementSibling();
    }

    // objects
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    Object *o;
    while(E) {
      o=ObjectFactory::createAndInit<Object>(E);
      addObject(o);
      E=E->getNextElementSibling();
    }

    // links
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    Link *l;
    while(E) {
      l=ObjectFactory::createAndInit<Link>(E);
      addLink(l);
      E=E->getNextElementSibling();
    }

    // constraints
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    Constraint *crt;
    while(E) {
      crt=ObjectFactory::createAndInit<Constraint>(E);
      addConstraint(crt);
      E=E->getNextElementSibling();
    }

    // observers
    e=e->getNextElementSibling();
    E=e->getFirstElementChild();
    Observer *obsrv;
    while(E) {
      obsrv=ObjectFactory::createAndInit<Observer>(E);
      addObserver(obsrv);
      E=E->getNextElementSibling();
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    if(e) {
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(e);
      I->setOpenMBVFrame(ombv.createOpenMBV());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"plotFeatureFrameI");
    while(e and MBXMLUtils::E(e)->getTagName()==MBSIM%"plotFeatureFrameI") {
      auto pf=getPlotFeatureFromXML(e);
      I->setPlotFeature(pf.first, pf.second);
      e=e->getNextElementSibling();
    }
  }

}
