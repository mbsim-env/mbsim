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
    DOMElement *e;
    Element::initializeUsingXML(element);
    e=element->getFirstElementChild();

    // search first element known by Group
    while(e && E(e)->getTagName()!=MBSIM%"frameOfReference" &&
        E(e)->getTagName()!=MBSIM%"position" &&
        E(e)->getTagName()!=MBSIM%"orientation" &&
        E(e)->getTagName()!=MBSIM%"frames")
      e=e->getNextElementSibling();

    if(e && E(e)->getTagName()==MBSIM%"frameOfReference") {
      saved_frameOfReference=E(e)->getAttribute("ref");
      e=e->getNextElementSibling();
    }

    if(e && E(e)->getTagName()==MBSIM%"position") {
      setPosition(getVec3(e));
      e=e->getNextElementSibling();
    }

    if(e && E(e)->getTagName()==MBSIM%"orientation") {
      setOrientation(getSqrMat3(e));
      e=e->getNextElementSibling();
    }

    // frames
    DOMElement *E=e->getFirstElementChild();
    while(E) {
      FixedRelativeFrame *f=new FixedRelativeFrame(MBXMLUtils::E(E)->getAttribute("name"));
      addFrame(f);
      f->initializeUsingXML(E);
      E=E->getNextElementSibling();
    }
    e=e->getNextElementSibling();

    // contours
    E=e->getFirstElementChild();
    while(E) {
      RigidContour *c=ObjectFactory::createAndInit<RigidContour>(E);
      addContour(c);
      E=E->getNextElementSibling();
    }
    e=e->getNextElementSibling();

    // groups
    E=e->getFirstElementChild();
    Group *g;
    while(E) {
      g=ObjectFactory::createAndInit<Group>(E);
      addGroup(g);
      E=E->getNextElementSibling();
    }
    e=e->getNextElementSibling();

    // objects
    E=e->getFirstElementChild();
    Object *o;
    while(E) {
      o=ObjectFactory::createAndInit<Object>(E);
      addObject(o);
      E=E->getNextElementSibling();
    }
    e=e->getNextElementSibling();

    // links
    E=e->getFirstElementChild();
    Link *l;
    while(E) {
      l=ObjectFactory::createAndInit<Link>(E);
      addLink(l);
      E=E->getNextElementSibling();
    }
    e=e->getNextElementSibling();

    // constraints
    if(e && MBXMLUtils::E(e)->getTagName()==MBSIM%"constraints") {
      E=e->getFirstElementChild();
      Constraint *crt;
      while(E) {
        crt=ObjectFactory::createAndInit<Constraint>(E);
        addConstraint(crt);
        E=E->getNextElementSibling();
      }
      e=e->getNextElementSibling();
    }

    // observers
    if(e && MBXMLUtils::E(e)->getTagName()==MBSIM%"observers") {
      E=e->getFirstElementChild();
      Observer *obsrv;
      while(E) {
        obsrv=ObjectFactory::createAndInit<Observer>(E);
        addObserver(obsrv);
        E=E->getNextElementSibling();
      }
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameI");
    if(e) {
      OpenMBVFrame ombv;
      I->setOpenMBVFrame(ombv.createOpenMBV(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"plotFeatureFrameI");
    while(e and MBXMLUtils::E(e)->getTagName()==MBSIM%"plotFeatureFrameI") {
      PlotFeatureStatus status = initializePlotFeatureStatusUsingXML(e);
      I->setPlotFeature(MBXMLUtils::E(e)->getAttribute("feature").substr(1), status);
      e=e->getNextElementSibling();
    }
  }

}
