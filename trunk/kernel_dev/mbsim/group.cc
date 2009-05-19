/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/group.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/order_one_dynamics.h"
#include "mbsim/frame.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/simpleattribute.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Group::Group(const string &name) : DynamicSystem(name) {}

  Group::~Group() {}

  void Group::facLLM() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->facLLM();

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->facLLM();
  }

  void Group::updateStateDependentVariables(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateStateDependentVariables(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateStateDependentVariables(t);
  }

  void Group::updateJacobians(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateJacobians(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Group::updatedu(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatedu(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updatedu(t,dt);
  }

  void Group::updatezd(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatezd(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updatezd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (*i)->updatexd(t);

    for(vector<OrderOneDynamics*>::iterator i = orderOneDynamics.begin(); i!= orderOneDynamics.end(); ++i) 
      (*i)->updatexd(t);
  }

  void Group::updateInverseKineticsJacobians(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateInverseKineticsJacobians(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateInverseKineticsJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Group::addDynamicSystem(DynamicSystem *sys, const Vec &RrRD, const SqrMat &ARD, const Frame* refFrame) {
    DynamicSystem::addDynamicSystem(sys);

    if(dynamic_cast<const Frame*>(refFrame)!=0) {
      int i = frameIndex(static_cast<const Frame*>(refFrame));
      IrOD.push_back(IrOF[i] + AIF[i]*RrRD);
      AID.push_back(AIF[i]*ARD);
    }
    else {
      IrOD.push_back(RrRD);
      AID.push_back(ARD);
    }
  }

  void Group::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMNS"ARD");
    e=e->NextSiblingElement();

    while(e->ValueStr()==MBSIMNS"Frame") {
      addFrame(e->Attribute("name"), Vec(e->FirstChildElement(MBSIMNS"RrRF")->GetText()),
                                     SqrMat(e->FirstChildElement(MBSIMNS"ARF")->GetText()));
      TiXmlElement *ee;
      if((ee=e->FirstChildElement(MBSIMNS"enableOpenMBV")))
        getFrame(e->Attribute("name"))->enableOpenMBV(atof(ee->FirstChildElement(MBSIMNS"size")->GetText()),
                                                      atof(ee->FirstChildElement(MBSIMNS"offset")->GetText()));
      e=e->NextSiblingElement();
    }
    Contour *c;
    while((c=ObjectFactory::createContour(e))) {
      addContour(c, Vec(e->FirstChildElement(MBSIMNS"RrRC")->GetText()),
                    SqrMat(e->FirstChildElement(MBSIMNS"ARC")->GetText()));
      c->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    Group *g;
    while((g=ObjectFactory::createGroup(e))) {
      addDynamicSystem(g, Vec(e->FirstChildElement(MBSIMNS"RrRD")->GetText()),
                          SqrMat(e->FirstChildElement(MBSIMNS"ARD")->GetText()));
      g->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    Object *o;
    while((o=ObjectFactory::createObject(e))) {
      addObject(o);
      o->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    Link *l;
    while((l=ObjectFactory::createLink(e))) {
      addLink(l);
      l->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
  }

}

