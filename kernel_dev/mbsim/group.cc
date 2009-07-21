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

#ifdef _OPENMP
#include <omp.h>
#endif

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
#pragma omp parallel for schedule(static) shared(t) default(none)
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateStateDependentVariables(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)object.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)object.size()>30) 
    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateStateDependentVariables(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }
  }

  void Group::updateJacobians(double t) {
#pragma omp parallel for schedule(static) shared(t) default(none)
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)object.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)object.size()>30) 
    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)link.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)link.size()>30) 
    for(int i=0; i<(int)link.size(); i++) {
      try { link[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); }
    }
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

  void Group::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    e=element->FirstChildElement();

    while(e && e->ValueStr()!=MBSIMNS"frameOfReference" &&
        e->ValueStr()!=MBSIMNS"relativePosition" &&
        e->ValueStr()!=MBSIMNS"relativeOrientation" &&
        e->ValueStr()!=MBSIMNS"frame" &&
        e->ValueStr()!=MBSIMNS"contour" &&
        ObjectFactory::getInstance()->createGroup(e)==0 &&
        ObjectFactory::getInstance()->createObject(e)==0 &&
        ObjectFactory::getInstance()->createLink(e)==0)
      e=e->NextSiblingElement();

    if(e && e->ValueStr()==MBSIMNS"frameOfReference") {
      string ref=e->Attribute("ref");
      if(ref.substr(0,3)!="../") { cout<<"ERROR! The reference frame "<<ref<<" must be one of the parent!"<<endl; _exit(1); }
      setFrameOfReference(getFrameByPath(ref));
      e=e->NextSiblingElement();
    }
    if(e && e->ValueStr()==MBSIMNS"relativePosition") {
      setRelativePosition(Vec(e->GetText()));
      e=e->NextSiblingElement();
    }
    if(e && e->ValueStr()==MBSIMNS"relativeOrientation") {
      setRelativeOrientation(SqrMat(e->GetText()));
      e=e->NextSiblingElement();
    }

    while(e && e->ValueStr()==MBSIMNS"frame") {
      TiXmlElement *ec=e->FirstChildElement();
      Frame *f=new Frame(ec->Attribute("name"));
#ifdef HAVE_OPENMBVCPPINTERFACE
      TiXmlElement *ee;
      if((ee=ec->FirstChildElement(MBSIMNS"enableOpenMBV")))
        f->enableOpenMBV(atof(ee->FirstChildElement(MBSIMNS"size")->GetText()), atof(ee->FirstChildElement(MBSIMNS"offset")->GetText()));
#endif
      ec=ec->NextSiblingElement();
      Frame *refF=0;
      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
        refF=getFrameByPath(ec->Attribute("ref"));
        ec=ec->NextSiblingElement();
      }
      Vec RrRF(ec->GetText());
      ec=ec->NextSiblingElement();
      SqrMat ARF(ec->GetText());
      addFrame(f, RrRF, ARF, refF);
      e=e->NextSiblingElement();
    }
    while(e && e->ValueStr()==MBSIMNS"contour") {
      TiXmlElement *ec=e->FirstChildElement();
      Contour *c=ObjectFactory::getInstance()->createContour(ec);
      TiXmlElement *contourElement=ec; // save for later initialization
      ec=ec->NextSiblingElement();
      Frame *refF=0;
      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
        refF=getFrameByPath(ec->Attribute("ref"));
        ec=ec->NextSiblingElement();
      }
      Vec RrRC(ec->GetText());
      ec=ec->NextSiblingElement();
      SqrMat ARC(ec->GetText());
      addContour(c, RrRC, ARC, refF);
      c->initializeUsingXML(contourElement);
      e=e->NextSiblingElement();
    }
    Group *g;
    while((g=ObjectFactory::getInstance()->createGroup(e))) {
      addDynamicSystem(g);
      g->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    Object *o;
    while((o=ObjectFactory::getInstance()->createObject(e))) {
      addObject(o);
      o->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
    Link *l;
    while((l=ObjectFactory::getInstance()->createLink(e))) {
      addLink(l);
      l->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
  }

}

