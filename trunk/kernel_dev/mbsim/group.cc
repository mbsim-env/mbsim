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
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)object.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)object.size()>30) 
    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateStateDependentVariables(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void Group::updateJacobians(double t) {
#pragma omp parallel for schedule(static) shared(t) default(none)
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)object.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)object.size()>30) 
    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

#pragma omp parallel for schedule(dynamic, max(1,(int)link.size()/(10*omp_get_num_threads()))) shared(t) default(none) if((int)link.size()>30) 
    for(int i=0; i<(int)link.size(); i++) {
      try { link[i]->updateJacobians(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
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

    // search first element known by Group
    while(e && e->ValueStr()!=MBSIMNS"frameOfReference" &&
        e->ValueStr()!=MBSIMNS"position" &&
        e->ValueStr()!=MBSIMNS"orientation" &&
        e->ValueStr()!=MBSIMNS"frames")
      e=e->NextSiblingElement();

    if(e && e->ValueStr()==MBSIMNS"frameOfReference") {
      string ref=e->Attribute("ref");
      if(ref.substr(0,3)!="../") { cout<<"ERROR! The reference frame "<<ref<<" must be one of the parent!"<<endl; _exit(1); }
      setFrameOfReference(getFrameByPath(ref));
      e=e->NextSiblingElement();
    }
    if(e && e->ValueStr()==MBSIMNS"position") {
      setPosition(Vec(e->GetText()));
      e=e->NextSiblingElement();
    }
    if(e && e->ValueStr()==MBSIMNS"orientation") {
      setOrientation(SqrMat(e->GetText()));
      e=e->NextSiblingElement();
    }

    TiXmlElement *E=e->FirstChildElement();
    while(E && E->ValueStr()==MBSIMNS"frame") {
      TiXmlElement *ec=E->FirstChildElement();
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
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();
    E=e->FirstChildElement();
    while(E && E->ValueStr()==MBSIMNS"contour") {
      TiXmlElement *ec=E->FirstChildElement();
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
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();
    E=e->FirstChildElement();
    Group *g;
    while((g=ObjectFactory::getInstance()->createGroup(E))) {
      addGroup(g);
      g->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();
    E=e->FirstChildElement();
    Object *o;
    while((o=ObjectFactory::getInstance()->createObject(E))) {
      addObject(o);
      o->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();
    E=e->FirstChildElement();
    Link *l;
    while((l=ObjectFactory::getInstance()->createLink(E))) {
      addLink(l);
      l->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
  }

}

