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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/objectfactory.h"
#include <mbsim/dynamic_system.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, KineticExcitation)

  KineticExcitation::KineticExcitation(const string &name) : FloatingFrameLink(name), F(NULL), M(NULL) {
    refFrameID = 1;
  }

  KineticExcitation::~KineticExcitation() {
    delete F;
    delete M;
  }

  void KineticExcitation::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_ref!="") connect(getByPath<Frame>(saved_ref));
      if(frame[0]==NULL) frame[0] = static_cast<DynamicSystem*>(parent)->getFrameI();
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and openMBVArrow) {
        if(forceDir.cols()) {
          openMBVForce=OpenMBV::ObjectFactory::create(openMBVArrow);
          openMBVForce->setName(name+"_Force");
          parent->getOpenMBVGrp()->addObject(openMBVForce);
        }
        if(momentDir.cols()) {
          openMBVMoment=OpenMBV::ObjectFactory::create(openMBVArrow);
          openMBVMoment->setName(name+"_Moment");
          parent->getOpenMBVGrp()->addObject(openMBVMoment);
        }
      }
    }
    else if(stage==unknownStage) {
      if(F  and (F->getRetSize().first!=forceDir.cols())) THROW_MBSIMERROR("Number of force directions does not match!");
      if(M  and (M->getRetSize().first!=momentDir.cols())) THROW_MBSIMERROR("Number of moment directions does not match!");
    }
    FloatingFrameLink::init(stage, config);
    if(F) F->init(stage, config);
    if(M) M->init(stage, config);
  }

  void KineticExcitation::updatelaF() {
    if(F) lambdaF = (*F)(getTime());
    updlaF = false;
  }

  void KineticExcitation::updatelaM() {
    if(M) lambdaM = (*M)(getTime());
    updlaM = false;
  }

  void KineticExcitation::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void KineticExcitation::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }

  void KineticExcitation::setForceFunction(Function<VecV(double)> *func) {
    F=func;
    F->setParent(this);
    F->setName("Force");
  }

  void KineticExcitation::setMomentFunction(Function<VecV(double)> *func) {
    M=func;
    M->setParent(this);
    M->setName("Moment");
  }

  void KineticExcitation::plot() {
    if(plotFeature[openMBV]) {
      if(openMBVForce) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=P[1]->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = evalForce();
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(nrm2(WF));
        openMBVForce->append(data);
      }
      if(openMBVMoment) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=P[1]->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WM = evalMoment();
        data.push_back(WM(0));
        data.push_back(WM(1));
        data.push_back(WM(2));
        data.push_back(nrm2(WM));
        openMBVMoment->append(data);
      }
    }
    FloatingFrameLink::plot();
  }

  void KineticExcitation::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat>(3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    if(e) setForceFunction(ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat>(3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentFunction");
    if(e) setMomentFunction(ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
        OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
        openMBVArrow=ombv.createOpenMBV(e); 
    }
  }

}
