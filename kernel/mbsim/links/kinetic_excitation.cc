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
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, KineticExcitation)

  KineticExcitation::KineticExcitation(const string &name) : FloatingFrameLink(name), F(nullptr), M(nullptr) {
    refFrame = secondFrame;
    evalOMBVForceColorRepresentation[0] = &KineticExcitation::evalNone;
    evalOMBVForceColorRepresentation[1] = &KineticExcitation::evalAboluteForce;
    evalOMBVMomentColorRepresentation[0] = &KineticExcitation::evalNone;
    evalOMBVMomentColorRepresentation[1] = &KineticExcitation::evalAboluteMoment;
  }

  KineticExcitation::~KineticExcitation() {
    delete F;
    delete M;
  }

  void KineticExcitation::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
      if(plotFeature[openMBV] and ombvArrow) {
        if(forceDir.cols()) {
          openMBVForce.resize(ombvArrow->getSideOfInteraction()==2?getNumberOfForces():getNumberOfForces()/2);
          for(size_t i=0; i<openMBVForce.size(); i++) {
            openMBVForce[i]=ombvArrow->createOpenMBV();
            openMBVForce[i]->setName(name+"_Force"+(openMBVForce.size()>1?to_string(i):string("")));
            parent->getOpenMBVGrp()->addObject(openMBVForce[i]);
          }
        }
        if(momentDir.cols()) {
          openMBVMoment.resize(ombvArrow->getSideOfInteraction()==2?getNumberOfForces():getNumberOfForces()/2);
          for(size_t i=0; i<openMBVMoment.size(); i++) {
            openMBVMoment[i]=ombvArrow->createOpenMBV();
            openMBVMoment[i]->setName(name+"_Moment"+(openMBVMoment.size()>1?to_string(i):string("")));
            parent->getOpenMBVGrp()->addObject(openMBVMoment[i]);
          }
        }
      }
    }
    else if(stage==unknownStage) {
      if(F and (F->getRetSize().first!=forceDir.cols())) throwError("Number of force directions does not match!");
      if(M and (M->getRetSize().first!=momentDir.cols())) throwError("Number of moment directions does not match!");
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
      if(openMBVForce.size()) {
        int off = ombvArrow->getSideOfInteraction()==0?getNumberOfForces()/2:0;
        for(size_t i=0; i<openMBVForce.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint=getPointOfApplication(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 WF = evalForce(off+i);
          data.push_back(WF(0));
          data.push_back(WF(1));
          data.push_back(WF(2));
          data.push_back((this->*evalOMBVForceColorRepresentation[ombvArrow->getColorRepresentation()])());
          openMBVForce[i]->append(data);
        }
      }
      if(openMBVMoment.size()) {
        int off = ombvArrow->getSideOfInteraction()==0?getNumberOfForces()/2:0;
        for(size_t i=0; i<openMBVMoment.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint=getPointOfApplication(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 WM = evalMoment(off+i);
          data.push_back(WM(0));
          data.push_back(WM(1));
          data.push_back(WM(2));
          data.push_back((this->*evalOMBVMomentColorRepresentation[ombvArrow->getColorRepresentation()])());
          openMBVMoment[i]->append(data);
        }
      }
    }
    FloatingFrameLink::plot();
  }

  void KineticExcitation::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat>(3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    if(e) setForceFunction(ObjectFactory::createAndInit<Function<VecV(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat>(3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentFunction");
    if(e) setMomentFunction(ObjectFactory::createAndInit<Function<VecV(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombvArrow = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,F?OpenMBVArrow::toHead:OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvArrow->initializeUsingXML(e);
    }
  }

}
