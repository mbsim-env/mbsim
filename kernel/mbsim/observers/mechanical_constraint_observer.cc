/* Copyright (C) 2004-2017 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/observers/mechanical_constraint_observer.h"
#include "mbsim/constraints/mechanical_constraint.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MechanicalConstraintObserver)

  MechanicalConstraintObserver::MechanicalConstraintObserver(const std::string &name) : Observer(name), constraint(nullptr) {
    evalOMBVForceColorRepresentation[0] = &MechanicalConstraintObserver::evalNone;
    evalOMBVForceColorRepresentation[1] = &MechanicalConstraintObserver::evalAbsoluteForce;
    evalOMBVMomentColorRepresentation[0] = &MechanicalConstraintObserver::evalNone;
    evalOMBVMomentColorRepresentation[1] = &MechanicalConstraintObserver::evalAbsoluteMoment;
  }

  void MechanicalConstraintObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_constraint!="")
        setMechanicalConstraint(getByPath<MechanicalConstraint>(saved_constraint));
      Observer::init(stage, config);
    }
    else if(stage==preInit) {
      if(sideOfForceInteraction==unknownSideOfInteraction)
        throwError("(MechanicalConstraintObserver::init): side of force interaction unknown");
      if(sideOfMomentInteraction==unknownSideOfInteraction)
        throwError("(MechanicalConstraintObserver::init): side of moment interaction unknown");
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        //      if(openMBVForce) plotColumns.push_back("Force");
        //      if(openMBVMoment) plotColumns.push_back("Moment");
        if(ombvForce) {
          openMBVForce.resize(sideOfForceInteraction==both?constraint->getMechanicalLink()->getNumberOfLinks():constraint->getMechanicalLink()->getNumberOfLinks()/2);
          for(size_t i=0; i<openMBVForce.size(); i++) {
            openMBVForce[i]=ombvForce->createOpenMBV();
            //openMBVForce[i]->setName(string("Force")+(i<size_t(constraint->getMechanicalLink()->getNumberOfLinks()/2)?"R":"A")+to_string(i%size_t(constraint->getMechanicalLink()->getNumberOfLinks()/2)));
            openMBVForce[i]->setName(string("Force")+(openMBVForce.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVForce[i]);
          }
        }
        if(ombvMoment) {
          openMBVMoment.resize(sideOfMomentInteraction==both?constraint->getMechanicalLink()->getNumberOfLinks():constraint->getMechanicalLink()->getNumberOfLinks()/2);
          for(size_t i=0; i<openMBVMoment.size(); i++) {
            openMBVMoment[i]=ombvMoment->createOpenMBV();
            //openMBVMoment[i]->setName(string("Moment")+(i<size_t(constraint->getMechanicalLink()->getNumberOfLinks()/2)?"R":"A")+to_string(i%size_t(constraint->getMechanicalLink()->getNumberOfLinks()/2)));
            openMBVMoment[i]->setName(string("Moment")+(openMBVMoment.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVMoment[i]);
          }
        }
      }
    }
    else if(stage==unknownStage) {
      Observer::init(stage, config);
      if((openMBVForce.size() or openMBVMoment.size()) and not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(MechanicalConstraintObserver::init()): inverse kinetics not enabled");
    }
    else
      Observer::init(stage, config);
  }

  void MechanicalConstraintObserver::plot() {
    if(plotFeature[openMBV]) {
      int off = sideOfForceInteraction==action?constraint->getMechanicalLink()->getNumberOfLinks()/2:0;
      for(size_t i=0; i<openMBVForce.size(); i++) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=constraint->getMechanicalLink()->getPointOfApplication(off+i)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = constraint->getMechanicalLink()->evalForce(off+i);
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back((this->*evalOMBVForceColorRepresentation[ombvForce->getColorRepresentation()])());
        openMBVForce[i]->append(data);
      }
      off = sideOfMomentInteraction==action?constraint->getMechanicalLink()->getNumberOfLinks()/2:0;
      for(size_t i=0; i<openMBVMoment.size(); i++) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=constraint->getMechanicalLink()->getPointOfApplication(off+i)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WM = constraint->getMechanicalLink()->evalMoment(off+i);
        data.push_back(WM(0));
        data.push_back(WM(1));
        data.push_back(WM(2));
        data.push_back((this->*evalOMBVMomentColorRepresentation[ombvMoment->getColorRepresentation()])());
        openMBVMoment[i]->append(data);
      }
    }

    Observer::plot();
  }

  void MechanicalConstraintObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"mechanicalConstraint");
    saved_constraint=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if(e) {
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfForceInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfForceInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfForceInteraction=both;
        else sideOfForceInteraction=unknownSideOfInteraction;
      }
      ombvForce = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint));
      ombvForce->initializeUsingXML(e);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if(e) {
      DOMElement* ee=E(e)->getFirstElementChildNamed(MBSIM%"sideOfInteraction");
      if(ee) {
        string sideOfInteractionStr=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
        if(sideOfInteractionStr=="action") sideOfMomentInteraction=action;
        else if(sideOfInteractionStr=="reaction") sideOfMomentInteraction=reaction;
        else if(sideOfInteractionStr=="both") sideOfMomentInteraction=both;
        else sideOfMomentInteraction=unknownSideOfInteraction;
      }
      ombvMoment = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint));
      ombvMoment->initializeUsingXML(e);
    }
  }

  double MechanicalConstraintObserver::evalAbsoluteForce() {
    return nrm2(constraint->getMechanicalLink()->evalForce());
  }

  double MechanicalConstraintObserver::evalAbsoluteMoment() {
    return nrm2(constraint->getMechanicalLink()->evalMoment());
  }

}
